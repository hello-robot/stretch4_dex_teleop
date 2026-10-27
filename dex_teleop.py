import numpy as np
import cv2
import time
import webcam_teleop_interface as wt
import simple_ik as si
import argparse
import gripper_to_goal as gg
import goal_from_teleop as gt
import dex_teleop_parameters as dt
import pprint as pp
import loop_timer as lt


def draw_slide_lift_overlay(image, goal_from_markers, lift_limits, wrist_to_lift,
                            actual_lift=None, x=20, y=110):
    """Draw the --slide_lift_range state onto the Dex Teleop window.

    Shown as lift values, the same numbers the robot reports via
    stretch_status and arm_lift_readout.py, so the overlay and the robot
    agree. The band is the stretch of lift you can reach right now; sliding
    moves it along the robot's full travel.

    lift_limits   -- (lowest, highest) lift the robot can reach
    wrist_to_lift -- fixed height of the wrist above the lift carriage
    actual_lift   -- the robot's current lift, drawn as a marker on the bar
    """
    g = goal_from_markers
    font = cv2.FONT_HERSHEY_SIMPLEX
    grey, green, amber, white = (170, 170, 170), (0, 255, 0), (0, 200, 255), (255, 255, 255)
    red = (0, 0, 255)
    lift_lo, lift_hi = lift_limits

    def line(text, colour, dy):
        cv2.putText(image, text, (x, y + dy), font, 0.8, colour, 2)

    dist = g.tongs_dist_from_camera
    line('tongs %s      down <%.2f   up >%.2f' %
         ('%.2f m' % dist if dist is not None else '  -- ',
          g.slide_lift_range_down, g.slide_lift_range_up), white, 0)

    sliding_down = g.in_sliding_region and dist is not None and dist < g.slide_lift_range_down
    sliding_up = g.in_sliding_region and dist is not None and dist > g.slide_lift_range_up
    if g.in_sliding_region:
        line('SLIDING ' + ('DOWN' if sliding_down else 'UP'), green, 32)
    else:
        line('idle', grey, 32)

    # With --slide_lift_range the marker is pinned to the region edge beyond
    # slide_lift_range_down/up, so tracking only spans that inner range -
    # past it you slide instead of tracking. Using the full tongs range here
    # would overstate the reach by the width of the two sliding regions.
    base = g.center_wrist_position[2] - g.teleop_origin[2] + g.lift_range_offset - wrist_to_lift
    win_lo = max(g.slide_lift_range_down + base, lift_lo)
    win_hi = min(g.slide_lift_range_up + base, lift_hi)

    # Amber only when sliding has stopped getting anywhere. The band resting
    # against a boundary you are not travelling towards is normal.
    stuck = ((sliding_down and win_lo <= lift_lo + 1e-4) or
             (sliding_up and win_hi >= lift_hi - 1e-4))
    text = 'lift  [%.2f - %.2f] m' % (win_lo, win_hi)
    if actual_lift is not None:
        text += '    now %.2f' % actual_lift
    line(text, amber if stuck else white, 64)

    bw, bh = 440, 26
    by = y + 86
    cv2.rectangle(image, (x, by), (x + bw, by + bh), grey, 2)
    span = lift_hi - lift_lo
    if span > 1e-9:
        def px(v):
            return x + int(bw * min(max((v - lift_lo) / span, 0.0), 1.0))
        cv2.rectangle(image, (px(win_lo), by), (px(win_hi), by + bh),
                      amber if stuck else green, -1)

    # Where the robot actually is, so you can see it track the band rather
    # than lag behind or sit against a stop.
    if actual_lift is not None and span > 1e-9:
        ax = x + int(bw * min(max((actual_lift - lift_lo) / span, 0.0), 1.0))
        tip, half, h = (ax, by - 2), 11, 16
        cv2.fillPoly(image, [np.array([tip, (ax - half, by - h), (ax + half, by - h)])], red)
        cv2.line(image, (ax, by - 2), (ax, by + bh + 2), red, 2)

    cv2.putText(image, '%.2f' % lift_lo, (x, by + bh + 26), font, 0.7, grey, 2)
    hi_text = '%.2f m' % lift_hi
    (hw, _), _ = cv2.getTextSize(hi_text, font, 0.7, 2)
    cv2.putText(image, hi_text, (x + bw - hw, by + bh + 26), font, 0.7, grey, 2)


if __name__ == '__main__':

    args = dt.get_arg_parser().parse_args()
    use_fastest_mode = args.fast
    manipulate_on_ground = args.ground
    left_handed = args.left
    slide_lift_range = args.slide_lift_range
    show_gui = not args.no_gui
        
    # 'slow', 'default', 'fast' and 'max' are the motion profiles Hello Robot
    # defines for this hardware, in robot_params.
    #
    # --fast previously asked for 'fastest_stretch_3', which robot_move does not
    # define: it warned and silently fell back to 'default'. Those hand-tuned
    # 'fastest_stretch_*' profiles were removed in d5ddb0e when this was forked
    # for Stretch 4; they were tuned on Stretch 3 hardware and exceeded the
    # factory 'max' values, so they are not reinstated here. 'fast' is the
    # factory profile between 'default' and 'max'.
    if use_fastest_mode:
        robot_speed = 'fast'
    else:
        robot_speed = 'slow'
    print('running with robot_speed =', robot_speed)
    if use_fastest_mode:
        print("WARNING: Expect lower contact sensitivity in Fast Mode")

    lift_middle = dt.get_lift_middle(manipulate_on_ground)
    center_configuration = dt.get_center_configuration(lift_middle)
    starting_configuration = dt.get_starting_configuration(lift_middle)
    
    if left_handed: 
        webcam_aruco_detector = wt.WebcamArucoDetector(tongs_prefix='left', visualize_detections=False)
    else:
        webcam_aruco_detector = wt.WebcamArucoDetector(tongs_prefix='right', visualize_detections=False)
    
    # Initialize IK
    simple_ik = si.SimpleIK()

    # For the --slide_lift_range overlay: the lift's reachable range, and the
    # fixed height of the wrist above the lift carriage. Derived from the IK
    # rather than hardcoded, so they follow the robot's own model.
    lift_limits = simple_ik.all_joint_limits['lift_joint']
    wrist_to_lift = simple_ik.fk_rotary_base(
        {'mobile_base_rotation_joint': 0.0, 'lift_joint': 0.0,
         'arm_l4_joint': 0.01})[2]

    # Define the center position for the wrist that corresponds with
    #the teleop origin.
    center_wrist_position = simple_ik.fk_rotary_base(center_configuration)

    gripper_to_goal = gg.GripperToGoal(robot_speed, starting_configuration, dt.robot_allowed_to_move)

    goal_from_markers = gt.GoalFromMarkers(dt.teleop_origin, center_wrist_position, slide_lift_range=slide_lift_range)


    loop_timer = lt.LoopTimer()
    print_timing = False
    print_goal = False
    
    if show_gui:
        print('Dex Teleop window: press q to quit.')

    while True:
        loop_timer.start_of_iteration()
        markers, teleop_image = webcam_aruco_detector.process_next_frame()
        goal_dict = goal_from_markers.get_goal_dict(markers)
        if goal_dict:
            if print_goal:
                print('goal_dict =')
                pp.pprint(goal_dict)
            gripper_to_goal.update_goal(**goal_dict)

        if show_gui:
            # The ArUco detector already drew the detected markers onto this
            # image, so it shows which markers are being tracked.
            if teleop_image is not None:
                display_image = teleop_image.copy()
            else:
                display_image = np.zeros((720, 1280, 3), dtype=np.uint8)

            if goal_dict:
                status, color = 'tracking tongs', (0, 255, 0)
            else:
                status, color = 'no tongs detected', (0, 0, 255)
            cv2.putText(display_image, status, (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 3)

            # Base debugging overlay and keys, all tied to base logging so the
            # controls can never be touched by accident during normal teleop,
            # where nothing on screen would show what changed.
            base_tuning = gripper_to_goal.base_command_log is not None
            if base_tuning:
                # Elapsed time on the same clock as the base command log, so
                # what you read off the screen matches the CSV's 't' column.
                elapsed = 't %.1f s' % (time.time() - gripper_to_goal.base_command_log_t0)
                (tw, _), _ = cv2.getTextSize(elapsed, cv2.FONT_HERSHEY_SIMPLEX, 1.2, 3)
                cv2.putText(display_image, elapsed,
                            (display_image.shape[1] - tw - 20, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)

                mm_per_rad, _ = gripper_to_goal.base_drift_rate()
                if mm_per_rad is None:
                    drift = 'drift --.- mm/rad  (rotate to measure)'
                else:
                    drift = 'drift %.1f mm/rad  (last %.0fs)' % (
                        mm_per_rad, gripper_to_goal.base_stats_window_s)
                clamp = gripper_to_goal.base_rotation_max_command
                clamp_text = ('clamp OFF        c toggle' if clamp <= 0.0 else
                              'clamp %.2f rad   c toggle, , / . adjust' % clamp)
                for i, line in enumerate(
                        ['base gain %.2f   [ slower / ] faster'
                         % gripper_to_goal.base_rotation_gain, clamp_text, drift]):
                    cv2.putText(display_image, line, (20, 110 + i * 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)

            if slide_lift_range:
                try:
                    actual_lift = gripper_to_goal.robot.status['lift']['pos']
                except (KeyError, AttributeError):
                    actual_lift = None
                draw_slide_lift_overlay(display_image, goal_from_markers,
                                        lift_limits, wrist_to_lift, actual_lift)

            display_image = cv2.resize(display_image, (1280, 720))
            cv2.imshow('Dex Teleop', display_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print('q pressed, quitting.')
                break
            elif base_tuning:
                if key in (ord('['), ord(']')):
                    gain = gripper_to_goal.base_rotation_gain
                    gain += 0.05 if key == ord(']') else -0.05
                    gripper_to_goal.base_rotation_gain = round(min(1.0, max(0.05, gain)), 2)
                    print('base_rotation_gain = %.2f' % gripper_to_goal.base_rotation_gain)
                elif key == ord('c'):
                    # Toggle between no clamp and the configured value, so the
                    # clamp can be judged independently of the gain.
                    gripper_to_goal.base_rotation_max_command = (
                        0.0 if gripper_to_goal.base_rotation_max_command > 0.0
                        else dt.base_rotation_max_command)
                    print('base_rotation_max_command = %.2f'
                          % gripper_to_goal.base_rotation_max_command)
                elif key in (ord(','), ord('.')):
                    clamp = gripper_to_goal.base_rotation_max_command
                    clamp += 0.05 if key == ord('.') else -0.05
                    gripper_to_goal.base_rotation_max_command = round(min(2.0, max(0.0, clamp)), 2)
                    print('base_rotation_max_command = %.2f'
                          % gripper_to_goal.base_rotation_max_command)

        loop_timer.end_of_iteration()
        if print_timing: 
            loop_timer.pretty_print()

    if show_gui:
        cv2.destroyAllWindows()




##############################################################
## NOTES
##############################################################

#######################################
#
# Overview
#
# Dexterous teleoperation uses a marker dictionary representing either
# a real or virtual ArUco marker specified with respect to the
# camera's frame of reference. The marker's position controls the
# robot's wrist position via inverse kinematics (IK). The marker's
# orientation directly controls the joints of the robot's dexterous
# wrist.
#
#######################################

#######################################
#
# The following coordinate systems are important to this teleoperation
# code
#
#######################################

#######################################
# ArUco Coordinate System
#
# Origin in the middle of the ArUco marker.
#
# x-axis
# right side when looking at marker is pos
# left side when looking at marker is neg

# y-axis
# top of marker is pos
# bottom of marker is neg

# z-axis
# normal to marker surface is pos
# pointing into the marker surface is neg
#
#######################################

#######################################
# Camera Coordinate System
#
# Camera on the floor looking with the top of the camer facing away
# from the person.
#
# This configuration matches the world frame's coordinate system with
# a different origin that is mostly just translated along the x and y
# axes.
#
# Origin likely at the optical cemter of a pinhole
# model of the camera.
#
# The descriptions below describe when the robot's mobile base is at
# theta = 0 deg.
#
# x-axis
# human left is pos / robot forward is pos
# human right is neg / robot backward is neg

# y-axis
# human arm extended is neg / robot arm extended is neg
# human arm retracted is pos / robot arm retracted is pos

# z-axis
# up is positive for person and the robot
# down is negative for person and the robot
#
#######################################

#######################################
# IK World Frame Coordinate System
#
# Origin at the axis of rotation of the mobile
# base on the floor.
#
# x-axis
# human/robot left is pos
# human/robot right is neg

# y-axis
# human/robot forward is neg
# human/robot backward is pos

# z-axis
# human/robot up is pos
# human/robot down is neg
#
#######################################

#######################################
# Robot Wrist Control

# wrist yaw
#     - : deployed direction
#     0 : straight out parallel to the telescoping arm
#     + : stowed direction

# wrist pitch
#     - : up
#     0 : horizontal
#     + : down

# wrist roll
#     - : 
#     0 : horizontal
#     + :
#
#######################################

##############################################################
