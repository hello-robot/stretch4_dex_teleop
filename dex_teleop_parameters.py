import numpy as np
import argparse


def get_arg_parser():
    parser = argparse.ArgumentParser(
        prog='Stretch Dexterous Teleop',
        description='This application enables a human operator to control Stretch using ArUco markers and a Logitech C930e webcam on the floor looking up with a light ring. Currently, the code supports tongs with ArUco markers attached.',
    )
    parser.add_argument('-l', '--left', action='store_true', help = 'Use Stretch as a left-handed robot. This requires left-hand tongs, which have distinct ArUco markers from right-hand tongs.')
    parser.add_argument('-f', '--fast', action='store_true', help = 'Move Stretch at the fastest available speed. The default is to move the robot at the slowest available speed')
    parser.add_argument('-g', '--ground', action='store_true', help = 'Manipulate at ground level up to around tabletop height. The default is to manipulate from tabletop height up to countertop height.')
    parser.add_argument('-m', '--multiprocessing', action='store_true', help = 'Write goals to shared memory using Python multiprocessing.')
    parser.add_argument('--no-gui', action='store_true', help = 'Do not open the Dex Teleop window. Use this when running headless or over SSH without X forwarding.')
    parser.add_argument('-i', '--slide_lift_range', action='store_true', help = 'Holding the tongs high will gradually slide the lift range of motion upward. Holding them low will gradually slide the lift range of motion downward. The default is to use a fixed range of motion for the lift.')
    
    return parser


# Measurements for the tongs
tongs_to_use = '56mm' #'50mm' #'44mm'

if tongs_to_use == '56mm':
    # 50mm ArUco marker tongs
    tongs_cube_side = 0.0735
    tongs_pin_joint_to_marker_center = 0.112
    tongs_pin_joint_to_tong_tip = 0.102
    #tongs_marker_center_to_tong_tip = (tongs_cube_side / 2.0) + 0.0125
    tongs_marker_center_to_tong_tip = (tongs_cube_side / 2.0) + 0.0105
    tongs_open_grip_width = 0.145
    tongs_closed_grip_width = 0.088
    

# The maximum and minimum goal_wrist_position z values do not
# need to be perfect due to joint limit checking performed by
# the SimpleIK based on the specialized URDF joint
# limits. They are specified with respect to the robot's
# coordinate system.
#
# On Stretch 4 the wrist reaches z = 1.42 m with the lift at its 1.20 m
# maximum. The Stretch 3 value here was 1.35, which put a ceiling 7 cm below
# what the robot can actually reach. Being slightly generous is safe, since
# SimpleIK does the real clamping; being short is not, because it silently
# removes travel.
goal_max_position_z = 1.45
goal_min_position_z = 0.07


# Regions at the top and bottom of the allowable tongs range
# are reserved for changing the range over which the lift is
# operating. This sliding region enables a user to use the
# lift's full range without restarting the code.
lift_sliding_region_height = 0.3


# Set how fast the lift will be translated when being slid.
lift_range_sliding_speed_multiplier = 10.0 #4.0
lift_range_offset_change_per_timestep = 0.001 * lift_range_sliding_speed_multiplier


# Fraction of the gripper's travel held back from a hard end stop.
#
# Commanding a gripper to its own limit makes it hold there against the stop.
# Measured on 4032 with the parallel jaw: commanding 0.0800 m reached only
# 0.0793 m and sat stalled at 73-82% effort, heating 65 to 71 C in under a
# minute with nothing visible to show for it, until the gripper stopped
# responding. Closing fully did the same, jaw against jaw.
#
# Always applied at the open end. Applied at the closed end only for tools
# with no squeeze range: the compliant grippers' closed position is past
# contact by design and the rubber absorbs it, but the parallel jaw's closed
# position is contact, so holding there is metal on metal. An object between
# the jaws stops them before the limit anyway, so the margin only limits how
# thin a thing can be gripped - 1.6 mm on the parallel jaw.
# The two ends need different margins because the shortfall differs. On the
# parallel jaw, commanding 0.0800 m reached only 0.0793 m (0.7 mm short) but
# closing reached 0.0002 m (0.2 mm short), so the closed end needs far less.
# At these values the jaw works between 0.4 mm and 78.4 mm of aperture.
gripper_open_margin = 0.02
gripper_closed_margin = 0.005


# The gripper's command range is no longer set here. It is read from the fitted
# tool at startup, because it differs between grippers: the Stretch Gripper v2
# opens to 1056 pct where the original opens to 300, and the parallel jaw
# gripper is commanded in metres of fingertip aperture. See find_gripper() in
# gripper_to_goal.py.


# Lower limit for wrist pitch. Allowing angles close to -Pi/2 will
# allow the gripper to point almost straight down, which is near a
# singularity. When the wrist is straight down with pitch = -Pi/2, the
# wrist roll and wrist yaw joints have rotational axes that are
# parallel resulting in gimbal lock. This can result in undesirable
# motions and unintuitive configurations. Changing this variable can
# be used to increase the risk of issues in order to point the gripper
# closer to straight down. A value of None will result in the pitch
# being allowed to move over the entire range of motion allowed by the
# robot's exported URDF. The default lowest pitch was -1.57 rad on the
# default URDF used during development (-89.954373836 deg)

#AFTER CHANGING THIS VARIABLE YOU NEED TO RUN
# generate_ik_urdfs.py FOR IT TO TAKE EFFECT.
#
# The generator clips to the robot's own limit, taking whichever is more
# conservative. On Stretch 4 the model already limits wrist pitch to -1.135
# rad, so the value below is currently inert - only a value above -1.135
# (less negative) will change the generated URDF.

wrist_pitch_lower_limit =  -0.8 * (np.pi/2.0)


# HOW DEX TELEOP CONNECTS TO THE ROBOT
#
# True  : connect through the running stretch_body_server using RobotClient.
#         Dex Teleop then shares the robot with ROS, nav and the web
#         interface, and keeps the server's safety sentries (joint runaway,
#         self collision, guarded contact).
# False : take the hardware directly with Robot(). This requires stopping the
#         server first, and runs without those sentries:
#             systemctl --user stop stretch_body_server.service
#
# Direct access is kept as an option because it is the shortest path to the
# hardware, which is useful when isolating a problem, and because the client
# adds a round trip per command that Dex Teleop's rate may not tolerate.

use_robot_client = True


# DROP GRIPPER ORIENTATION GOALS WITH LARGE JOINT ANGLE CHANGES
#
# Dropping goals that result in extreme changes in joint
# angles over a single time step avoids the nearly 360
# degree rotation in an opposite direction of motion that
# can occur when a goal jumps across a joint limit for a
# joint with a large range of motion like the roll joint.
#
# This also reduces the potential for unexpected wrist
# motions near gimbal lock when the yaw and roll axes are
# aligned (i.e., the gripper is pointed down to the
# ground). Goals representing slow motions that traverse
# near this gimbal lock region can still result in the
# gripper approximately going upside down in a manner
# similar to a pendulum, but this results in large yaw
# joint motions and is prevented at high speeds due to
# joint angles that differ significantly between time
# steps. Inverting this motion must also be performed at
# low speeds or the gripper will become stuck and need to
# traverse a trajectory around the gimbal lock region.
#
#max_allowed_wrist_yaw_change = np.pi/2.0
#max_allowed_wrist_roll_change = np.pi/2.0
max_allowed_wrist_yaw_change = 1.8 * (np.pi/2.0)
max_allowed_wrist_roll_change = 1.8 * (np.pi/2.0)


# How many consecutive frames the guard above may reject before it gives up and
# accepts the new orientation.
#
# The guard compares against the last *accepted* command, so while it rejects it
# never updates its reference. A genuine discontinuity - the Euler decomposition
# flipping branch near large pitch, say - therefore leaves it comparing against a
# stale value forever, and the wrist stays frozen until the operator happens to
# rotate back. Observed on stretch-se4-4032: ~50 frames (about 4 s) of
# 'extreme wrist_yaw change' while the hand was holding still.
#
# An orientation that persists for several frames is where the hand actually is,
# not a one-frame glitch, so resync to it.
max_consecutive_wrist_rejections = 4

        
# This is the weight between 0.0 and 1.0 multiplied by the current
# command when performing exponential smoothing. A higher weight leads
# to less lag, but higher noise. A lower weight leads to more lag, but
# smoother motions with less noise.

# current_filtered_value = ((1.0 - exponential_smoothing) * previous_filtered_value) + (exponential_smoothing * current_value)
exponential_smoothing_for_orientation = 0.5 #0.2 # 0.25 #0.1
exponential_smoothing_for_position = 0.5 #0.2 #0.5 #0.1


# Proportional gain applied to the mobile base rotation error before it is
# sent to base.rotate_by(). A gain of 1.0 asks the base to cover the whole
# remaining error every control cycle.
#
# rotate_by() issues an *incremental* trapezoidal trajectory per wheel
# (MODE_POS_TRAJ_INCR). At our ~13 Hz command rate no trajectory ever runs to
# completion, so each cycle re-plans a fresh acceleration ramp for the
# remaining error and the base never decelerates into the goal. Measured on
# stretch-se4-4032, a gain of 1.0 overshot a stationary goal by 9-12 degrees,
# then crawled back. That crawl is where most unwanted base translation
# happens, because an incremental position command lets unequal wheel motion
# accumulate as x/y drift instead of correcting it.
#
# A lower gain might be expected to feel laggy, but it does not: 0.5 and 1.0
# were indistinguishable to the operator, and 0.5 actually settled sooner,
# because the base no longer spends that time recovering from its own
# overshoot. Use the clamp below as well; they address different failures.
base_rotation_gain = 0.5


# Largest single rotation increment sent to base.rotate_by(), in radians.
#
# The gain above tames the steady case, but a large error - typically after
# the tongs are lost and reacquired somewhere else - still produces one big
# command. Measured on stretch-se4-4032, a 0.62 rad command issued while the
# base was reversing direction leaked 15 cm of translation in two seconds,
# 18x the normal rate. Capping how far ahead the planner aims stops it
# slewing that violently.
#
# This costs almost no speed. rotate_by() is reissued every control cycle
# (~75 ms), so a cap of 0.3 rad still allows about 4 rad/s of sustained
# rotation, well above what the base actually does.
base_rotation_max_command = 0.3


# When False, the robot should only move to its initial position
# and not move in response to ArUco markers. This is helpful when
# first trying new code and interface objects.
robot_allowed_to_move = True

# Camera stand at maximal height
# Minimum distance from the tongs to the camera in meters
#min_dist_from_camera_to_tongs = 0.3
# Maximum distance from the tongs to the camera in meters
#max_dist_from_camera_to_tongs = 0.8

# Camera stand at minimal height

# This range represents the manipulation range. Commands for sliding
# the lift range are outside of this range.

# Minimum distance from the tongs to the camera in meters
min_dist_from_camera_to_tongs = 0.5 #0.5
# Maximum distance from the tongs to the camera in meters
max_dist_from_camera_to_tongs = 1.0 #1.0

# Maximum height range of tongs
max_tongs_height_range = max_dist_from_camera_to_tongs - min_dist_from_camera_to_tongs


# The origin for teleoperation with respect to the camera's frame
# of reference. Holding the teleoperation interface at this
# position with respect to the camera results in the robot being
# commanded to achieve the center_wrist_position. The camera's
# frame of reference has its origin in the optical center of the
# camera and its z-axis points directly out of the camera on the
# camera's optical axis.
teleop_origin_x = 0.0
teleop_origin_z = min_dist_from_camera_to_tongs + max_tongs_height_range/2.0
teleop_origin_y = 0.24

teleop_origin = np.array([teleop_origin_x, teleop_origin_y, teleop_origin_z])


# Robot configuration used to define the center wrist position

def get_lift_maximum():
    # Read the lift's upper limit from the robot's own parameters rather than
    # hardcoding it. This was 1.09, the Stretch 3 lift maximum, which left the
    # working range sitting below what a Stretch 4 can reach.
    from stretch4_body.core.robot_params import RobotParams
    return RobotParams().get_params()[1]['lift']['range_m'][1]


def get_lift_middle(manipulate_on_ground): 
    # Additional joint limits for the lift beyond what the URDF
    # specifies. A None value is ignored in favor of the URDF joint
    # limits. These can only be more restrictive than the URDF joint
    # limits.
    if manipulate_on_ground:
        lift_minimum = None
        lift_maximum = max_tongs_height_range
        lift_middle = max_tongs_height_range / 2.0
    else:
        lift_minimum = get_lift_maximum() - max_tongs_height_range
        lift_maximum = None
        lift_middle = lift_minimum + max_tongs_height_range/2.0
    return lift_middle


def get_center_configuration(lift_middle): 
    # manipulate lower objects
    center_configuration = {
        'mobile_base_rotation_joint': 0.0,
        'lift_joint': lift_middle,
        'arm_l4_joint': 0.01,
        'wrist_yaw_joint': 0.0,
        'wrist_pitch_joint': 0.0,
        'wrist_roll_joint': 0.0
    }
    return center_configuration

# def get_starting_configuration(lift_middle): 

#     # The robot will attempt to achieve this configuration before
#     # teleoperation begins. Teleoperation commands for the robot's
#     # wrist position are made relative to the wrist position
#     # associated with this starting configuration.
#     starting_configuration = {
#         'joint_mobile_base_rotate_by': 0.0,
#         'lift_joint': lift_middle,
#         'arm_l4_joint': 0.01,
#         'wrist_yaw_joint': 0.9 * np.pi,
#         'wrist_pitch_joint': 0.0,
#         'wrist_roll_joint': 0.0
#     }
#     return starting_configuration

def get_starting_configuration(lift_middle): 

    # The robot will attempt to achieve this configuration before
    # teleoperation begins. Teleoperation commands for the robot's
    # wrist position are made relative to the wrist position
    # associated with this starting configuration.
    starting_configuration = {
        'joint_mobile_base_rotate_by': 0.0,
        'lift_joint': lift_middle,
        'arm_l4_joint': 0.01,
        'wrist_yaw_joint': 0.0,
        'wrist_pitch_joint': 0.0,
        'wrist_roll_joint': 0.1
    }
    return starting_configuration



# String used by processes to communicate via shared memory
shared_memory_name = 'gripper_goal_20231222'


def goal_dict_to_array(goal_dict):
    if goal_dict is None:
        return None

    goal_array = np.zeros((5, 3), dtype=np.float64)
    grip_width = goal_dict['grip_width']
    if grip_width is not None: 
        goal_array[0,0] = grip_width
    else:
        goal_array[0,0] = -10000.0
        
    goal_array[1,:] = goal_dict['wrist_position']
    goal_array[2,:] = goal_dict['gripper_x_axis']
    goal_array[3,:] = goal_dict['gripper_y_axis']
    goal_array[4,:] = goal_dict['gripper_z_axis']
    
    return goal_array


def goal_array_to_dict(goal_array):
    goal_dict = {}
    grip_width = goal_array[0,0]
    if grip_width < -1000.0:
        grip_width = None
    goal_dict['grip_width'] = grip_width
    goal_dict['wrist_position'] = goal_array[1,:]
    goal_dict['gripper_x_axis'] = goal_array[2,:]
    goal_dict['gripper_y_axis'] = goal_array[3,:]
    goal_dict['gripper_z_axis'] = goal_array[4,:]
    return goal_dict


def get_example_goal_array():
    
    goal_grip_width = 1.0
    goal_wrist_position = np.array([-0.03, -0.4, 0.9])
    goal_x_axis = np.array([ 1.0,  0.0, 0.0])
    goal_y_axis = np.array([ 0.0, -1.0, 0.0])
    goal_z_axis = np.array([ 0.0, 0.0, -1.0])

    example_goal = {'grip_width': goal_grip_width,
                    'wrist_position': goal_wrist_position,
                    'gripper_x_axis': goal_x_axis,
                    'gripper_y_axis': goal_y_axis,
                    'gripper_z_axis': goal_z_axis}

    goal_array = goal_dict_to_array(example_goal)
    return goal_array


def get_do_nothing_goal_array():
    
    goal_grip_width = -10000.0
    goal_wrist_position = np.array([-10000.0, -10000.0, -10000.0])
    goal_x_axis = np.array([ 1.0,  0.0, 0.0])
    goal_y_axis = np.array([ 0.0, -1.0, 0.0])
    goal_z_axis = np.array([ 0.0, 0.0, -1.0])

    example_goal = {'grip_width': goal_grip_width,
                    'wrist_position': goal_wrist_position,
                    'gripper_x_axis': goal_x_axis,
                    'gripper_y_axis': goal_y_axis,
                    'gripper_z_axis': goal_z_axis}

    goal_array = goal_dict_to_array(example_goal)
    return goal_array


def is_a_do_nothing_goal_array(goal_array):
    test = goal_array[1:,:] < -1000.0
    return np.any(test)
