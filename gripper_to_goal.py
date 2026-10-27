import stretch4_body.robot.robot as rb
import numpy as np
import math
import time
from functools import partial
from scipy.spatial.transform import Rotation
from stretch4_body.core.robot_params import RobotParams
import urchin as urdf_loader
import os
from collections import deque
import simple_ik as si
import argparse
import loop_timer as lt
import dex_teleop_parameters as dt
from multiprocessing import shared_memory
import pprint as pp
import robot_move as rm
from copy import deepcopy


def angle_diff_rad(target_rad, current_rad):
    # Returns the smallest signed difference between two angles, in
    # [-pi, pi]. Previously imported from hello_helpers.hello_misc,
    # which on Stretch 4 is the ROS 2 node helper and no longer
    # provides this function.
    diff_rad = target_rad - current_rad
    diff_rad = ((diff_rad + math.pi) % (2.0 * math.pi)) - math.pi
    return diff_rad


def load_urdf(file_name):
    if not os.path.isfile(file_name):
        print()
        print('*****************************')
        print('ERROR: ' + file_name + ' was not found. Simple IK requires a specialized URDF saved with this file name. prepare_base_rotation_ik_urdf.py can be used to generate this specialized URDF.')
        print('*****************************')
        print()
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), file_name)
    urdf = urdf_loader.URDF.load(file_name, lazy_load_meshes=True)
    return urdf


def deg2rad(deg):
    return deg * (np.pi/180.0)

def nan_in_configuration(configuration):
    for k, v in configuration.items():
        if math.isnan(v) or np.isnan(v):
            return True
    return False

class BaseOdom:
    def __init__(self, robot):
        self.base = robot.base
        self.start_odom = self._read_odom()
        self.reset_odom = deepcopy(self.start_odom)

    def _read_odom(self):
        # Read the fields the direct omnibase's get_odom() returns, but from
        # status directly. RobotClient exposes status and not get_odom, and the
        # direct omnibase's get_odom() is itself only a wrapper over these same
        # fields, so this works for both.
        status = self.base.status
        return {'x': status['x'],
                'y': status['y'],
                'theta': status['theta'],
                'time': status['pose_time_s']}

    def reset(self):
        self.reset_odom = self._read_odom()

    def get(self):
        odom = self._read_odom()
        #print()
        #print(f'{self.reset_odom=}')
        #print(f'{odom=}')
        relative_odom = {
            'x': odom['x'] - self.reset_odom['x'],
            'y': odom['y'] - self.reset_odom['y'],
            'theta': odom['theta'] - self.reset_odom['theta'],
            'time': odom['time']
        }
        return relative_odom
        


    
def open_base_command_log():
    """Open a CSV of base commands and odometry, if DEX_TELEOP_BASE_LOG is set.

    Dex Teleop only ever calls base.rotate_by(), which commands
    move_by(0, 0, w_r) - translation is explicitly zero. This logs what was
    commanded alongside where the base actually went, so unwanted translation
    can be correlated with the commands that preceded it.

        DEX_TELEOP_BASE_LOG=1 python3 dex_teleop.py
    """
    if not os.environ.get('DEX_TELEOP_BASE_LOG'):
        return None
    name = time.strftime('base_commands_%Y%m%d_%H%M%S.csv')
    f = open(name, 'w')
    f.write('t,gain,clamp,rotate_by_cmd,goal_base_angle,current_base_angle,base_x,base_y,base_theta\n')
    print('logging base commands to', name)
    return f


class GripperToGoal:

    # Rolling window for the live base drift readout.
    base_stats_window_s = 10.0

    def track_base_motion(self, base_status):
        """Record recent base motion for the live drift readout."""
        now = time.time()
        self.base_motion_history.append(
            (now, base_status['x'], base_status['y'], base_status['theta']))
        while (len(self.base_motion_history) > 1 and
               now - self.base_motion_history[0][0] > self.base_stats_window_s):
            self.base_motion_history.popleft()

    def base_drift_rate(self):
        """Translation leaked per radian rotated, over the recent window.

        Dex Teleop never commands base translation, so any translation seen
        here is unwanted. Dividing by the rotation actually performed makes
        the number comparable between moments when the operator was turning
        a lot and moments when they were barely turning.

        Returns (mm_per_rad, rotated_rad). mm_per_rad is None when too
        little rotation has happened for the ratio to mean anything.
        """
        h = list(self.base_motion_history)
        if len(h) < 2:
            return None, 0.0
        translated = 0.0
        rotated = 0.0
        for a, b in zip(h, h[1:]):
            translated += math.hypot(b[1] - a[1], b[2] - a[2])
            rotated += abs(b[3] - a[3])
        if rotated < 0.2:
            return None, rotated
        return 1000.0 * translated / rotated, rotated

    def __init__(self, robot_speed, starting_configuration, robot_allowed_to_move):
        # The gripper joint is added once the fitted tool is known, below.
        self.joints_allowed_to_move = ['arm_l4_joint', 'lift_joint', 'wrist_yaw_joint', 'wrist_pitch_joint', 'wrist_roll_joint', 'joint_mobile_base_rotate_by']

        self.use_urdf = True
        if not self.use_urdf:
            
            ######################################
            # stretch_params.py | grep range
            # stretch_body.robot_params.nominal_params
            # param.lift.range_m [0.0, 1.1]
            # param.arm.range_m [0.0, 0.555]
            # param.wrist_pitch.range_deg [-150, 90]
            # param.wrist_roll.range_deg [-150, 150]
            # param.wrist_yaw.range_deg [-200, 100]
            # param.stretch_gripper.range_deg [-260, 400]
            ######################################

            wrist_joints = ['wrist_yaw_joint', 'wrist_pitch_joint', 'wrist_roll_joint']
            s = 10.0
            self.wrist_joint_limits = {
                'wrist_yaw_joint': (deg2rad(-200 + s), deg2rad(100 - s)),
                'wrist_pitch_joint': (deg2rad(-150 + s), deg2rad(90 - s)),
                'wrist_roll_joint': (deg2rad(-150 + s), deg2rad(150 - s))
                }

            # self.wrist_joint_limits = {
            #     'wrist_yaw_joint': (-np.pi/d, np.pi/d),
            #     'wrist_pitch_joint': (-np.pi/d, np.pi/d),
            #     'wrist_roll_joint': (-np.pi/d, np.pi/d)
            #     }

        else:
            # Get the wrist joint limits from the robot's own parameters.
            #
            # These used to be read from the IK URDF, but all three wrist
            # joints are type="fixed" in it - the arm is merged and the wrist
            # frozen - so their <limit> tags are vestigial and happen to carry
            # a different convention: the model defines wrist_roll_joint with
            # axis [1,0,0] and limits [-245, +65] deg, mirrored against the
            # hardware's [-65, +245]. Since these values are commanded to the
            # hardware, the hardware's parameters are the authority. Yaw and
            # pitch are [-65, +245] in both, so only roll changes.
            robot_params = RobotParams().get_params()[1]
            self.wrist_joint_limits = {}
            for joint_name in ('wrist_yaw', 'wrist_pitch', 'wrist_roll'):
                lower_deg, upper_deg = robot_params[joint_name]['range_deg']
                self.wrist_joint_limits[joint_name + '_joint'] = (deg2rad(lower_deg),
                                                                  deg2rad(upper_deg))

        self.robot_allowed_to_move = robot_allowed_to_move
        self.drop_extreme_wrist_orientation_change = True
            
        # Initialize the filtered wrist orientation that is used to
        # command the robot. Simple exponential smoothing is used to
        # filter wrist orientation values coming from the interface
        # objects.
        self.filtered_wrist_orientation = np.array([0.0, 0.0, 0.0])

        # Initialize the filtered wrist position that is used to command
        # the robot. Simple exponential smoothing is used to filter wrist
        # position values coming from the interface objects.
        self.filtered_wrist_position_configuration = np.array([
            starting_configuration['joint_mobile_base_rotate_by'],
            starting_configuration['lift_joint'],
            starting_configuration['arm_l4_joint']
        ])
    
        self.prev_commanded_wrist_orientation = {'wrist_yaw_joint':None,
                                                 'wrist_pitch_joint':None,
                                                 'wrist_roll_joint':None}

        
        # This is the weight multiplied by the current wrist angle command when performing exponential smoothing.
        # 0.5 with 'max' robot speed was too noisy on the wrist
        self.wrist_orientation_filter = dt.exponential_smoothing_for_orientation

        # This is the weight multiplied by the current wrist position command when performing exponential smoothing.
        # commands before sending them to the robot
        self.wrist_position_filter = dt.exponential_smoothing_for_position 

        # Initialize IK
        self.simple_ik = si.SimpleIK()
        
        ##########################################################
        # Prepare the robot last to avoid errors due to blocking calls
        # associated with other aspects of setting things up, such as
        # initializing SimpleIK.
        self.robot = rm.start_robot()

        # The grip width from the tongs is 0.0 (closed) to 1.0 (open). Map it
        # onto whatever the fitted gripper actually accepts, rather than a
        # fixed span.
        #
        # poses and move_to share units for every tool - percent for the
        # Stretch grippers, fingertip aperture in metres for the parallel jaw -
        # so this works without knowing which one is attached.
        #
        # It matters: the Stretch Gripper v2 has range_deg [-32, 338] against
        # the original's [-100, 300], which makes its open position 1056 pct
        # rather than 300. The previous fixed span of 400 opened a v2 to 19% of
        # its travel.
        self.gripper = rm.find_gripper(self.robot)
        self.gripper_name = self.gripper.name
        self.grip_closed = self.gripper.poses['close']
        self.grip_open = self.gripper.poses['open']
        self.joints_allowed_to_move.append(self.gripper_name)

        # Does this tool have a squeeze range below "fingertips touching"?
        # The compliant grippers do - closed is -100 pct, well past contact -
        # so the tongs' lower half drives grip force and their midpoint means
        # touching. The parallel jaw's closed position is its zero aperture,
        # so there is no squeeze to map and the whole travel is opening.
        # The tolerance matters: the jaw reports closed as -4.6e-06 m, which
        # is negative but not a squeeze range.
        self.gripper_squeezes = self.grip_closed < -0.01 * abs(self.grip_open)

        # Hold back from the hard stops. See dex_teleop_parameters for why the
        # closed end is only backed off when there is no squeeze range.
        span = self.grip_open - self.grip_closed
        self.grip_open -= dt.gripper_open_margin * span
        if not self.gripper_squeezes:
            self.grip_closed += dt.gripper_closed_margin * span
        # %g so metres of aperture and percent both read sensibly.
        print('gripper: %s, commands %g (closed) to %g (open)'
              % (self.gripper_name, self.grip_closed, self.grip_open))

        if dt.use_robot_client:
            print('connected to the robot through stretch_body_server (RobotClient)')
        else:
            print('stretch4_body file imported =', rb.__file__)
            # arm.motor is only reachable with direct hardware access
            transport_version = self.robot.arm.motor.transport.version
            print('stretch4_body using transport version =', transport_version)

        self.robot_move = rm.RobotMove(self.robot, speed=robot_speed)
        self.robot_move.print_settings()

        self.robot_move.to_configuration(starting_configuration, speed='default')
        self.robot.push_command()
        self.robot.wait_command()

        # Set the current mobile base angle to be 0.0 radians.
        #self.robot.base.reset_odometry()
        #
        # Refresh the status first. RobotClient populates status once at
        # startup and then only on pull_status(), so taking the odometry
        # baseline without this captures a stale pose and leaves every later
        # base angle offset by whatever moved in between.
        if dt.use_robot_client:
            self.robot.pull_status()
        self.base_odometry = BaseOdom(self.robot)
        self.base_odometry.reset()

        self.base_command_log = open_base_command_log()
        self.base_command_log_t0 = time.time()

        # Live-tunable so it can be compared against the original behaviour
        # without restarting. A gain of 1.0 is exactly what the code did
        # before this parameter existed.
        self.base_rotation_gain = dt.base_rotation_gain
        # 0.0 disables the clamp, which is how it behaved before the clamp
        # existed. Live-tunable so gain and clamp can be compared separately.
        self.base_rotation_max_command = dt.base_rotation_max_command
        self.base_motion_history = deque()
        
        ##########################################################

        self.print_robot_status_thread_timing = False
        # Print the commanded wrist angles every frame, and say when a frame is
        # rejected. Enable with DEX_TELEOP_DEBUG_WRIST=1 to watch yaw approach
        # its 245 deg limit.
        self.debug_wrist_orientation = bool(os.environ.get('DEX_TELEOP_DEBUG_WRIST'))

        self.max_allowed_wrist_yaw_change = dt.max_allowed_wrist_yaw_change
        self.max_allowed_wrist_roll_change = dt.max_allowed_wrist_roll_change
        self.max_consecutive_wrist_rejections = dt.max_consecutive_wrist_rejections
        self.consecutive_wrist_rejections = 0
    

    def __del__(self):
        # __del__ can run while the interpreter is shutting down, after Ctrl-C,
        # by which point builtins the stop path needs are already gone. There
        # is nothing useful to do then, and the traceback only obscures
        # whatever actually ended the run.
        try:
            print('GripperToGoal.__del__: stopping the robot')
            self.robot.stop()
        except Exception:
            pass
        
        
    def update_goal(self, grip_width, wrist_position, gripper_x_axis, gripper_y_axis, gripper_z_axis):

        # RobotClient caches the status dict and only refreshes it when
        # pull_status() is called, which stretch4_body documents as the user's
        # job at the start of each control cycle. Without this the base
        # odometry below never changes, so the mobile base rotation loop is
        # blind and winds itself up. The status socket is CONFLATE, so this
        # yields the latest status rather than a backlog.
        if dt.use_robot_client:
            self.robot.pull_status()

        ###############################
        #INPUT: wrist_position

        # Use Simple IK to find configurations for the
        # mobile base angle, lift distance, and arm
        # distance to achieve the goal wrist position in
        # the world frame.
        new_goal_configuration = self.simple_ik.ik_rotary_base(wrist_position)
        if new_goal_configuration is None:
            print(f'WARNING: SimpleIK failed to find a valid new_goal_configuration so skipping this iteration by continuing the loop. Input to IK: wrist_position = {wrist_position}, Output from IK: new_goal_configuration = {new_goal_configuration}')
        else: 

            new_wrist_position_configuration = np.array([new_goal_configuration['mobile_base_rotation_joint'],
                                                         new_goal_configuration['lift_joint'],
                                                         new_goal_configuration['arm_l4_joint']])

            # Use exponential smoothing to filter the wrist
            # position configuration used to command the
            # robot.
            self.filtered_wrist_position_configuration = (((1.0 - self.wrist_position_filter) * self.filtered_wrist_position_configuration) +
                                                          (self.wrist_position_filter * new_wrist_position_configuration))

            new_goal_configuration['lift_joint'] = self.filtered_wrist_position_configuration[1]
            new_goal_configuration['arm_l4_joint'] = self.filtered_wrist_position_configuration[2]


            self.simple_ik.clip_with_joint_limits(new_goal_configuration)

            #################################

            #################################
            # INPUT: grip_width between 0.0 and 1.0
        
            if (grip_width is not None) and (grip_width > -1000.0): 
                if self.gripper_squeezes:
                    # Half-closed tongs mean fingertips touching, as they did
                    # when this was a fixed span centred on zero. Below that
                    # the gripper squeezes, above it opens, each half scaled
                    # to the fitted tool's own limit.
                    if grip_width < 0.5:
                        command = -self.grip_closed * (grip_width - 0.5) / 0.5
                    else:
                        command = self.grip_open * (grip_width - 0.5) / 0.5
                else:
                    # No squeeze range, so spend the whole tongs travel on
                    # opening rather than wasting half of it on zero.
                    command = (self.grip_closed
                               + grip_width * (self.grip_open - self.grip_closed))
                new_goal_configuration[self.gripper_name] = command


            ##################################################
            # INPUT: x_axis, y_axis, z_axis

            # Use the gripper pose marker's orientation to directly control the robot's wrist yaw, pitch, and roll. 
            aruco_rotation = np.zeros((3, 3))

            aruco_rotation[:,0] = gripper_x_axis
            aruco_rotation[:,1] = gripper_y_axis
            aruco_rotation[:,2] = gripper_z_axis

            r = Rotation.from_matrix(aruco_rotation)
            # capital letters represent intrinsic
            # rotations, lowercase letters represent
            # extrinsic rotations
            ypr = r.as_euler('ZXY', degrees=False)
            
            wrist_yaw = angle_diff_rad(ypr[0] + np.pi, 0.0)
            lower_limit, upper_limit = self.wrist_joint_limits['wrist_yaw_joint']
            if (wrist_yaw < lower_limit):
                wrist_yaw = wrist_yaw + (2.0*np.pi)
                
            # The yaw and roll offsets above are a rigid reframing of the
            # marker pose: Rz(180) * R * Ry(180) reproduces (yaw+pi, pitch,
            # roll+pi) exactly, and leaves pitch untouched. So the negation
            # here is not part of that reframing - it reflects the Stretch 4
            # wrist pitch joint turning the opposite way to the Stretch 3 one.
            # Determined by observing the robot: without it the gripper
            # pitches away from the direction the tongs are tilted.
            wrist_pitch = -ypr[1]

            # angle_diff_rad wraps into [-pi, pi], but the roll joint travels
            # from -65 to +245 deg. Without this the wrapped values below the
            # lower limit can never reach the +180..+245 band, which cost most
            # of the joint's range. Same rescue wrist_yaw gets above.
            wrist_roll = angle_diff_rad(ypr[2] + np.pi, 0.0)
            lower_limit, upper_limit = self.wrist_joint_limits['wrist_roll_joint']
            if (wrist_roll < lower_limit):
                wrist_roll = wrist_roll + (2.0*np.pi)

            if self.debug_wrist_orientation:
                print('___________')
                print('wrist_yaw, wrist_pitch, wrist_roll = {:.2f}, {:.2f}, {:.2f} deg'.format((180.0 * (wrist_yaw/np.pi)),
                                                                                               (180.0 * (wrist_pitch/np.pi)),
                                                                                               (180.0 * (wrist_roll/np.pi))))

            # Clamp out-of-range angles rather than discarding the frame.
            #
            # Each wrist joint spans 310 deg of a 360 deg circle, so 50 deg of
            # orientations have no valid joint angle at all - neither the
            # wrapped value nor that value plus 360 lands inside the range.
            # Those poses are simply unreachable, which is a property of the
            # hardware rather than something to solve here.
            #
            # What did need solving: one unreachable joint used to drop the
            # whole orientation frame, so pitch and roll stopped responding too
            # for as long as the operator held the pose. Clamping lets the
            # reachable joints keep tracking while the unreachable one rests at
            # its limit, which is what hitting a joint limit should feel like.
            # Hold the joint's last good value rather than clamping to the
            # limit. The two differ by about a degree when the operator simply
            # rotates past the limit, but they differ a lot for a one-frame
            # marker glitch: clamping turns a wild reading into a legal command
            # well away from where the wrist is, and the extreme-change guard
            # below is far too loose (162 deg) to catch it. Seen on 4032 as a
            # single frame of yaw 271 between 212 and 165 - a 33 deg twitch
            # once clamped. Holding the previous value ignores it entirely.
            limits_violated = False
            limited = []
            for name, value in (('wrist_yaw_joint', wrist_yaw),
                                ('wrist_pitch_joint', wrist_pitch),
                                ('wrist_roll_joint', wrist_roll)):
                lower_limit, upper_limit = self.wrist_joint_limits[name]
                if lower_limit <= value <= upper_limit:
                    limited.append(value)
                    continue
                limits_violated = True
                previous = self.prev_commanded_wrist_orientation[name]
                # Nothing commanded yet, so there is no good value to hold.
                limited.append(previous if previous is not None else
                               min(upper_limit, max(lower_limit, value)))
            wrist_yaw, wrist_pitch, wrist_roll = limited


            ################################################################
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
            extreme_difference_violated = False
            if self.drop_extreme_wrist_orientation_change:
                prev_wrist_yaw = self.prev_commanded_wrist_orientation['wrist_yaw_joint']
                if prev_wrist_yaw is not None:
                    diff = abs(wrist_yaw - prev_wrist_yaw)
                    if diff > self.max_allowed_wrist_yaw_change:
                        print('extreme wrist_yaw change of {:.2f} deg'.format((180.0 * (diff/np.pi))))
                        extreme_difference_violated = True
                prev_wrist_roll = self.prev_commanded_wrist_orientation['wrist_roll_joint']
                if prev_wrist_roll is not None:
                    diff = abs(wrist_roll - prev_wrist_roll)
                    if diff > self.max_allowed_wrist_roll_change:
                        print('extreme wrist_roll change of {:.2f} deg'.format((180.0 * (diff/np.pi))))
                        extreme_difference_violated = True
            #
            # The guard compares against the last *accepted* command and does
            # not update that reference while it rejects, so a real
            # discontinuity would leave it comparing against a stale value
            # indefinitely. An orientation that survives several frames is
            # where the hand actually is, so give up and resync to it.
            if extreme_difference_violated:
                self.consecutive_wrist_rejections += 1
                if self.consecutive_wrist_rejections >= self.max_consecutive_wrist_rejections:
                    if self.debug_wrist_orientation:
                        print('wrist orientation held for {:d} frames, resyncing'.format(
                            self.consecutive_wrist_rejections))
                    extreme_difference_violated = False
            else:
                self.consecutive_wrist_rejections = 0
            #
            ################################################################

            if self.debug_wrist_orientation:
                if limits_violated:
                    print('A wrist angle was out of range; holding its last good value.')

            # limits_violated no longer blocks the command - the angles above
            # have already been clamped into range.
            if not extreme_difference_violated:
                new_wrist_orientation = np.array([wrist_yaw, wrist_pitch, wrist_roll])
                    
                # Use exponential smoothing to filter the wrist
                # orientation configuration used to command the
                # robot.
                self.filtered_wrist_orientation = (((1.0 - self.wrist_orientation_filter) * self.filtered_wrist_orientation) +
                                                   (self.wrist_orientation_filter * new_wrist_orientation))
                
                new_goal_configuration['wrist_yaw_joint'] = self.filtered_wrist_orientation[0]
                new_goal_configuration['wrist_pitch_joint'] = self.filtered_wrist_orientation[1]
                new_goal_configuration['wrist_roll_joint'] = self.filtered_wrist_orientation[2]

                self.prev_commanded_wrist_orientation = {'wrist_yaw_joint':self.filtered_wrist_orientation[0],
                                                         'wrist_pitch_joint':self.filtered_wrist_orientation[1],
                                                         'wrist_roll_joint':self.filtered_wrist_orientation[2]}

            # Convert from the absolute goal for the mobile
            # base to an incremental move to be performed
            # using rotate_by. This should be performed just
            # before sending the commands to make sure it's
            # using the most rececnt mobile base angle
            # estimate to reduce overshoot and other issues.

            # convert base odometry angle to be in the range -pi to pi
            #
            # On Stretch 4 the odometry angle and base.rotate_by() share a sign
            # convention: both follow the body twist, positive counterclockwise.
            # Measured on stretch-se4-4032: rotate_by(+0.10) moved theta by
            # +0.0992 rad. The Stretch 3 comment here described the opposite
            # convention, which is why the sign below has changed.
            odom_dict = self.base_odometry.get()
            #print(f'{odom_dict=}')

            #base_odom_theta = angle_diff_rad(self.robot.base.status['theta'], 0.0)
            base_odom_theta = angle_diff_rad(odom_dict['theta'], 0.0)
            current_mobile_base_angle = base_odom_theta
            #print(f'{current_mobile_base_angle=}')
            
            new_goal_configuration['mobile_base_rotation_joint'] = self.filtered_wrist_position_configuration[0]

            # Only command a fraction of the remaining error. rotate_by() plans
            # a fresh trapezoid every cycle and never finishes one at our
            # command rate, so a full-error command overshoots the goal and the
            # recovery from that overshoot is what drags the base sideways.
            # See dex_teleop_parameters.base_rotation_gain.
            base_angle_error = (new_goal_configuration['mobile_base_rotation_joint'] - current_mobile_base_angle)
            rotate_by = self.base_rotation_gain * base_angle_error
            if self.base_rotation_max_command > 0.0:
                rotate_by = max(-self.base_rotation_max_command,
                                min(self.base_rotation_max_command, rotate_by))
            new_goal_configuration['joint_mobile_base_rotate_by'] = rotate_by

            bs = self.robot.base.status
            if self.base_command_log is not None:
                self.track_base_motion(bs)

            if self.base_command_log is not None:
                self.base_command_log.write('%.4f,%.3f,%.3f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f\n' % (
                    time.time() - self.base_command_log_t0, self.base_rotation_gain,
                    self.base_rotation_max_command, rotate_by,
                    new_goal_configuration['mobile_base_rotation_joint'],
                    current_mobile_base_angle, bs['x'], bs['y'], bs['theta']))
                self.base_command_log.flush()
            #print(f'{rotate_by=}')
            #print()
                  
            #new_goal_configuration['joint_mobile_base_rotate_by'] = 0.0
            # remove virtual joint and approximate motion with rotate_by using joint_mobile_base_rotate_by
            del new_goal_configuration['mobile_base_rotation_joint']


            # If motion allowed, command the robot to move to the target configuration
            if self.robot_allowed_to_move:
                if nan_in_configuration(new_goal_configuration): 
                    print()
                    print('******************************************************************')
                    print('WARNING: dex_teleop: new_goal_configuration has a nan, so skipping execution on the robot')
                    print()
                    print('     new_goal_configuration =', new_goal_configuration)
                    print()
                    print('******************************************************************')
                    print()
                else: 
                    self.robot_move.to_configuration(new_goal_configuration, self.joints_allowed_to_move)
                    self.robot.push_command()

            # Print robot status timing stats, if desired.
            # These status threads only exist with direct hardware access.
            if self.print_robot_status_thread_timing and not dt.use_robot_client:
                self.robot.non_dxl_thread.stats.pretty_print()
                print()
                self.robot.dxl_end_of_arm_thread.stats.pretty_print()
                print()
                self.robot.dxl_head_thread.stats.pretty_print()
                print()
                self.robot.sys_thread.stats.pretty_print()

            #####################################################


        
if __name__ == '__main__':

    
    args = dt.get_arg_parser().parse_args()
    use_fastest_mode = args.fast
    manipulate_on_ground = args.ground
    left_handed = args.left
    use_multiprocessing = args.multiprocessing

    
    # When False, the robot should only move to its initial position
    # and not move in response to ArUco markers. This is helpful when
    # first trying new code and interface objects.
    robot_allowed_to_move = True


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

    lift_middle = dt.get_lift_middle(manipulate_on_ground)
    center_configuration = dt.get_center_configuration(lift_middle)
    starting_configuration = dt.get_starting_configuration(lift_middle)
    
    gripper_to_goal = GripperToGoal(robot_speed, starting_configuration, robot_allowed_to_move)
    
    if use_multiprocessing:
        shm = shared_memory.SharedMemory(name=dt.shared_memory_name, create=False)
        example_goal_array = dt.get_example_goal_array()
        received_goal_array = np.ndarray(example_goal_array.shape, dtype=example_goal_array.dtype, buffer=shm.buf)
    else:
        
        goal_grip_width = 1.0
        goal_wrist_position = np.array([-0.03, -0.4, 0.9])
        goal_x_axis = np.array([ 1.0,  0.0, 0.0])
        goal_y_axis = np.array([ 0.0, -1.0, 0.0])
        goal_z_axis = np.array([ 0.0, 0.0, -1.0])
        goal_dict = {'grip_width': goal_grip_width,
                     'wrist_position': goal_wrist_position,
                     'gripper_x_axis': goal_x_axis,
                     'gripper_y_axis': goal_y_axis,
                     'gripper_z_axis': goal_z_axis}

    loop_timer = lt.LoopTimer()
    # As in goal_from_teleop: printing every iteration costs more than the work
    # being measured.
    print_goal = False
    print_timing = True
    timing_report_period = 100
    first_goal_received = False
    
    try:
        while True:
            loop_timer.start_of_iteration()
            if use_multiprocessing: 
                if not dt.is_a_do_nothing_goal_array(received_goal_array):
                    goal_dict = dt.goal_array_to_dict(received_goal_array)
                    gripper_to_goal.update_goal(**goal_dict)
                    if print_goal:
                        print('goal_dict =')
                        pp.pprint(goal_dict)
                    if not first_goal_received:
                        loop_timer.reset()
                        first_goal_received = True
                elif print_goal:
                    print('received do nothing goal array')
            else: 
                gripper_to_goal.update_goal(**goal_dict)
                if not first_goal_received:
                    loop_timer.reset()
                    first_goal_received = True

            time.sleep(0.03)
            loop_timer.end_of_iteration()
            if print_timing and (loop_timer.loop_iterations % timing_report_period == 0):
                loop_timer.pretty_print()
    finally:
        print('cleaning up shared memory multiprocessing')
        shm.close()
        


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
