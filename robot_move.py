import stretch4_body.robot.robot as rb
import dex_teleop_parameters as dt
from stretch4_body.robot.robot_client import RobotClient
from stretch4_body.core.robot_params import RobotParams

from functools import partial
import sys


def red(text):
    """Wrap text in ANSI red, unless stdout is redirected to a file."""
    if not sys.stdout.isatty():
        return text
    return '\033[1;31m' + text + '\033[0m'


def find_gripper(robot):
    """Return the end-of-arm gripper, whichever tool is fitted.

    RobotClient aliases the configured tool joint as end_of_arm.gripper; the
    direct Robot path does not, so fall back to whichever joint declares itself
    a tool. Both expose the same .name, .poses and .move_to, so Dex Teleop does
    not need to know which gripper is attached.
    """
    gripper = getattr(robot.end_of_arm, 'gripper', None)
    if gripper is not None:
        return gripper

    from stretch4_body.utils.tool_metadata import is_tool_joint
    for joint_name in robot.end_of_arm.joints:
        if is_tool_joint(joint_name):
            return getattr(robot.end_of_arm, joint_name)
    raise Exception('No gripper found on the end of arm. Is a tool configured? '
                    'Check with: stretch_configure_tool')


def start_robot():
    """Return a started robot handle, direct or via the body server.

    See dex_teleop_parameters.use_robot_client. RobotClient talks to the
    running stretch_body_server; Robot() takes the hardware directly and needs
    the server stopped first.
    """
    robot = RobotClient() if dt.use_robot_client else rb.Robot()
    if not robot.startup():
        if dt.use_robot_client:
            raise Exception('Could not start RobotClient. Is stretch_body_server running? '
                            'Check with: stretch_body_server --status')
        raise Exception('Could not start Robot. Is stretch_body_server holding the hardware? '
                        'Stop it with: systemctl --user stop stretch_body_server.service')
    return robot


class RobotMove:
    def __init__(self, robot, speed='default'):
        
        self.params = RobotParams().get_params()[1]
        self.available_speeds = ['default', 'slow', 'fast', 'max']
        # Every tool names its gripper joint differently - 'stretch_gripper'
        # for the compliant grippers, 'parallel_gripper' for the parallel jaw -
        # and robot params carries whichever one is fitted. Key off the name
        # rather than assuming.
        self.gripper_name = find_gripper(robot).name
        self.create_all_commands(robot)

        if speed not in self.all_speeds.keys():
            print(f"WARNING: RobotMove create_commands called with unrecognized speed = '{speed}'")
            print(f"         Using 'default' speed instead.")
            speed = 'default'

        # Check robot is homed
        is_homed = robot.is_homed()
        if not is_homed:
            print(red("ERROR: Robot must be homed before using Dex Teleop."))
            print(red("To home the robot, use the cli command `stretch_robot_home`.\n"))
            raise Exception("Robot must be homed before using Dex Teleop")

        self.speed = speed
        self.move_to_functions = self.all_speeds[self.speed]['move_to_functions']
        self.move_to_settings = self.all_speeds[self.speed]['move_to_settings']

        
    def print_settings(self):
        print(self.move_to_settings)

        
    def create_all_commands(self, robot):
        self.all_speeds = {}
        for speed in self.available_speeds:
            
            if speed == 'custom':
                custom_parameters = {
                    'joint_mobile_base_rotate_by': (None, {'v_r': 0.3, 'a_r': 0.5}),
                    'lift_joint': (None, {'v_m': 0.2, 'a_m': 1.0}),
                    'arm_l4_joint': (None, {'v_m': 0.18, 'a_m': 1.0}),
                    'wrist_yaw_joint': ('wrist_yaw', {'v_r': 3.0, 'a_r': 10}),
                    'wrist_pitch_joint': ('wrist_pitch', {'v_r': 3.0, 'a_r': 10.0}),
                    'wrist_roll_joint': ('wrist_roll', {'v_r': 4.5, 'a_r': 12}),
                    'stretch_gripper':  ('stretch_gripper', {'v_r': 12, 'a_r': 18})
                }

                joint_functions = {
                    'joint_mobile_base_rotate_by': robot.base.rotate_by,
                    'lift_joint': robot.lift.move_to, 
                    'arm_l4_joint': robot.arm.move_to,
                    'wrist_yaw_joint': robot.end_of_arm.move_to,
                    'wrist_pitch_joint': robot.end_of_arm.move_to,
                    'wrist_roll_joint': robot.end_of_arm.move_to,
                    'stretch_gripper' : robot.end_of_arm.move_to
                    }

                f = joint_functions
                p = custom_parameters

                move_to_functions = { j: ((partial(f[j], **p[j][1]))
                                               if (p[j][0] is None) else
                                                   (partial(f[j], p[j][0], **p[j][1])))
                                           for j in joint_functions.keys()}

                move_to_settings = custom_parameters

            else: 
                base_speed = self.params['omnibase']['motion'][speed]
                base_rot_v = base_speed['vel_w_r']
                base_rot_a = base_speed['accel_w_r']

                lift_speed = self.params['lift']['motion'][speed]
                lift_v = lift_speed['vel_m']
                lift_a = lift_speed['accel_m']

                arm_speed = self.params['arm']['motion'][speed]
                arm_v = arm_speed['vel_m']
                arm_a = arm_speed['accel_m']

                yaw_speed = self.params['wrist_yaw']['motion'][speed]
                yaw_v = yaw_speed['vel']
                yaw_a = yaw_speed['accel']

                pitch_speed = self.params['wrist_pitch']['motion'][speed]
                pitch_v = pitch_speed['vel']
                pitch_a = pitch_speed['accel']

                roll_speed = self.params['wrist_roll']['motion'][speed]
                roll_v = roll_speed['vel']
                roll_a = roll_speed['accel']

                gripper_speed = self.params[self.gripper_name]['motion'][speed]
                gripper_v = gripper_speed['vel']
                gripper_a = gripper_speed['accel']

                # Make sure the gripper moves at a reasonable
                # speed. For example, the slow gripper values for
                # Atlanta Basquiat+on July 14, 2025 (accel 4.0 and vel
                # 1.0) resulted is very slow motion that was
                # mismatched to the motions of the other joints.
                if gripper_v < 6.0:
                    gripper_v = 6.0
                if gripper_a < 6.0:
                    gripper_a = 6.0

                move_to_functions = {
                    'joint_mobile_base_rotate_by': partial(robot.base.rotate_by, v_r=base_rot_v, a_r=base_rot_a),
                    'lift_joint': partial(robot.lift.move_to, v_m=lift_v, a_m=lift_a), 
                    'arm_l4_joint': partial(robot.arm.move_to, v_m=arm_v, a_m=arm_a), 
                    'wrist_yaw_joint': partial(robot.end_of_arm.move_to, 'wrist_yaw', v_r=yaw_v, a_r=yaw_a),
                    'wrist_pitch_joint': partial(robot.end_of_arm.move_to, 'wrist_pitch', v_r=pitch_v, a_r=pitch_a),
                    'wrist_roll_joint': partial(robot.end_of_arm.move_to, 'wrist_roll', v_r=roll_v, a_r=roll_a),
                    self.gripper_name: partial(robot.end_of_arm.move_to, self.gripper_name, v_r=gripper_v, a_r=gripper_a)
                }

                move_to_settings = {
                    'joint_mobile_base_rotate_by': (None, {'v_r':base_rot_v, 'a_r':base_rot_a}),
                    'lift_joint': (None, {'v_m':lift_v, 'a_m':lift_a}), 
                    'arm_l4_joint': (None, {'v_m':arm_v, 'a_m':arm_a}), 
                    'wrist_yaw_joint': ('wrist_yaw', {'joint': 'wrist_yaw','v_r':yaw_v, 'a_r':yaw_a}),
                    'wrist_pitch_joint': ('wrist_pitch', {'joint': 'wrist_pitch','v_r':pitch_v, 'a_r':pitch_a}),
                    'wrist_roll_joint': ('wrist_roll', {'joint': 'wrist_roll', 'v_r':roll_v, 'a_r':roll_a}),
                    self.gripper_name: (self.gripper_name, {'joint': self.gripper_name, 'v_r':gripper_v, 'a_r':gripper_a})
                }

                
            self.all_speeds[speed] = {'move_to_functions': move_to_functions, 'move_to_settings': move_to_settings}
                

    def clip_configuration(self, config, min_config, max_config):
        pass

    
    def to_configuration(self, config, valid_joints=None, speed=None):
        if speed is None:
            move_to_functions = self.move_to_functions
        else:
            if speed not in self.all_speeds.keys():
                print('WARNING: RobotMove create_commands called with unrecognized speed = \'' + speed + '\'')
                print('         Using \'default\' speed instead.')
                speed = 'default'
            move_to_functions = self.all_speeds[speed]['move_to_functions']
            
        for j in config.keys():
            if valid_joints is None:
                move_to_functions[j](config[j])
            elif j in valid_joints: 
                move_to_functions[j](config[j])
            

                
if __name__ == '__main__':
    
    robot = start_robot()
    
    robot_move = RobotMove(robot, speed='slow')
    robot_move.print_settings()

    starting_configuration = {
        'joint_mobile_base_rotate_by': 0.1,
        'lift_joint': 0.7,
        'arm_l4_joint': 0.01,
        'wrist_yaw_joint': 0.1 * 3.14,
        'wrist_pitch_joint': 0.1,
        'wrist_roll_joint': 0.1
    }
    
    robot_move.to_configuration(starting_configuration, speed='slow')
    robot.push_command()
    robot.wait_command()
    robot.stop()
    
