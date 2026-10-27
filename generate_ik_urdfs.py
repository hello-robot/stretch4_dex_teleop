#!/usr/bin/env python3
"""Generate the specialized IK URDFs that Dex Teleop needs on Stretch 4.

This replaces prepare_specialized_urdfs.py, the Stretch 3 script, which read
the calibrated URDF from <fleet dir>/exported_urdf/stretch.urdf. Stretch 4 has
no exported_urdf directory; the calibrated model is built from the robot's own
parameters by the stretch4_urdf package instead.

This is a thin wrapper around stretch4_urdf's urdf_utils_generate_ik_urdfs. It
calls that module's functions rather than its command line entry point, for the
reason given in generate() below.

Two files are written into the repository root, which is where simple_ik.py
and gripper_to_goal.py load them from:

    stretch4_dex_teleop_base_rotation_ik_with_fixed_wrist.urdf
    stretch4_dex_teleop_base_translation_ik_with_fixed_wrist.urdf

Note on why each URDF is generated from a freshly built robot model below.
"""

import argparse
import pathlib
import re

from stretch4_urdf.utils import urdf_utils_generate_ik_urdfs as gen

import dex_teleop_parameters as dt

PREFIX = 'stretch4_dex_teleop'

# The true arm range, used only to sanity check the generated files.
MAX_PLAUSIBLE_ARM_EXTENSION_M = 1.0


def generate(output_dir):
    # stretch4_urdf.generate_ik_urdfs() generates all six variants from a
    # single robot model, taking only a shallow copy.copy() of it between
    # variants. merge_arm() mutates the merged arm joint's limit in place, so
    # each successive variant re-merges an already-merged joint and the arm
    # limit accumulates: the rotation and translation variants Dex Teleop uses
    # come out with an arm limit of 1.69 m and 2.08 m rather than 0.52 m.
    # Building a fresh robot model per file avoids this. The joint origins are
    # unaffected, so only the limits were ever wrong.
    # stretch4_urdf clips the wrist pitch using its own module level
    # constant. Dex Teleop keeps that limit in dex_teleop_parameters.py, so
    # point the generator at ours and preserve it as the single place to tune.
    gen.wrist_pitch_lower_limit = dt.wrist_pitch_lower_limit

    paths = []
    for make in (gen.make_rotary_ik_urdf, gen.make_translation_ik_urdf):
        robot = gen.generate_robot_from_base_xacro()
        paths.append(make(robot, PREFIX, output_dir,
                          is_fixed_wrist=True, is_merge_arm=True))
    return paths


def check_arm_limit(path):
    """Guard against the accumulating-limit bug reappearing."""
    text = pathlib.Path(path).read_text()
    joint = re.search(r'<joint name="arm_l4_joint".*?</joint>', text, re.S)
    if joint is None:
        return 'no arm_l4_joint found'
    limit = re.search(r'<limit[^>]*upper="([0-9.]+)"', joint.group(0))
    if limit is None:
        return 'no arm_l4_joint limit found'
    upper = float(limit.group(1))
    if upper > MAX_PLAUSIBLE_ARM_EXTENSION_M:
        return f'arm_l4_joint upper limit {upper} m is implausible'
    return None


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', default='.',
                        help='where to write the URDFs (default: repository root)')
    args = parser.parse_args()

    written = generate(args.output_dir)

    problems = False
    for path in written:
        problem = check_arm_limit(path)
        if problem is None:
            print('Wrote', path)
        else:
            problems = True
            print('PROBLEM:', path, '-', problem)

    if problems:
        raise SystemExit('Generated URDFs failed their sanity check.')
    print('\nDone. Dex Teleop loads these two files from the repository root.')
