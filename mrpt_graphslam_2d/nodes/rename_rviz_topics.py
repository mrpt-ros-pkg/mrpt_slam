#!/usr/bin/env python
# Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""
Generate rviz files from templates using the current hostname.

Current script generates rviz files based on the templates files of
rviz/templates and using the current computer's hostname for determining the
namespaces of the topics. This is a required step for visualizing multi-robot
graphSLAM when running based on rosbags or based on measurements from Gazebo
since each multi-robot agent runs on a separate ROS core to simulate as much
as possible a real-time setup.
"""

import logging
import os
import socket
import sys

from colorlog import ColoredFormatter

LOG_LEVEL = logging.DEBUG
LOGFORMAT = ('%(log_color)s%(levelname)-5s%(reset)s '
             '| %(log_color)s%(message)s%(reset)s')

logging.root.setLevel(LOG_LEVEL)
formatter = ColoredFormatter(LOGFORMAT)
stream = logging.StreamHandler()
stream.setLevel(LOG_LEVEL)
stream.setFormatter(formatter)

logger = logging.getLogger('RvizRenamer')
logger.setLevel(LOG_LEVEL)
logger.addHandler(stream)


def rename_topics_in_rviz_file(templ_file, replace_dict):
    """Read from a template file and write modified contents to rviz file."""
    with open(templ_file, 'r') as templ:
        templ_lines = templ.readlines()
        head, tail = os.path.split(templ_file)

        actual_file = os.path.join(os.path.dirname(head), tail)
        logger.info('Writing file: %s' % os.path.abspath(actual_file))
        with open(actual_file, 'w') as f:
            f.writelines([
                line.format(**replace_dict) for line in templ_lines
            ])


def validate_args():
    """Validate that the script is invoked without extra arguments."""
    if len(sys.argv) > 1:
        logger.warning('Current script modifies the template rviz files '
                       'found in rviz/templates so that their topics match the '
                       "running computer's hostname.\n"
                       'Run this without any additional arguments.')
        logger.warning('Exiting...')
        sys.exit(-1)


def main():
    """Run the rviz topic renaming script."""
    logger.info('Initializing...')
    validate_args()

    script_dir = os.path.dirname(os.path.realpath(__file__))
    rviz_dir_path = os.path.join(script_dir, '..', 'rviz', 'templates')

    rviz_templ_files = [os.path.join(rviz_dir_path, i)
                        for i in os.listdir(rviz_dir_path)]

    rviz_templ_files = filter(lambda name: 'bag' in name or 'gazebo' in name,
                              rviz_templ_files)

    curr_hostname = socket.gethostname()
    replace_dict = {'HOSTNAME_PLACEHOLDER': curr_hostname}

    logger.info('Rviz files to operate on:\n%s\n\n' %
                os.linesep.join([os.path.abspath(f)
                                 for f in rviz_templ_files]))

    logger.info('Replacing: %s ==> %s' % replace_dict.items()[0])

    for templ_file in rviz_templ_files:
        rename_topics_in_rviz_file(templ_file, replace_dict)
    logger.info('All done.')


if __name__ == '__main__':
    main()
