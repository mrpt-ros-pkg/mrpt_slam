#!/usr/bin/env python
# Tue Jun 13 17:21:40 EEST 2017, Nikos Koukis

"""
Current script generates rviz files based on the templates files of
rviz/templates and usign the current computer's hostname for determining the
namespaces of the topics. This is a required step for visualizing multi-robot
graphSLAM when running based on rosbags or based on measurements from Gazebo
since each multi-robot agent runs on a separate ROS core to simulate as much as
possible a real-time setup.

"""

import os
import logging
import socket

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("RvizRenamer")


def rename_topics_in_rviz_file(templ_file, replace_dict):
    """Read from a single template file and write the modified contents to an
    actual rviz file one directory prior to the templates directory.
    """

    with open(templ_file, 'r') as templ:
        templ_lines = templ.readlines()
        head, tail = os.path.split(templ_file)

        # create the actual rviz files in the parent dir of templates
        actual_file = os.path.join(os.path.dirname(head), tail)
        logger.info("Writing file: %s" % os.path.abspath(actual_file))
        with open(actual_file, 'w') as f:
            # rewrite its line of the templ file by making the appropriate
            # substitution
            f.writelines([l.format(**replace_dict) for l in templ_lines])



def main():
    """Main."""

    logger.info("Initializing...")

    # fetch the rviz files that topics renaming is to happen
    script_dir = os.path.dirname(os.path.realpath(__file__))
    rviz_dir_path = os.path.join(script_dir, "..", "rviz", "templates")

    rviz_templ_files = [os.path.join(rviz_dir_path, i)
                        for i in os.listdir(rviz_dir_path)]

    # do the renaming only for files used for "gazebo" (suffix "gazebo") or
    # recorded with rosbag (suffix "gazebo")

    rviz_templ_files = filter(lambda name: "bag" in name or "gazebo" in name,
                              rviz_templ_files)

    curr_hostname = socket.gethostname()
    replace_dict = {"HOSTNAME_PLACEHOLDER": curr_hostname}

    logger.info("Rviz files to operate on:\n%s\n\n" %
                os.linesep.join([os.path.abspath(f)
                                 for f in rviz_templ_files]))

    # print the strings to be replaced
    logger.info("Replacing: %s ==> %s" % replace_dict.items()[0])

    for templ_file in rviz_templ_files:
        rename_topics_in_rviz_file(templ_file, replace_dict)
    logger.info("All done.")


if __name__ == "__main__":
    main()



