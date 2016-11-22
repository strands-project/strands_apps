#!/usr/bin/env python

import rospy
from topological_roslaunch import BringUpAtNodes
import yaml
import sys

if __name__ == '__main__':

    rospy.init_node('bring_up_at_nodes')

    try:
        config_file =  rospy.get_param("~config")
    except:
        rospy.logerr("Could not get config parameter. Make sure to supply a YAML "
                     "configuration file.")
        sys.exit(1)

    with open(config_file, "r") as f:
        config = yaml.load(f.read())

    for c in config:
        # rospy.loginfo()
        executor = BringUpAtNodes(bring_up_nodes = c['nodes'], launch_files = c['bring_up'])

    rospy.spin()
