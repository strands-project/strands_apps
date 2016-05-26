#!/usr/bin/env python

import rospy
from topological_roslaunch import TearDownAtNodes
import yaml
import sys

if __name__ == '__main__':

    rospy.init_node('tear_down_at_nodes')

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
        executor = TearDownAtNodes(tear_down_nodes = c['nodes'], launch_files = c['tear_down'])        

    
    rospy.spin()
