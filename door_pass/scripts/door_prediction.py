#!/usr/bin/env python

import sys
import rospy

from strands_navigation_msgs.msg import TopologicalMap
from mongodb_store.message_store import MessageStoreProxy
from door_pass.msg import DoorWaitStat

def usage():
    print "\nFor using all the available stats use:"
#    print "\t rosrun topological_navigation topological_prediction.py"
#    print "For all the stats in a range use:"
#    print "\t rosrun topological_navigation topological_prediction.py -range from_epoch to_epoch"
#    print "For all the stats from a date until now use:"
#    print "\t rosrun topological_navigation topological_prediction.py -range from_epoch -1"
#    print "For all the stats until one date:"
#    print "\t rosrun topological_navigation topological_prediction.py -range 0 to_epoch"


class door_prediction(object):


    def __init__(self, epochs) :
        self.top_map = []
        self.map_received =False
        self.range = epochs

        rospy.Subscriber('/topological_map', TopologicalMap, self.MapCallback)
        rospy.loginfo("Waiting for Topological map ...")       
        while not self.map_received:
            rospy.sleep(rospy.Duration(0.1))
            rospy.loginfo("Waiting for Topological map ...")
        rospy.loginfo("... Got Topological map")

        stats=self.gather_stats()
        self.extract_doors(stats)
        
        self.find_doors()

        rospy.loginfo("All Done ...")
        rospy.spin()

      
    """
     MapCallback
     
     This function receives the Topological Map
    """
    def MapCallback(self, msg) :
        self.top_map = msg
        self.map_received = True

      
    """
     gather_stats
     
     This function retrieves all the door passing stats
    """
    def gather_stats(self):
        msg_store = MessageStoreProxy(collection='door_stats')

#        query = {}
        query_meta={}
#        available = msg_store.query(door_pass.msg.DoorWaitStat._type, query, query_meta)
        stats=[]
    
        available = len(msg_store.query(DoorWaitStat._type, {}, query_meta))
        if available <= 0 :
            rospy.logerr("not in datacentre")
        else:
            message_list = msg_store.query(DoorWaitStat._type, {}, query_meta)    
            for i in message_list:
                print i[0]
                print "----------------------------"
                stats.append(i[0])

        return stats            

    """
     extract_doors
     
     This function takes all the stats and sorts them by door in the environment
    """
    def extract_doors(self, stats):
        waypoints=[]
        for i in stats:
            if not i.waypoint in waypoints:
                waypoints.append(i.waypoint)
        
        waypoints.sort()
        print waypoints

    """
     find_doors
     
     This function finds all the doors in the environment using the topological map
    """
    def find_doors(self):
        doors=[]
        to_pop=[]
        for i in self.top_map.nodes:
            cwp = i.name
            for j in i.edges:
                if j.action == 'door_wait_and_pass':
                    d={}
                    d['nodes']=[]
                    d['nodes'].append(cwp)
                    d['nodes'].append(j.node)
                    doors.append(d)

        print len(doors)
        print doors        
        
        for k in range(len(doors)):
            print "Looking for", k, doors[k]['nodes'][0], doors[k]['nodes'][1]
            for l in range(k, len(doors)):
                if k!=l:
                    print "L", l, doors[l]['nodes']
                    if doors[k]['nodes'][0] in doors[l]['nodes'] and doors[k]['nodes'][1] in doors[l]['nodes'] :
                        print "POP"
                        to_pop.append(l)
        
        to_pop.sort()
        to_pop.reverse()
        print "TO POP:", to_pop
        
        for m in to_pop:
            doors.pop(m)
        print doors

if __name__ == '__main__':
    rospy.init_node('door_prediction')
    epochs=[]
    #if len(sys.argv) < 2:
    if '-h' in sys.argv or '--help' in sys.argv:
        usage()
        sys.exit(1)
    else:
        if '-range' in sys.argv:
            ind = sys.argv.index('-range')
            epochs.append(int(sys.argv[ind+1]))
            epochs.append(int(sys.argv[ind+2]))
            print epochs
        else:
            print "gathering all the stats"        
            epochs=[0, rospy.get_rostime().to_sec()]

    server = door_prediction(epochs)