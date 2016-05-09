#!/usr/bin/env python

import sys
import numpy
import random

import rospy
import actionlib

from actionlib_msgs.msg import *

from threading import Lock
from door_pass.srv import PredictDoorState
from mongodb_store.message_store import MessageStoreProxy
import fremenserver.msg
from strands_navigation_msgs.msg import TopologicalMap
from door_pass.msg import DoorWaitStat


def usage():
    print "\Using all the available stats"


class door_prediction(object):


    def __init__(self, epochs) :
        self.top_map = []
        self.map_received =False
        self.range = epochs
        self.lock = Lock()
        self.wait_timeout = rospy.get_param('/door_pass_timeout', 240)
        
        # Get Topological Map
        rospy.Subscriber('/topological_map', TopologicalMap, self.MapCallback)
        rospy.loginfo("Waiting for Topological map ...")       
        while not self.map_received:
            rospy.sleep(rospy.Duration(0.1))
        rospy.loginfo("... Got Topological map")


        # Creating fremen server client
        rospy.loginfo("Creating fremen server client")
        self.FremenClient= actionlib.SimpleActionClient('fremenserver', fremenserver.msg.FremenAction)
        self.FremenClient.wait_for_server()
        rospy.loginfo(" ...done")

        # Create Door models        
        self.create_models()

        #Advertise Service
        self.predict_srv=rospy.Service('/door_prediction/predict_doors', PredictDoorState, self.predict_door_cb)

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
     create_models
     
     This function finds all the doors in the environment 
     retrieves all the door passing stats,
     sorts them by door
    """
    def create_models(self):
        self.doors =[]                  #Contains statistics per door used to create fremen models
              
        stats=self.gather_stats()       # retrieves all the door passing stats
        print self.doors
        self.extract_doors(stats)       # sorts stats by door
        
        for i in self.doors:
            i['order']=self.create_fremen_model(i)
            print "-------------"


    """
     create_fremen_model
     
     This function creates the fremen model for each door
    """
    def create_fremen_model(self, door):
        to_ret={}
        epochs = [x['epoch'] for x in door['stats']]
        res = [x['result'] for x in door['stats']]
        tepochs = [x['epoch'] for x in door['times']]
        times = [x['time'] for x in door['times']]
        
        print str(len(epochs))+' samples using '+str(int(numpy.ceil(len(epochs)*0.75)))+' for model building'# and '+str(int(numpy.ceil(len(epochs)*0.2)))+' for evaluation'


        # Choosing the samples used for model building and evaluation
        sampling_type = rospy.get_param('/door_prediction/model_building/sampling_type', 0) #0 for ordered (extrapolation), 1 for random (intrapolation)       
        if sampling_type == 0:
            #ordered sampling

            #samples for result sampling
            index_b = range(int(numpy.ceil(len(epochs)*0.75)))
            index_e = range(int(numpy.ceil(len(epochs)*0.75)),len(epochs))
            
            #samples for time sampling
            tindex_b = range(int(numpy.ceil(len(tepochs)*0.75)))
            tindex_e = range(int(numpy.ceil(len(tepochs)*0.75)),len(tepochs))
        else:
            #random sampling
        
           #samples for result sampling        
            index_b = sorted(random.sample(xrange(len(epochs)), int(numpy.ceil(len(epochs)*0.8)))) 
            index_e = []
            for i in range(len(epochs)):
                if i not in index_b:
                    index_e.append(i)

            #samples for time sampling
            tindex_b = sorted(random.sample(xrange(len(tepochs)), int(numpy.ceil(len(tepochs)*0.8)))) 
            tindex_e = []
            for i in range(len(tepochs)):
                if i not in tindex_b:
                    tindex_e.append(i)

   
        if not index_e:
            index_e = random.sample(xrange(len(epochs)), 1)

        if not tindex_e:
            tindex_e = random.sample(xrange(len(tepochs)), 1)
            
        epochs_build = [ epochs[i] for i in index_b]
        epochs_eval = [ epochs[i] for i in index_e]
        res_build = [ res[i] for i in index_b]
        res_eval = [ res[i] for i in index_e]

        time_epochs_build = [ tepochs[i] for i in tindex_b]
        time_epochs_eval = [ tepochs[i] for i in tindex_e]
        times_build = [ times[i] for i in tindex_b]
        times_eval = [ times[i] for i in tindex_e]
        
        print index_b, epochs_build, res_build, times_build
        print '---'
        print index_e, epochs_eval, res_eval, times_eval
        print '---'
        print door['model_id']['res']
        to_ret['res']=self.add_and_eval_models(door['model_id']['res'], epochs_build, res_build, epochs_eval, res_eval)
        to_ret['time']=self.add_and_eval_value_models(door['model_id']['time'], time_epochs_build, times_build, time_epochs_eval, times_eval)
        print to_ret
        return to_ret


    """
     add_and_eval_models
     
     This function creates and evaluates fremen models for binary states
     it returns the recommended order for the predictions
    """
    def add_and_eval_models(self, model_id, a_epochs, a_states, e_epochs, e_states):
        fremgoal = fremenserver.msg.FremenGoal()
        fremgoal.operation = 'add'
        fremgoal.id = model_id
        fremgoal.times = a_epochs
        fremgoal.states = a_states
        
        # print "--- BUILD ---"
        self.FremenClient.send_goal(fremgoal)
        self.FremenClient.wait_for_result()
        ps = self.FremenClient.get_result()
        print ps
        
        # print "--- EVALUATE ---"
        frevgoal = fremenserver.msg.FremenGoal()
        frevgoal.operation = 'evaluate'
        frevgoal.id = model_id
        frevgoal.times = e_epochs
        frevgoal.states = e_states
        frevgoal.order = 5
        
        self.FremenClient.send_goal(frevgoal)
        self.FremenClient.wait_for_result()
        pse = self.FremenClient.get_result()  
        print pse.errors
        print "chosen order %d" %pse.errors.index(min(pse.errors))
        return pse.errors.index(min(pse.errors))


    """
     add_and_eval_value_models
     
     This function creates and evaluates fremen models for float values
     it returns the recommended order for the predictions
    """
    def add_and_eval_value_models(self, model_id, a_epochs, a_states, e_epochs, e_states):
        print a_states
        fremgoal = fremenserver.msg.FremenGoal()
        fremgoal.operation = 'addvalues'
        fremgoal.id = model_id
        fremgoal.times = a_epochs
        fremgoal.values = a_states
        
        # Sends the goal to the action server.
        self.FremenClient.send_goal(fremgoal)
        print "Sending data to fremenserver"
        
        
        # Waits for the server to finish performing the action.
        self.FremenClient.wait_for_result()
        
        print "fremenserver done"
        
        # Prints out the result of executing the action
        ps = self.FremenClient.get_result()
        print "fremenserver result:"
        print ps
        
        # print "--- EVALUATE ---"
        frevgoal = fremenserver.msg.FremenGoal()
        frevgoal.operation = 'evaluate'
        frevgoal.id = model_id
        frevgoal.times = e_epochs
        frevgoal.states = e_states
        frevgoal.order = 5
        
        # Sends the goal to the action server.
        self.FremenClient.send_goal(frevgoal)
        
        # Waits for the server to finish performing the action.
        self.FremenClient.wait_for_result()
        
        # Prints out the result of executing the action
        pse = self.FremenClient.get_result()  # A FibonacciResult
        print pse.errors
        print "chosen order %d" %pse.errors.index(min(pse.errors))
        return pse.errors.index(min(pse.errors))


    """
     create_new_door
     
     Creates a new door to be appended in the doors list
    """
    def create_new_door(self, source, target):
        d={}
        a = 'door'+'_'+source+'_'+target+'_res'
        b = 'door'+'_'+source+'_'+target+'_time'
        c = source+'_'+target
        d['model_id']={'name':c,'res':a, 'time':b}
        d['order']={'state':0, 'time':0}
        d['nodes']=[]
        d['nodes'].append(source)
        d['nodes'].append(target)
        d['stats']=[]
        d['times']=[]        
        return d
       
    
    """
     gather_stats
     
     This function retrieves all the door passing stats
     and finds all the doors in the environment from the stats
    """
    def gather_stats(self):
        stats=[]
        doors=[]
        #to_pop=[]
        
        msg_store = MessageStoreProxy(collection='door_stats')

        query = {}
        query['topological_map_name']= self.top_map.pointset
        query_meta={}
       
        #find all the stats for the current topological map
        message_list = msg_store.query(DoorWaitStat._type, query, query_meta)
        if len(message_list) <= 0 :
            rospy.logerr("not in datacentre")
        else:
            for i in message_list:
                stats.append(i)
                #create temporal door models
                if doors:
                    found=False
                    for j in doors:                    
                        if i[0].source_waypoint in j['nodes'] and i[0].target_waypoint in j['nodes']:
                            found=True
                            break
                    if not found:
                        d = self.create_new_door(i[0].source_waypoint, i[0].target_waypoint)
                        doors.append(d)
                else:
                    d = self.create_new_door(i[0].source_waypoint, i[0].target_waypoint)
                    doors.append(d)

        self.doors=doors
        for l in self.doors:
            print l['nodes']
        return stats


    """
     extract_doors
     
     This function takes all the stats and sorts them by door in the environment
    """
    def extract_doors(self, stats):
        #waypoints=[]
        for i in stats:
            for j in self.doors:
                if i[0].source_waypoint in j['nodes'] and i[0].target_waypoint in j['nodes']:
                    tstat ={}
                    if i[0].opened:
                        tstat['result']=1.0
                        tistat={}
                        tistat['epoch']=int(i[1]['inserted_at'].strftime('%s'))
                        tistat['time']=(i[0].wait_time/(self.wait_timeout+1))
                        j['times'].append(tistat)
                    else:
                        tstat['result']=0.0
                    tstat['epoch']=int(i[1]['inserted_at'].strftime('%s'))
                    j['stats'].append(tstat)
        
        total_stats=0
        for h in self.doors:
            print "----------"
            print h['model_id']
            print 'number of stats:' + str(len(h['stats']))
            total_stats=total_stats+len(h['stats'])
        print "----------"
        print total_stats  


    def predict_door_cb(self, req):
        with self.lock:
            return self.get_predict(req.epoch.secs)

    def get_predict(self, epoch):
        print "requesting prediction for time %d" %epoch

        dur=[]
        prob=[]
        
        dids = [x['model_id']['name'] for x in self.doors] #doors ids

        #door_pass outcome forecast
        resids = [x['model_id']['res'] for x in self.doors]     #door_pass outcome model_id
        resords = [x['order']['res'] for x in self.doors]       #door_pass outcome model order
        prob = self.forecast_outcome(epoch, resids, resords)    #door_pass outcome prediction
        
        #door_pass time forecast
        timids = [x['model_id']['time'] for x in self.doors]    #door_pass time model_id
        timords = [x['order']['time'] for x in self.doors]      #door_pass time model order
        dur2 = self.forecast_outcome(epoch, timids, timords)    #door_pass time prediction
        dur = [rospy.Duration.from_sec(x* self.wait_timeout) for x in dur2]
        
        #print resords
        for i in range(len(prob)):
            if prob[i] <= 0.0:
                prob[i] = 0.000001
            elif prob[i] >= 1.0:
                prob[i] = 0.999999

        return dids, prob, dur


    def forecast_outcome(self, epoch, mods, ords):
        print epoch, mods, ords
        fremgoal = fremenserver.msg.FremenGoal()
        fremgoal.operation = 'forecast'
        fremgoal.ids = mods
        fremgoal.times.append(epoch)
        
        fremgoal.order = -1
        fremgoal.orders = ords
        
        self.FremenClient.send_goal(fremgoal)
        self.FremenClient.wait_for_result(timeout=rospy.Duration(10.0))

#        if self.FremenClient.get_state() == actionlib.GoalStatus.SUCCEEDED:
        ps = self.FremenClient.get_result()
        print ps
        prob = list(ps.probabilities)
        return prob
        




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