#!/usr/bin/env python

import sys
import numpy
import random

import rospy
import actionlib
import time


import std_msgs.msg
import door_pass.msg

from threading import Lock
from door_pass.srv import PredictDoorState
from mongodb_store.message_store import MessageStoreProxy

import fremenserver.msg
from strands_navigation_msgs.msg import TopologicalMap
from door_pass.msg import DoorWaitStat


def usage():
    print "\Using all the available stats"


class door_prediction(object):

    _feedback = door_pass.msg.BuildPredictionFeedback()
    _result   = door_pass.msg.BuildPredictionResult()

    def __init__(self, epochs) :
        rospy.on_shutdown(self._on_node_shutdown)
        self.top_map = []
        self.map_received =False
        self.range = epochs
        self.lock = Lock()
        action_name = '/door_prediction/build_temporal_model'
        self.wait_timeout = rospy.get_param('/door_pass_timeout', 240)
        
        # Get Topological Map
        rospy.Subscriber('/topological_map', TopologicalMap, self.MapCallback)
        rospy.loginfo("Waiting for Topological map ...")       
        while not self.map_received:
            rospy.sleep(rospy.Duration(0.1))
        rospy.loginfo("... Got Topological map")

        #Creating Action Server
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(action_name, door_pass.msg.BuildPredictionAction, execute_cb = self.build_callback, auto_start = False)
        self._as.register_preempt_callback(self.preempt_callback)
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")

        # Creating fremen server client
        rospy.loginfo("Creating fremen server client")
        self.FremenClient= actionlib.SimpleActionClient('fremenserver', fremenserver.msg.FremenAction)
        self.FremenClient.wait_for_server()
        rospy.loginfo(" ...done")

        # Create Door models        
        #self.create_models() not needed anymore its done on fremen_restart_cb

        #Advertise Service
        self.predict_srv=rospy.Service('/door_prediction/predict_doors', PredictDoorState, self.predict_door_cb)

        rospy.loginfo("Set-Up Fremenserver monitors")
        #Fremen Server Monitor
        self.fremen_monitor = rospy.Timer(rospy.Duration(10), self.monitor_cb)
        # Subscribe to fremen server start topic
        rospy.Subscriber('/fremenserver_start', std_msgs.msg.Bool, self.fremen_restart_cb)
        rospy.loginfo("... Done")

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
     fremen_start_cb
     
     This function creates the models when the fremenserver is started
    """
    def fremen_restart_cb(self, msg):
        if msg.data:
            rospy.logwarn("FREMENSERVER restart detected will generate new models now")
            with self.lock:
                self.create_models()
            

    """
     BuildCallBack
     
     This Functions is called when the Action Server is called to build the models again
    """
    def build_callback(self, goal):
        with self.lock:
            self.cancelled = False
    
            # print goal
    
            # set epoch ranges based on goal
            if goal.start_range.secs > 0:
                self.range[0] = goal.start_range.secs
            if goal.end_range.secs > 0:
                self.range[1] = goal.end_range.secs
    
            rospy.loginfo('Building model for epoch range: %s' % self.range)
    
            start_time = time.time()
            #self.get_list_of_edges()
            #elapsed_time = time.time() - start_time
            #self._feedback.result = "%d edges found in %.3f seconds \nGathering stats ..." %(len(self.eids),elapsed_time)
            #self._as.publish_feedback(self._feedback)
            #self.gather_stats()        
            self.create_models()
            elapsed_time = time.time() - start_time
            self._feedback.result = "Finished after %.3f seconds" %elapsed_time 
            self._as.publish_feedback(self._feedback)       #Publish Feedback
    
            #rospy.loginfo("Finished after %.3f sechttp://9gag.com/gag/aGR3z60onds" %elapsed_time)
            
            if not self.cancelled :     
                self._result.success = True
                self._as.set_succeeded(self._result)
            else:
                self._result.success = False
                self._as.set_preempted(self._result)


    """
     BuildCallBack
     
     This Functions is called when the Action Server is called to build the models again
    """
    def preempt_callback(self):
        self.cancelled = True


    """
     create_models
     
     This function finds all the doors in the environment 
     retrieves all the door passing stats,
     sorts them by door
    """
    def create_models(self):
        #self.unknowns=[]                #Contains model ids without statistics
        self.doors =[]                  #Contains statistics per door used to create fremen models
              
        stats=self.gather_stats()       # retrieves all the door passing stats
#        print "++++++++++++++"
        self.unknowns=self.find_doors()
#        print self.unknowns
#        print "++++++++++++++"
        #print self.doors
        self.extract_doors(stats)       # sorts stats by door
        
        for i in self.doors:
            i['order']=self.create_fremen_model(i)
            #print "-------------"

        print "Modelled Doors:"
        for i in self.doors:
            print i['model_id']['name']
        print "Unknown Doors:"
        for i in self.unknowns:
            print i['name']


    """
     find_doors
     
     This function finds all the doors in the environment using the topological map
    """
    def find_doors(self):
        doors=[]
        to_pop=[]
        for i in self.top_map.nodes:
            cwp = i.name
            #For all nodes on the environment find doorpassing actions and create a list of doors 
            for j in i.edges:
                if j.action == 'door_wait_and_pass':    
                    d={}
                    a = 'door'+'_'+cwp+'_'+j.node+'_res'
                    b = 'door'+'_'+cwp+'_'+j.node+'_time'
                    c = cwp+'_'+j.node
                    d['model_id']={'name':c,'res':a, 'time':b}
#                    d['order']={'state':0, 'time':0}
                    d['nodes']=[]
                    d['nodes'].append(cwp)
                    d['nodes'].append(j.node)
#                    d['stats']=[]
                    doors.append(d)

        #Find Repeated doors        
        for k in range(len(doors)):
            for l in range(k, len(doors)):
                if k!=l:
                    if doors[k]['nodes'][0] in doors[l]['nodes'] and doors[k]['nodes'][1] in doors[l]['nodes'] :
                        to_pop.append(l)
        
        #Remove Repeated doors
        to_pop.sort()
        to_pop.reverse()       
        for m in to_pop:
            doors.pop(m)

        to_pop=[]
        for k in range(len(doors)):
            for i in self.doors:
                if doors[k]['nodes'][0] in i['nodes'] and doors[k]['nodes'][1] in i['nodes'] :
                    to_pop.append(k)        

        to_pop.sort()
        to_pop.reverse()       
        for m in to_pop:
            doors.pop(m)        
        
        udoors=[x['model_id'] for x in doors]
        return udoors

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
               
        if len(tepochs)>0 and len(epochs):
            create_models=True
            #print 'Times: '+str(len(tepochs))+' samples using '+str(int(numpy.ceil(len(tepochs)*0.75)))+' for model building'# and '+str(int(numpy.ceil(len(epochs)*0.2)))+' for evaluation'
            #print 'Pass: '+str(len(epochs))+' samples using '+str(int(numpy.ceil(len(epochs)*0.75)))+' for model building'# and '+str(int(numpy.ceil(len(epochs)*0.2)))+' for evaluation'
        else:
            create_models=False
            rospy.logwarn("No MODELS created for %s, as there are no success stats for this door yet. It will be treated as unknown" %door['model_id']['res'])
            self.unknowns.append(door['model_id'])

        if create_models:
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
            to_ret['res']=self.add_and_eval_models(door['model_id']['res'], epochs_build, res_build, epochs_eval, res_eval)
    

            time_epochs_build = [ tepochs[i] for i in tindex_b]
            time_epochs_eval = [ tepochs[i] for i in tindex_e]
            times_build = [ times[i] for i in tindex_b]
            times_eval = [ times[i] for i in tindex_e]
            to_ret['time']=self.add_and_eval_value_models(door['model_id']['time'], time_epochs_build, times_build, time_epochs_eval, times_eval)
            
            #print index_b, epochs_build, res_build, times_build
            #print '---'
            #print index_e, epochs_eval, res_eval, times_eval
            #print '---'
            #print door['model_id']['res']
            #print to_ret

        else:
            to_ret={}

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
        #print ps
        
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
        #print pse.errors
        #print "chosen order %d" %pse.errors.index(min(pse.errors))
        return pse.errors.index(min(pse.errors))


    """
     add_and_eval_value_models
     
     This function creates and evaluates fremen models for float values
     it returns the recommended order for the predictions
    """
    def add_and_eval_value_models(self, model_id, a_epochs, a_states, e_epochs, e_states):
        #print a_states
        fremgoal = fremenserver.msg.FremenGoal()
        fremgoal.operation = 'addvalues'
        fremgoal.id = model_id
        fremgoal.times = a_epochs
        fremgoal.values = a_states
        
        # Sends the goal to the action server.
        self.FremenClient.send_goal(fremgoal)
        #print "Sending data to fremenserver"
        
        
        # Waits for the server to finish performing the action.
        self.FremenClient.wait_for_result()
        
        #print "fremenserver done"
        
        # Prints out the result of executing the action
        ps = self.FremenClient.get_result()
        #print "fremenserver result:"
        #print ps
        
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
        #print pse.errors
        #print "chosen order %d" %pse.errors.index(min(pse.errors))
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
            rospy.logerr("NO stats in datacentre. All doors will be found from map")
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
#        for l in self.doors:
#            print l['nodes']
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
        
#        total_stats=0
#        for h in self.doors:
#            print "----------"
#            print h['model_id']
#            print 'number of stats:' + str(len(h['stats']))
#            total_stats=total_stats+len(h['stats'])
#        print "----------"
#        print total_stats  


    def predict_door_cb(self, req):
        with self.lock:
            return self.get_predict(req.epoch.secs)

    def get_predict(self, epoch):
        rospy.loginfo("requesting prediction for time %d" %epoch)

        dids=[]
        dur=[]
        prob=[]

        if len(self.doors)>0:
        #        print self.unknowns
            unknowndoor = [x['name'] for x in self.unknowns]
            
            dids = [x['model_id']['name'] for x in self.doors if x['model_id']['name'] not in unknowndoor] #doors ids
        
            #door_pass outcome forecast
            unknowndoor = [x['res'] for x in self.unknowns]
            resids = [x['model_id']['res'] for x in self.doors if x['model_id']['res'] not in unknowndoor]     #door_pass outcome model_id
            resords = [x['order']['res'] for x in self.doors if x['model_id']['res'] not in unknowndoor]       #door_pass outcome model order
            prob = self.forecast_outcome(epoch, resids, resords)    #door_pass outcome prediction
            
            unknowndoor = [x['time'] for x in self.unknowns]
            #door_pass time forecast
            timids = [x['model_id']['time'] for x in self.doors if x['model_id']['time'] not in unknowndoor]    #door_pass time model_id
            timords = [x['order']['time'] for x in self.doors if x['model_id']['time'] not in unknowndoor]      #door_pass time model order
            dur2 = self.forecast_outcome(epoch, timids, timords)    #door_pass time prediction
            dur = [rospy.Duration.from_sec(x* self.wait_timeout) for x in dur2]
        

        for i in self.unknowns:
            dids.append(i['name'])
            prob.append(0.5)
            dur.append(rospy.Duration.from_sec(self.wait_timeout/2))
        
        #print resords
        for i in range(len(prob)):
            if prob[i] <= 0.0:
                prob[i] = 0.000001
            elif prob[i] >= 1.0:
                prob[i] = 0.999999

        return dids, prob, dur


    def forecast_outcome(self, epoch, mods, ords):
        #print epoch, mods, ords
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
        #print ps
        prob = list(ps.probabilities)
        return prob


    """
     monitor_cb
     
     This function monitors if fremenserver is still active
    """
    def monitor_cb(self, events) :
        if not self.FremenClient.wait_for_server(timeout = rospy.Duration(1)):
            rospy.logerr("NO FREMEN SERVER FOUND. Fremenserver restart might be required")


    def _on_node_shutdown(self):
        self.fremen_monitor.shutdown()

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