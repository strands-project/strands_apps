from . import MonitorType
from threading import Timer
import rospy
import rostopic

class TopicAlive(MonitorType):
    name = "TopicAlive"
    description = ("This monitor triggers the actions if there are no messages"
                   " on the given topic for a given period")
    config_keys = [('topic', "The topic to monitor"),
                   ('max_duration', "The maximum number of seconds to accept "
                    "not receiving a message")]
    
    def __init__(self, monitor_config, invalid_cb):
        super(TopicAlive, self).__init__(monitor_config, invalid_cb)
        self.max_duration = rospy.Duration(self.max_duration)
        
    def start(self):
        self._validity_timer = Timer(self.max_duration.to_sec(), self.timout_cb)
        self._last_time = None
        self._topic_sub = None
        self._retry_timer = None
        MsgClass, topic, _func = rostopic.get_topic_class(self.topic)
        
        if topic is not None:
            self._topic_sub = rospy.Subscriber(topic, MsgClass, self.topic_cb)
            self._last_time = rospy.Time.now()
            self._validity_timer.start()
        else:
            rospy.logwarn("Could not determine type of '{}' topic. Retry in "
                          "10s.".format(self.topic))
            self._retry_timer = Timer(10, self.start)
            self._retry_timer.start()

    def stop(self):
        if self._topic_sub is not None:
            self._topic_sub.unregister()
        if self._retry_timer is not None:
            self._retry_timer.cancel()
        self._validity_timer.cancel()

    def topic_cb(self, msg):
        self._last_time = rospy.Time.now()
        
    def timout_cb(self):
        duration =  rospy.Time.now() - self._last_time
        if duration > self.max_duration:
            rospy.logwarn("No message received on {} for {}s.".format(self.topic,
                                                                      duration.to_sec()))
            self.set_invalid()
        else:
            self._validity_timer = Timer((self.max_duration - duration).to_sec(),
                                         self.timout_cb)
            self._validity_timer.start()
        
class TopicPublished(MonitorType):
    name = "TopicPublished"
    description =  ("This monitor triggers the actions if a message "
                            "is published on a given topic")
    config_keys = [('topic', "The topic to listen to")]
 
    def start(self):
        self._topic_sub = None
        self._retry_timer = None
        MsgClass, topic, _func = rostopic.get_topic_class(self.topic)
        print topic
        if topic is not None:
            self._topic_sub = rospy.Subscriber(topic, MsgClass, self.topic_cb)
        else:
            rospy.logwarn("Could not determine type of '{}' topic. Retry in "
                          "10s.".format(self.topic))
            self._retry_timer = Timer(10, self.start)
            self._retry_timer.start()
    
    def stop(self):
        if self._topic_sub is not None:
            self._topic_sub.unregister()
        if self._retry_timer is not None:
            self._retry_timer.cancel()
    
    def topic_cb(self, msg):
        self.set_invalid()
        
    
class TopicFieldCondition(MonitorType):
    name = "TopicFieldCondition"
    description = ("This monitor checks a field in a given topic and triggers "
                   "the actions when a condition is met.")
    config_keys = [('topic', "The topic & field to watch, eg /topic/field"),
                   ('condition', "A python expression containing {value} "
                    "as a place holder for the value of the topic field. For "
                    "example '{value} < 30'")]
   
    def start(self):
        self._topic_sub = None
        self._retry_timer = None
        MsgClass, topic, self._eval_func = rostopic.get_topic_class(self.topic)
        if topic is not None:
            if self._eval_func is None:
                self._eval_func = lambda x: x
            self._topic_sub = rospy.Subscriber(topic, MsgClass, self.topic_cb)
        else:
            rospy.logwarn("Could not determine type of '{}' topic. Retry in "
                          "10s.".format(self.topic))
            self._retry_timer = Timer(10, self.start)
            self._retry_timer.start()

    def topic_cb(self, msg):
        value = self._eval_func(msg)
        test =  self.condition.format(value=value)
        try:
            condition = eval(test) # don't expect moronic users.
        except:
            raise Exception("Problem evaluating expression '{}'".format(test))
        if condition: 
            rospy.logwarn("TopicFieldCondition monitor triggering.")
            self.set_invalid()
        
    
    def stop(self):
        if self._topic_sub is not None:
            self._topic_sub.unregister()
        if self._retry_timer is not None:
            self._retry_timer.cancel()
        