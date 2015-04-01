#!/usr/bin/env python
import rospy
import tf
from static_transform_manager.srv import ( SetTransformation, SetTransformationResponse, 
                                   StopTransformation, StopTransformationResponse
                                   )


class StaticTransformationsManager(object):
    def __init__(self, freq):
        self._tf_broadcaster =  tf.TransformBroadcaster()
        self._rater =  rospy.Rate(freq)
        self._running =  True
        self._transforms = {} # Indexed by child name
        
        # set up services
        self._start_srv = rospy.Service(rospy.get_name()+"/set_tf",
                                        SetTransformation,
                                        self._start_transform_srv)
        self._start_srv = rospy.Service(rospy.get_name()+"/stop_tf",
                                        StopTransformation,
                                        self._stop_tranform_srv)
        pass
    
    def _start_transform_srv(self, req):
        resp =  SetTransformationResponse()
        self._transforms[req.transform.child_frame_id] = req.transform
        resp.is_ok = True
        resp.response =  "Frame added"
        return resp
    
    def _stop_tranform_srv(self, req):
        resp =  StopTransformationResponse()
        if self._transforms.has_key(req.child_frame_id):
            self._transforms.pop(req.child_frame_id)
            resp.response =  "ok"
            resp.is_ok =  True
        else:
            resp.response =  "frame not known by this managerx"
            resp.is_ok = False
        return resp
    
    def start(self):
        while self._running and not rospy.is_shutdown():
            self._rater.sleep()
            for transform in self._transforms.values():
                self._tf_broadcaster.sendTransform((transform.transform.translation.x,
                                                    transform.transform.translation.y,
                                                    transform.transform.translation.z), 
                                                   (transform.transform.rotation.x,
                                                    transform.transform.rotation.y,
                                                    transform.transform.rotation.z,
                                                    transform.transform.rotation.w), 
                                                   rospy.Time.now(),
                                                   transform.child_frame_id,
                                                   transform.header.frame_id)

                
if __name__ == '__main__':
    ''' Main Program '''
    rospy.init_node("static_transforms_manager")
    manager = StaticTransformationsManager(30)
    manager.start()
