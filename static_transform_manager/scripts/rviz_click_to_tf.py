#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PointStamped, TransformStamped
from static_transform_manager.srv import ( SetTransformation,
                                            SetTransformationResponse, 
                                            StopTransformation,
                                            StopTransformationResponse
                                        )
class RVizClickToTF(object):
    def __init__(self, frame_name):
        self._frame_name = frame_name
        self._click_sub = rospy.Subscriber("/clicked_point",
                                           PointStamped,
                                           self._on_clicked)
        self._set_transform =  rospy.ServiceProxy("/static_transforms_manager/set_tf",
                                                  SetTransformation)


    def _on_clicked(self, pt):
        t = TransformStamped()
        t.child_frame_id = "clicked"
        t.header.frame_id = pt.header.frame_id 
        t.transform.translation.x = pt.point.x 
        t.transform.translation.y = pt.point.y
        t.transform.rotation.w = 1
        self._set_transform(t)


if __name__ == "__main__":
    rospy.init_node("ptu_follow_frame_click_tester")
    click_to_tf = RVizClickToTF("clicked")
    rospy.spin()

