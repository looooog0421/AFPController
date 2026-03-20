#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolRequest


class SemanticRelayController:
    def __init__(self):
        rospy.init_node("scissor_controller", anonymous=False)

        self.enable_topic = rospy.get_param("~enable_topic", "/scissor/enable")
        self.service_name = rospy.get_param("~service_name", "/relay_driver/output1/set")
        self.label = rospy.get_param("~label", "scissor")

        rospy.loginfo("Waiting for relay service: %s", self.service_name)
        rospy.wait_for_service(self.service_name)
        self.proxy = rospy.ServiceProxy(self.service_name, SetBool)

        rospy.Subscriber(self.enable_topic, Bool, self._callback, queue_size=10)
        rospy.loginfo("%s controller subscribed to %s", self.label, self.enable_topic)
        rospy.spin()

    def _callback(self, msg: Bool):
        try:
            request = SetBoolRequest(data=bool(msg.data))
            response = self.proxy(request)
            if response.success:
                rospy.loginfo("%s command applied: %s", self.label, msg.data)
            else:
                rospy.logwarn("%s command rejected: %s", self.label, response.message)
        except rospy.ServiceException as e:
            rospy.logerr("%s controller service call failed: %s", self.label, e)


if __name__ == "__main__":
    try:
        SemanticRelayController()
    except rospy.ROSInterruptException:
        pass
