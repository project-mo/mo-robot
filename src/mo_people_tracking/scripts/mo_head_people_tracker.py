#!/usr/bin/python

import math

import rospy
from sensor_msgs.msg import CameraInfo
from image_recognition_msgs.msg import Recognitions
from std_msgs.msg import Float64
import image_geometry


class MoHeadPeopleTracker(object):
    def __init__(self):
        self._cam_info_subscriber = rospy.Subscriber("camera_info", CameraInfo, self._process_camera_info)
        self._detections_subscriber = rospy.Subscriber("detections", Recognitions, self._process_recognitions)

        self._head_reference_publisher = rospy.Publisher("head_yaw_reference", Float64, queue_size=1)

        self.cam_model = image_geometry.PinholeCameraModel()
        self.cam_model_configured = False

        self.latest_detection_time = rospy.Time(0)
        self.yaw = None

    def _process_camera_info(self, camera_info):
        if not self.cam_model_configured:
            self.cam_model.fromCameraInfo(camera_info)
            cam_model_configured = True
    
    def _process_recognitions(self, recognitions):
        # Find the largest ROI (we assume this is the closest person)
        largest_roi_width = 0
        roi = None
        for r in recognitions.recognitions:
            if r.roi.width > largest_roi_width:
                largest_roi_width = r.roi.width
                roi = r.roi

        # If no ROI is found, return
        if not roi:
            return
        
        # extract the yaw angle from the center of the ROI
        x_min = roi.x_offset
        y_min = roi.y_offset
        x_max = x_min + roi.width
        y_max = y_min + roi.height

        u = (x_min + x_max) // 2
        v = (y_min + y_max) // 2

        ray = self.cam_model.projectPixelTo3dRay((u, v))

        self.latest_detection_time = recognitions.header.stamp
        self.yaw = math.atan2(ray[1], ray[0])
        rospy.loginfo("Detected person, remembering yaw = {}".format(self.yaw))

    def publish_head_reference(self):
        if rospy.Time.now() - self.latest_detection_time < rospy.Duration(3):
            rospy.loginfo("Person detected, sending head reference yaw = {}".format(self.yaw))
            self._head_reference_publisher.publish(Float64(self.yaw))
        else:
            rospy.loginfo("No person detected, sending head reference yaw = 0")
            self._head_reference_publisher.publish(Float64(0))


def main():
    rospy.init_node("mo_head_people_tracking")

    mo_head_people_tracker = MoHeadPeopleTracker()

    r = rospy.Rate(0.1)

    while not rospy.is_shutdown():
        mo_head_people_tracker.publish_head_reference()
        r.sleep()


if __name__ == '__main__':
    main()
