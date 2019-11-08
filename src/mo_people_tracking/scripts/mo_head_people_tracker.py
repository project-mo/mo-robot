#!/usr/bin/python

import rospy
from sensor_msgs.msg import CameraInfo
from image_recognition_msgs.msg import Recognitions
import image_geometry

class MoHeadPeopleTracker(object):
    def __init__(self):
        self._cam_info_subscriber = rospy.Subscriber("camera_info", CameraInfo, self._process_camera_info)
        self._detections_subscriber = rospy.Subscriber("detections", Recognitions, self._process_recognitions)

        cam_model = image_geometry.PinholeCameraModel()
        cam_model_configured = False

        self.yaw = None

    def _process_camera_info(self, camera_info):
        if not cam_model_configured:
            cam_model.fromCameraInfo(camera_info)
            cam_model_configured = True
    
    def _process_recognitions(self, recognitions):
        # Find the largest ROI (we assume this is the closest person)
        largest_roi_width = 0
        roi = None
        for r in recognitions.recognitions:
            if r.roi.width > largest_roi_width:
                largest_roi_width = r.roi.width
                largest_roi = r.roi

	# If no ROI is found, return
        if not roi:
            return
        
	# extract the yaw angle from the center of the ROI
        x_min = roi.x_offset
        y_min = roi.y_offset
        x_max = x_min + width
        y_max = y_min + height

        u = (x_min + x_max) // 2
        v = (y_min + y_max) // 2

        ray = cam_model.projectPixelTo3dRay((u, v))

        self.yaw = math.atan2(ray[1], ray[0])
#        yaw_publisher.publish(self.yaw)


def main():
    rospy.init_node("mo_head_people_tracking")

    mo_head_people_tracker = MoHeadPeopleTracker()

    rospy.spin()

if __name__ == '__main__':
    main()
