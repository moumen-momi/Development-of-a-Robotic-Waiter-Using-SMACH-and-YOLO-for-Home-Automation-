#!/usr/bin/env python3
# coding=utf-8
import rospy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from second_coursework.srv import YOLOLastFrame, YOLOLastFrameResponse
from second_coursework.msg import YOLODetection
from yolov4 import Detector


class YOLOv4ROSITR:
    def __init__(self):
        self.cv_image = None
        self.bridge = CvBridge()
        self.cam_subs = rospy.Subscriber("/camera/image", Image, self.img_callback)
        self.yolo_srv = rospy.Service('/detect_frame', YOLOLastFrame, self.yolo_service)
        self.detector = Detector(gpu_id=0, config_path='/opt/darknet/cfg/yolov4.cfg',
                                 weights_path='/opt/darknet/yolov4.weights',
                                 lib_darknet_path='/opt/darknet/libdarknet.so',
                                 meta_path='/home/k23015707/ros_ws/src/second_coursework/cfg/coco.data')

    def img_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        rospy.loginfo("I have received a new image")

    def yolo_service(self, request):
        res = YOLOLastFrameResponse()
        if self.cv_image is None:
            rospy.logwarn("I have not yet received any image")
        else:
            img_arr = cv2.resize(self.cv_image, (self.detector.network_width(), self.detector.network_height()))
            cv_height, cv_width, _ = self.cv_image.shape
            detections = self.detector.perform_detect(image_path_or_buf=img_arr, show_image=True)
            for detection in detections:
                box = detection.left_x, detection.top_y, detection.width, detection.height
                print(f'{detection.class_name.ljust(10)} | {detection.class_confidence * 100:.1f} % | {box}')
                d = YOLODetection(detection.class_name, detection.class_confidence,
                                  detection.left_x, detection.top_y, detection.width, detection.height)

                # Convert bbox to image space
                d.bbox_x = int((d.bbox_x / self.detector.network_width()) * cv_width)
                d.bbox_y = int((d.bbox_y / self.detector.network_height()) * cv_height)
                d.width = int((d.width / self.detector.network_width()) * cv_width)
                d.height = int((d.height / self.detector.network_height()) * cv_height)
                res.detections.append(d)
        return res

if __name__ == '__main__':
    rospy.init_node('yolo_ros_itr')
    yolo_ros = YOLOv4ROSITR()
    rospy.spin()
    # To detect image you run rosservice call /detect_frame
