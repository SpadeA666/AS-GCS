#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from yolov8_ros_msgs.msg import BoundingBox, BoundingBoxes

class ArUcoDetector:
    def __init__(self):
        # 1. 初始化 ROS 节点
        rospy.init_node('aruco_detector_node', anonymous=True)
        self.bridge = CvBridge()

        # 2. 兼容性配置 ArUco 字典和参数
        # 针对你图片中的 6x6_250 字典
        try:
            # 尝试旧版 OpenCV API (Noetic 默认)
            self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            self.parameters = aruco.DetectorParameters_create()
            rospy.loginfo("使用 OpenCV 旧版 ArUco API")
        except AttributeError:
            # 尝试新版 OpenCV API (4.7+)
            self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
            self.parameters = aruco.DetectorParameters()
            rospy.loginfo("使用 OpenCV 新版 ArUco API")

        # 3. 话题配置
        # 订阅：请确保这里的 /usb_cam/image_raw 与你的相机驱动话题一致
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)

        # 发布：发布识别标注后的图像
        self.image_pub = rospy.Publisher("/aruco/detection_result", Image, queue_size=1)
        # 发布：ArUco 像素坐标（复用 BoundingBoxes 格式，供 C++ 端 trackArUcoDown 使用）
        self.aruco_box_pub = rospy.Publisher("/aruco/bounding_boxes", BoundingBoxes, queue_size=1)

        rospy.loginfo("ArUco 识别节点已启动，等待图像输入...")

    def callback(self, data):
        try:
            # 将 ROS 图像转为 OpenCV 格式
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # 4. ArUco 检测核心逻辑
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        # 5. 标注处理
        if ids is not None:
            # 在原图上画出方框和 ID
            aruco.drawDetectedMarkers(frame, corners, ids, borderColor=(0, 0, 255))

            # 构造 BoundingBoxes 消息（复用 YOLO 格式，发给 C++ 端做视觉伺服）
            bbox_msg = BoundingBoxes()
            bbox_msg.header = Header(stamp=rospy.Time.now())
            bbox_msg.len = len(ids)

            for i in range(len(ids)):
                pts = np.int32(corners[i][0])
                cv2.polylines(frame, [pts], True, (0, 0, 255), thickness=10)
                id_val = ids[i][0]
                rospy.loginfo("OK成功 检测到标记 ID: {}".format(id_val))

                # 计算角点包围框的 min/max（像素坐标）
                xmin = int(np.min(pts[:, 0]))
                xmax = int(np.max(pts[:, 0]))
                ymin = int(np.min(pts[:, 1]))
                ymax = int(np.max(pts[:, 1]))

                # 填入 BoundingBox 消息
                box = BoundingBox()
                box.Class = "aruco_{}".format(id_val)
                box.probability = 1.0
                box.xmin = xmin
                box.xmax = xmax
                box.ymin = ymin
                box.ymax = ymax
                bbox_msg.bounding_boxes.append(box)

                # 计算并显示中心像素坐标
                cx = (xmin + xmax) // 2
                cy = (ymin + ymax) // 2
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

                # 在画面上加标注
                cv2.putText(frame, "TARGET ID: {} ({},{})".format(id_val, cx, cy),
                            (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), thickness=5)

            # 发布像素坐标
            self.aruco_box_pub.publish(bbox_msg)
        else:
            # 如果没识别到，可以在画面角落显示状态
            cv2.putText(frame, "Searching...", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # 6. 发布结果图像
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        detector = ArUcoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("正在关闭识别节点...")
        cv2.destroyAllWindows()