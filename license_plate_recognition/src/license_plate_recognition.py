#!/usr/bin/env python3
#coding=utf-8

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import hyperlpr3 as lpr3
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from FisheyeParam import *
from showtrajectory.msg import OCRResult
from geometry_msgs.msg import Polygon, Point32
from sensor_msgs.msg import NavSatFix

license_pub = rospy.Publisher('license_with_box', OCRResult, queue_size=1)
gps_1 = 0

def gps_time(gps):
   gps_t = 1


def license_plate_recognition(compressed_image):
    bridge = CvBridge()
    global gps_1
    if gps_1 % 10 == 0 :
      #  cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        np_arr = np.fromstring(compressed_image.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
      # 执行车牌识别算法
        catcher = lpr3.LicensePlateCatcher(detect_level=lpr3.DETECT_LEVEL_HIGH)
        results = catcher(cv_image)
        pil_image = Image.fromarray(cv_image)  # 将 OpenCV 图像转换为 PIL 图像
        draw = ImageDraw.Draw(pil_image)  # 创建绘制对象
        font_ch = ImageFont.truetype("/home/user/下载/platech.ttf", 20, 0)
        for code, confidence, type_idx, box in results:
      # 在图像上绘制车牌框和文本
            x1, y1, x2, y2 = box
          
            draw.rectangle([(x1, y1), (x2, y2)], outline=(0, 255, 0))
            draw.text((x1, y1 - 30), code, fill=(0, 0, 255), font=font_ch)
            draw.text((x1, y1 - 60), f"Confidence: {confidence:.2f}", fill=(0, 0, 255), font=font_ch)
            # print(corners)
            M = np.array([
            [6.19087214455194, -13.6783918575607, -5114.04257928648],
            [-0.07103670281201, -7.1647058925724, -2210.0437877731],
            [-0.000452864931393468, -0.0140691132595466, -5.29547420841964]
            ])
            # 将点转换为齐次坐标
            point_1 = np.array([[x1],
                                    [y1],
                                  [1]])

    # 使用变换矩阵 T 对点进行逆透视变换
            transformed_point_homogeneous = np.dot(M, point_1)

    # 将齐次坐标转换回普通坐标
            transformed_x1 = transformed_point_homogeneous[0] / transformed_point_homogeneous[2]
            transformed_y1 = transformed_point_homogeneous[1] / transformed_point_homogeneous[2]
            point_2 = np.array([[x2],
                                    [y2],
                                  [1]])

    # 使用变换矩阵 T 对点进行逆透视变换
            transformed_point_homogeneous = np.dot(M, point_2)

    # 将齐次坐标转换回普通坐标
            transformed_x2 = transformed_point_homogeneous[0] / transformed_point_homogeneous[2]
            transformed_y2 = transformed_point_homogeneous[1] / transformed_point_homogeneous[2]
            license_plate_recognition_with_box = OCRResult()
                  # 创建 Header 对象，设置时间戳和其他字段
              # text_with_box.stamp=gps_1
            # x1_1 = 540-transformed_x1
            # y1_1 = 480-transformed_y1
            # x2_1 = 540-transformed_x2
            # y2_1= 480-transformed_y2
            x1_1 = transformed_x1-540
            y1_1 = transformed_y1-480
            x2_1 = transformed_x2-540
            y2_1= transformed_y2-480
            license_plate_recognition_with_box.polygon.points = [
                      Point32(x=x1_1/108, y=y1_1/108),
                      Point32(x=x2_1/108, y=y1_1/108),
                      Point32(x=x2_1/108, y=y2_1/108),
                      Point32(x=x1_1/108, y=y2_1/108)
                      # Point32(x=x1_1, y=y1_1),
                      # Point32(x=x2_1, y=y1_1),
                      # Point32(x=x2_1, y=y2_1),
                      # Point32(x=x1_1, y=y2_1)
                  ]
            license_plate_recognition_with_box.text = code
            license_plate_recognition_with_box.confidence=confidence
              # print(text_with_box.text)
        # 发布边界框和文本消息
            license_pub.publish(license_plate_recognition_with_box)


    cv_image = np.array(pil_image)  # 将 PIL 图像转换回 OpenCV 格式
        #cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
   # cv2.putText(cv_image, f"Confidence: {confidence:.2f}", (x1, y1 - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
   # cv2.putText(cv_image, f"{code} - {confidence:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.imshow("License Plate Recognition", cv_image)
    cv2.waitKey(1)

def main():
    rospy.init_node("license_plate_recognition_node", anonymous=True)
    rospy.Subscriber("/driver/fisheye/front/compressed", CompressedImage, license_plate_recognition)
    rospy.Subscriber("/Inertial/gps/fix", NavSatFix, gps_time)
    rospy.spin()

if __name__ == '__main__':
    main()
