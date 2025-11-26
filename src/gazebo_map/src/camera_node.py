#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from ultralytics import YOLO


model = YOLO('/home/liu/ai_community/src/gazebo_map/src/yolo5su_ai_community_new.pt')

def image_callback(msg):
    global model
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        results = model(cv_image_rgb)

        for result in results:
            final_img=result.plot()

        cv2.imshow("Camera Image", final_img)
        cv2.waitKey(1)
    except Exception as e:
        rospy.logerr(f"Error converting image: {e}")


if __name__ == '__main__':
    try:
        rospy.init_node('camera_image_reader', anonymous=True)

        rospy.Subscriber("/image_raw", Image, image_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
