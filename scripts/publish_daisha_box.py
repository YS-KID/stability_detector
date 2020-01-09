#!/usr/bin/env python
import rospy
import numpy as np

from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray

class DaishaBox:

    def __init__(self):
        self.pub_box = rospy.Publisher("/daisha_box", BoundingBox, queue_size=10)
        rospy.init_node("daisha_box_publisher", anonymous=True)
        rospy.Subscriber("/HSI_color_filter_blue/boxes", BoundingBoxArray, self.publish_box)
        rospy.spin()

    def publish_box(self, msg):
        filtered_boxes = []
        filtered_dimension_z = []
        for box in msg.boxes:
            if box.pose.position.x < 1.0:
                filtered_boxes.append(box)
                filtered_dimension_z.append(box.dimensions.z)
        msg_box = BoundingBox(header=msg.header)
        self.pub_box.publish(filtered_boxes[np.argmax(filtered_dimension_z)])

if __name__ == '__main__':
    ins = DaishaBox()

