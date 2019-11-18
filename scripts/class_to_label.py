#!/usr/bin/env python
import rospy
from jsk_recognition_msgs.msg import Label, LabelArray
from jsk_recognition_msgs.msg import ClassificationResult

class ClassToLabel:

    def __init__(self):
        self.pub_labels = rospy.Publisher("/ssd_label", LabelArray, queue_size=10)
        rospy.init_node("class_to_label", anonymous=True)
        rospy.Subscriber("/ssd_object_detector/output/class", ClassificationResult, self.pub_label)
        rospy.spin()

    def pub_label(self, msg):
        label_msg = LabelArray(header=msg.header)
        for label, label_name in zip(msg.labels, msg.label_names):
            label = Label(id=label, name=label_name)
            label_msg.labels.append(label)
        self.pub_labels.publish(label_msg)

if __name__ == '__main__':
    ins = ClassToLabel()

