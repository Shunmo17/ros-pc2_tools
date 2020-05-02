#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import ros_numpy
import numpy as np

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

class Pc2ToImage():
    def __init(self):
        self.pc2_topic = rospy.get_param("~input_pc2_topic")
        self.image_topic = rospy.get_param("~output_image_topic")

        # node
        self.sub_pc2 = rospy.Subscriber(self.pc2_topic, PointCloud2, self.pc2_callback)
        self.pub_image = rospy.Publisher(self.image_topic, Image, queue_size=1)
    
    def pc2_callback(self, _pc2_msg):
        self.pub_image.publish(self.generate_image(_pc2_msg))

    #  generate image from pc2
    def generate_image(self, pc2_msg):
        # picture size
        height = pc2_msg.height
        width = pc2_msg.width

        # convert
        array = np.zeros((height, width), dtype=np.float32)

        # save data to arrary
        pc2_original = ros_numpy.numpify(pc2_msg)
        array[:, :] = pc2_original['rgb']

        # convert color (html color -> rgb)
        data = array.view(np.uint8).reshape(array.shape + (4,))[..., :3]
        image = ros_numpy.msgify(Image, data, encoding='bgr8')
        return image

def main():
    rospy.init_node("pc2_to_image")

    print("===== PointCloud2 to Image Convertor ======================")
    pc2_to_image = Pc2ToImage()

    rospy.spin()


if __name__ == '__main__':
    main()
