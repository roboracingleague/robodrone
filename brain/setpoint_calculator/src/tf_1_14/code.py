#!/usr/bin/env python3
from __future__ import print_function
import sys
import rospy
import cv2
import tensorflow as tf
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
class image_converter:
    def __init__(self, filepath):
        self.bridge = CvBridge()
        self.lock = False
        self.graph = self.load_graph(filepath)
        self.sess = tf.Session(graph=self.graph)
        self.x = self.graph.get_tensor_by_name('ssd_mobilnet/image_tensor:0')
        self.y1 = self.graph.get_tensor_by_name('ssd_mobilnet/detection_classes:0')
        self.y2 = self.graph.get_tensor_by_name('ssd_mobilnet/detection_boxes:0')
        self.image_sub = rospy.Subscriber("/camera/fisheye1/image_raw",Image,self.callback)
    def callback(self,data):
        if not self.lock:
            self.lock = True
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
            except CvBridgeError as e:
                print(e)
            cv_image = cv2.resize(cv_image,(318,300),interpolation=cv2.INTER_AREA)
            last_axis = -1
            dim_to_repeat = 2
            repeats = 3
            grscale_img_3dims = np.expand_dims(cv_image, last_axis)
            image = np.repeat(grscale_img_3dims, repeats, dim_to_repeat).astype('uint8')
            y1_out, y2_out = self.sess.run([self.y1, self.y2], feed_dict={
                self.x: [image] # < 45
            })
            self.lock = False
            print(y1_out)
            # print(y2_out)
        else:
            print("lock")
    def load_graph(self, frozen_graph_filename):
        # We load the protobuf file from the disk and parse it to retrieve the
        # unserialized graph_def
        with tf.io.gfile.GFile(frozen_graph_filename, "rb") as f:
            graph_def = tf.compat.v1.GraphDef()
            graph_def.ParseFromString(f.read())
        # Then, we import the graph_def into a new Graph and returns it
        with tf.Graph().as_default() as graph:
        # The name var will prefix every op/nodes in your graph
        # Since we load everything in a new graph, this is not needed
            tf.import_graph_def(graph_def, name="ssd_mobilnet")
        return graph
def main(args):
    ic = image_converter('./model/frozen_inference_graph.pb')
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
if __name__ == '__main__':
    main(sys.argv)
