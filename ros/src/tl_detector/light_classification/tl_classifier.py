from styx_msgs.msg import TrafficLight

import rospy
import yaml

import tensorflow as tf
import numpy as np
import cv2
import sys
import os

CONFIDENCE_CUTOFF = 0.7  # Scores threshold 
PATH_GRAPH_FILE_SIM  = '/light_classification/models/sim/'   # Simulator
PATH_GRAPH_FILE_SITE = '/light_classification/models/site/'  # Real site
SALVE_IMG = False # Salve imagem to Data Training

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        sys.path.append("..")

        # Launch file will distinguish either site or sim
        config_string = rospy.get_param("/traffic_light_config")

        # config is a  dictionary
        config = yaml.load(config_string)  
        
        # Check site or simulator 
        self.is_site = config['is_site']  

        if self.is_site:
            self.graph_path = os.getcwd() + PATH_GRAPH_FILE_SITE + 'frozen_inference_graph.pb'
        else:
            self.graph_path = os.getcwd() + PATH_GRAPH_FILE_SIM + 'frozen_inference_graph.pb'


        rospy.logwarn('TLClassifier self.is_site={}, graph_path={}'.format(self.is_site, self.graph_path))

        # Loads a frozen inference graph
        detection_graph = self.load_graph(self.graph_path) 

        # To put Session graph
        self.sess = tf.Session(graph=detection_graph)

        # The input placeholder for the image.
        # `get_tensor_by_name` returns the Tensor with the associated name in the Graph.
        self.image_tensor = self.sess.graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.sess.graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.sess.graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.sess.graph.get_tensor_by_name('detection_classes:0')
        
        # initialize as RED for safety at the begining
        self.light_state = TrafficLight.RED 

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        #TODO implement light color prediction
        
        # Actual detection.
        boxes, scores, classes = self.sess.run([self.detection_boxes, self.detection_scores, self.detection_classes], 
                                                feed_dict={self.image_tensor: np.expand_dims(image, 0)})
        # Remove unnecessary dimensions
        # boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes)

        # Filter boxes with a confidence score less than `confidence_cutoff`
        #classes must get alighed with what is defined in label_map.pbtxt
        if (scores[0] > CONFIDENCE_CUTOFF):
            rospy.logwarn("scores[0]={} > CONFIDENCE_CUTOFF={}".format(scores[0], CONFIDENCE_CUTOFF))
            if (classes[0] == 1.0):  
                self.light_state = TrafficLight.GREEN
            elif  (classes[0] == 2.0):
                self.light_state = TrafficLight.RED
            elif  (classes[0] == 3.0):
                self.light_state = TrafficLight.YELLOW

        # Write image to disk
        if SALVE_IMG:
            save_file = "./home/workspace/CarND-Capstone/data/images_dataset/img.jpg"
            #save_file = ".C:/Users/ednaldo.goncalves/Documents/GitHub/CarND-Capstone-P9/data/images_dataset/img.jpg"
            cv2.imwrite(save_file, image)

        return self.light_state

    
    def load_graph(self, graph_file):
        # Loads a frozen inference graph
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return graph