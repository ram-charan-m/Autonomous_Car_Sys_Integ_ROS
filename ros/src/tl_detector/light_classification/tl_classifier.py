from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import cv2
import os

class TLClassifier(object):
    def __init__(self):
        SSD_GRAPH_FILE = 'light_classification/TF_model/frozen_inference_graph.pb'
        detection_graph = self.load_graph(SSD_GRAPH_FILE)
        
        self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
        self.confidence_cutoff = 0.30
        self.sess = tf.Session(graph=detection_graph)
        
    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        # Light color prediction
        image_np = np.expand_dims(np.asarray(image, dtype=np.uint8), 0)
 (boxes, scores, classes) = self.sess.run([self.detection_boxes, self.detection_scores, self.detection_classes],
                                        feed_dict={self.image_tensor: image_np})
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes)
        boxes, scores, classes = self.filter_boxes(self.confidence_cutoff, boxes, scores, classes)
        width = image.shape[1]
        height = image.shape[0]
        box_coords = self.to_image_coords(boxes, height, width)
        
        # Light color classification
        red_thresh = 0
        for i in range(len(box_coords)):
            bottom, left, top, right = box_coords[0, ...]
            buffer = (top - bottom)*0.1 # adding a buffer for creating a larger search area for red block within traffic signal 
            bottom = int(bottom + buffer)
            top = int(top - buffer)
            left = int(left + buffer/2)
            right = int(right - buffer/2)
            block_height = int((top - bottom)/3.0)
            red_block = image[int(bottom):int(bottom+block_height),int(left):int(right)]
            val = np.mean(red_block[:,:,2])/(np.mean(red_block[:,:,0]) + np.mean(red_block[:,:,1]) + np.mean(red_block[:,:,2]))
            print('Val: ', val)
            if(val > 0.40):
                red_thresh +=1

        if red_thresh > 0:
            print("<<----------BRAKING!----------->>")
            return TrafficLight.RED
        else:
            return TrafficLight.UNKNOWN

    def load_graph(self, graph_file):
        """Loads a frozen inference graph"""
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return graph

    def filter_boxes(self, min_score, boxes, scores, classes):
        """Return boxes with a confidence >= `min_score` and class of traffic lights(10)"""
        n = len(classes)
        idxs = []
        for i in range(n):
            if scores[i] >= min_score and classes[i] == 10 and boxes[i][2] - boxes[i][0] > 0.07:
                idxs.append(i)

        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_boxes, filtered_scores, filtered_classes

    def to_image_coords(self, boxes, height, width):
        """
        The original box coordinate output is normalized, i.e [0, 1].
        This converts it back to the original coordinate based on the image
        size.
        """
        box_coords = np.zeros_like(boxes)
        box_coords[:, 0] = boxes[:, 0] * height
        box_coords[:, 1] = boxes[:, 1] * width
        box_coords[:, 2] = boxes[:, 2] * height
        box_coords[:, 3] = boxes[:, 3] * width
        return box_coords