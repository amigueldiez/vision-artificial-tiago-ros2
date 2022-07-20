# Import the necessary libraries
from email.header import Header
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import torch
import numpy as np
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose
 
class ImageAnalyser(Node):
  
  def __init__(self):
    """
    Class constructor to set up the node
    """ 
    # Initiate the Node class's constructor and give it a name
    super().__init__('img_analyzer')
      

    weights= './best.pt'
    self.model = torch.hub.load('yolov5/', 'custom', weights, source='local')
    #self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')


    self.classes = self.model.names

    self.colors = [(255,0,0), (0,0,255)]

    self.frame_num = 0
    

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      'xtion/rgb/image_raw', 
      self.listener_callback, 
      10)

    self.bbox_publisher = self.create_publisher(Detection2DArray, 'cv_basics/detections', 10)

    self.subscription # prevent unused variable warning


    self.get_logger().info("Node Initialized")
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    #self.get_logger().info('Receiving video frame')
  
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)

    p = self.model(current_frame)

    # Send bounding boxes with Detection2DArray message
    detection_msg = Detection2DArray()
    detection_msg.header.stamp = data.header.stamp


    for i in range(len(p.tolist()[0].pred[0])):
            detection = Detection2D()
            object_hypothesis = ObjectHypothesisWithPose()

            object_hypothesis.id = str(self.classes[int(p.tolist()[0].pred[0][i][4:6].data.cpu().numpy()[1])])
            object_hypothesis.score = float(p.tolist()[0].pred[0][i][4:6].data.cpu().numpy()[0])

            detection.results.append(object_hypothesis)

            bbox = p.tolist()[0].pred[0][i][0:4].data.cpu().numpy().astype(np.int32)

            detection.bbox.center.x = bbox[2] - ((bbox[2] - bbox[0]) / 2.0)
            detection.bbox.center.y = bbox[3] - ((bbox[3] - bbox[1]) / 2.0)
            detection.bbox.size_x = float(bbox[2] - bbox[0])
            detection.bbox.size_y = float(bbox[3] - bbox[1])

            detection_msg.detections.append(detection)
                
    self.bbox_publisher.publish(detection_msg)

    cv2.waitKey(10)

    def destroy_node(self):
      """
      Destroy the node
      """
      self.destroy_node()
      self.video_writer.release()
      cv2.destroyAllWindows()
    
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageAnalyser()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)

  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()