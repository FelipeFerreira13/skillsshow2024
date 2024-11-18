# Import the necessary libraries
import rospy # Python library for ROS
import cv2 # OpenCV library

from camera.srv import Status

def take_image(req):
 
    # Go through the loop 2 times per second
    rate = rospy.Rate(2) # 2Hz
        
    # Create a VideoCapture object
    cap = cv2.VideoCapture(0)
   
    # While ROS is still running.
    while not rospy.is_shutdown():
        
        # Capture frame-by-frame
        ret, frame = cap.read()
            
        if ret == True:
            # Print debugging information to the terminal
            rospy.loginfo('publishing video frame')
                
            cv2.imshow('image', frame)

            cv2.waitKey(1)
                
        # Sleep just enough to maintain the desired rate
        rate.sleep()

    return True

def simple_cam_node():
  rospy.init_node('simple_cam_node')
  srv = rospy.Service('/camera/simple_cam/start', Status, take_image)

  rospy.spin()
  
if __name__ == '__main__':
  simple_cam_node()