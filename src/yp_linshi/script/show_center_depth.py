#coding:utf-8
import rospy
import cv2
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np
import pyrealsense2 as rs2


from geometry_msgs.msg import Pose 

if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

class ImageListener:
    def __init__(self, depth_image_topic, depth_info_topic, camera_color_image):
        self.bridge = CvBridge()
	self.sub_image = rospy.Subscriber(camera_color_image, msg_Image, self.imageColorCallback)             #声明三个订阅者（深度图、相机内参、彩色图）和一个发布者（目标pose）
        self.sub_info = rospy.Subscriber(depth_info_topic, CameraInfo, self.imageDepthInfoCallback)
        self.sub = rospy.Subscriber(depth_image_topic, msg_Image, self.imageDepthCallback)
        self.pose_pub = rospy.Publisher("/target_pose_camera", Pose, queue_size=1)        
	self.intrinsics = None  #内参为类成员，在InfoCallback中读取赋值，然后供depthCallback使用
        self.pix = None         #target的像素坐标，由ColoCallback中提取赋值，供depthCallback使用

    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
	    pix=self.pix
            line = '\nDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])

            if self.intrinsics:
                depth = cv_image[pix[1], pix[0]]
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
            line += '\n'
            sys.stdout.write(line)
            sys.stdout.flush()

	    target=Pose ()
            target.position.x=result[0]/1000
	    target.position.y=result[1]/1000
	    target.position.z=result[2]/1000
	    #print('***************')
	    #print(target)
	    rate = rospy.Rate(5) # 5hz
	    self.pose_pub.publish(target)
	    rate.sleep()

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return


    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return

    def imageColorCallback(self, data):

        cv_image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
	HSV_image=cv2.cvtColor(cv_image_color,cv2.COLOR_BGR2HSV)
	#lower= np.array([170,100,100]) 	
	#upper= np.array([180,255,255])
        lower= np.array([95,125,65]) 	#95 125 100
	upper= np.array([105,255,255])
	mask=cv2.inRange(HSV_image,lower,upper)   #分离目标
	img_Result=cv2.bitwise_and(cv_image_color,cv_image_color,mask=mask)#颜色恢复
	kernel= cv2.getStructuringElement(cv2.MORPH_RECT,(4,5)) #定义矩形结构元素
	img_erode=cv2.erode(mask,kernel,iterations=1)
	img_dilated=cv2.dilate(mask,kernel,iterations=3)
	
	binary,contours,hierarchy = cv2.findContours(img_dilated,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	for i in range(len(contours)):                     #len(contours)show numble of contours
    		area = cv2.contourArea(contours[i])        #calculate the area of each contours 
   		if area > 1000:
        
        		rect = cv2.minAreaRect(contours[i]) #提取矩形坐标

			#print("形心坐标为:")
			#print(rect[0])
			#print("矩形面积为:")
			#print(area)
        		box = cv2.boxPoints(rect)	#get the xyz of thr four vertices of the rect
       		 	box = np.int0(box)                  #change type of box to int
        		#print(area)        
       			cv2.drawContours(cv_image_color, [box], 0, (0, 0, 255), 2) 
			#M=cv2.moents(contours[i])
			center_x=int(rect[0][0])
			center_y=int(rect[0][1])
			text="("+str(center_x)+","+str(center_y)+")"
			cv2.putText(cv_image_color,text,(center_x,center_y),cv2.FONT_HERSHEY_PLAIN,2.0,(0,0,255),2)
       			#cv2.drawContours(cv_image_color, [box], 0, (0, 0, 255), 2) 
			cv2.circle(cv_image_color,(center_x,center_y),7,128,-1)
			pix = (center_x,center_y)
			self.pix=pix


	#cv2.imshow("chuli",img_dilated)
        #cv2.imshow("HSV_Image window", img_Result)
        #cv2.waitKey(3)
        # 显示Opencv格式的图像
        cv2.imshow("targe_Image window", cv_image_color)
        cv2.waitKey(3)




def main():
    depth_image_topic = '/camera/aligned_depth_to_color/image_raw'
    depth_info_topic = '/camera/aligned_depth_to_color/camera_info'
    camera_color_image = '/camera/color/image_raw'
    

    print ('')
    print ('show_center_depth.py')
    print ('--------------------')
    print ('App to demontrate the usage of the /camera/depth topics.')
    print ('')
    print ('Application subscribes to %s and %s topics.' % (depth_image_topic, depth_info_topic))
    print ('Application then calculates and print the range to the closest object.')
    print ('If intrinsics data is available, it also prints the 3D location of the object')
    print ('If a confedence map is also available in the topic %s, it also prints the confidence grade.' % depth_image_topic.replace('depth', 'confidence'))
    print ('')
    
    listener = ImageListener(depth_image_topic, depth_info_topic, camera_color_image)
    rospy.spin()
if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    main()
