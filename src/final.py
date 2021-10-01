#!/usr/bin/env python
import rospy, rospkg
import numpy as np
import cv2, time
import numpy as np
import rospy, math, os, rospkg

from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image ,LaserScan 
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge
from std_msgs.msg import String , Int64


def PID(input_data, kp, ki, kd):
    	global start_time, end_time
    	end_time = time.time()
    	dt = end_time - start_time
    	start_time = end_time

    	error = 320 - input_data
    	p_error = kp * error
    	i_error = ki * error * dt
    	d_error = kd * error / dt
    	output = p_error + i_error + d_error
    	if output > 50:
		output = 50
    	elif output < -50:
		output = -50
    	return -output



#time funct
start_time = 0.0
now_time = 0.0
late_time = 0.0

#flags
DEBUG = 1
time_done = 0
print_once_stage = -1 #each var num mean stage
print_once_artag = -1 #each var num mean ar tag num

##timeflags
forked_road_drive_cnt = 5000
#opencv
bridge = CvBridge()
cv_image = np.empty(shape=[0])

#hough transform
threshold_60 = 60
threshold_100 = 100
width_640 = 640
scan_width_200, scan_height_20 = 200, 20
lmid_200, rmid_440 = scan_width_200, width_640 - scan_width_200
area_width_20, area_height_10 = 20, 10
vertical_430 = 430
row_begin_5 = (scan_height_20 - area_height_10) // 2
row_end_15 = row_begin_5 + area_height_10
pixel_threshold_160 = 0.8 * area_width_20 * area_height_10

#default xycar status
stage = 0
Speed = 5
Angle = 0


# lidar
done = 0
distance = [] 
traffic_left = 'A'
traffic_right = 'A'
traffic_count = 0

# ar viewer
arData = {"DX":0.0,"DY":0.0,"DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0}
roll, pitch, yaw = 0, 0, 0
ar_viewer_id = -1 


def img_callback(img_data):
    	global cv_image
   	global bridge
    	cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")

def lidar_callback(data):
	global distance, motor_msg
	distance = data.ranges

def traffic_left_callback(data):
    	global traffic_left
	traffic_left = data.data

def traffic_right_callback(data):
	global traffic_right
	traffic_right = data.data

def count_callback(data):
    	global traffic_count
	traffic_count = data.data

def ar_viewer_callback(data):
     	global arData  , ar_viewer_id, roll, pitch, yaw
 	for i in data.markers:
 		arData["DX"] = i.pose.pose.position.x
 		arData["DY"] = i.pose.pose.position.y
 		arData["DZ"] = i.pose.pose.position.z

 		arData["AX"] = i.pose.pose.orientation.x
 		arData["AY"] = i.pose.pose.orientation.y
 		arData["AZ"] = i.pose.pose.orientation.z
 		arData["AW"] = i.pose.pose.orientation.w
         	(roll, pitch, yaw) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"])) 
         # ar??? ID
         	ar_viewer_id = i.id


def drive(Angle, Speed):
    	global pub

    	msg = xycar_motor()
    	msg.angle = Angle
    	msg.speed = Speed

    	pub.publish(msg)

def lidar_stop():
        global distance , done
        ok = 0
        for degree in range(90,160): # 60,120 -> 120,180
                if distance[degree] <=0.45:
			ok+=1
		if ok >2:
			return 1
			break
        if ok<=2 :
		if done == 1 :
			stage = 1
			return 0


def start():
    	global pub
        global cv_image
    	global stage, Speed, Anlge 
    	global ar_viewer_id, pitch
	global done, DEBUG, time_done, print_once_stage, print_once_artag 
	global forked_road_drive_cnt 
	global traffic_left, traffic_right, traffic_count
    	rospy.init_node('auto_driver')
    	rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)
    	pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    	

    #lidar
        rospy.Subscriber('/scan',LaserScan, lidar_callback, queue_size=1)

    # traffic 
        rospy.Subscriber('/Left_color', String, traffic_left_callback, queue_size=1)
	rospy.Subscriber('/Right_color', String, traffic_right_callback, queue_size = 1 )
        rospy.Subscriber('/time_count', Int64, count_callback, queue_size = 1 )

    #ar viewer
    	rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_viewer_callback, queue_size = 1)
    
	time.sleep(3)#delay for lidar setup
    
    	while not rospy.is_shutdown():
        	while not cv_image.size == (640*480*3):
            		continue
        
        	frame = cv_image
        	if cv2.waitKey(1) & 0xFF == 27:
            		break
#######################################################################################################
#######################################################################################################
        	roi = frame[vertical_430:vertical_430 + scan_height_20, :]
        	frame = cv2.rectangle(frame, (0, vertical_430), (width_640 - 1, vertical_430 + scan_height_20), (255, 0, 0), 3)
        	hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        	lbound = np.array([0, 0, threshold_100], dtype=np.uint8)
        	ubound = np.array([131, 255, 255], dtype=np.uint8)
        	bin = cv2.inRange(hsv, lbound, ubound)
        	view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)

        	left, right = -1, -1

        	for l in range(300, 0, -1):
            		area = bin[row_begin_5:row_end_15, l - area_width_20:l]
            		if cv2.countNonZero(area) > pixel_threshold_160:
                		left = l
                		break
        	for r in range(320 , 620):
            		area = bin[row_begin_5:row_end_15, r:r + area_width_20]
            		if cv2.countNonZero(area) > pixel_threshold_160:
               			right = r
                		break


		if left != -1:
			lsquare = cv2.rectangle(view, (left - area_width_20, row_begin_5), (left, row_end_15), (0, 255, 0), 3)
        	else:
            		#print("Lost left line")
	    		left = 0

        	if right != -1:
            		rsquare = cv2.rectangle(view, (right, row_begin_5), (right + area_width_20, row_end_15), (0, 255, 0), 3)
        	else:
            		#print("Lost right line")
	    		right = 640
#######################################################################################################

        	center = (right + left)/2
	    	Angle = PID(center, 0.05, 0.0007, 0.005)
 
        
        	#cv2.imshow("origin", frame)
        	#cv2.imshow("view", view)

		#DEBUG
		if DEBUG == 1 :
			stage = 0
			DEBUG = 0
		

		

        	# stage 0 :lidar
        	if (stage == 0 and lidar_stop() == 1):
			if print_once_stage == -1:
				print("stage 0 lidar stop\n")
				print_once_stage = 0
                        Speed = 0
            	        drive(0, Speed)
                        done = 1
            	        continue
        
        	if (stage ==0 and Speed ==0 and lidar_stop() == 0):
            	        stage = 1 
                        Speed = 4
			if print_once_stage == 0 :
	            		print("stage 0 clear\n")
				print_once_stage = 1
            	        continue


        	# stage 1 : traffic_corss
        	cross = 0.4# white pixcels
        	if ((stage == 1) and ((float(cv2.countNonZero(bin)) / (640*20)) > cross)) :
			print(float(cv2.countNonZero(bin)) / (640*20))
			if print_once_stage == 1:                      
				print("Stop line, stage 1")
				print_once_stage = 2
			if traffic_right == 'R' or traffic_right == 'Y':
				Speed = 0
				Angle = 0
			elif traffic_right == 'G':				
				stage = 2
				Speed = 5
				print 'stage 1 clear'
                
        	# stage 2 : slow zone
		if ((stage == 2) and (ar_viewer_id == 0) and (float(cv2.countNonZero(bin)) / (640*20)) > 0.4) :
			if print_once_stage == 2 :
				print('Real Slow zone, stage 2')
				print_once_stage = 3
			if time_done == 0 :
				#for n sec drive cnt
				n = 10
				start_t = time.time()
				time_done = 1
				#DEBUG
				print (time.time())
		if (stage == 2 and time_done == 1):
			if time.time() - start_t < n :
				Speed = 3
				
			else:
				stage = 3
				Speed = 5
				print 'stage2 clear'

        	# stage 3 : Forked road recognition
		if (stage == 3 and ar_viewer_id == 2):
			if (float(cv2.countNonZero(bin)) / (640*20) < 0.27):
				Speed = 0
				stage = 4
				print('stage 3 clear')
			

		# stage 4 : turn according to two traffic signals
		if (stage == 4):
			if traffic_left == 'G' or (traffic_left == 'Y' and traffic_count > 2):
				#DEBUG
				print 'go to Left'
				Angle = -30
				Speed = 4
				while forked_road_drive_cnt:
					drive(Angle,Speed)
					forked_road_drive_cnt = forked_road_drive_cnt -1
					

			elif traffic_right == 'G' or (traffic_right == 'Y' and traffic_count > 2):
				#DEBUG
				print 'go to right'				
				Angle = 30
				Speed = 4
				while forked_road_drive_cnt:
					drive(Angle,Speed)
					forked_road_drive_cnt = forked_road_drive_cnt -1

			elif traffic_left == 'Y' and traffic_count < 2 : #goto r
				#DEBUG
				print 'go to Right'
				Angle = 30
				Speed = 4
				while forked_road_drive_cnt:
					drive(Angle,Speed)
					forked_road_drive_cnt = forked_road_drive_cnt -1

			elif traffic_right == 'Y' and traffic_count < 2 : #goto l
				#DEBUG
				print 'go to Left'
				Angle = -30
				Speed = 4
				while forked_road_drive_cnt:
					drive(Angle,Speed)
					forked_road_drive_cnt = forked_road_drive_cnt -1
			#DEBUG
			print 'stage 4 claer'
			Speed = 5
			stage = 5 #next stage						


	        # stage 5 : parking
        	if (stage == 5 and ar_viewer_id == 6 and arData["DZ"] < 2.4):
			Speed = 3
			if (arData["DZ"] < 0.7):
				drive(0,0)
				stage = 6
				print('stage 5 clear')

		# stage 6 : backward right
		if (stage == 6):
			start_t = time.time()
			while time.time() - start_t < 1:
				drive(-50,3)
			start_t = time.time()
			while time.time() - start_t < 1.5:
				drive(50,-3)
			start_t = time.time()
			while time.time() - start_t < 2.5:
				drive(-50,-3)
			print('stage 6 clear')
			stage = 7

		# go forward and stop
		if (stage == 7):
			pitch = pitch * 180/math.pi
			
			if arData["DZ"] > 1.1:
				print pitch
				if pitch < -5:
					Angle = -20
					Speed = 2
				elif pitch > 5:
					Angle = 20
					Speed = 2
				else:
					Angle = 0
					Speed = 3
			else:
				drive(0,0)
				print('driving finished')
				break


			
            
 
		drive(Angle, Speed)
		#cv2.destroyAllWindows()

if __name__ == '__main__':
        
    	start()
    	start_time = time.time()


