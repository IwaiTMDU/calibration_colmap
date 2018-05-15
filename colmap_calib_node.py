#!/usr/bin/env python
import rospy
import subprocess
import os.path
import imghdr
import cv2
import threading
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge, CvBridgeError

class colmap:

	def __init__(self):
		self.bridge = CvBridge()
		self.image_count = 0
		self.image_path = "./tmp/images"
		self.pre_sec = -1
		self.pre_nsec = -1
		self.x_distance = 0
		self.image_save_distance = 2.0
		self.calib_image_num = 100
		self.flag_calib = False
		rospy.Subscriber("/kitti/camera_color_left/image_raw", Image, self.Image_graber)
		rospy.Subscriber("/kitti/oxts/gps/vel", TwistStamped, self.vel_callback)

	def __del__(self):
		self.flag_calib = False

	def Image_graber(self,data):#
		self.CreateDirIfnotExist(self.image_path)

		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)

		self.Calib_Check()

	def vel_callback(self,vel):
		if self.pre_sec<0:
			self.x_distance = 0
		else:
			dt = vel.header.stamp.secs-self.pre_sec + (vel.header.stamp.nsecs - self.pre_nsec)*0.000000001
			if dt < 0:
				dt = 0
			self.x_distance += vel.twist.linear.x * dt
		self.pre_sec = vel.header.stamp.secs
		self.pre_nsec = vel.header.stamp.nsecs

	def Calib_Check(self):
		if not self.flag_calib:
			if self.x_distance > self.image_save_distance:
				self.x_distance = 0
				image_name = self.image_path + "/" + str(self.image_count)+".jpg"
				self.image_count += 1
				cv2.imwrite(image_name, self.cv_image)
				cv2.imshow('Image',self.cv_image)
				cv2.waitKey(1)
				print("Save Image : "+image_name)
		
			if self.image_count > self.calib_image_num:
				self.flag_calib = True
				cv2.destroyAllWindows()
				th = threading.Thread(target=self.CameraCalib())
				th.start()
				self.x_distance = 0
		else:
			self.x_distance = 0
				

	def CameraCalib(self):
		if self.Sparse_reconstruction():
			self.CreateDirIfnotExist("./model")
			txt_cmd = "rm -rf ./model/* ; colmap model_converter --input_path ./tmp/sparse/0 --output_path ./model --output_type 'TXT'"
			subprocess.call(txt_cmd, shell = True)
			print("Success : CameraCalib")
		else:
			print("Failure : CameraCalib")
			self.calib_image_num += 10
			print("Modify : Distance = "+str(self.image_save_distance)+" , Image num = "+str(self.calib_image_num))

		subprocess.call("rm -rf "+self.image_path+"/*", shell = True)
		self.image_count = 0
		self.flag_calib = False

	def Sparse_reconstruction(self):
		self.CreateDirIfnotExist("./tmp")
		self.CreateDirIfnotExist("./tmp/sparse")
		if not self.CheckImageDir():
			return False

		feature_extract_cmd = "colmap feature_extractor --ImageReader.single_camera 1 --database_path ./tmp/database.db --ImageReader.camera_model OPENCV --image_path "+self.image_path
		feature_matching_cmd = "colmap exhaustive_matcher --database_path ./tmp/database.db"
		sparse_cmd = "colmap mapper --database_path ./tmp/database.db  --export_path ./tmp/sparse --image_path "+self.image_path
	
		cmd = "rm -rf ./tmp/*.db ; rm -rfd ./tmp/sparse/* ; " + feature_extract_cmd + " ; " + feature_matching_cmd + " ; " + sparse_cmd

		print(cmd)

		subprocess.call(cmd, shell = True)
		if os.path.isdir("./tmp/sparse/0"):
			return True
		else:
			print("Failure : Sparse reconstruction")
			return False

	def Dense_reconstruction(self):
		if not self.Sparse_reconstruction():
			return None
	
		self.CreateDirIfnotExist("./tmp/dense")
		undistorter_cmd = "colmap image_undistorter --input_path ./tmp/sparse/0 --output_path ./tmp/dense --output_type COLMAP --max_image_size 2000 --image_path " + self.image_path
		stereo_cmd = "colmap dense_stereo --workspace_path ./tmp/dense --workspace_format COLMAP --DenseStereo.geom_consistency true"
		fuser_cmd = "colmap dense_fuser --workspace_path ./tmp/dense --workspace_format COLMAP --input_type geometric --output_path ./model/fused.ply"
		mesher_cmd = "colmap dense_mesher --input_path ./model/fused.ply --output_path ./model/meshed.ply"

		cmd = undistorter_cmd + " ; " + stereo_cmd + " ; " + fuser_cmd + " ; " + mesher_cmd
	
		subprocess.call(cmd, shell = True)
		return None

	def CreateDirIfnotExist(self,path):
		if not os.path.isdir(path):
			os.mkdir(path)

	def CheckImageDir(self):
		image_cnt = 0
		if not os.path.isdir(self.image_path):
			print("Cannot find image directory " + self.image_path)
			return False
		else:
			return True
#		else:
#			for file in os.listdir(self.image_path):
#				if imghdr.what(self.image_path+"/"+file) != None:
#					image_cnt +=1

#			if image_cnt >= self.calib_image_num:		
#				return True
#			else:
#				print("Insufficient image files")
#				return False

if __name__ == '__main__':
	rospy.init_node('colmap_calib_node', anonymous=True)
	colmap = colmap()
	#colmap.CameraCalib()
	rospy.spin()
