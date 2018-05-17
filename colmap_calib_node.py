#!/usr/bin/env python
import rospy
import subprocess
import os.path
import imghdr
import cv2
import threading
import math
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from tf.msg import tfMessage
from cv_bridge import CvBridge, CvBridgeError

class colmap:

	def __init__(self):
		self.bridge = CvBridge()
		self.image_count = 0
		self.image_path = "./tmp/images"
		self.calib_image_num = 100
		self.flag_calib = False
		self.snap_distance = 3 * 3
		self.snap_rot = 2
		self.snap_point = (0.0,0.0,0.0,0)
		rospy.Subscriber("/kitti/camera_color_left/image_raw", Image, self.Image_graber)
		rospy.Subscriber("/tf", tfMessage, self.Tf_callback)

	def __del__(self):
		self.flag_calib = False

	def Image_graber(self,data):#
		self.CreateDirIfnotExist(self.image_path)

		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)

		self.Calib_Check()
 
	def Tf_callback(self,tf):
		rot = tf.transforms[0].transform.rotation
		x = rot.x*rot.x - rot.y*rot.y - rot.z*rot.z + rot.w*rot.w
		y = 2*(rot.x*rot.y+rot.z*rot.w)
		self.theta = math.atan2(y,x)*180.0/math.pi
		self.pos = tf.transforms[0].transform.translation

		if self.flag_calib:
			self.snap_point = (self.pos.x,self.pos.y,self.pos.z, self.theta)

	def Calib_Check(self):
		if not self.flag_calib:
			displace = (self.pos.x - self.snap_point[0]) * (self.pos.x - self.snap_point[0]) + (self.pos.y - self.snap_point[1]) * (self.pos.y - self.snap_point[1]) + (self.pos.z - self.snap_point[2]) * (self.pos.z - self.snap_point[2])

			rot = abs(self.theta-self.snap_point[3])
			

			if (displace > self.snap_distance) or (rot > self.snap_rot):
				self.snap_point = (self.pos.x,self.pos.y,self.pos.z, self.theta)
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

	def CameraCalib(self):
		if self.Sparse_reconstruction():
			rm_cmd = "rm -rf ./model/*"
			subprocess.call(rm_cmd, shell = True)
			self.CreateDirIfnotExist("./model")
			txt_cmd = "colmap model_converter --input_path ./tmp/sparse/0 --output_path ./model --output_type 'TXT'"
			subprocess.call(txt_cmd, shell = True)
			print("Success : CameraCalib")
		else:
			print("Failure : CameraCalib")
			self.calib_image_num += 10
			print("Modify : Distance = "+str(self.snap_distance)+" , Image num = "+str(self.calib_image_num))

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

if __name__ == '__main__':
	rospy.init_node('colmap_calib_node', anonymous=True)
	colmap = colmap()
	rospy.spin()
