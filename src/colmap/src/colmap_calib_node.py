#!/usr/bin/env python
import rospy
import subprocess
import os.path
import imghdr
import cv2
import threading
import math
import yaml
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from tf.msg import tfMessage
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from cv_bridge import CvBridge, CvBridgeError

class colmap:

	def __init__(self):
		self.image_path = "./tmp/images"
		self.model_path = "./tmp/model"
		self.yml_path = "./yml"
		self.filename_yml = "camera_param.yml"
		self.calib_image_num = 100
		self.snap_distance = 3
		self.snap_rot = 3

		self.bridge = CvBridge()
		self.image_count = 0
		self.flag_calib = False
		self.pos = Vector3(0.0, 0.0, 0.0)
		self.rot = Quaternion(0.0, 0.0, 1.0, 0.0)
		self.snap_point = (self.pos,0.0)
		rospy.Subscriber("/kitti/camera_color_left/image_raw", Image, self.Image_graber)
		rospy.Subscriber("/tf", tfMessage, self.Tf_callback)

	def Image_graber(self,image):#
		self.CreateDirIfnotExist(self.image_path)

		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(image,"bgr8")
		except CvBridgeError as e:
			print(e)

		self.Calib_Check()
 
	def Tf_callback(self,tf):
		self.rot = tf.transforms[0].transform.rotation
		self.pos = tf.transforms[0].transform.translation

	def Calib_Check(self):
		if not self.flag_calib:
			displace = math.sqrt((self.pos.x - self.snap_point[0].x) * (self.pos.x - self.snap_point[0].x) + (self.pos.y - self.snap_point[0].y) * (self.pos.y - self.snap_point[0].y) + (self.pos.z - self.snap_point[0].z) * (self.pos.z - self.snap_point[0].z))

			x = self.rot.x*self.rot.x - self.rot.y*self.rot.y - self.rot.z*self.rot.z + self.rot.w*self.rot.w
			y = 2*(self.rot.x*self.rot.y+self.rot.z*self.rot.w)
			theta = math.atan2(y,x)*180.0/math.pi
			abstheta = abs(theta-self.snap_point[1])

			if (displace > self.snap_distance) or (abstheta > self.snap_rot):
				self.snap_point = (self.pos,theta)
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
			self.CreateDirIfnotExist(self.model_path)
			rm_cmd = "rm -rf " + self.model_path + "/*"
			subprocess.call(rm_cmd, shell = True)
			txt_cmd = "colmap model_converter --input_path ./tmp/sparse/0 --output_type 'TXT' --output_path " + self.model_path
			subprocess.call(txt_cmd, shell = True)
			self.WriteIntrinsics()

			print("Success : CameraCalib")
		else:
			print("Failure : CameraCalib")
			self.calib_image_num += 10
			print("Modify : Distance = "+str(self.snap_distance)+" , Image num = "+str(self.calib_image_num))

		subprocess.call("rm -rf "+self.image_path+"/*", shell = True)
		self.image_count = 0
		self.flag_calib = False

	def WriteIntrinsics(self):
		with open(self.model_path + "/cameras.txt") as cf:
			cfstr = cf.readlines()
			intr = cfstr[3].split(" ")[:-1]
			self.CreateDirIfnotExist(self.yml_path) 
			yaml_content = ("camera_model: " + intr[1] + "\n"
							+ "image_width: " + intr[2] + "\n"
							+ "image_height: " + intr[3] + "\n"
							+ "camera_matrix:\n"
							+ "[ " + intr[4] + " 0.0 " + intr[6] + " 0.0 " + intr[5] + " " + intr[7] 						+ " 0.0 0.0 1.0 ]\n"
							+ "distortion_coefficients:\n"
							+ "[ " + " ".join(intr[8:]) + " ]\n")
			print(yaml_content)
			#yaml_data = yaml.dump(yaml_content, default_flow_style=False)
			with open(self.yml_path + "/" + self.filename_yml, "wt") as fp:
				if fp is None:
					print(self.yaml_path + "/" + self.filename_yml+ "No such file or directry")
				else:
					fp.write(yaml_content)
					print("Output : ./yml/camera_param.yml")

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
		if not os.path.isdir(self.image_path):
			print("Cannot find image directory " + self.image_path)
			return False
		else:
			return True

if __name__ == '__main__':
	rospy.init_node('colmap_calib_node', anonymous=True)
	colmap = colmap()
	rospy.spin()
