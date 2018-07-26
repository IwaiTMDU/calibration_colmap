#!/usr/bin/env python
import rospy
import subprocess
import os.path
import imghdr
import cv2
import threading
import math
import numpy as np
from COLMAP import COLMAP

from sensor_msgs.msg import Image
from collections import OrderedDict
import tf
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from cv_bridge import CvBridge, CvBridgeError

class COLMAPCalib:

	def __init__(self):
		self.colmap = COLMAP()
		self.colmap.CheckColmapInstallation()

		self.dense = rospy.get_param('~dense', False)
		self.workspace_path = rospy.get_param('~workspace', ".tmp_colmap")
		self.calib_image_num = rospy.get_param('~image_num', 100)
		self.snap_distance = rospy.get_param('~distance', 1)
		self.snap_rot = rospy.get_param('~rotation', 5)
		self.colmap.yml_path = rospy.get_param('~yml_dir', "./yml")
		self.colmap.filename_yml = rospy.get_param('~yml_name', "camera_param.yml")

		self.bridge = CvBridge()
		self.cv_image = None
		self.image_time = None
		self.image_sub = rospy.Subscriber("/image_raw", Image, self.Image_graber)
		subprocess.call("mkdir -p "+self.workspace_path, shell = True)
		subprocess.call("rm -rf "+self.workspace_path+"/*", shell = True)
		subprocess.call("mkdir -p "+self.workspace_path+"/images_pool", shell = True)
		self.listener = tf.TransformListener()
		self.flag_calibcheck = True
		self.tf_th = threading.Thread(target=self.Tf_thread)
		self.check_th = threading.Thread(target=self.Calib_Check)
		self.tf_th.start()
		self.check_th.start()

	def Image_graber(self,image):
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(image,"bgr8")
			self.image_time = image.header.stamp
		except CvBridgeError as e:
			print(e)
 
	def Tf_thread(self):
		initial = False;
		image_count = 0
		rot = 0
		distance = 0
		snap_point = [np.array([0,0,0]),0]

		while not rospy.is_shutdown() and self.flag_calibcheck:
			try:
				if self.cv_image is not None:
					(self.trans,rot) = self.listener.lookupTransform("/world", "/base_link", self.image_time)

					if not initial:
						euler = tf.transformations.euler_from_quaternion(rot)
						snap_point[0] = np.array(self.trans)
						snap_point[1] = euler[2]*180.0/math.pi
						initial = True
					else:
						yaw = (tf.transformations.euler_from_quaternion(rot))[2]*180.0/math.pi
						rot = abs(yaw-snap_point[1])
						distance = np.linalg.norm(np.array(self.trans)-snap_point[0])
					
						if (distance > self.snap_distance) or (rot > self.snap_rot):
							snap_point = (np.array(self.trans),yaw)
						
							image_name = self.workspace_path+"/images_pool/" + str(image_count)+".jpg"
							image_count += 1
							cv2.imwrite(image_name, self.cv_image)
							show_image = cv2.resize(self.cv_image, (600, 480))
							cv2.putText(show_image, "Save Image : "+image_name, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0.0, 0.0, 255.0))
							cv2.imshow('Image',show_image)
							cv2.waitKey(1)

			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

	def Calib_Check(self):
		image_group = 0
		image_count = 0
		while not rospy.is_shutdown() and self.flag_calibcheck:
			images = os.listdir(self.workspace_path+"/images_pool")
			if len(images) > self.calib_image_num:
				image_pool_path = self.workspace_path+"/images_pool"
				group_path = self.workspace_path+"/group"+str(image_group)
				subprocess.call("mkdir -p "+group_path+"/images", shell = True)

				for i in range(0, self.calib_image_num):
					subprocess.call("mv "+image_pool_path+"/"+str(image_count)+".jpg "+group_path+"/images/", shell = True)
					image_count +=1

				if self.CameraCalib(_group_path=group_path, _group=image_group):
					cv2.destroyAllWindows()
					self.flag_calibcheck = False
					self.tf_th.join()
					subprocess.call("rm -rf "+image_pool_path, shell = True)
				else:
					self.calib_image_num += 10
					print("Modify : Distance = "+str(self.snap_distance)+" , Image num = "+str(self.calib_image_num))
					image_group += 1
			

	def CameraCalib(self, _group_path, _group):
		subprocess.call("rm -rf "+_group_path+"/*.db ; rm -rfd "+_group_path+"/sparse/*", shell = True)

		if self.colmap.Sparse_reconstruction(_group_path = _group_path):
			_sparse_path = _group_path+"/sparse/0"
			_model_path = _group_path + "/model"
			subprocess.call("mkdir -p "+_model_path, shell = True)
			rm_cmd = "rm -rf " + _model_path + "/*"
			subprocess.call(rm_cmd, shell = True)
			txt_cmd = "colmap model_converter --input_path "+_sparse_path+" --output_type 'TXT' --output_path " + _model_path
			subprocess.call(txt_cmd, shell = True)
			self.colmap.WriteIntrinsics(_group_path = _group_path)

			print("Success : CameraCalib")
			if self.dense:
				self.colmap.Dense_reconstruction(_group_path = _group_path)
				print("Carried out : Dense reconstruction")
			return True

		else:
			print("Failure : CameraCalib")
			return False

	

if __name__ == '__main__':
	rospy.init_node('colmap_calib_node', anonymous=True)
	colmap = COLMAPCalib()
	rospy.spin()

