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

		subprocess.call("mkdir -p "+self.workspace_path+" ; rm -rf "+self.workspace_path+"/* ; mkdir -p "+self.workspace_path+"/images_pool", shell = True)

		self.pooled_image = 0
		self.bridge = CvBridge()
		self.snap_point = None
		self.show_image = None
		self.listener = tf.TransformListener()
		self.image_sub = rospy.Subscriber("/image_raw", Image, self.Image_graber, queue_size = 10)
		self.check_th = threading.Thread(target=self.Calib_Check)
		self.check_th.start()

	def Image_graber(self,image):
		writing_image = False

		try:
			cv_image = self.bridge.imgmsg_to_cv2(image,"bgr8")
			try:
				
				self.listener.waitForTransform("/world", "/base_link", image.header.stamp, rospy.Duration(1.0))
				(trans,rot) = self.listener.lookupTransform("/world", "/base_link", image.header.stamp)
				if self.snap_point is None:
					self.image_count = 0
					writing_image = True
					self.snap_point = [np.array(trans), (tf.transformations.euler_from_quaternion(rot))[2]*180.0/math.pi]
				else:
					yaw = (tf.transformations.euler_from_quaternion(rot))[2]*180.0/math.pi
					rot = abs(yaw-self.snap_point[1])
					distance = np.linalg.norm(np.array(trans)-self.snap_point[0])
				
					if (distance > self.snap_distance) or (rot > self.snap_rot):
						self.snap_point = (np.array(trans),yaw)
						writing_image = True
						
				
				if writing_image:
					image_name = self.workspace_path+"/images_pool/" + str(self.image_count)+".jpg"
					self.image_count += 1
					self.pooled_image += 1
					cv2.imwrite(image_name, cv_image)
					self.show_image = cv2.resize(cv_image, (600, 480))
					cv2.putText(self.show_image, "Save Image : "+image_name, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0.0, 0.0, 255.0))

				if self.show_image is not None:
					cv2.imshow('Image',self.show_image)
					cv2.waitKey(1)

			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				pass
				

		except CvBridgeError as e:
			print(e)

	def Calib_Check(self):
		image_group = 0
		image_count = 0
		image_pool_path = self.workspace_path+"/images_pool"

		while not rospy.is_shutdown():
			if self.pooled_image > self.calib_image_num:
				group_path = self.workspace_path+"/group"+str(image_group)
				subprocess.call("mkdir -p "+group_path+"/images", shell = True)

				print("Start Calibration")
				for i in range(0, self.calib_image_num):
					subprocess.call("mv "+image_pool_path+"/"+str(image_count)+".jpg "+group_path+"/images/", shell = True)
					image_count +=1
				self.pooled_image -= image_count

				if self.CameraCalib(_group_path=group_path, _group=image_group):
					cv2.destroyAllWindows()
					cv2.waitKey(1)
					subprocess.call("rm -rf "+image_pool_path, shell = True)
					self.image_sub.unregister()

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

