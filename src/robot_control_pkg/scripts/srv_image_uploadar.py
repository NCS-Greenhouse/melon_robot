# -*- coding: utf-8 -*-
"""
Created on Fri Aug 25 21:28:21 2023

@author: jun94
@Modified By: ShengDao Du

"""

import pymongo
import cv2
import numpy as np
import os
import base64
import rospy
from greenhouse_arm_pkg.srv import upload_img, upload_imgRequest, upload_imgResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageUploader():
    def __init__(self) -> None:
        rospy.init_node("image_uploader")
        self.upload_haldlar = rospy.Service('upload_img', upload_img, self.upload_image)
        self.cli = pymongo.MongoClient("mongodb://65feb189-94b4-4b4e-9c48-9f41752e8a2b:2aDcNa6FiDmRf293IoBnjBHy@apps-mongodb-single-9f379c12-32ce-477b-8871-d0436bdd634d-pub.education.wise-paas.com:27017/a745f4c1-dac0-4af1-a27d-84ae4da47ec5")
        self.db=self.cli["a745f4c1-dac0-4af1-a27d-84ae4da47ec5"]
        self.collection=self.db["ctlp_img_rt"]
        self.collection.delete_many({})
        self.bridge = CvBridge()
        pass

    def read_img_b64(self, cvimg):
        b64_str = cv2.imencode(".jpg",cvimg)[1].tobytes()
        b64_str = base64.b64encode(b64_str)
        return b64_str
    
    def upload_image(self, request:upload_imgRequest) -> upload_imgResponse:
        #save sensor_msgs/Image to jpg to somewhere.
        ros_img = Image()
        ros_img = request.image

        cv_image = self.bridge.imgmsg_to_cv2(request.image)

        # cv2.imwrite(request.image_path + "/" + str(ros_img.header.stamp.to_nsec()) + ".jpg")

        imgb64 = self.read_img_b64(cv_image)
        # filename = os.path.splitext(os.path.basename(request.image_path.data))[0]
        data = {"no.":str(ros_img.header.stamp.to_nsec()),"cont":imgb64}
        self.collection.insert_one(data)
        res = upload_imgResponse()
        res.result.data = True
        rospy.loginfo("Uploaded")
        response = upload_imgResponse()
        response.result.data = True
        return response


upload_Img = ImageUploader()

rospy.spin()

