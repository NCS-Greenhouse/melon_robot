# -*- coding: utf-8 -*-
#
# Created on Fri Aug 25 21:28:21 2023
#
# Copyright (c) 2023 NCS-Greenhouse-Group
#
# Author:ShengDao Du, Email: duchengdao@gmail.com
# For more information: https://github.com/Runnlion
# 
# Credit: jun94
#

import numpy as np
import pymongo, cv2
import rospy, rospkg, configparser, base64, os
from robot_control_pkg.srv import upload_img, upload_imgRequest, upload_imgResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageUploader():
    def __init__(self) -> None:
        rospy.init_node("image_uploader")
        rospy.on_shutdown(self.close_connection)

        #read configuration file
        # Source: https://docs.python.org/3/library/configparser.html
        self.conf = configparser.ConfigParser() 
        self.rospack = rospkg.RosPack()
        #Add Try-Except Clause
        self.package_root = self.rospack.get_path("robot_control_pkg") #Get current package absolute location
        self.conf.read(self.package_root + "/package.conf") # Try to load this configuration file
        if((self.conf.get('forge.mango', 'mango-credentials',fallback='err')) == 'err' \
            or self.conf.get('forge.mango', 'mango-db-cli',fallback='err')  == 'err'):
            # OR not 'forge.mango' in self.conf
            rospy.logerr("Configuration Loading Fail.")
            rospy.signal_shutdown("Configuration Loading Fail.")
            return
        else:
            pass
        rospy.loginfo("Configuration Loaded.")
        #Check Configuration File
        self.upload_haldlar = rospy.Service('upload_img', upload_img, self.upload_image)
        self.cli = pymongo.MongoClient(self.conf['forge.mango']['mango-credentials'])
        self.db=self.cli[self.conf['forge.mango']['mango-db-cli']]
        self.collection=self.db["ctlp_img_rt"]
        self.collection.delete_many({})
        self.bridge = CvBridge()
        pass
    
    def close_connection(self):
        try:
            #if mango is connected
            self.cli.close()
            rospy.loginfo("Disconnect from Mango Server.")
        except:
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

