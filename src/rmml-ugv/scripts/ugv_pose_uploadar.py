#!/usr/bin/env python3
#
# Transplanted on Sun Dec 17 2023
#
# Copyright (c) 2023 NCS-Greenhouse-Group
#
# Modified by:ShengDao Du, Email: duchengdao@gmail.com
# Github Page: https://github.com/Runnlion
# Personal Page: https://shengdao.me
#
# Author:kiven, Email: ${kiven-repo}
# Github Page: https://github.com/kevin88121698
#

import rospy, rospkg, configparser
import json
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
import datetime
import time
import random
import math

from wisepaasdatahubedgesdk.EdgeAgent import EdgeAgent
import wisepaasdatahubedgesdk.Common.Constants as constant
from wisepaasdatahubedgesdk.Model.Edge import EdgeAgentOptions, DCCSOptions, EdgeData, EdgeTag


# 230323 update. Save the last position to a json file.
x = 0
y = 0
theta = 0
q_w = 0
q_z = 1
x_origin = 0
y_origin = 0
flag = 5
last_position = Pose()  # initialize last position variable

def on_connected(edgeAgent, isConnected):
  print("connected !")
  config = __generateConfig()
  _edgeAgent.uploadConfig(action = constant.ActionType['Create'], edgeConfig = config)
  
  
def on_disconnected(edgeAgent, isDisconnected):
  print("disconnected !")

def edgeAgent_on_message(agent, messageReceivedEventArgs):
  print("edgeAgent_on_message !")

def __sendData():
  data = __generateData()     # change speed
  _edgeAgent.sendData(data)



def __generateBatchData():    # send data in batch for high frequency data
  array = []
  for n in range(1, 10):
    edgeData = EdgeData()
    for i in range(1, 1 + 1):
      for j in range(1, 1 + 1):
        deviceId = 'Device' + str(i)
        tagName = 'ATag' + str(j)
        value = random.uniform(0, 100)
        tag = EdgeTag(deviceId, tagName, value)
        edgeData.tagList.append(tag)
    array.append(edgeData)

def __generateData(): 
  # global controlCommand
  edgeData = EdgeData()
  for i in range(1, 1 + 1):
    for j in range(1, 1 + 1):
      deviceId = 'Device' + str(i)
      tagName = 'ATag' + str(j)
      value = x							# position X
      tag = EdgeTag(deviceId, tagName, value)
      edgeData.tagList.append(tag)
    for j in range(2, 2 + 1):
      deviceId = 'Device' + str(i)
      tagName = 'ATag' + str(j)
      value = y							# position Y
      tag = EdgeTag(deviceId, tagName, value)
      edgeData.tagList.append(tag)
    for j in range(3, 3 + 1):
      deviceId = 'Device' + str(i)
      tagName = 'ATag' + str(j)
      value =  theta 						# Oriantation	
      tag = EdgeTag(deviceId, tagName, value)
      edgeData.tagList.append(tag)
######################################## position, quaternion save #########################################
    for j in range(4, 4 + 1):
      deviceId = 'Device' + str(i)
      tagName = 'ATag' + str(j) 
      value =  q_w						
      tag = EdgeTag(deviceId, tagName, value)
      edgeData.tagList.append(tag)
    for j in range(5, 5 + 1):
      deviceId = 'Device' + str(i)
      tagName = 'ATag' + str(j)
      value =  q_z					
      tag = EdgeTag(deviceId, tagName, value)
      edgeData.tagList.append(tag)
    for j in range(8, 8 + 1):
      deviceId = 'Device' + str(i)
      tagName = 'ATag' + str(j) 
      value =  x_origin						
      tag = EdgeTag(deviceId, tagName, value)
      edgeData.tagList.append(tag)
    for j in range(9, 9 + 1):
      deviceId = 'Device' + str(i)
      tagName = 'ATag' + str(j) 
      value =  y_origin						
      tag = EdgeTag(deviceId, tagName, value)
      edgeData.tagList.append(tag)
########################################################################################################
  edgeData.timestamp = datetime.datetime.now()
  return edgeData


def amcl_pose_callback(msg:PoseWithCovarianceStamped):
  global x, y, theta, r, q_z, q_w, x_origin, y_origin, last_position
  r = 0.60
  theta = -(math.atan2(2*(msg.pose.pose.orientation.w*msg.pose.pose.orientation.z),1-2*math.pow(msg.pose.pose.orientation.z,2)) + 3.14159/2)
  x0 = msg.pose.pose.position.x +0.0*math.cos(theta)-0.0*math.sin(theta) #- 22.919# in order to fit the 3D simulation (map change origin)
  y0 = msg.pose.pose.position.y -0.0*math.cos(theta)-0.0*math.sin(theta) #- 5.627
  x = x0*67.61 + y0*1.2*0.85 + 619.29     # cm-> m, simulation ,slam map
  y = x0*1.2 - y0*67.61*0.85 - 289.9 +28
  x_origin = msg.pose.pose.position.x
  y_origin = msg.pose.pose.position.y
  q_z = msg.pose.pose.orientation.z
  q_w = msg.pose.pose.orientation.w

  last_position = msg.pose.pose  # update last position variable with the latest position data


rospack = rospkg.RosPack()
conf = configparser.ConfigParser() 
forge_package_root = rospack.get_path("robot_control_pkg") #Get current package absolute location
conf.read(forge_package_root + "/package.conf") # Try to load this configuration file

package_root = rospack.get_path("rmml-ugv") #Get current package absolute location

rospy.init_node('upload')
rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback, queue_size = 1, buff_size = 52428800)                    #current position
# _edgeAgent = None
edgeAgentOptions = EdgeAgentOptions(nodeId = 'e3093067-073e-45d8-8c79-c37c6f1e2bb2')
edgeAgentOptions.connectType = constant.ConnectType['DCCS']
dccsOptions = DCCSOptions(apiUrl = 'https://api-dccs-ensaas.education.wise-paas.com/', credentialKey = conf['forge.datahub']['credential_ugv_second'])
edgeAgentOptions.DCCS = dccsOptions
_edgeAgent = EdgeAgent(edgeAgentOptions)
_edgeAgent.on_connected = on_connected
_edgeAgent.on_disconnected = on_disconnected
_edgeAgent.on_message = edgeAgent_on_message

_edgeAgent.connect()
time.sleep(5)  # Waiting for connection to be established


while not rospy.is_shutdown():
  __sendData()
  # time.sleep(1)
  rospy.Rate(1).sleep()
  rospy.loginfo("Last position: %s.",str(last_position))

if last_position is not None:
  # save last position data to a JSON file
  print("save last position data to a JSON file")
  json_file_path = package_root + "/config/last_position.json"
  with open(json_file_path, 'w') as f:
      position_dict = {
          'x': last_position.position.x,
          'y': last_position.position.y,
          'z': last_position.position.z,
          'qx': last_position.orientation.x,
          'qy': last_position.orientation.y,
          'qz': last_position.orientation.z,
          'qw': last_position.orientation.w
      }
  # json.dump(position_dict, f)

rospy.spin()
_edgeAgent.disconnect()


