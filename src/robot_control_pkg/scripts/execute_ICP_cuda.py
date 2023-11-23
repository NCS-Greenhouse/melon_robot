#
# Created on Thu Nov 23 2023
#
# Copyright (c) 2023 NCS-Greenhouse-Group
#
# Author:ShengDao Du, Email: duchengdao@gmail.com
# Github Page: https://github.com/Runnlion
# Personal Page: https://shengdao.me
#

import tf.transformations as tr
import rospy, timeit
import cupoch as cph
from robot_control_pkg.srv import execute_icp_cuda, execute_icp_cudaResponse, execute_icp_cudaRequest


def ICP(req:execute_icp_cudaRequest):
    start = timeit.default_timer()
    cloudOrig = cph.io.create_from_pointcloud2_msg(
        req.pc_orig.data, cph.io.PointCloud2MsgInfo.default_dense( req.pc_orig.width,
                                                        req.pc_orig.height,
                                                        req.pc_orig.point_step)
    )
    cloudNew = cph.io.create_from_pointcloud2_msg(
        req.pc_new_vp.data, cph.io.PointCloud2MsgInfo.default_dense(req.pc_new_vp.width,
                                                              req.pc_new_vp.height,
                                                               req.pc_new_vp.point_step)
    )
    result = cph.registration.registration_icp(cloudNew,cloudOrig,req.dist_R2)
    rospy.loginfo("ICP Calculated.")
    response = execute_icp_cudaResponse()
    q = tr.quaternion_from_matrix(result.transformation)
    response.transform.orientation.x = q[0]
    response.transform.orientation.y = q[1]
    response.transform.orientation.z = q[2]
    response.transform.orientation.w = q[3]
    response.transform.position.x = result.transformation[0,3]
    response.transform.position.y = result.transformation[1,3]
    response.transform.position.z = result.transformation[2,3]
    response.RMSE = result.inlier_rmse

    stop = timeit.default_timer()
    rospy.loginfo('Time: '+ str(stop - start))
    return response

def ICP_server():
    rospy.init_node('execute_icp_gpu')
    rospy.Service('execute_icp_cuda', execute_icp_cuda, ICP)
    rospy.loginfo("Cuda ICP Service Started.")
    rospy.spin()

ICP_server()
