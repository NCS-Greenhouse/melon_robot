
import GPUtil
import rospy

ARUCO_CAMERA_TYPE = 'color'

class parameter_initializer:
    def __init__(self) -> None:
        rospy.init_node("python_utils",anonymous=True)
        self.__check_gpu_status__()
        self.initialize_parameters()

    def __check_gpu_status__(self)->None:
        # print()
        gpu_availability = len(GPUtil.getAvailable())
        rospy.set_param("gpu_number",gpu_availability)
        rospy.loginfo("Set the parameter 'gpu_number' to %s.",gpu_availability)
    
    def initialize_parameters(self)->None:
        rospy.set_param("/aruco_detector/camera_type",ARUCO_CAMERA_TYPE)


parameter_initializer()

