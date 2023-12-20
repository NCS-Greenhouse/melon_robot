
import GPUtil
import rospy


class python_utils:
    def __init__(self,argc) -> None:
        rospy.init_node("python_utils",anonymous=True)
        if(argc == 'check-gpu'):
            self.__check_gpu_status__()
    
    def __check_gpu_status__(self)->None:
        # print()
        gpu_availability = len(GPUtil.getAvailable())
        rospy.set_param("gpu_number",gpu_availability)
        rospy.loginfo("Set the parameter 'gpu_number' to %s.",gpu_availability)

python_utils("check-gpu")

