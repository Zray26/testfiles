import rospy
# import tf
from hrclib_client_v6 import odyssey_Interface
import geometry_msgs.msg 
from visualization_msgs.msg import Marker
pick_pos_dict={'h': (9.853695701167453e-06, -0.6027460694313049), 'lin': 0.4585753381252289, 'l': (-0.13635384781205717, 0.9114743399306715, 0.850791817648235, 1.2031456067289128, -0.3104654609270394, -1.3138586727237413, -0.8174240070861964), 'r': (0.2505741638044654, -0.9006784175037083, -0.4091691351442108, -1.670779516757796, 0.6212879185982039, 1.0429657882244618, -2.4777225202159308)}
import math

class ARchasing_client(object):
    def __init__(self):
        print "Instantiated"
        self.x_pos = None
        self.y_pos = None
        self.z_pos = None
        self.x_offset = -0.2
        self.y_offset = 0
        self.z_offset = -0.2
        ods=odyssey_Interface()
        self.ods = ods


    def ARcallback(self,data):
        print "In callback"
        x = data.pose.position.x
        y = data.pose.position.y
        z = data.pose.position.z
        # print x
        self.ods._L0_single_task_move_safe("right", [x, y, z],[-math.pi/2 + 0.05, 0, 0], [20 for i in range (6)])

    def ARlistener(self):
        rospy.Subscriber("/visualization_marker",Marker,self.ARcallback, queue_size =1, buff_size= 2**24)
        print "In listener"
        rospy.spin()


#temp_pickbolt_client.marker_CallBack()

if __name__=="__main__":
    rospy.init_node("Pick_Bolt_Test")
    ar = ARchasing_client()
    ar.ARlistener()

    # ods.grip("right",1)
    # ods.grip("right",0)
    # ods.grip("right",1)
    # ods.grip("right",0)


############################ Pick Bolt ##################################################################
    # ods=odyssey_Interface()

    # ods._L0_single_task_move_safe("right",[0.7263-0.1, -0.309544, 0.93824+0.2],
    #                               [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
    #                               [20 for i in range(6)])
    # ods._L0_single_task_move_safe("right",[ods.marker0[0]-0.03,ods.marker0[1]-0.05,ods.marker0[2]+0.2],
    #                               [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
    #                               [20 for i in range(6)])
    # ods.grip("right",1)
    # ods.single_move_relate(arm="right",move=[0,0,-0.09],maxforce=[25 for i in range(6)],time=2)
    # ods.grip("right",0)
    # ods.single_move_relate(arm="right",move=[0,0,0.09],maxforce=[25 for i in range(6)],time=2)
    
#########################################################################################################

############################### Follow Marker ###########################################################
    # while not rospy.is_shutdown():
    #     ods=odyssey_Interface()
    #     ods._L0_single_task_move_safe("right",[0.7263-0.1, -0.309544, 0.93824+0.2],
    #                             [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
    #                             [20 for i in range(6)])
    #     # ods.go_upper_default_jp(pick_pos_dict,hard=False,f=30)
    #     ods._L0_single_task_move_safe("right",[ods.marker0[0],ods.marker0[1],ods.marker0[2]+0.3],
    #                               [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
    #                               [20 for i in range(6)])
    #     rospy.Rate(0.1).sleep()
#########################################################################################################


#     ods.go_upper_default_jp(posdict=ods.gval.default_pose_screw_rightarm_observe,hard=False,f=15)
#     #ods.go_upper_default_jp(posdict=ods.gval.default_pose_ready_insert_and_screw,hard=False,f=15)
#     print(ods.rpose)#[0.7375103572717412, -0.25048753446663785, 1.2731132364376299
# #    ods.get_rpose
#     print('The position of marker0:', ods.marker0)#[0.9548131752342998, -0.30152793713899934, 0.9200864503752137]
# #    ods.arm_cart_move(arm="right",pos=[ods.marker0[0],ods.marker0[1],ods.marker0[2]+0.1],orn=[0.2+math.pi/2, math.pi / 2, -math.pi / 2],maxforce=[15 for i in range(6)])

# #    ods._L0_single_task_move_safe(arm="right",pos=[ods.rpose[0],ods.rpose[1],ods.rpose[2]+0.1],orn=[0, math.pi / 2, -math.pi / 2],maxforce=[15 for i in range(6)])

#     ods._L0_single_task_move_safe("right",[0.7263, -0.309544, 0.93824+0.2],
#                                   [0.1, math.pi / 2, -math.pi / 2],
#                                   [50 for i in range(6)],hard=True)
                                
#     ods.go_upper_default_jp(posdict=ods.gval.default_pose_screw_rightarm_observe,hard=False,f=15)
#    ods._L0_single_task_move_safe("right",[0.74,-0.25,1.17],[0, math.pi / 2, -math.pi / 2],  [20 for i in range(6)])
    # print(ods.rpose)#[0.7375103572717412, -0.25048753446663785, 1.2731132364376299
    #tp=temp_pickbolt_client()
       
#    print(ods.lpose)
#    print(ods.rpose)
#    print(ods.get_jp_dict())
    # ods.get
    # ods.go_upper_default_jp(pick_pos_dict,hard=False,f=30)
    # ods.get_jp_dict()
    # ods.go_upper_default_jp(pick_pos_dict,hard=False,f=30)

    # ods.set_grippers(0)
    # ods.set_grippers(1)
    # ods.grip(rl="left",v=1)
    # ods.grip(rl="left",v=0)
    # ods.grip(rl="left",v=1)



def subscriber():
    rospy.init_node('transform_subscriber', anonymous= True)
    rospy.Subscriber()
