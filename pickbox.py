import rospy
# import tf
from hrclib_client_v6 import odyssey_Interface
import geometry_msgs.msg 
pick_pos_dict={'h': (9.853695701167453e-06, -0.6027460694313049), 'lin': 0.4585753381252289, 'l': (-0.13635384781205717, 0.9114743399306715, 0.850791817648235, 1.1031456067289128, -0.3104654609270394, -1.3138586727237413, -0.8174240070861964), 'r': (0.2505741638044654, -0.9006784175037083, -0.4091691351442108, -1.670779516757796, 0.6212879185982039, 0.8429657882244618, -2.4777225202159308)}
import math
x_offset = 0
y_offset = 0
z_offset = 0.05
b_offset = 0.04
move_points={'p1':(0.6, -0.309544, 1.1), 'p2':(0.6, -0.309544, 0.9), 'p3':(0.6, -0.309544, 1.1), 'p4':(0.85, -0.15, 1.1), 'p5':(0.85, -0.15, 0.9), 'p6':(0.85, -0.15, 1.1), 'p7':(0.6, -0.309544, 1.1)}
point_list = ["p1","p2","p3","p4","p5","p6","p7"]
class temp_pickbolt_client(object):
    def __init__(self):
        rospy.init_node('transform_subscriber', anonymous= True)
        rospy.Subscriber("/marker",geometry_msgs.msg.Transform,self.marker_CallBack)
        
        self.msg=None
    def marker_CallBack(self,msg):
        print(msg)
        self.msg=msg

class seq_move(object):
    def __init__(self):
        # rospy.init_node('move_node', anonymous= True)
        a = 1

    def sequential_move(self,point_list,move_points):
        ods=odyssey_Interface()
        for i in range(5):
            print "=========== move to next place =========="
            raw_input()
            x = move_points[point_list[i]][0]
            y = move_points[point_list[i]][1]
            z = move_points[point_list[i]][2]
            ods._L0_single_task_move_safe("right",[x + x_offset, y + y_offset, z + z_offset],
                                        [0.05 + math.pi/2, math.pi / 2, -math.pi / 2],
                                        [20 for i in range(6)])

class box_detection(object):
    def __init__(self):
        # b_x = 0.82
        # b_y = -0.4
        # b_z = 0.8
        b_x = 0
        b_y = 0
        b_z = 0
        success = False
        ods = odyssey_Interface()
        b_offset = 0.04
        self.b_x=b_x
        self.b_y=b_y
        self.b_z=b_z
        self.success = success
        self.ods = ods
        self.b_offset = b_offset


    def marker_Callback(self,data):
        # print "In callback"
        print self.success
        if self.success == False:
            # self.b_x = data.pose.position.x
            # self.b_y = data.pose.position.y
            # self.b_z = data.pose.position.z
            self.b_x = data.translation.x
            self.b_y = data.translation.y
            self.b_z = data.translation.z
            self.success=True
            self.pick_and_place()

    def ARlistener(self):
        rospy.Subscriber("/marker",geometry_msgs.msg.Transform,self.marker_Callback)
        print "In listener"
        rospy.spin()
    
    def pick_and_place(self):
        if self.success == True:
            ods = odyssey_Interface()
            print "Moving to Initial pose"
            raw_input()
            ods._L0_single_task_move_safe("right",[0.7263-0.1, -0.309544, 0.93824+0.2],
                                    [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
                                    [20 for i in range(6)])
            # gripper oppen
            print "gripper oppen"
            raw_input()
            ods.grip("right",1)

            # move to the top of box
            print "move to the top"
            raw_input()
            ods._L0_single_task_move_safe("right",[self.b_x + self.b_offset, self.b_y, 0.93824+0.2],
                                [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
                                [20 for i in range(6)]) 
                                
            # move downwards
            print "move downwards"
            raw_input()
            ods._L0_single_task_move_safe("right",[self.b_x + self.b_offset, self.b_y, self.b_z+0.1],
                [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
                [20 for i in range(6)]) 

            # grasp the box 
            print "gripper close"
            raw_input()
            ods.grip("right",0) 

            # move upwards
            print "move upwards"
            raw_input()
            ods._L0_single_task_move_safe("right",[self.b_x + self.b_offset, self.b_y, 0.93824+0.2],
                                [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
                                [20 for i in range(6)])   

            # move to 2nd place
            print "move to second place"
            raw_input()
            ods._L0_single_task_move_safe("right",[self.b_x + self.b_offset +0.2, self.b_y +0.1, 0.93824+0.2],
                                [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
                                [20 for i in range(6)])   
            
            # move downwards
            print "move downwards"
            raw_input()
            ods._L0_single_task_move_safe("right",[self.b_x + self.b_offset +0.2, self.b_y +0.1, self.b_y+0.1],
                                [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
                                [20 for i in range(6)])  
            
            # gripper open
            print "gripper open"
            raw_input()
            ods.grip("right",1)

            # move upwards
            print "move upwards"
            raw_input()
            ods._L0_single_task_move_safe("right",[dbox.b_x + b_offset +0.2, dbox.b_y +0.1, 0.93824+0.2],
                                [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
                                [20 for i in range(6)])

            # move to initial pose
            print "back to initial pose"
            raw_input()
            ods._L0_single_task_move_safe("right",[0.7263-0.1, -0.309544, 0.93824+0.2],
                            [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
                            [20 for i in range(6)]) 
        
#temp_pickbolt_client.marker_CallBack()

if __name__=="__main__":
    rospy.init_node("Pick_Box_Test")
    dbox = box_detection()
    dbox.ARlistener()
    dbox.pick_and_place()
    # if dbox.success == True:
    #     ods = odyssey_Interface()
    #     print "Moving to Initial pose"
    #     raw_input()
    #     ods._L0_single_task_move_safe("right",[0.7263-0.1, -0.309544, 0.93824+0.2],
    #                             [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
    #                             [20 for i in range(6)])
    #     # gripper oppen
    #     print "gripper oppen"
    #     raw_input()
    #     ods.grip("right",1)

    #     # move to the top of box
    #     print "move to the top"
    #     raw_input()
    #     ods._L0_single_task_move_safe("right",[dbox.b_x + b_offset, dbox.b_y, 0.93824+0.2],
    #                         [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
    #                         [20 for i in range(6)]) 
                            
    #     # move downwards
    #     print "move downwards"
    #     raw_input()
    #     ods._L0_single_task_move_safe("right",[dbox.b_x + b_offset, dbox.b_y, dbox.b_y+0.1],
    #         [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
    #         [20 for i in range(6)]) 

    #     # grasp the box 
    #     print "gripper close"
    #     raw_input()
    #     ods.grip("right",0) 

    #     # move upwards
    #     print "move upwards"
    #     raw_input()
    #     ods._L0_single_task_move_safe("right",[dbox.b_x + b_offset, dbox.b_y, 0.93824+0.2],
    #                         [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
    #                         [20 for i in range(6)])   

    #     # move to 2nd place
    #     print "move to second place"
    #     raw_input()
    #     ods._L0_single_task_move_safe("right",[dbox.b_x + b_offset +0.2, dbox.b_y +0.1, 0.93824+0.2],
    #                         [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
    #                         [20 for i in range(6)])   
        
    #     # move downwards
    #     print "move downwards"
    #     raw_input()
    #     ods._L0_single_task_move_safe("right",[dbox.b_x + b_offset +0.2, dbox.b_y +0.1, dbox.b_y+0.1],
    #                         [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
    #                         [20 for i in range(6)])  
        
    #     # gripper open
    #     print "gripper open"
    #     raw_input()
    #     ods.grip("right",1)

    #     # move upwards
    #     print "move upwards"
    #     raw_input()
    #     ods._L0_single_task_move_safe("right",[dbox.b_x + b_offset +0.2, dbox.b_y +0.1, 0.93824+0.2],
    #                         [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
    #                         [20 for i in range(6)])

    #     # move to initial pose
    #     print "back to initial pose"
    #     raw_input()
    #     ods._L0_single_task_move_safe("right",[0.7263-0.1, -0.309544, 0.93824+0.2],
    #                     [0.2+math.pi/2, math.pi / 2, -math.pi / 2],
    #                     [20 for i in range(6)]) 
        

    # mtp = seq_move()
    # mtp.sequential_move(point_list,move_points)
    

# def subscriber():
#     rospy.init_node('transform_subscriber', anonymous= True)
#     rospy.Subscriber()
