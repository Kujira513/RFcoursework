#!/usr/bin/env python
import sys
import copy

import rospy
import rospkg
import tf2_ros
from std_msgs.msg import String
from std_msgs.msg import (
    Empty,
)

from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from chess_baxter.srv import *
import baxter_interface
import moveit_commander

'''
Group R
Mingfeng Ye: 2325714Y
Haowen Li: 2327962L
Shengxian Liu: 2284864L
Chung Ki Yau: 2359033Y
'''

'''
this is the node for play a three legal pre-fined move once the chessboard
is setting up

NOTE: the principle is that this node publish to "chatter" topic to make the 
      TF boardcaster <detect_chess_pieces> to boardcast the postion of a chess piece
      this node want!! and then have a listener to listen the position of this chess piece.

      And one more thing is that we use the motion planning here.
'''
class PickAndPlaceMoveIt(object):
    def __init__(self, limb, hover_distance=0.15, verbose=True):
        self._limb_name = limb  # string
        self._hover_distance = hover_distance  # in meters
        self._verbose = verbose  # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")

         # This is an interface to the world surrounding the robot.
        self._scene = moveit_commander.PlanningSceneInterface()

        self._robot = moveit_commander.RobotCommander()
        # This is an interface to one group of joints.  In our case, we want to use the "right_arm".
        # We will use this to plan and execute motions
        self._group = moveit_commander.MoveGroupCommander(limb+"_arm")

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))

        self.gripper_open()
        self._group.set_pose_target(start_angles)
        plan = self._group.plan()
        self._group.execute(plan)
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        rospy.sleep(0.9)
        self._gripper.open()
        rospy.sleep(0.9)

    def gripper_close(self):
        rospy.sleep(1.0)
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance

        self._group.set_pose_target(approach)
        plan = self._group.plan()
        self._group.execute(plan)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w

        # servo up from current pose
        self._group.set_pose_target(ik_pose)
        plan = self._group.plan()
        self._group.execute(plan)

    def _servo_to_pose(self, pose):
        # servo down to release
        self._group.set_pose_target(pose)
        plan = self._group.plan()
        self._group.execute(plan)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()


def main():
    moveit_commander.roscpp_initialize(sys.argv)

    '''
    make this node a publisher of the topic <chatter>
    '''
    pub = rospy.Publisher('chatter', String, queue_size=1)

    rospy.init_node("ik_play_choreographed_game", anonymous=True)

   
    rospy.wait_for_message("/robot/sim/started", Empty)

    '''
    create a listener to the TF boardCaster
    '''
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    '''
    receive the positonmap by ROS Paremeter Server
    '''
    piece_positionmap = rospy.get_param('piece_target_position_map')
   
    limb = 'left'
    hover_distance = 0.15  # meters

    



    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    #starting_pose = Pose(position=Point(x=0.40, y=0.63, z=0.15), orientation=overhead_orientation)
    pnp = PickAndPlaceMoveIt(limb, hover_distance)
     
        
    '''
    this is the pre-define chess piece that need to moves
    '''    
    three_moves_chess_piece  = ["r01","R72","k20"]
    '''
    this is the postion we want them to move, these three are legal moves
    '''
    three_moves_predefined_position = ["21","70","11"]

    '''
    here to play the legal moves
    '''
    rospy.loginfo("\n\nNow play the choreographed game for three moves!! \n\n")
    for number,each_move_chess_piece in enumerate(three_moves_chess_piece):
        '''
        publish the name of chess piece we want to move
        '''
        pub.publish(each_move_chess_piece)

        '''
        keep listen to the TF boardcaster, until get the postion back
        '''
        while True:
            try:
                
                transformation1 = tfBuffer.lookup_transform('base', each_move_chess_piece, rospy.Time(0))
           
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
        
        '''
        positon get here
        '''
        x = transformation1.transform.translation.x
        y = transformation1.transform.translation.y
        z = transformation1.transform.translation.z

        '''
        pick and place pieces here
        '''
        rospy.loginfo("\n\nNow we play the %d move, three moves totally\n\n" % (number+1))
        pnp.pick(Pose(position=Point(x=x, y=y, z=z-0.003), orientation=overhead_orientation))
        place_pose = piece_positionmap[three_moves_predefined_position[number]]       
        pnp.place(Pose(position=Point(x=place_pose[0], y=place_pose[1], z=place_pose[2]+0.01),orientation=overhead_orientation))

    

        '''
        if the piece is not in the correct position, re-pick and re-place the chess pieces again!!
        '''
        transformation2 = tfBuffer.lookup_transform('base', each_move_chess_piece, rospy.Time(0))            
        x2 = transformation2.transform.translation.x
        y2 = transformation2.transform.translation.y
        z2 = transformation2.transform.translation.z
        print(x2,y2,z2)
        while not (place_pose[0] - 0.02 < x2 < place_pose[0] + 0.02 and place_pose[1] -0.02 < y2 < place_pose[1] + 0.02):
            
            rospy.loginfo("\n\nThe chess piece is not in the correct postion, so re-pick and re-place again!!!!\n\n")
            rospy.loginfo("\n\nre-pick now...\n\n")    
            pnp.pick(Pose(position=Point(x=x2, y=y2, z=z2-0.003), orientation=overhead_orientation))
            rospy.loginfo("\n\nre-place now...\n\n")           
            pnp.place(Pose(position=Point(x=place_pose[0], y=place_pose[1], z=place_pose[2]+0.01),orientation=overhead_orientation))
            transformation2 = tfBuffer.lookup_transform('base', each_move_chess_piece, rospy.Time(0))            
            x2 = transformation2.transform.translation.x
            y2 = transformation2.transform.translation.y
            z2 = transformation2.transform.translation.z
    
    rospy.loginfo("\n\nNow all the legal moves has been excuted! \n\n")
    return 0
    
    


if __name__ == '__main__':
    sys.exit(main())
