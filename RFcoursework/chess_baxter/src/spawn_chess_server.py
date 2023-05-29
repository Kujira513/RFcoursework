#!/usr/bin/env python

import rospy
import sys
import rospkg

'''
Group R
Mingfeng Ye: 2325714Y
Haowen Li: 2327962L
Shengxian Liu: 2284864L
Chung Ki Yau: 2359033Y
'''

'''
this is the ROS service for spawning a chess piece to a pre-defined postion.

once it running, the node <pick_and_place_chess_pieces.py> will call this service
to spawn a chess piece to a pre-defined posiotn.
'''




from std_msgs.msg import (
    Empty,
)

from gazebo_msgs.srv import *
from geometry_msgs.msg import *

from chess_baxter.srv import spawnChessResponse, spawnChess

'''
the pre-defined postion is indicated here
'''
pre_define_block_pose=Pose(position=Point(x=0.40, y=0.63, z=0.78))

# this is a sercice that use to spawn sdf model
rospy.wait_for_service("gazebo/spawn_sdf_model")
srv_call = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

'''
NOTE:this service will receive two string argument, 
req.a -> the name of the piece
req.b -> the piece model info

'''
def spawnChess_handle(req):
    
    srv_call(req.a, req.b, "", pre_define_block_pose, "world")
    
    return spawnChessResponse(success = True)



def main():
    

    rospy.init_node('spawn_chess_server')
    # service
    rospy.Service('spawn_Chess', spawnChess, spawnChess_handle)
    rospy.loginfo("Service ready to spawn chess pieces!")
    rospy.spin()

# ********* MAIN *********
if __name__ == '__main__':
    
    main()