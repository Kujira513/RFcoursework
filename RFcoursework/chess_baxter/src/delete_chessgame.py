#!/usr/bin/env python
import sys, rospy, tf, rospkg
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy

'''
Group R
Mingfeng Ye: 2325714Y
Haowen Li: 2327962L
Shengxian Liu: 2284864L
Chung Ki Yau: 2359033Y
'''

'''
this is the node for deleting the chess pieces, chessboard, table
'''

# http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials

if __name__ == '__main__':
    rospy.init_node("delete_chessboard")
    # call the service of deleting model 
    rospy.wait_for_service("gazebo/delete_model")

    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

    list_pieces = rospy.get_param('list_pieces')
    board_setup = rospy.get_param('board_setup')
    for row, each in enumerate(board_setup):
        for col, piece in enumerate(each):
            if piece in list_pieces:
                piece_name = "%s%d%d" % (piece, row, col)
                print "Deleting "+piece_name
                delete_model(piece_name)
    
    delete_model("cafe_table")
    delete_model("chessboard")

