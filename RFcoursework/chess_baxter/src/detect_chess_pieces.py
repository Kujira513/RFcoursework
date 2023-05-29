#!/usr/bin/env python
import rospy
import tf
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import String


'''
Group R
Mingfeng Ye: 2325714Y
Haowen Li: 2327962L
Shengxian Liu: 2284864L
Chung Ki Yau: 2359033Y
'''

'''
this is the Gazebo to TF ROS node for detect the chess pieces.

it is actually a TF broadcaster to boardcast the position of chess pieces

it will update to detect a chess piece by receive the specfic chess piece name from a topic
in other word, it subscribe to a topic "chatter"


NOTE: the node <pick_and_place_chess_pieces> and 
      the node <play_choreographed_game> will use this TF broadcaster
      to pick the chess piece, they all have the publisher to publish a
      chess piece name and then use TF listener to get the postion of a
      chess piece it want!!!

'''

# this is the deault chess piece that this node to detect the position
input_linkname = "r00" # default

# Global variable where the object's pose is stored
pose = None


# once this node receive the specfic chess piece name, it will use this callback function to update 
# the chess piece this node gonna be detect!!!
'''
    because the input_linkname is global, so it gonna update the chess piece this node gonna dectect, 
    once it receive the chess piece name from a topic, it will excute this function automatically.
    '''
def callback(data):
    #print("ddfdsfafasdfasfadsf")
    print("change to chess piece <%s> now" % data.data)
    global input_linkname
    input_linkname = data.data
 

'''
this function is main part that use to dectect a specfic chess piece
'''
def get_links_gazebo(link_states_msg):
    # Call back to retrieve the object you are interested in
    try:
        global pose
        poses = {'world': link_states_msg.pose[0]} # get world link
        for (link_idx, link_name) in enumerate(link_states_msg.name):
            modelname = link_name.split('::')[0]
            if input_linkname == modelname:
                poses[modelname] = link_states_msg.pose[link_idx]

        pose = poses[input_linkname]
    except:
        pass


def main():
    rospy.init_node('detect_chess_pieces', anonymous=True)
    
    # Create TF broadcaster -- this will publish a frame give a pose
    tfBroadcaster = tf.TransformBroadcaster()
    
    
    global input_linkname
    
    # subscribe to the chatter topic, receive the name of the chess piece we want to detect
    # and then will excute the callback function to change the chess piece we want to detect
    op = rospy.Subscriber("chatter", String, callback)
    print(op)
    # SUbscribe to Gazebo's topic where all links and objects poses within the simulation are published
    rospy.Subscriber('gazebo/link_states', LinkStates, get_links_gazebo)
    rospy.loginfo('Spinning')
    global pose
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
       if pose is not None:

            pos = pose.position
            ori = pose.orientation
            
            rospy.loginfo("%s" % input_linkname)
            print(pos)
            print(ori)
            print("..........\n")
            
            # Publish transformation given in pose
            # the node which pick the chess piece where receive the positon by calling this function <sendTransform>
            tfBroadcaster.sendTransform((pos.x, pos.y, pos.z - 0.93), (ori.x, ori.y, ori.z, ori.w), rospy.Time.now(), input_linkname, 'world')
            rate.sleep()
   
    
    rospy.spin()


if __name__ == '__main__':
    main()