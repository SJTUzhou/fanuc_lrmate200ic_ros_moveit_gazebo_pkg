#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math

# set the following constants according to ROBOT urdf
GRIPPER_MIN_LIMIT = 0.000
GRIPPER_MAX_LIMIT = 0.036
GRIPPER_JOINT_NAMES = ['joint_claw_finger1', 'joint_claw_finger2']



# in ROS: roll yaw pitch represnts the rotation angles around the x\y\z axis
# convert quaternions[w, x, y, z] to roll, pitch, yaw
def quaternions2rpy(w, x, y, z):
    roll = math.atan2(2*(w*x+y*z), 1-2*(x**2+y**2))
    pitch = math.arcsin(2*(w*y-z*x))
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y**2+z**2))
    return roll, pitch, yaw


# convert roll, pitch, yaw to quaternions[w, x, y, z]
def rpy2quaternions(roll, pitch, yaw):
    w = math.cos(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
    x = math.sin(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) - math.cos(roll/2)*math.sin(pitch/2)*math.sin(yaw/2) 
    y = math.cos(roll/2)*math.sin(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.cos(pitch/2)*math.sin(yaw/2)
    z = math.cos(roll/2)*math.cos(pitch/2)*math.sin(yaw/2) - math.sin(roll/2)*math.sin(pitch/2)*math.cos(yaw/2)   
    return w, x, y, z


# set end effector pose with position(x, y, z) and orientation(roll, pitch, yaw)
def set_eef_pose(px, py, pz, roll, pitch, yaw):
    if px == 'no_value':
        print('No pose value in param sever!')
        return None
    else:
        w, x, y, z = rpy2quaternions(roll, pitch, yaw)
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = w
        pose_target.orientation.x = x
        pose_target.orientation.y = y
        pose_target.orientation.z = z
        pose_target.position.x = px
        pose_target.position.y = py
        pose_target.position.z = pz
        return pose_target


def set_gripper_position(group_gripper, gripper_position):
    show_joint_info(group_gripper)
    if GRIPPER_MIN_LIMIT <= gripper_position <= GRIPPER_MAX_LIMIT:
        target_dict = {GRIPPER_JOINT_NAMES[0]: -gripper_position, GRIPPER_JOINT_NAMES[1]: gripper_position}
        group_gripper.set_joint_value_target(target_dict)
        plan = group_gripper.plan()
        group_gripper.execute(plan)
        # Uncomment below line when working with a real robot
        # group_gripper.go()
        show_joint_info(group_gripper)
    else:
        rospy.loginfo('The value of gripper position should be set in [%f, %f]' %(GRIPPER_MIN_LIMIT, GRIPPER_MAX_LIMIT))
        rospy.loginfo('Param: Gripper position %f' %gripper_position)


# get the parameters from the param server
def param_get():
    node_name = 'set_lrmate200ic_pose'
    roll = rospy.get_param(node_name + '/roll', 'no_value')
    pitch = rospy.get_param(node_name + '/pitch', 'no_value')
    yaw = rospy.get_param(node_name + '/yaw', 'no_value')
    px = rospy.get_param(node_name + '/px', 'no_value')
    py = rospy.get_param(node_name + '/py', 'no_value')
    pz = rospy.get_param(node_name + '/pz', 'no_value')
    gripper_position = rospy.get_param(node_name + '/gripper_position', 'no_value')
    return roll, pitch, yaw, px, py, pz, gripper_position

# print information about the joints of a given group
def show_joint_info(group):
    rospy.loginfo("Current joint values: ")
    joint_name_lst = group.get_active_joints()
    joint_value_lst = group.get_current_joint_values()
    for (name, value) in zip(joint_name_lst, joint_value_lst):
        rospy.loginfo("%s: %.3f" %(name, value))
    #rospy.loginfo(str(joint_name_lst))


# print information about a given group
def show_group_information(group):
    eef_name = group.get_end_effector_link()
    if eef_name != '':
        reference_frame = group.get_pose_reference_frame()
        eef_pose_stamped = group.get_current_pose(eef_name)
        # pose.orientation / pose.position
        eef_pose = eef_pose_stamped.pose 
        rpy_lst = group.get_current_rpy(eef_name)       
        rospy.loginfo("End effector pose reference frame: %s" %str(reference_frame))
        rospy.loginfo("End effector name: %s" %str(eef_name))        
        rospy.loginfo("End effector current pose:\n%s" %str(eef_pose))
        rospy.loginfo("End effector Euler angles:\n[roll: %.3f, pitch: %.3f, yaw: %.3f]" %(rpy_lst[0], rpy_lst[1], rpy_lst[2]))
    else:
        rospy.loginfo("No end effector found in Group %s!" % group.get_name())
    show_joint_info(group)
    




def set_end_effector_pose():
    # sys.argv: a list containing the parameters from the command line; the first element is the name of the script.
    moveit_commander.roscpp_initialize(sys.argv)
    # anonymous = True to ensure the uniqueness of the node
    rospy.init_node('set_lrmate200ic_pose', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # group name 'manipulator' is the name of the arm group including claw
    group_manipulator = moveit_commander.MoveGroupCommander('manipulator')
    group_gripper = moveit_commander.MoveGroupCommander('gripper')

    show_group_information(group_manipulator)
    # show_group_information(group_gripper)
    
    # get the param from the param sever
    roll, pitch, yaw, px, py, pz, gripper_position = param_get()

    # set a pose target for the end effector
    pose_target = set_eef_pose(px, py, pz, roll, pitch, yaw)
    # pose_target is either a pose msg or None
    if pose_target:
        group_manipulator.set_pose_target(pose_target)

        # plan and execute
        plan = group_manipulator.plan()
        rospy.sleep(0.5)
        group_manipulator.execute(plan)
        # Uncomment below line when working with a real robot
        # group_manipulator.go()

        show_group_information(group_manipulator)
    
    # set group gripper
    set_gripper_position(group_gripper, gripper_position)

    # wait for printing information
    rospy.sleep(0.5)
    moveit_commander.roscpp_shutdown()


def main():
    set_end_effector_pose()



if __name__ == '__main__':
    main()
