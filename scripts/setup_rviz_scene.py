#!/usr/bin/env python

from catkin.find_in_workspaces import find_in_workspaces
import openravepy
import os
import numpy
import logging
import yaml
from harmonic_playback.msg import ProbabilityUpdate
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from matplotlib import cm

project_name = 'harmonic_playback'
logger = logging.getLogger(project_name)

URDF_PATH = 'package://harmonic_playback/data/mico.urdf'
SRDF_PATH = 'package://harmonic_playback/data/mico.srdf'

class JointStateClient(object):
    def __init__(self, robot, topic_name):
        from rospy import Subscriber
        from sensor_msgs.msg import JointState

        self._robot = robot
        self._subscriber = Subscriber(topic_name, JointState, self._callback)
        self._dof_mapping = {
            joint.GetName(): joint.GetDOFIndex() for joint in robot.GetJoints()
        }

    def _callback(self, joint_msg):
        from openravepy import KinBody

        # Map from joint names to DOF indices.
        dof_indices = []
        dof_values = []

        for name, position in zip(joint_msg.name, joint_msg.position):
            dof_index = self._dof_mapping.get(name)
            if dof_index is not None:
                dof_indices.append(dof_index)
                dof_values.append(position)

        # Update joint values.
        with self._robot.GetEnv():
            self._robot.SetDOFValues(dof_values, dof_indices,
                                     KinBody.CheckLimitsAction.Nothing)
            
def rodrigues(r):
    def S(n):
        Sn = numpy.array([[0,-n[2],n[1]],[n[2],0,-n[0]],[-n[1],n[0],0]])
        return Sn
    theta = numpy.linalg.norm(r)
    if theta > 1e-30:
        n = r/theta
        Sn = S(n)
        R = numpy.eye(3) + numpy.sin(theta)*Sn + (1-numpy.cos(theta))*numpy.dot(Sn,Sn)
    else:
        Sr = S(r)
        theta2 = theta**2
        R = numpy.eye(3) + (1-theta2/6.)*Sr + (.5-theta2/24.)*numpy.dot(Sr,Sr)
#    return mat(R)
    return R

def setup(sim=False, viewer=None, debug=False):

    data_base_path = find_in_workspaces(
        search_dirs=['share'],
        project=project_name,
        path='data',
        first_match_only=True)
    if len(data_base_path) == 0:
        raise Exception('Unable to find environment path. Did you source devel/setup.bash?')

    env_path = os.path.join(data_base_path[0], 'environments', 'table.env.xml')
    
    # Initialize logging
    if debug:
        openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Debug)
    else:
        openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
    openravepy.misc.InitOpenRAVELogging()

    # Load the environment
    env = openravepy.Environment()
    if not env.Load(env_path):
        raise IOError(
            'Unable to load environment "{:s}".'.format(env_path))
    
    with env:
        or_urdf = openravepy.RaveCreateModule(env, 'urdf')
        ada_name = or_urdf.SendCommand(
            'LoadURI {:s} {:s}'.format(URDF_PATH, SRDF_PATH))
        
    env.SetViewer('InteractiveMarker')
        
        
    robot = env.GetRobot(ada_name)

    #TODO get this from a rosparam
    right_handed = True

    # Set the active manipulator on the robot
    robot.arm = robot.GetManipulator('Mico')
#     robot.arm.SetActive()

    # Now set everything to the right location in the environment
    #if using jaco, assume we have the portable mount, and robot should have a different distance to table
    using_jaco = robot.GetName() == 'JACO'
    if using_jaco:
      robot_pose = numpy.array([[1., 0., 0., 0.409],
                              [0., 1., 0., 0.338],
                              [0., 0., 1., 0.754],
                              [0., 0., 0., 1.]])
    else:
      robot_pose = numpy.array([[1., 0., 0., 0.409],
                              [0., 1., 0., 0.338],
                              [0., 0., 1., 0.795],
                              [0., 0., 0., 1.]])


    with env:
        robot.SetTransform(robot_pose)

    #if sim is True:
    #   startConfig = numpy.array([  3.33066907e-16,   2.22044605e-16,   1.66608370e+00,
    #    -1.65549603e+00,  -1.94424475e-01,   1.06742772e+00,
    #    -1.65409614e+00,   1.30780704e+00])
    logger.info('Setting Initial Robot Configuration to Serving')
    #    robot.SetDOFValues(startConfig)

    logger.info('Initial Config Set')
    # Load the fork into the robot's hand
    tool = env.ReadKinBodyURI('objects/kinova_tool.kinbody.xml')
    env.Add(tool)
     
     # Fork in end-effector
     #ee_in_world = robot.GetLink('j2n6a300_link_6').GetTransform()
    tool_in_ee = numpy.array([[ -1., 0.,  0., 0.],
                             [ 0.,  1., 0., -0.002],
                             [ 0.,  0.,  -1., -0.118],
                             [ 0.,  0.,  0., 1.]])


    ee_in_world = robot.arm.GetEndEffectorTransform()
    if right_handed:
        y_trans_tool = 0.004
    else:
        y_trans_tool = -0.004
    tool_in_ee = numpy.array([[ 1., 0.,  0., 0.],
                            [ 0.,  1., 0., y_trans_tool],
                            [ 0.,  0.,  1., -0.042],
                            [ 0.,  0.,  0., 1.]])
    rotate_tool_in_ee = rodrigues([0., 0., 0.])
    rotate_tool_in_ee = rodrigues([0., 0., numpy.pi/32.])
    #rotate_tool_in_ee = rodrigues([0., 0., -np.pi/32.])
    tool_in_ee[0:3, 0:3] = numpy.dot(rotate_tool_in_ee, tool_in_ee[0:3, 0:3])

    tool_in_world = numpy.dot(ee_in_world, tool_in_ee)
    tool.SetTransform(tool_in_world)
    
    fork = env.ReadKinBodyURI('objects/fork.kinbody.xml')
    env.Add(fork)
    fork_in_hole = numpy.array([[1.,0.,0.,0.],
                                [0.,1.,0.,0.],
                                [0.,0.,1.,-0.03],
                                [0.,0.,0.,1.]])
    hole_in_tool = numpy.array([[0.,0.,1.,0.],
                                [0.,1.,0.,-0.0225],
                                [-1.,0.,0.,0.0408],
                                [0.,0.,0.,1.]])
    fork_in_tool = numpy.dot(hole_in_tool, fork_in_hole)                           
    fork_in_ee = numpy.dot(tool_in_ee, fork_in_tool)
    fork_in_world = numpy.dot(ee_in_world, fork_in_ee)
    fork.SetTransform(fork_in_world)
    logger.info('creating fork and tool boxes')
    # fork_box = make_collision_box_body(fork, add_to_pos=np.array([0.0, 0.0, 0.05]), add_to_extents=np.array([0.02, 0.02, 0.1]))
    # tool_box = make_collision_box_body(tool, add_to_pos=np.array([0.0, 0.0, 0.04]), add_to_extents=np.array([0.055, 0.055, 0.055]))

    # logger.info('fork and tool boxes created')

    
    #find all finger links
#     finger_link_inds = []
#     grab_link = None
#     for ind,link in enumerate(robot.GetLinks()):
#         if 'inger' in link.GetName():
#             finger_link_inds.append(ind)
#         if 'end_effector' in link.GetName():
#             grab_link = link
# 
#     logger.info('Grabbing tool and fork')
    robot.Grab(tool)
    robot.Grab(fork)
 #   robot.Grab(tool_box, grablink=grab_link, linkstoignore=finger_link_inds)
  #  robot.Grab(fork_box, grablink=grab_link, linkstoignore=finger_link_inds)
    return robot, env

class MorselDisplay:
    def __init__(self, env, morsel_file, morsel_topic):
        with open(morsel_file, 'rb') as f:
            morsels = yaml.load(f)
        
        self.morsels = {}      
        
        object_base_path = find_in_workspaces(
            search_dirs=['share'],
            project=project_name,
            path='data',
            first_match_only=True)[0]
        ball_path = os.path.join(object_base_path, 'objects', 'smallsphere.kinbody.xml')
        delta_path = os.path.join(object_base_path, 'objects', 'mediumsphere.kinbody.xml')
    
        for name in ['morsel0', 'morsel1', 'morsel2']:
            if name not in morsels:
                continue
            with env:
                morsel = env.ReadKinBodyURI(ball_path)
                delta = env.ReadKinBodyURI(delta_path)
                morsel.SetName(name)
                delta.SetName(name + '_delta')
                env.Add(morsel)
                env.Add(delta)
                morsel.Enable(False)
                delta.Enable(False)
        
            morsel.SetTransform(morsels[name])
            delta.SetTransform(morsels[name])
            self.morsels[name] = {'morsel': morsel, 'delta': delta}
    
        self.sub = rospy.Subscriber(morsel_topic, ProbabilityUpdate, self._callback)
        self.marker_publisher = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=50)
        
    def _callback(self, msg):
        marker_msg = MarkerArray()
        for ind, (name, prob) in enumerate(zip(msg.names, msg.probabilities)):
            try:
                delta = self.morsels[name]['delta']
                position = delta.GetTransform()[0:3,3]
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = msg.header.stamp
                marker.type = Marker.TEXT_VIEW_FACING
                marker.id = ind
                marker.ns = 'probabilities'
                marker.lifetime.secs = 1
            
                marker.pose.position.x = position[0]
                marker.pose.position.y = position[1]
                marker.pose.position.z = position[2] + 0.1
                marker.pose.orientation.x = 0.
                marker.pose.orientation.y = 0.
                marker.pose.orientation.z = 0.
                marker.pose.orientation.w = 1.
            
                marker.text = str(np.ceil(prob*100.)/100.)
                marker.scale.x = 0.09
                marker.scale.y = 0.09
                marker.scale.z = 0.09
            
                color_jet = cm.jet(prob)
                marker.color.r = color_jet[0]
                marker.color.g = color_jet[1]
                marker.color.b = color_jet[2]
                marker.color.a = color_jet[3]
                
                delta.GetLinks()[0].GetGeometries()[0].SetDiffuseColor(color_jet)

                marker_msg.markers.append(marker)
            except KeyError:
                rospy.logwarn('Got probability update for unknown morsel {}'.format(name))
        self.marker_publisher.publish(marker_msg)
        
if __name__ == "__main__":
    import rospy
    rospy.init_node('setup', anonymous=True)
    # Wait for bag file to be set up
#     if rospy.get_param('use_sim_time', default=False):
#         import time
#         while (rospy.get_time() <= 0.01):
#             time.sleep(0.1)
    
    
    robot, env = setup()
    if rospy.has_param('~morsel_file') and rospy.get_param('~morsel_file') != '':
        morsel_display = MorselDisplay(env, rospy.get_param('~morsel_file'), rospy.get_param('~morsel_topic'))
    joint_client = JointStateClient(robot, rospy.get_param('~joint_state_topic'))
    
    rospy.spin()
    
    