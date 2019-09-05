#!/usr/bin/env python

import csv
import os
import argparse
import numpy as np
import cv2
import yaml
import progressbar

import rosbag
import rospy

import geometry_msgs.msg
import sensor_msgs.msg
import std_msgs.msg
import visualization_msgs.msg
import ibmmpy.msg
import tf2_msgs.msg
import tf.transformations as tf

import contextlib
from tf2_geometry_msgs.tf2_geometry_msgs import from_msg_msg

class SubDirs:
    GAZE_DIR = 'gaze'
    RAW_DIR = 'raw_data'
    TEXT_DIR = 'text_data'
    VID_DIR = 'videos'
    PROC_DIR = 'processed'
    STATS_DIR = 'stats'
    def __init__(self, run_dir):
        self._run_dir = run_dir
    def path(self, d):
        return os.path.join(self._run_dir, d)

def make_dir(directory):
    if not os.path.isdir(directory):
        try:
            os.makedirs(directory)
        except OSError as e:
            if e.errno != errno.EEXIST:
                raise
            
class MessageContainer():
    def __init__(self, msg):
        self.msg = msg

class CsvDataGenerator:
    def __init__(self, filename, ts):
        self.filename = filename
        self._seq = 0
        self._ts = ts
    
    def __iter__(self):
        with open(self.filename, 'rb') as f:
            reader = csv.DictReader(f)
            
            try:
                for next_t in self._ts:
                    for vals in reader:
                        t, msg = self.build_message(vals)
                        if t.to_sec() >= next_t:
                            if hasattr(msg.msg, 'header'):
                                msg.msg.header.seq = self._seq
                                self._seq += 1
                            yield t, msg
                            break
            except StopIteration: # out of time steps
                pass
                        
    
    def build_message(self, vals):
        raise NotImplementedError()
  

class MultiCsvDataGenerator:
    def __init__(self, filenames, ts):
        self.filenames = filenames
        self._each_seq = [0] * len(self.filenames)
        self._seq = 0
        self._ts = ts
    
    def __iter__(self):
        with contextlib.nested(*(open(f, 'rb') for f in self.filenames)) as files:
            # can only use "nested" for this until python3.3 :(
            # see https://docs.python.org/2.7/library/contextlib.html#contextlib.ExitStack
            # for 3.3, use contextlib.ExitStack
            readers = [csv.DictReader(f) for f in files]
            iters = [iter(r) for r in readers]
            cache = [ [] for _ in readers ]
            next_cache = [ [] for _ in readers ]
            
            for next_t in self._ts:
                for i_reader, reader_it in enumerate(iters):
                    for vals in reader_it:
                        t, msg = self.build_message(vals, i_reader)
                        if hasattr(msg, 'header'):
                            msg.header.seq = self._each_seq[i_reader]
                            self._each_seq[i_reader] += 1
                        if t.to_sec() < next_t:
                            cache[i_reader].append((t, msg))
                        else:
                            next_cache[i_reader].append((t, msg)) # catch the past-the-end message
                            break
                merged_t, merged_msg = self.combine_messages(cache)
                if hasattr(merged_msg.msg, 'header'):
                    merged_msg.msg.header.seq = self._seq
                    merged_msg.msg.header.stamp = rospy.Time.from_sec(next_t)
                    self._seq += 1
                yield merged_t, merged_msg
                cache = next_cache
                next_cache = [ [] for _ in readers ]
                
    
    def build_message(self, vals, idx):
        raise NotImplementedError()
    
class JoystickDataGenerator(CsvDataGenerator):
    topic = '/joystick'
    def __init__(self, base_dir, *args):
        filename = os.path.join(base_dir, SubDirs.TEXT_DIR, 'ada_joy.csv')
        CsvDataGenerator.__init__(self, filename, *args)
    
    def build_message(self, vals):
        t = rospy.Time.from_sec(float(vals['timestamp']))
        axes = [float(vals['axes_x']), float(vals['axes_y']), float(vals['axes_z'])]
        buttons = [int(vals['buttons_0']), int(vals['buttons_1'])]
        msg = sensor_msgs.msg.Joy(axes=axes, buttons=buttons)
        msg.header.stamp = t
        return t, MessageContainer(msg)
    
class JointInfoGenerator(CsvDataGenerator):
    topic = '/joint_states'
    
    _JOINT_NAMES = ['mico_joint_1', 'mico_joint_2', 'mico_joint_3', 'mico_joint_4', 
                    'mico_joint_5', 'mico_joint_6', 'mico_joint_finger_1',
                    'mico_joint_finger_2']
    def __init__(self, base_dir, *args):
        filename = os.path.join(base_dir, SubDirs.TEXT_DIR, 'joint_positions.csv')
        CsvDataGenerator.__init__(self, filename, *args)
        
    def build_message(self, vals):
        t = rospy.Time.from_sec(float(vals['timestamp']))
        
        pos = [ float(vals[j + '_pos']) for j in JointInfoGenerator._JOINT_NAMES ]
        vel = [ float(vals[j + '_vel']) for j in JointInfoGenerator._JOINT_NAMES ]
        eff = [ float(vals[j + '_eff']) for j in JointInfoGenerator._JOINT_NAMES ]
        msg = sensor_msgs.msg.JointState(
            name = JointInfoGenerator._JOINT_NAMES,
            position = pos,
            velocity = vel,
            effort = eff
            )
        msg.header.stamp = t
        return t, MessageContainer(msg)
    
class RobotSkeletonGenerator(CsvDataGenerator):
    topic = '/visualization_marker'
    
    _POSITION_NAMES = ['mico_link_base', 'mico_link_1', 'mico_link_2', 'mico_link_3', 
                    'mico_link_4', 'mico_link_5', 'mico_link_hand',
                    'mico_end_effector', 'mico_fork_tip']
    
    def __init__(self, base_dir, *args):
        filename = os.path.join(base_dir, SubDirs.TEXT_DIR, 'robot_position.csv')
        CsvDataGenerator.__init__(self, filename, *args)
        
    def build_message(self, vals):
        t = rospy.Time.from_sec(float(vals['timestamp']))
        
        msg = visualization_msgs.msg.Marker(
            id = 100,
            ns = 'robot_skeleton',
            type = visualization_msgs.msg.Marker.LINE_STRIP,
            action = visualization_msgs.msg.Marker.MODIFY,
            pose = geometry_msgs.msg.Pose(
                    position = geometry_msgs.msg.Point(0., 0., 0.),
                    orientation = geometry_msgs.msg.Quaternion(0., 0., 0., 1.)
                ),
            scale = geometry_msgs.msg.Vector3(0.01, 0., 0.),
            points = [ geometry_msgs.msg.Point(float(vals[j + '_x']), float(vals[j + '_y']), float(vals[j + '_z'])) for j in RobotSkeletonGenerator._POSITION_NAMES ],
            color = std_msgs.msg.ColorRGBA(r=0, g=1, b=0, a=1)
            )
        msg.header.stamp = t
        msg.header.frame_id = 'robot_base'
        return t, MessageContainer(msg)
        

ros_myo = None
class MyoEmgGenerator(CsvDataGenerator):
    topic = '/myo_emg'
    
    def __init__(self, base_dir, *args):
        filename = os.path.join(base_dir, SubDirs.TEXT_DIR, 'myo_emg.csv')
        CsvDataGenerator.__init__(self, filename, *args)
        
    def build_message(self, vals):
        global ros_myo
        if not ros_myo:
            import ros_myo.msg
        t = rospy.Time.from_sec(float(vals['timestamp']))
        msg = ros_myo.msg.EmgArray(
            data = [float(vals['emg{}'.format(n)]) for n in range(8)],
            moving = bool(vals['moving'])
            )
        return t, MessageContainer(msg)


harmonic_playback = None
class GoalProbabilityGenerator(CsvDataGenerator):
    topic = '/probabilities'
    
    def __init__(self, base_dir, *args):
        filename = os.path.join(base_dir, SubDirs.TEXT_DIR, 'assistance_info.csv')
        CsvDataGenerator.__init__(self, filename, *args)
        
        with open(os.path.join(base_dir, SubDirs.TEXT_DIR, 'morsel.yaml'), 'rb') as f:
            morsels = yaml.load(f)
        self.morsel_names = [ k for k in morsels.keys() if k.startswith('morsel') ]
    
    def build_message(self, vals):
        global harmonic_playback
        if harmonic_playback is None:
            import harmonic_playback.msg
        t = rospy.Time.from_sec(float(vals['timestamp']))
        msg = harmonic_playback.msg.ProbabilityUpdate(
            names = self.morsel_names,
            probabilities = [ float(vals['p_goal_{}'.format(i)]) for i in range(len(self.morsel_names)) ]
            )
        return t, MessageContainer(msg)

class EgoTransformGenerator(CsvDataGenerator):
    topic = '/tf'
    
    def __init__(self, base_dir, *args):
        filename = os.path.join(base_dir, SubDirs.PROC_DIR, 'world_camera_pose.csv')
        CsvDataGenerator.__init__(self, filename, *args)
    def build_message(self, vals):
        t = rospy.Time.from_sec(float(vals['timestamp']))
        tf_msg = geometry_msgs.msg.TransformStamped()
        tf_msg.header.stamp = t
        tf_msg.header.frame_id = 'mico_link_base'
        tf_msg.child_frame_id = 'pupil_world'
        
        # build up a transform so we can invert it
        tf_mat = tf.quaternion_matrix([float(vals['qx']), float(vals['qy']), float(vals['qz']), float(vals['qw'])])
        tf_mat[:3,3] = [float(vals['x']), float(vals['y']), float(vals['z'])]
        tf_mat_inv = np.linalg.inv(tf_mat)
        
        tf_msg.transform.translation = geometry_msgs.msg.Vector3(*tf_mat[:3,3])
        tf_msg.transform.rotation = geometry_msgs.msg.Quaternion(*tf.quaternion_from_matrix(tf_mat))
        
        tf_msg_inv = geometry_msgs.msg.TransformStamped()
        tf_msg_inv.header.stamp = t
        tf_msg_inv.header.frame_id = 'mico_link_base'
        tf_msg_inv.child_frame_id = 'pupil_world_inv'
        tf_msg_inv.transform.translation = geometry_msgs.msg.Vector3(*tf_mat_inv[:3,3])
        tf_msg_inv.transform.rotation = geometry_msgs.msg.Quaternion(*tf.quaternion_from_matrix(tf_mat_inv))
        
        return t, MessageContainer(tf2_msgs.msg.TFMessage([tf_msg, tf_msg_inv]))

class GazeDataGenerator(MultiCsvDataGenerator):
    topic = '/gaze'
    def __init__(self, base_dir, *args):
        filenames = [os.path.join(base_dir, SubDirs.TEXT_DIR, 'gaze_positions.csv'),
                     os.path.join(base_dir, SubDirs.TEXT_DIR, 'pupil_eye0.csv'),
                     os.path.join(base_dir, SubDirs.TEXT_DIR, 'pupil_eye1.csv')]
        
        GazeDataGenerator.__selectors = [GazeDataGenerator.select_world, GazeDataGenerator.select_eye, GazeDataGenerator.select_eye]
        MultiCsvDataGenerator.__init__(self, filenames, *args)
    
    @staticmethod
    def select_world(vals):
        return (float(vals['norm_pos_x']), float(vals['norm_pos_y']), np.nan)
    
    @staticmethod
    def select_eye(vals):
        return (float(vals['circle_3d_normal_x']), float(vals['circle_3d_normal_y']), float(vals['circle_3d_normal_z']))
        
    def build_message(self, vals, idx):
        msg = ibmmpy.msg.GazeDataPoint(
                confidence=float(vals['confidence']),
                position=geometry_msgs.msg.Point(*GazeDataGenerator.__selectors[idx](vals)))
        t = rospy.Time.from_sec(float(vals['timestamp']))
        msg.header.stamp = t
        return t, msg
    
    def combine_messages(self, msgs):
        t = np.max([[m[0] for ms in msgs for m in ms]])
        container = MessageContainer(
            ibmmpy.msg.GazeData(
                    world_data=[x[1] for x in msgs[0]],
                    eye0_data=[x[1] for x in msgs[1]],
                    eye1_data=[x[1] for x in msgs[2]]
                )
            )
        return t, container
        

DATA_GENERATORS = {
        'joystick': JoystickDataGenerator,
        'joint': JointInfoGenerator,
        'myo_emg': MyoEmgGenerator,
        'goal_probabilities': GoalProbabilityGenerator,
        'robot_skeleton': RobotSkeletonGenerator,
        'ego_pose': EgoTransformGenerator,
        'gaze': GazeDataGenerator
    }
MORSEL_COMPONENT = 'morsel_tfs'
ALL_COMPONENTS = DATA_GENERATORS.keys()
ALL_COMPONENTS.extend([
    MORSEL_COMPONENT
    ])


def generate_morsel_tfs(base_dir):
    with open(os.path.join(base_dir, SubDirs.TEXT_DIR, 'morsel.yaml'), 'r') as f:
        morsel_data = yaml.load(f)
        
    morsel_data = {k: v for k, v in morsel_data.iteritems() if k.startswith('morsel') and v is not None}
    
    with open(os.path.join(base_dir, SubDirs.PROC_DIR, 'playback', 'morsel_transforms.launch'), 'w') as launch_file:
        launch_file.write('<launch>\n')
        for morsel, transform in morsel_data.iteritems():
            tf_mat = np.array(transform)
            trans = tf.translation_from_matrix(tf_mat)
            quat = tf.quaternion_from_matrix(tf_mat)
            launch_file.write('    <node pkg="tf2_ros" type="static_transform_publisher" name="{morsel}_tf_pub" args="{x} {y} {z} {qx} {qy} {qz} {qw} world {morsel}" />\n'.format(
                morsel=morsel, x=trans[0], y=trans[1], z=trans[2], qx=quat[0], qy=quat[1], qz=quat[2], qw=quat[3]
                ))
        launch_file.write('</launch>\n')
        
    

def build_bag(base_dir, hz=10, keys=DATA_GENERATORS.keys(), output_file=None): 
    run_stats_file = os.path.join(base_dir, SubDirs.STATS_DIR, 'run_info.yaml')
    with open(run_stats_file, 'rb') as f:
        d = yaml.load(f)
        run_stats = { r[0]: r[1] for r in d }
    
    if hz is not None:
        sample_ts = np.arange(float(run_stats['start_time']), float(run_stats['end_time']), 1./hz)
    else:
        sample_ts = None
        
    print('Directory: {}'.format(base_dir))
    if MORSEL_COMPONENT in keys:
        print('\t{}:'.format(MORSEL_COMPONENT))
        generate_morsel_tfs(base_dir)
        keys.remove(MORSEL_COMPONENT)
    if len(keys) == 0:
        return
    
    bag_file = output_file or os.path.join(base_dir, SubDirs.PROC_DIR, 'playback/viz_data.bag')
    with rosbag.Bag(bag_file, 'w') as bag:
        for key in keys:
            print('\t{}:'.format(key))
            data_generator = DATA_GENERATORS[key]
            try:
                with progressbar.ProgressBar(max_value=sample_ts[-1]-sample_ts[0]) as bar:
                    for t, msg in data_generator(base_dir, sample_ts):
                        bag.write(data_generator.topic, msg.msg, t=t)
                        bar.update(min(max(t.to_sec()-sample_ts[0], 0), sample_ts[-1]-sample_ts[0]))
            except StopIteration:
                pass
            
def downsample_video(in_vid, in_ts, out_vid, out_ts, factor):
    in_video = cv2.VideoCapture(in_vid)
    ts = np.load(in_ts)
    down_video = cv2.VideoWriter(out_vid, int(in_video.get(cv2.CAP_PROP_FOURCC)), 
                                        in_video.get(cv2.CAP_PROP_FPS)/factor, 
                                        (int(in_video.get(cv2.CAP_PROP_FRAME_WIDTH)), int(in_video.get(cv2.CAP_PROP_FRAME_HEIGHT))), 
                                        True)
    down_ts = ts[np.arange(0, len(ts), factor)]
    print('Downsampling video {} by {}X:'.format(in_vid, factor))
    with progressbar.ProgressBar(max_value=len(ts)) as bar:
        for i in range(len(ts)):
            ok, data = in_video.read()
            if ok:
                if i % factor == 0:
                    down_video.write(data)
            else:
                break
            bar.update(i)
    np.save(out_ts, down_ts)    

def process_videos(base_dir, hz=10):
    vid_dir = os.path.join(base_dir, SubDirs.VID_DIR)
    proc_dir = os.path.join(base_dir, SubDirs.PROC_DIR)
    
    downsample_video(in_vid=os.path.join(proc_dir, 'gaze_overlay.mp4'),
                     in_ts=os.path.join(vid_dir, 'world_timestamps.npy'),
                     out_vid=os.path.join(proc_dir, 'playback/world.mp4'),
                     out_ts=os.path.join(proc_dir, 'playback/world_timestamps.npy'),
                     factor = int(30 / hz)) 
    
    downsample_video(in_vid=os.path.join(vid_dir, 'eye0.mp4'),
                     in_ts=os.path.join(vid_dir, 'eye0_timestamps.npy'),
                     out_vid=os.path.join(proc_dir, 'playback/eye0.mp4'),
                     out_ts=os.path.join(proc_dir, 'playback/eye0_timestamps.npy'),
                     factor = int(120 / hz)) 
    
    downsample_video(in_vid=os.path.join(vid_dir, 'eye1.mp4'),
                     in_ts=os.path.join(vid_dir, 'eye1_timestamps.npy'),
                     out_vid=os.path.join(proc_dir, 'playback/eye1.mp4'),
                     out_ts=os.path.join(proc_dir, 'playback/eye1_timestamps.npy'),
                     factor = int(120 / hz)) 
    
    if os.path.isfile(os.path.join(vid_dir, 'zed_left.avi')):
        downsample_video(in_vid=os.path.join(vid_dir, 'zed_left.avi'),
                         in_ts=os.path.join(vid_dir, 'zed_timestamps.npy'),
                         out_vid=os.path.join(proc_dir, 'playback/zed_left.avi'),
                         out_ts=os.path.join(proc_dir, 'playback/zed_timestamps.npy'),
                         factor = int(30 / hz)) 
                    
def main():   
    parser = argparse.ArgumentParser('Build playback bag file')
    parser.add_argument('export_dir')
    parser.add_argument('--hz', default=10, type=float)
    parser.add_argument('--components', nargs='+', default=ALL_COMPONENTS, choices=ALL_COMPONENTS, help='Which components to use')
    parser.add_argument('--disable-videos', default=False, action='store_true', help='Disable video generation')
    parser.add_argument('--output-file', default=None, help='Output file (defaults to $data_dir/processed/playback/viz_data.bag)')
    args = parser.parse_args()
    
    for base_dir, dirs, _ in os.walk(args.export_dir):
        if SubDirs.TEXT_DIR in dirs:
            make_dir(os.path.join(base_dir, SubDirs.PROC_DIR, 'playback'))
            build_bag(base_dir, hz=args.hz, keys=args.components, output_file=args.output_file)
            if not args.disable_videos:
                process_videos(base_dir, hz=args.hz)
            
if __name__ == "__main__":
    main()
            
                
