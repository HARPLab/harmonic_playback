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
import harmonic_playback.msg
import ros_myo.msg
import sensor_msgs.msg
import std_msgs.msg
import visualization_msgs.msg

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
    def __init__(self, filename):
        self.filename = filename
        self._seq = 0
    
    def __iter__(self):
        with open(self.filename, 'rb') as f:
            reader = csv.DictReader(f)
            for vals in reader:
                t, msg = self.build_message(vals)
                if hasattr(msg.msg, 'header'):
                    msg.msg.header.seq = self._seq
                    self._seq += 1
                yield t, msg
    
    def build_message(self, vals):
        raise NotImplementedError()
        

class JoystickDataGenerator(CsvDataGenerator):
    topic = '/joystick'
    def __init__(self, base_dir):
        filename = os.path.join(base_dir, SubDirs.TEXT_DIR, 'ada_joy.csv')
        CsvDataGenerator.__init__(self, filename)
    
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
    def __init__(self, base_dir):
        filename = os.path.join(base_dir, SubDirs.TEXT_DIR, 'joint_positions.csv')
        CsvDataGenerator.__init__(self, filename)
        
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
    
    def __init__(self, base_dir):
        filename = os.path.join(base_dir, SubDirs.TEXT_DIR, 'robot_position.csv')
        CsvDataGenerator.__init__(self, filename)
        
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
        
    
class MyoEmgGenerator(CsvDataGenerator):
    topic = '/myo_emg'
    
    def __init__(self, base_dir):
        filename = os.path.join(base_dir, SubDirs.TEXT_DIR, 'myo_emg.csv')
        CsvDataGenerator.__init__(self, filename)
        
    def build_message(self, vals):
        t = rospy.Time.from_sec(float(vals['timestamp']))
        msg = ros_myo.msg.EmgArray(
            data = [float(vals['emg{}'.format(n)]) for n in range(8)],
            moving = bool(vals['moving'])
            )
        return t, MessageContainer(msg)
    
class GoalProbabilityGenerator(CsvDataGenerator):
    topic = '/probabilities'
    
    def __init__(self, base_dir):
        filename = os.path.join(base_dir, SubDirs.TEXT_DIR, 'assistance_info.csv')
        CsvDataGenerator.__init__(self, filename)
        
        with open(os.path.join(base_dir, SubDirs.TEXT_DIR, 'morsel.yaml'), 'rb') as f:
            morsels = yaml.load(f)
        self.morsel_names = [ k for k in morsels.keys() if k.startswith('morsel') ]
    
    def build_message(self, vals):
        t = rospy.Time.from_sec(float(vals['timestamp']))
        msg = harmonic_playback.msg.ProbabilityUpdate(
            names = self.morsel_names,
            probabilities = [ float(vals['p_goal_{}'.format(i)]) for i in range(len(self.morsel_names)) ]
            )
        return t, MessageContainer(msg)
    
    
# class VideoDataGenerator:
#     def __init__(self, vid_name, ts_name):
#         self.vid_name = vid_name
#         self.ts = np.load(ts_name)
#         
#     class ImageMessageContainer:
#         def __init__(self, bridge, reader):
#             self._bridge = bridge
#             self._reader = reader
#         
#         @property
#         def msg(self):
#             _, img = self._reader.retrieve()
#             return self._bridge.cv2_to_imgmsg(img, encoding='bgr8')
#     
#     def __iter__(self):
#         video_reader = cv2.VideoCapture(self.vid_name)
#         bridge = cv_bridge.CvBridge()
#         for t in self.ts:
#             ret = video_reader.grab()
#             if not ret:
#                 return
#             t = rospy.Time.from_sec(t)
#             yield t, VideoDataGenerator.ImageMessageContainer(bridge, video_reader)
#         self.video_reader.release()
    

# class EgoVideoDataGenerator(VideoDataGenerator):
#     topic = '/ego_video'
#     def __init__(self, base_dir):
#         vid_name = os.path.join(base_dir, build_data_stats.SubDirs.VID_DIR, 'world.mp4')
#         ts_name = os.path.join(base_dir, build_data_stats.SubDirs.VID_DIR, 'world_timestamps.npy')
#         VideoDataGenerator.__init__(self, vid_name, ts_name)
        


DATA_GENERATORS = {
        'joystick': JoystickDataGenerator,
        'joint': JointInfoGenerator,
        'myo_emg': MyoEmgGenerator,
        'goal_probabilities': GoalProbabilityGenerator,
        'robot_skeleton': RobotSkeletonGenerator
    }

def build_bag(base_dir, hz=10, keys=DATA_GENERATORS.keys()): 
    run_stats_file = os.path.join(base_dir, SubDirs.STATS_DIR, 'run_info.yaml')
    with open(run_stats_file, 'rb') as f:
        d = yaml.load(f)
        run_stats = { r[0]: r[1] for r in d }
    
    sample_ts = np.arange(float(run_stats['start_time']), float(run_stats['end_time']), 1./hz)
    
    print('Directory: {}'.format(base_dir))
    bag_file = os.path.join(base_dir, SubDirs.PROC_DIR, 'playback/viz_data.bag')
    with rosbag.Bag(bag_file, 'w') as bag:
        for key in keys:
            print('\t{}:'.format(key))
            data_generator = DATA_GENERATORS[key]
            try:
                with progressbar.ProgressBar(max_value=sample_ts[-1]-sample_ts[0]) as bar:
                    iter_t = iter(sample_ts)
                    next_t = next(iter_t)
                    for t, msg in data_generator(base_dir):
                        if t.to_sec() >= next_t:
                            bag.write(data_generator.topic, msg.msg, t=t)
                            next_t = next(iter_t)
                            bar.update(next_t-sample_ts[0])
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
    parser.add_argument('--components', nargs='+', default=DATA_GENERATORS.keys(), help='Which components to use ({})'.format(DATA_GENERATORS.keys()))
    parser.add_argument('--disable-videos', default=False, action='store_true', help='Disable video generation')
    args = parser.parse_args()
    
    for base_dir, dirs, _ in os.walk(args.export_dir):
        if SubDirs.TEXT_DIR in dirs:
            make_dir(os.path.join(base_dir, SubDirs.PROC_DIR, 'playback'))
            build_bag(base_dir, hz=args.hz, keys=args.components)
            if not args.disable_videos:
                process_videos(base_dir, hz=args.hz)
            
if __name__ == "__main__":
    main()
            
                