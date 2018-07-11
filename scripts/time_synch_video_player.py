#!/usr/bin/env python

import cv2
import Queue
import threading
import numpy as np

import rospy
import sensor_msgs.msg
import cv_bridge

import rosgraph_msgs.msg

# def get_default_camera_info_from_image(img):
#     cam_info = sensor_msgs.msg.CameraInfo()
#     cam_info.header.frame_id = img.header.frame_id
#     cam_info.height = img.height
#     cam_info.width = img.width
#     cam_info.distortion_model = "plumb_bob"
#     cam_info.D = [0., 0., 0., 0., 0.]
#     cam_info.K = [1., 0., img.width/2, 
#                   0., 1., img.height/2., 
#                   0., 0., 1.]
#     cam_info.R = [1., 0., 0.,
#                   0., 1., 0.,
#                   0., 0., 1.]
#     cam_info.P = [1., 0., img.width/2., 0.,
#                   0., 1., img.height/2., 0.,
#                   0., 0., 1., 0.]
#     return cam_info

class ImageQueue:
    def __init__(self, cap, ts, queue_size=1, flip_code=None):
        self.cap = cap
        self.ts = ts
        self.flip_code = flip_code
        self._queue = Queue.LifoQueue(maxsize=queue_size)
        self._thread = threading.Thread(target=self.run)
        self._thread.start()
        self._is_finished = False
        
    def run(self):
        for t in self.ts:
            ok, img = self.cap.read()
            if self.flip_code is not None:
                img = cv2.flip(img, self.flip_code)
            if ok:
                item = (t, img)
                placed = False
                while not placed:
                    if rospy.is_shutdown() or self._is_finished:
                        break
                    try:
                        self._queue.put(item, block=True, timeout=1.)
                        placed = True
                    except Queue.Full:
                        pass
            else:
                break
            if rospy.is_shutdown() or self._is_finished:
                break
        self._is_finished = True
        
    def get(self, *args, **kwargs):
        return self._queue.get(*args, **kwargs)
    
    def stop(self):
        self._is_finished = True
        self._thread.join()
        
    
    @property
    def is_finished(self):
        return self._is_finished

class SimClockDelay:
    def __init__(self):
        self.sub = rospy.Subscriber('/clock', rosgraph_msgs.msg.Clock, self._callback)
        self.cond = threading.Condition()
        self.next_time = None
        self.last_time = None
        
    def _callback(self, msg):
        self.last_time = msg.clock
        with self.cond:
            if self.next_time is not None and msg.clock >= self.next_time:
                self.cond.notify_all()
                
    def wait(self, tm, *args, **kwargs):
        with self.cond:
            self.next_time = tm
            self.cond.wait(*args, **kwargs)
            self.next_time = None
    
    def get_time(self):
        if self.last_time is not None:
            return self.last_time
        else:
            return rospy.Time(0,0)

def publish_image():
    rospy.init_node('image_publisher')
    provider = rospy.get_param('~video_stream_provider')
    cap = cv2.VideoCapture(provider)
    
    flip_horz = rospy.get_param('~flip_horizontal', default=False)
    flip_vert = rospy.get_param('~flip_vertical', default=False)
    if flip_horz and flip_vert:
        flip_code = -1
    elif flip_horz:
        flip_code = 1
    elif flip_vert:
        flip_code = 0
    else:
        flip_code = None
    
    queue_size = rospy.get_param('~queue_size', default=10)
    ts = np.load(rospy.get_param('~ts_file'))
    queue = ImageQueue(cap, ts, queue_size, flip_code)
    
    topic = rospy.get_param('~camera_name')
    pub = rospy.Publisher(topic + '/image_raw', sensor_msgs.msg.Image, queue_size=1)
    
    bridge = cv_bridge.CvBridge()
    
    use_manual_sim = rospy.get_param('~use_manual_sim', default=False)
    offset_from_start = rospy.get_param('~offset_from_start', default=False)
    if use_manual_sim:
        sim_clock = SimClockDelay()
        get_time = sim_clock.get_time
    elif offset_from_start:
        time_offset = rospy.Time.from_sec(ts[0]) - rospy.get_rostime()
        get_time = lambda: rospy.get_rostime() + time_offset
    else:
        get_time = rospy.get_rostime
        
    while not rospy.is_shutdown() and not queue.is_finished:
        try:
            t, img_raw = queue.get(block=True, timeout=1.)
            tm = rospy.Time.from_sec(t)
            cur_tm = get_time()
            if tm < cur_tm:
#                 rospy.loginfo("Video frame time has passed already (offset={}), discarding".format( (cur_tm - tm).to_sec()))
                continue
#             rospy.loginfo("got frame for time: {}".format(tm))
            
            img = bridge.cv2_to_imgmsg(img_raw, encoding="bgr8")
            img.header.stamp = tm
            img.header.frame_id = topic
            
            if use_manual_sim:
                done = False
                while not done:
                    try:
                        sim_clock.wait(tm, timeout=1.)
                        done = True
                    except RuntimeError:
                        pass
            else:
                while cur_tm < tm:
                    sleep_len = min(tm - cur_tm, rospy.Duration(1))
    #                 rospy.loginfo("Waiting {} to publish frame (total: {})".format(sleep_len.to_sec(), (tm - cur_tm).to_sec()))
                    rospy.sleep(sleep_len)
                    cur_tm = rospy.get_rostime()
            pub.publish(img)
            
        except Queue.Empty:
            pass
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            rospy.loginfo('ROS time move backwards, assuming the bag restarted and reloading the queue')
            queue.stop()
#             rospy.loginfo('Queue stopped')
            cap = cv2.VideoCapture(provider)
            queue = ImageQueue(cap, ts, queue_size, flip_code)
    
    
if __name__ == "__main__":
    publish_image()

