#!/usr/bin/env python

import cv2
import Queue
import threading
import numpy as np

import rospy
import sensor_msgs.msg
import cv_bridge
from hgext.zeroconf import publish

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
    def __init__(self, cap, ts, queue_size=1):
        self.cap = cap
        self.ts = ts
        self._queue = Queue.LifoQueue(maxsize=queue_size)
        self._thread = threading.Thread(target=self.run)
        self._thread.start()
        self._is_finished = False
        
    def run(self):
        for t in self.ts:
            ok, img = self.cap.read()
            if ok:
                item = (t, img)
                placed = False
                while not placed:
                    if rospy.is_shutdown():
                        break
                    try:
                        self._queue.put(item, block=True, timeout=1.)
                        placed = True
                    except Queue.Full:
                        pass
            else:
                break
        self._is_finished = True
        
    def get(self, *args, **kwargs):
        return self._queue.get(*args, **kwargs)
    
    @property
    def is_finished(self):
        return self._is_finished
    
def publish_image():
    rospy.init_node('image_publisher')
    provider = rospy.get_param('~video_stream_provider')
    cap = cv2.VideoCapture(provider)
    
    queue_size = rospy.get_param('~queue_size', default=10)
    ts = np.load(rospy.get_param('~ts_file'))
    queue = ImageQueue(cap, ts, queue_size)
    
    topic = rospy.get_param('~camera_name')
    pub = rospy.Publisher(topic, sensor_msgs.msg.Image, queue_size=1)
    
    bridge = cv_bridge.CvBridge()
        
    while not rospy.is_shutdown() and not queue.is_finished:
        try:
            rospy.loginfo("Waiting for next frame")
            t, img_raw = queue.get(block=True, timeout=1.)
            tm = rospy.Time.from_sec(t)
            cur_tm = rospy.get_rostime()
            if tm < cur_tm:
                rospy.loginfo("Video frame time has passed already (offset={}), discarding".format( (cur_tm - tm).to_sec()))
                continue
            rospy.loginfo("got frame for time: {}".format(tm))
            
            img = bridge.cv2_to_imgmsg(img_raw, encoding="bgr8")
            img.header.stamp = tm
            
            while cur_tm < tm:
                sleep_len = min(tm - cur_tm, rospy.Duration(1))
                rospy.loginfo("Waiting {} to publish frame (total: {})".format(sleep_len.to_sec(), (tm - cur_tm).to_sec()))
                try:
                    rospy.sleep(sleep_len)
                except rospy.exceptions.ROSTimeMovedBackwardsException:
                    pass
                cur_tm = rospy.get_rostime()
            pub.publish(img)
            
        except Queue.Empty:
            pass 
    
    
if __name__ == "__main__":
    publish_image()

