# Message that contains a list of detected objects - ie warnings 
# Each PyTrackerMsg object has a min_angle, max_angle, time stamp, frame_id, and is_static field

std_msgs/Header header

py_tracker_msg/PyTrackerMsg [] warnings 