#Specifies each object ID
int64 object_id

#Specifies the minimum angle detected from the polygon
float32 min_angle

#Specifies the maximum angle detected fom the polygon
float32 max_angle

#Distance to polygon
float32 distance

#Frame ID
string frame_id 

#Specifices if the object is static or dynamic 
bool is_dynamic 

#Time stamp at creation, (could be time timestamp)
builtin_interfaces/Time time_created
