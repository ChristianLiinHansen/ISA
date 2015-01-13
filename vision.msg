# The distance r and the angle theta which is detected by vision
# Header contains <time>stamp, <string>frame_id and <uint32>seq
# Use like msg.header.stamp = ros::Time::now(); // This creates a timestamped...
Header header

float64 r
float64 theta

# The end of line detection
bool end_of_line
