# Multi-zone ToF distances from Parsec (mm). -1 = disabled, -2 = invalid.
std_msgs/Header header
uint8 seq
uint8 zones          # 16 or 64
int16[] fdist        # length == zones