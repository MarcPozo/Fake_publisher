#!/usr/bin/env python

import rospy

import dynamic_reconfigure.client

def callback(config):

    global y
    y = config['int_param']
    print(y)

if __name__ == "__main__":
    rospy.init_node("dynamic_client")

    y = 0
    client = dynamic_reconfigure.client.Client("dynamic_tutorials", timeout=30, config_callback=callback)

    r = rospy.Rate(0.1)
    x = 0
    b = False
    while not rospy.is_shutdown():
        #client.update_configuration({"int_param":x, "double_param":(1/(x+1)), "str_param":str(rospy.get_rostime()), "bool_param":b, "size":1})
	print(y)
        r.sleep()
