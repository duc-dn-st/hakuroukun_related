#!/usr/bin/env python3
#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist

class KengakuNode:
    def __init__(self):
        
        rospy.init_node("kengaku_run_node", anonymous=True)

        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def _stop(self):

        pub_msg = Twist()

        pub_msg.linear.x = 0.0

        pub_msg.angular.x = 0.0

        self.vel_pub.publish(pub_msg)

    def _go_straight(self):

        pub_msg = Twist()

        pub_msg.linear.x = 1.2

        pub_msg.angular.x = 0.0


        self.vel_pub.publish(pub_msg)

    def _go_backward(self):

        pub_msg = Twist()

        pub_msg.linear.x = -1.5

        pub_msg.angular.x = 0.0

        self.vel_pub.publish(pub_msg)

    def _run_demo(self):

        rate = rospy.Rate(10) # 10hz

        now = time.time()        
        while time.time() - now < 13:
            self._go_straight()
            rospy.loginfo("running foward")
            rate.sleep()

        now = time.time()
        while time.time() - now < 2:
            self._stop()
            rospy.loginfo("Stop")
            rate.sleep()
        
        now = time.time()
        while time.time() - now < 18:
            self._go_backward()
            rospy.loginfo("running backward")
            rate.sleep()

        now = time.time()
        while time.time() - now < 2:
            self._stop()
            rospy.loginfo("Stop")
            rate.sleep()

if __name__ == "__main__":
    
    kn = KengakuNode()
    try:
        kn._run_demo()
    except rospy.ROSInterruptException:
        pass