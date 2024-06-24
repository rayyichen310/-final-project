import rospy
import tf2_ros
from costmap_2d import Costmap2DROS
from path_planning import PathPlanning

def main():
    rospy.init_node('path_planning')

    tf_buffer = tf2_ros.Buffer()
    tf2_listener = tf2_ros.TransformListener(tf_buffer)
    lcr = Costmap2DROS("cleaning_costmap", tf_buffer)

    rospy.sleep(5)
    clr = PathPlanning(lcr)
    clr.getPathInROS()  # 根據格柵地圖做路徑規劃

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():  # 循環發布規劃好的路徑
        clr.publishCoveragePath()
        rospy.spinOnce()
        rate.sleep()

    rospy.signal_shutdown('Shutting down')

if __name__ == '__main__':
    main()
