import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def main():
    # 初始化 ROS node
    rospy.init_node('test_path')
    n = rospy.NodeHandle()

    # 創建一個路徑pub
    path_pub = rospy.Publisher('plan_path', Path, queue_size=10)

    # 創建路徑消息
    path = Path()
    path.header.frame_id = "map"  
    path.header.stamp = rospy.Time.now()

    # 添加路徑點
    for i in range(10):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()

        pose.pose.position.x = i  
        pose.pose.position.y = i
        pose.pose.orientation.w = 1.0

        path.poses.append(pose)

    # 發布路徑
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        path_pub.publish(path)
        rospy.spinOnce()
        rate.sleep()

if __name__ == '__main__':
    main()
