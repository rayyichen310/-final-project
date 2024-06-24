#!/usr/bin/env python3
import rospy
import roslaunch
import signal
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Point, Quaternion

# 向explore伺服器發送終止請求
def stop_explore_lite():
    pass

# save map 
def save_map():
    # 獲取amcl的launch文件路徑
    launch_file_path = roslaunch.rlutil.resolve_launch_arguments(['auto_nav', 'save_map.launch'])
    # create Launch
    launch = roslaunch.parent.ROSLaunchParent(rospy.get_param('/run_id'), launch_file_path)
    # 啟動launch文件
    launch.start()
    try:
        # 等待節點結束
        launch.spin()
    finally:
        # 關閉launch
        launch.shutdown()

# 啟動amcl返航
def run_amcl():
    # 獲取amcl的launch文件路徑
    launch_file_path = roslaunch.rlutil.resolve_launch_arguments(['auto_nav', 'amcl.launch'])
    # 創建Launch
    launch = roslaunch.parent.ROSLaunchParent(rospy.get_param('/run_id'), launch_file_path)
    # 啟動launch文件
    launch.start()
    try:
        # 等待節點結束
        launch.spin()
    finally:
        # 關閉launch
        launch.shutdown()

class GoalMonitorNode:
    def __init__(self):
        rospy.init_node('return_to_start', anonymous=True)

        # 處理SIGINT和SIGTERM信號
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)

        # 訂閱/move_base/goal話題
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_status_callback)
        
        # 定義pub
        self.goal_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)

        # set timer ，每秒觸發一次回調函數
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)

        # 記錄上次收到消息的時間
        self.last_goal_time = rospy.Time.now()

        rospy.spin()

    def signal_handler(self, signum, frame):
        rospy.loginfo("Received signal {}, shutting down...".format(signum))
        # 實現清理或其他關閉邏輯（如果需要）
        rospy.signal_shutdown("Received signal, shutting down")

    def goal_status_callback(self, msg):
        # 更新上次收到消息的時間
        self.last_goal_time = rospy.Time.now()

    def timer_callback(self, event):
        # 檢查是否超過20秒沒有收到消息
        if rospy.Time.now() - self.last_goal_time > rospy.Duration(25):
            rospy.loginfo("No goal received in the last 20 seconds. Sending a new goal.")

            stop_explore_lite()
            rospy.loginfo("1")
            rospy.sleep(1.0)    # 確保停止
            rospy.loginfo("2")
            save_map()
            rospy.loginfo("3")
            #run_amcl()

            # 創建返航的目標消息
            new_goal = MoveBaseActionGoal()
            new_goal.goal_id.stamp = rospy.Time.now()
            new_goal.goal.target_pose.header.frame_id = 'map'
            new_goal.goal.target_pose.pose.position = Point(x=-8.0, y=0.0, z=0.0)
            new_goal.goal.target_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            # 發布新的目標消息到/move_base/goal話題
            self.goal_publisher.publish(new_goal)

if __name__ == '__main__':
    try:
        
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)

        node = GoalMonitorNode()
    except rospy.ROSInterruptException:
        pass
