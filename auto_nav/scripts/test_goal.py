import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def main():
    # 初始化 ROS 節點
    rospy.init_node('test_goal')

    # 告訴 action cli，我們想要連接到 move_base 伺服器
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # 等待 move_base action 伺服器啟動
    rospy.loginfo("Waiting for the move_base action server")
    ac.wait_for_server()

    # 設置目標
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # 定義目標位置和方向
    goal.target_pose.pose.position.x = 0.0  
    goal.target_pose.pose.position.y = 1.0
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo("Sending goal")
    ac.send_goal(goal)

    # 等待目標完成
    ac.wait_for_result()

    if ac.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("The base moved to the goal position")
    else:
        rospy.loginfo("The base failed to move to the goal position")

if __name__ == '__main__':
    main()
