import rospy
import math
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler


x_current = 0.0
y_current = 0.0
normeNextGoal = 0.0  


def eulerAngles2Quaternion(yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cr = math.cos(0 * 0.5)
    sr = math.sin(0 * 0.5)
    cp = math.cos(0 * 0.5)
    sp = math.sin(0 * 0.5)

    w = cy * cr * cp + sy * sr * sp
    x = cy * sr * cp - sy * cr * sp
    y = cy * cr * sp + sy * sr * cp
    z = sy * cr * cp - cy * sr * sp

    return w, x, y, z

class PathPlanner:
    class Goal:
        def __init__(self, x, y, visited):
            self.x = x
            self.y = y
            self.visited = visited

    def __init__(self):
        self.Path = []

    def addGoal(self, x, y, visit):
        newGoal = PathPlanner.Goal(x, y, visit)
        self.Path.append(newGoal)

pathPlanner = PathPlanner()
passedPath = Path()
pubPassedPath = None

def pose_callback(poses):
    global x_current, y_current, passedPath, pubPassedPath
    x_current = poses.pose.pose.position.x
    y_current = poses.pose.pose.position.y
    passedPath.header = poses.header
    p = PoseStamped()
    p.header = poses.header
    p.pose = poses.pose.pose
    passedPath.poses.append(p)
    pubPassedPath.publish(passedPath)

taille_last_path = 0
new_path = False

def path_callback(path):
    global taille_last_path, new_path, pathPlanner
    if len(pathPlanner.Path) == 0 or len(path.poses) != taille_last_path:
        pathPlanner.Path.clear()
        new_path = True
        for pose in path.poses:
            pathPlanner.addGoal(pose.pose.position.x, pose.pose.position.y, False)
        taille_last_path = len(path.poses)

def main():
    global normeNextGoal, new_path, pathPlanner, pubPassedPath
    rospy.init_node('next_goal')
    rospy.Subscriber('/odom', Odometry, pose_callback)
    rospy.Subscriber('/plan_path', Path, path_callback)

    pub1 = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1000)
    pubPassedPath = rospy.Publisher('passedPath', Path, queue_size=1000)

    loop_rate = rospy.Rate(10)
    goal_msgs = PoseStamped()
    count = 0
    goal_reached = False

    if not rospy.get_param("/NextGoal/tolerance_goal", normeNextGoal):
        rospy.logerr("Please set your tolerance_goal")
        return

    while not rospy.is_shutdown():
        rospy.spinOnce()
        if new_path:
            count = 0
            new_path = False

        if pathPlanner.Path:
            if math.sqrt((x_current - pathPlanner.Path[count].x)**2 + (y_current - pathPlanner.Path[count].y)**2) <= normeNextGoal:
                count += 1
                goal_reached = False
            
            if not goal_reached:
                goal_msgs.header.frame_id = "odom"
                goal_msgs.header.stamp = rospy.Time.now()
                goal_msgs.pose.position.x = pathPlanner.Path[count].x
                goal_msgs.pose.position.y = pathPlanner.Path[count].y
                goal_msgs.pose.position.z = 0

                if count < len(pathPlanner.Path) - 1:
                    angle = math.atan2(pathPlanner.Path[count + 1].y - pathPlanner.Path[count].y, pathPlanner.Path[count + 1].x - pathPlanner.Path[count].x)
                else:
                    angle = math.atan2(pathPlanner.Path[0].y - pathPlanner.Path[count].y, pathPlanner.Path[0].x - pathPlanner.Path[count].x)
                
                w, x, y, z = eulerAngles2Quaternion(angle)
                goal_msgs.pose.orientation.w = w
                goal_msgs.pose.orientation.x = x
                goal_msgs.pose.orientation.y = y
                goal_msgs.pose.orientation.z = z

                goal_reached = True
                pub1.publish(goal_msgs)

        loop_rate.sleep()

if __name__ == '__main__':
    main()
