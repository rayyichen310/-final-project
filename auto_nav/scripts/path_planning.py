import rospy
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from costmap_2d import Costmap2DROS, Costmap2D
import numpy as np
import math
import cv2

class PathPlanning:
    def __init__(self, costmap2dRos):
        self.m_costmap2dRos = costmap2dRos
        self.m_costmap2d = costmap2dRos.getCostmap()

        private_nh = rospy.get_param

        self.m_planPub = rospy.Publisher("plan_path", Path, queue_size=1)                # 用於在rviz中畫出規劃路徑
        self.m_gridPub = rospy.Publisher("covered_grid", OccupancyGrid, queue_size=1)    # 用於在rviz中畫出已清掃區域

        self.m_cellSize = rospy.get_param("size_of_cell", 3)
        self.m_gridCoveredValue = rospy.get_param("grid_covered_value", 0)

        costmap2dX = self.m_costmap2d.getSizeInCellsX()
        costmap2dY = self.m_costmap2d.getSizeInCellsY()

        self.m_srcMap = np.zeros((costmap2dY, costmap2dX), dtype=np.uint8)
        for r in range(costmap2dY):
            for c in range(costmap2dX):
                self.m_srcMap[r, c] = self.m_costmap2d.getCost(c, costmap2dY - r - 1)

        self.initMat()
        self.initCoveredGrid()

        self.m_initialized = not self.m_srcMap.size == 0

    def getPathInROS(self):
        if self.m_pathVecInROS:
            self.m_pathVecInROS.clear()
        self.getPathInCV()  # 得到m_pathVec，即在CV中的路徑

        sizey = self.m_cellMat.shape[0]
        for cell in self.m_pathVec:
            pose = PoseStamped()
            x, y = self.m_costmap2d.mapToWorld(
                cell.col * self.m_cellSize + self.m_cellSize / 2,
                (sizey - cell.row - 1) * self.m_cellSize + self.m_cellSize / 2
            )
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = math.cos(cell.theta * math.pi / 180 / 2)
            pose.pose.orientation.z = math.sin(cell.theta * math.pi / 180 / 2)
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            self.m_pathVecInROS.append(pose)
        return self.m_pathVecInROS

    def initMat(self):
        self.m_cellMat = np.zeros((self.m_srcMap.shape[0] // self.m_cellSize, self.m_srcMap.shape[1] // self.m_cellSize), dtype=np.uint8)
        self.m_freeSpaceVec = []

        for r in range(self.m_cellMat.shape[0]):
            for c in range(self.m_cellMat.shape[1]):
                isFree = True
                for i in range(self.m_cellSize):
                    for j in range(self.m_cellSize):
                        if self.m_srcMap[r * self.m_cellSize + i, c * self.m_cellSize + j] != Costmap2D.FREE_SPACE:
                            isFree = False
                            break
                    if not isFree:
                        break
                if isFree:
                    cell = CellIndex(row=r, col=c, theta=0)
                    self.m_freeSpaceVec.append(cell)
                    self.m_cellMat[r, c] = Costmap2D.FREE_SPACE
                else:
                    self.m_cellMat[r, c] = Costmap2D.LETHAL_OBSTACLE

        self.m_neuralMat = np.full(self.m_cellMat.shape, 50.0, dtype=np.float32)
        self.m_neuralMat[self.m_cellMat == Costmap2D.LETHAL_OBSTACLE] = -100000.0

    def initCoveredGrid(self):
        resolution = self.m_costmap2d.getResolution()

        self.m_coveredPathGrid = OccupancyGrid()
        self.m_coveredPathGrid.header.frame_id = "map"
        self.m_coveredPathGrid.header.stamp = rospy.Time.now()
        self.m_coveredPathGrid.info.resolution = resolution
        self.m_coveredPathGrid.info.width = self.m_costmap2d.getSizeInCellsX()
        self.m_coveredPathGrid.info.height = self.m_costmap2d.getSizeInCellsY()

        wx, wy = self.m_costmap2d.mapToWorld(0, 0)
        self.m_coveredPathGrid.info.origin.position.x = wx - resolution / 2
        self.m_coveredPathGrid.info.origin.position.y = wy - resolution / 2
        self.m_coveredPathGrid.info.origin.orientation.w = 1.0

        self.m_coveredPathGrid.data = list(self.m_costmap2d.getCharMap())

    def getPathInCV(self):
        initPoint = CellIndex(theta=0)
        isok = self.m_costmap2dRos.getRobotPose(self.m_initPose)

        mx, my = self.m_costmap2d.worldToMap(self.m_initPose.pose.position.x, self.m_initPose.pose.position.y)
        initPoint.row = self.m_cellMat.shape[0] - my // self.m_cellSize - 1
        initPoint.col = mx // self.m_cellSize

        currentPoint = initPoint
        self.m_pathVec = [initPoint]

        c_0 = 50
        PI = 3.14159
        thetaVec = [0, 45, 90, 135, 180, 225, 270, 315]

        for _ in range(9000):
            self.m_neuralMat[currentPoint.row, currentPoint.col] = -250.0
            lastTheta = currentPoint.theta

            max_v = -300
            maxIndex = 0
            for id, theta in enumerate(thetaVec):
                deltaTheta = abs(theta - lastTheta)
                if deltaTheta > 180:
                    deltaTheta = 360 - deltaTheta
                e = 1 - abs(deltaTheta) / 180

                if id == 0:
                    v = self.m_neuralMat[currentPoint.row, currentPoint.col + 1] + c_0 * e if currentPoint.col != self.m_neuralMat.shape[1] - 1 else -100000
                elif id == 1:
                    v = self.m_neuralMat[currentPoint.row - 1, currentPoint.col + 1] + c_0 * e - 200 if currentPoint.col != self.m_neuralMat.shape[1] - 1 and currentPoint.row != 0 else -100000
                elif id == 2:
                    v = self.m_neuralMat[currentPoint.row - 1, currentPoint.col] + c_0 * e if currentPoint.row != 0 else -100000
                elif id == 3:
                    v = self.m_neuralMat[currentPoint.row - 1, currentPoint.col - 1] + c_0 * e - 200 if currentPoint.col != 0 and currentPoint.row != 0 else -100000
                elif id == 4:
                    v = self.m_neuralMat[currentPoint.row, currentPoint.col - 1] + c_0 * e if currentPoint.col != 0 else -100000
                elif id == 5:
                    v = self.m_neuralMat[currentPoint.row + 1, currentPoint.col - 1] + c_0 * e - 200 if currentPoint.col != 0 and currentPoint.row != self.m_neuralMat.shape[0] - 1 else -100000
                elif id == 6:
                    v = self.m_neuralMat[currentPoint.row + 1, currentPoint.col] + c_0 * e if currentPoint.row != self.m_neuralMat.shape[0] - 1 else -100000
                elif id == 7:
                    v = self.m_neuralMat[currentPoint.row + 1, currentPoint.col + 1] + c_0 * e - 200 if currentPoint.col != self.m_neuralMat.shape[1] - 1 and currentPoint.row != self.m_neuralMat.shape[0] - 1 else -100000

                if v > max_v:
                    max_v = v
                    maxIndex = id
                elif v == max_v and id > maxIndex:
                    max_v = v
                    maxIndex = id

            if max_v <= 0:
                minDist = float('inf')
                minIndex = -1
                for ii, cell in enumerate(self.m_freeSpaceVec):
                    if self.m_neuralMat[cell.row, cell.col] > 0 and self.boundingJudge(cell.row, cell.col):
                        dist = np.hypot(currentPoint.row - cell.row, currentPoint.col - cell.col)
                        if dist < minDist:
                            minDist = dist
                            minIndex = ii
                if minIndex != -1 and minDist != float('inf'):
                    currentPoint = self.m_freeSpaceVec[minIndex]
                    self.m_pathVec.append(currentPoint)
                    continue
                else:
                    rospy.loginfo("The program has been dead because of the self-locking")
                    rospy.loginfo("The program has gone through %d steps", len(self.m_pathVec))
                    break

            nextPoint = CellIndex()
            if maxIndex == 0:
                nextPoint.row = currentPoint.row
                nextPoint.col = currentPoint.col + 1
            elif maxIndex == 1:
                nextPoint.row = currentPoint.row - 1
                nextPoint.col = currentPoint.col + 1
            elif maxIndex == 2:
                nextPoint.row = currentPoint.row - 1
                nextPoint.col = currentPoint.col
            elif maxIndex == 3:
                nextPoint.row = currentPoint.row - 1
                nextPoint.col = currentPoint.col - 1
            elif maxIndex == 4:
                nextPoint.row = currentPoint.row
                nextPoint.col = currentPoint.col - 1
            elif maxIndex == 5:
                nextPoint.row = currentPoint.row + 1
                nextPoint.col = currentPoint.col - 1
            elif maxIndex == 6:
                nextPoint.row = currentPoint.row + 1
                nextPoint.col = currentPoint.col
            elif maxIndex == 7:
                nextPoint.row = currentPoint.row + 1
                nextPoint.col = currentPoint.col + 1

            nextPoint.theta = thetaVec[maxIndex]
            currentPoint = nextPoint
            self.m_pathVec.append(nextPoint)

    def boundingJudge(self, a, b):
        for i in range(-1, 2):
            for m in range(-1, 2):
                if i == 0 and m == 0:
                    continue
                if self.m_neuralMat[a + i, b + m] == -250.0:
                    return True
        return False

    def publishPlan(self, path):
        if not self.m_initialized:
            rospy.logerr("This planner has not been initialized yet, but it is being used, please call initialize() before use")
            return

        gui_path = Path()
        gui_path.poses = path
        gui_path.header.frame_id = "map"
        gui_path.header.stamp = rospy.Time.now()

        self.m_planPub.publish(gui_path)

    def publishCoveragePath(self):
        self.publishPlan(self.m_pathVecInROS)

class CellIndex:
    def __init__(self, row=0, col=0, theta=0):
        self.row = row
        self.col = col
        self.theta = theta
