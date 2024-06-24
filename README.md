# 基於ROS的掃地機器人專案

## 目標
開發一個基於ROS的掃地機器人，並在Gazebo環境中模擬實作結果。該機器人具備以下功能：
1. 自動建立地圖
2. 保存地圖
3. 自動清掃

## 系統設計
### 自動建圖
![image](https://github.com/rayyichen310/-final-project/assets/173726579/646e153f-5165-44c4-a70b-c9b5e23c5cba)


### 自動清掃
![image](https://github.com/rayyichen310/-final-project/assets/173726579/7d7946f2-4431-484b-8f9d-ac9aa6aa40fb)


主要模組和文件夾包含以下內容：

1. **robot**：包含掃地機器人的URDF文件和環境的.world文件
   - **URDF文件**：用於描述掃地機器人的結構和組成部分，如機器人的連接、關節、形狀和感測器配置。
   - **.world文件**：用於描述模擬環境，包括地圖、障礙物和其他靜態元素，提供機器人運行的模擬場景。

2. **m-explore**：開源explore_lite算法
   - **explore_lite算法**：用於自主探索未知環境，通過移動機器人並建立環境地圖，確保機器人能夠覆蓋所有區域，適應不同場景的變化。

3. **auto_nav**：自動建圖和導航
   - **auto_slam.launch**：利用SLAM和explore_lite，機器人能夠在未知環境中自動建立地圖。
   - **clean_work.launch**：機器人根據建立的地圖，自動規劃和導航，清掃房間。

4. **scripts**
   - **path_planning.py**：規劃掃地機器人的路徑。
     - **路徑規劃**：基於Dijkstra算法和A*算法，確保機器人能夠高效地規劃覆蓋所有需要清掃的區域，避開障礙物。
   - **path_planning_node.py**：發佈規劃好的路徑的節點。
     - **路徑節點發佈**：通過ROS節點發佈規劃好的路徑，指導機器人按規劃路徑進行運動。
   - **next_goal.py**：發佈下一個目標點的節點。
     - **目標點發佈**：動態生成和發佈下一個清掃目標點，確保機器人能夠連續有效地覆蓋所有清掃區域，提高清掃效率。

## 測試結果

1. **自動掃描建圖**：
   - 指令：`roslaunch auto_nav auto_slam.launch`
   - ![image](https://github.com/rayyichen310/-final-project/assets/173726579/443390b1-6591-4f50-8ae0-763408622925)

   

2. **保存地圖**：
   - 指令：`roslaunch auto_nav save_map.launch`
  

3. **自動清掃**：
   - 指令：`roslaunch auto_nav clean_work.launch`
   - ![image](https://github.com/rayyichen310/-final-project/assets/173726579/72ef3836-5306-4fe0-af04-c4f52880f0f9)
   - 測試結果：能正確運作清掃房間，但偶爾機器人在清掃80%左右的地圖後，會卡在某個點不再前進
   - 對邊緣的處理不夠好

## 使用說明

1. 克隆此專案到您的工作區：
   ```bash
   cd ~/catkin_ws
   catkin_make
   cd src 
   git clone https://github.com/rayyichen310/-final-project.git Sweeping-Robot-main
   cd Sweeping-Robot-main


## 參考
- https://github.com/hrnr/m-explore
- https://github.com/nobleo/full_coverage_path_planner
- https://github.com/hjr553199215/SLAM-Clean-Robot-Path-Coverage-in-ROS
- https://github.com/peterWon/CleaningRobot
- https://github.com/mywisdomfly/Clean-robot-turtlebot3
- https://github.com/gezongbo/Wander_bot
- https://github.com/li-haojia/Clean-robot-turtlebot3?tab=readme-ov-file


