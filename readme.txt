roscore
./test.sh 
source /opt/ros/kinetic/setup.bash 
roslaunch catkin_ws/src/distance_measure/pass_through_filter.launch 
python catkin_ws/src/distance_measure/readDistance.py
python catkin_ws/src/ACC/ACC.py
python catkin_ws/src/data_creation/src/sub_autoPID_work.py
