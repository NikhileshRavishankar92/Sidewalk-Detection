# Sidewalk-Detection
Program to detect the sidewalk in an image. 
- The approach is to back project the red, green, hue and saturation values of the sidewalk onto the whole image.
- Outliers are rejected by selecting the maximum contour region in the thresholded image which will invariably be the sidewalk
- The algorithm will work only if we can assume that the region immediately infront of the robot is part of the sidewalk. 

Instructions for downloading and implementing the vision algorithm
- Create an ROS package in the source directory of your local ROS workspace (~/catkin_ws/src). 
- You can create the package along with necessary dependencies using the instruction "catkin_create_pkg pkg_name sensor_msgs cv_bridge rospy std_msgs roscpp pcl_ros pcl_msgs pcl_conversions geometry_msgs "
- Create a folder called scripts inside the newly created pkg (~/catkin_ws/src/pkg_name)
- Download and save the files sidewalk_detect.py and Robot_vision.py in the scripts folder(~/catkin_ws/src/pkg_name/scripts)
- Make the two files executable so that ROS can recognize and use them. (Run "chmod +x sidewalk_detect.py" and "chmod +x Robot_vision.py")
- Change directory to ~/catkin_ws in your terminal and run catkin_make. 
- Run roscore in a separate terminal
- Enter the command rosrun pkg_name sidewalk_detect.py. The node will now wait for Image topic "/camera/color/image_raw"
- Play the rosbag file to view the results (rosbag play -l bag_name.bag)
- Press q to exit the image window

Detecting the sidewalk in a point cloud
-To run the point cloud algorithm, download and save the file sidewalk_detect.cpp inside your ROS package. 

-Update the CMakelists.txt with add_executable(sidewalk_detect sidewalk_detect.cpp) and target_link_libraries(sidewalk_detect ${catkin_LIBRARIES}) under the sub-heading build.

-Run catkin_make from the catkin_ws directory.

-Run roscore in a separate terminal.

-Enter the command rosrun pkg_name sidewalk_detect. The node will now wait for the topic /camera/depth/points. 

-To visualize the point cloud use rvix (Enter "rosrun rviz rviz" in a separate terminal).

-Play rosbag file containing the topic /camera/depth/points (rosbag play -l bag_name.bag) in a separate terminal.

-In the rviz window, set Global Options -> Fixed frame -> camera_depth_optical_frame ,Global Options -> Background color -> 0;0;0, Global Options -> Frame rate -> 30 ,Grid -> plane -> XY.

-Add PointCloud2 display and set topic to /camera/depth/points_in or /camera/depth/points_out to view the sidewalk and non-sidewalk parts of the pointcloud respectively.
