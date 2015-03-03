# RosDataConversion
ROS packages that contain nodes to convert (extract) data from bag files or published topics.  

- General Notes:
  - I work with ROS Fuerte distribution (using rosbuild) and ROS Hydro distribution (using catkin), so this repository contains a branch/buildsystem.
  - Most of the time I wont create package for both buildsystems (only the one I needed).
  - You can usually convert data obtained in one distribution using the node of the other distribution.

- Notes for git repository use
  - To work with the desired branch, I suggest to clone it as follow:
    - git clone -b [catkin|rosbuild] https://github.com/smichaud/RosDataConversion.git RosDataConversion_[catkin|rosbuild]

- Notes for package use:
  - For catkin, I create a softlink of the package directory in my catkin workspace src directory and build the workspace. 
  - For rosbuild I simply add the package path to the ROS_PACKAGE_PATH.
