# platoon_simulations
This package implements platoon architecture and control in simulation using Turtlebot3 platform in Gazebo classic.

## Requirements : 
- Ubuntu-22.04 LTS Desktop OS
- ROS2-Humble
- Python 3.10+

## Install Dependencies :

1. Install Gazebo classic (v11) :  

    ```bash
    sudo apt install gazebo
    sudo apt install ros-humble-gazebo-*
    ```

2. Install ros2 package dependencies :  

    ```bash
    sudo apt install ros-humble-turtlebot3-msgs
    sudo apt install ros-humble-turtlebot3-gazebo
    sudo apt install ros-humble-tf2-ros
    sudo apt install ros-humble-tf2-tools
    sudo apt install ros-humble-nav2-bringup
    sudo apt install ros-humble-tf-transformations
    ```  

## Configuring the ros2 workspace :   

1. Create ROS2 workspace :  

    ```bash
    mkdir -p ~/turtlebot3_ws/src
    cd ~/turtlebot3_ws/src
    git clone https://github.com/ab31mohit/platoon_simulations.git
    cd ~/turtlebot3_ws
    colcon build
    echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

## Running the package :

1. Launch multiple turtlebot3 robots in empty world :  

    ```bash
    ros2 launch platoon_simulations platoon_bringup.launch.py
    ```   
    This launch file will spawn the robots at custom poses defined within the [***params***](/params/) directory.     
    You add multiple robots with custom prefixes(namespaces) & poses in this config file.   

2. Running the formation control launch file :  

    ```bash
    ros2 launch platoon_simulations formation_control.launch.py
    ```   
    This launch file runs the platoon control for a set of 2 robots (leader & follower).    
    The leader and follower robots are defined as launch argument and passed to the formation_control_node.py node.   
    The platoon control launch file basically extends the formation control launch file for multiple sets of leader & follower (basically for a platoon containing more than 2 robots).   

3. Teleoperate the leader to test platoon control : 

- Export the namespace for leader : 

    ```bash
    export TURTLEBOT3_NAMESPACE=waffle1
    ```  
- Run the teleoperation node :  

    ```bash
    ros2 run platoon_simulations teleop.py
    ```
