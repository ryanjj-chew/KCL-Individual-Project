# Readme
1. Install Ubuntu 22.04 Jammy

2. Set up the ROS 2 Humble environment  
   Follow the official tutorial:  
   https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

3. Install prerequisites  
   a. Install colcon:  
      sudo apt install python3-colcon-common-extensions  

   b. Install ROS 2 Controllers:  
      sudo apt install ros-humble-ros2-control  

   c. Install MoveIt 2:  
      sudo apt install ros-humble-moveit  

   d. Install Ultralytics (YOLO):  
      pip3 install ultralytics  

   e. Install Python packages (if not already installed):  
      pip3 install scipy numpy opencv-python  

   f. Install Nvidia CUDA environment:  
      sudo apt install ubuntu-drivers-common  
      sudo ubuntu-drivers autoinstall  
      sudo reboot  
      # Verify driver version:  
      nvidia-smi  
      # Install CUDA Toolkit 12.8:  
      https://developer.nvidia.com/cuda-toolkit-archive  

   g. Install Isaac Sim 4.5.0:  
      https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_ros.html  

4. Set up the workspace  
   a. Source the ROS 2 environment:  
      source /opt/ros/humble/setup.bash  

   b. Create and enter workspace:  
      mkdir -p ~/ros2_ws/src  
      cd ~/ros2_ws/src  

   c. Clone the repository:  
      git clone https://github.com/ryanjj-chew/KCL-Individual-Project.git  

   d. Build the workspace:  
      cd ~/ros2_ws  
      colcon build  

   e. Source the overlay:  
      source install/setup.bash  

5. Execute the program  
   a. Launch Isaac Sim:  
      cd <isaac_sim_install_path>  
      source /opt/ros/humble/setup.bash  
      ./isaac-sim.selector.sh  
      # Ensure the ROS 2 Bridge extension is enabled, then click Start  

   b. Open three terminals, source the overlay in each:  
      source ~/ros2_ws/install/setup.bash  
      ros2 run space yolo_inference  
      ros2 run space depth  
      ros2 run space direction  

   c. In a separate terminal, launch MoveIt 2:  
      source ~/ros2_ws/install/setup.bash  
      ros2 launch ur5_moveit_config demo.launch.py  

   d. Open `/ros2_ws/src/space/IsaacSim/space.usd` in Isaac Sim and click Play  
