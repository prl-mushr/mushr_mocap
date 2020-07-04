# mushr-mocap

Code for running mocap for MuSHR cars.



This instruction assumes that you're using the windows machine setup in CSE 014 which is connected to the Optitrack system.



## Start the Mocap System (Windows)

1. Make sure the cameras are powered (Plug in the NetGear Switch to the Wall)
2. Open Windows Security, *turn off* the public network firewall. This is necessary for Windows machine to stream VRPN data. Make sure to turn it on after the mocap is done, or plug off the router from the wall so that it does not have internet access during the mocap, to be safer.
3. Start Motive Application (2.2.0 Final). Have the these panels on "Camera Calibration", "Assets", Devices".
   - It should automatically pick up cameras and show them in "Devices" panel. It takes some time.
4. Open the latest calibration. "File->Open->" and choose the latest ".cal" file. See [calibration instruction] to do the calibration again.
5. Since there's no visual markers in the scene, I recommend putting the floor marker to see get a frame of reference. In "Assets" tab, you can "File->Open" and double-click "floor.motive" to import the asset.
6. Check the tracked car in Motive. To create another rigid body, see [rigid-body instruction]. The x-axis may be in a wrong direction. You can fix it in Motive, though I prefer to fix it through ROS `static_transform_publisher`.
7. __Data streaming__ should be on by default, you can see this by checking the "Streaming" on the lower right. Make sure the settings are correct in the "Streaming" panel. "Local Interface" should be the IP of the windows machine.



## Setting up MuSHR-Mocap

1. Install `vrpn_client_ros` via :
```bash
sudo apt install ros-[melodic]-vrpn_client_ros
```

2. Clone this repository

   ```bash
   git clone https://github.com/prl-mushr/mushr_mocap.git
   ```

3. Build

   ```bash
   catkin build
   ```

4. Check that the machine is connected to the same wifi router (TP_Link_F0D9_5G).

5. Check that you can ping to the windows machine.

   ```bash
   ping 192.168.0.183
   ```

   should return some bytes.
   
6. Set $ROS_IP to match the IP of this machine, e.g. 
```bash
export ROS_IP=192.168.0.156
```

7. Launch the vrpn client. See the launch file as an example.

   ```bash
   roslaunch mushr_mocap vrpn.launch car_name:=car35 # in a new terminal
   ```

8. You should be able to check the raw mocap topic published as `vrpn_client_node/car_name/pose`

   ```bash
   rostopic echo /vrpn_client_node/car35/pose
   ```

9. Publish the transformed car pose (changes the axis orientation and takes into account the offset to base_link):

   ```bash
   roslaunch mushr_mocap car_pose_publisher car_name:=car35
   ```
Check the topic
   ```bash
   rostopic echo /car35/mocap_pose
   ```

   If you want this to be in a different topic name, modify `car_pose_publisher.py`.

10. To publish to `car_name/initialpose` topic,
   ```bash
   roslaunch mushr_mocap set_initial_pose.launch car_name:=car35
   ```
