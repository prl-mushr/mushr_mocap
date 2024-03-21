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
6. Check the tracked car in Motive. To create another rigid body, see [rigid-body instruction]. The orientation may be wrong. This can be fixed using the offsets file, though we recommend aligning the axes with the vehicle through motive to the extent possible.
7. __Data streaming__ should be on by default, you can see this by checking the "Streaming" on the lower right. Make sure the settings are correct in the "Streaming" panel. "Local Interface" should be the IP of the windows machine. We will henceforth refer to this as "WINDOWS_IP".


## Setting up MuSHR-Mocap

1. Install `vrpn_client_ros` via :
```bash
sudo apt install ros-[melodic]-vrpn-client-ros
```

2. Clone this repository

   ```bash
   git clone https://github.com/prl-mushr/mushr_mocap.git
   ```

3. Build

   ```bash
   catkin_make
   ```

4. Check that the machine is connected to the same wifi router (TP_Link_F0D9_5G).

5. Check that you can ping to the windows machine. 
   ```bash
   ping <WINDOWS_IP>
   ```

   should return some bytes.
   
6. Set the offset in the file `./configs/mushr_tf_offset.yaml`:
   ```yaml
   x: 0.0
   y: 0.0
   z: 0.0
   roll: 180.0 # degrees!
   pitch: 0.0
   yaw: 0.0
   ```

7. Set the asset name and car name in `./launch/mushr_mocap.launch`.
```xml
   <launch>
     <!-- this is the IP address of the windows system publishing the mocap poses -->
     <arg name="server" default="WINDOWS_IP"/>
     <!-- this is the asset name you have assigned in the mocap system to the car -->
     <arg name="asset_name" default="mushrv4"/> 
     <!-- This is the prefix that the mocap pose will be published to, i.e., car/mocap_pose -->
     <arg name="car_name" default="car" />
     ...
     ...
   ```
   You can also set these during launch.

## Running the MuSHR mocap:

1. Make sure that Motive is already running on the windows machine and publishing the pose of the vehicle, and that the MuSHR is connected to the same network as the windows machine (in UW's case, the TP_Link_F0D9_5G).

2. If you're using MuSHR V4, start the docker using:
   ```bash
   mushr_noetic
   ```

3. Now, run the following command:
   ```bash
   roslaunch mushr_mocap mushr_mocap.launch server:=<WINDOWS_IP> asset_name:=<asset_name> car_name:=<car_name>
   ```
   Note that as the arguments do have default values, if you're only tracking one asset, you could just set the default values in the launch file and run:
   ```bash
   roslaunch mushr_mocap mushr_mocap.launch
   ```

<!-- 6. Set $ROS_IP to match the IP of this machine, e.g. 
```bash
export ROS_IP=[current computer's IP]
```

7. Launch the vrpn client. You need to update `vrpn.launch` with the list of rigid body names you want to be streamed from Optitrack Motive. The names of the rigid bodies should match those in Motive's `Assets` list. See the launch file as an example.

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

   If you want this to be in a different topic name, modify `car_pose_publisher.py`. -->

4. To publish to `car_name/initialpose` topic (this one is a bit WIP),
   ```bash
   roslaunch mushr_mocap set_init_pose.launch car_name:=car35
   ```