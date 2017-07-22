# Purpose of this repo
This repo is Baxter learnig pick project, which uses ROS DMP package and RL off-policy Q-learning.

You can follow this README to know how it works.

![exprement_image](https://github.com/Dennis-BIRL-GDUT/baxter_DMP_RL/blob/master/image/exprement.png)
## Dependencies

Before running this project, you need install this **dependencies**:

1. Install ROS full-desk-version : [ ROS indigo wiki](http://wiki.ros.org/ROS/Installation).

2. Install Baxter simulator workspace : [Install Baxter simulator workspace](http://sdk.rethinkrobotics.com/wiki/Simulator_Installation).
    * If you don't have a real Baxter robot, pls run this project in the Baxter gazebo simulator.
    * If you have a real Baxter robot, follow this website and  make sure that you can control you robot : [Baxter getting start](http://sdk.rethinkrobotics.com/wiki/Getting_Started).
3. Install Robotiq force torque driver : [force torque sensor driver](https://github.com/ros-industrial/robotiq/tree/jade-devel/robotiq_force_torque_sensor) , force torque sensor sensing the envrionemnt and give a reward to the RL agent. Don't worry, if you don't have this sensor, I will add this force torque sensor model to gazebo simulator, and update.

# Let's Go
## 1. Generating a Baxter joint trajectory

 * If you have a **real Baxter robot**, and follow this tutorial and generating a joint trajectory : [Baxter Joint Trajectory Playback Example. ](http://sdk.rethinkrobotics.com/wiki/Joint_Trajectory_Playback_Example)
 * Or you can navigate to this repo datasets file, the baxter_joint_input_data.csv is the joint trajectory csv file.

## 2. Generating a DMP trajectory

 * This repo are using ROS DMP package to generate DMP trajectorys, move to this website to learn more: [ROS DMP wiki](http://wiki.ros.org/dmp).

  How to do:

```
rosrun dmp dmp_server
rosrun dmp baxter_r_ram_dmp.py
```
**[TIPS]** : You need rewrite the code.

 * This repo are generated the DMP trajectorys already, in the **datasets** data0 - data4 csv file are the DMP trajectorys.

## 3. Learnig a optomal DMP trajectroy.
This part are using the Reinforcement leaning off-polociy agent.

 * After genarating the DMP trajectroys then running :

 ```
 rosrun dmp dmp_joint_trajectory_action_server.py
 rosrun dmp Baxter_DMP_RL_joint_trajectory_learn_client.py
 ```
 **[TIPS]** : You need rewrite the code.
