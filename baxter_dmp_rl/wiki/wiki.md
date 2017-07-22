<a name="Top"></a>
[\<< back to Mainpage](https://github.com/Dennis-BIRL-GDUT/baxter_DMP_RL/wiki/Baxter-DMP-RL-Demo)

EN | [中文](https://github.com/Dennis-BIRL-GDUT/baxter_DMP_RL/wiki/Baxter-DMP-RL-Demo-cn)

## Demo: Baxter pick project which uses DMP trajectory and reinforcement learning.
![exprement_image](https://github.com/Dennis-BIRL-GDUT/baxter_DMP_RL/blob/master/image/exprement.png)

<br /><br /><br />

>## Navigation
1. <a href="#Summary">Summary</a>
2. <a href="#Video">Video</a>
3. <a href="#Quickstart">Quickstart</a>
4. <a href="#Structure">Structure</a>
5. <a href="#FAQ">FAQ</a>

<br /><br /><br />





<a name="Summary"></a>
### 1 Summary
***
**Baxter DMP RL Demo** is a package that makes the baxter arm genarate DMP trajectory and uses reinforcement learning to learn which trajectory is the optimal one .<br />
So far this package has been tested on **ROS indigo** powered by Ubuntu 14.04 only. If you got any problem with the code, please contact us.<br />
<a name="Files Location">
##### 1.1 Files Location:
- ./baxter_DMP_RL/scripts/...
- ./baxter_DMP_RL/src/...
<a name="Subscriptions">
##### 1.2 Subscriptions:
- ./robotiq/robotiq_force_torque_sensor/nodes/rq_sensor.cpp
- geometry_msgs.msg.WrenchStamped massage<br />
<a name="Publications">
##### 1.3 Publications:
- In this demo i publish 5 topics which is :

  > action1_reward, type is std_msgs.Float32<br />
   action2_reward, type is std_msgs.Float32<br />
   action3_reward, type is std_msgs.Float32<br />
   action4_reward, type is std_msgs.Float32<br />
   action5_reward, type is std_msgs.Float32<br />

   This topic which used to plot reward curve.

<br /><br /><br />


<a name="Video"></a>
### 2 Video
<video id="video" controls="" preload="none" poster="https://github.com/Dennis-BIRL-GDUT/baxter_DMP_RL/blob/master/image/exprement.png">
      <source id="mp4" src="http://media.w3.org/2010/05/sintel/trailer.mp4" type="video/mp4">
      <p>Your user agent does not support the HTML5 Video element.</p>
    </video>


<br /><br /><br />


<a name="Quickstart"></a>
### 3 Quickstart

***

Beofore you run this demo, please read the README file carefully, and then make sure your computer have been connected to the baxter. And run:
```
$ rosrun dmp dmp_joint_trajectory_action_server.py
$ rosrun dmp Baxter_DMP_RL_joint_trajectory_learn_client.py
```
To read more detail or find the solutions to some problems, please refer to <a href="#FAQ">**FAQ**</a> part below.
<br /><br /><br />


<a name="Structure"></a>
### 4 Structure

***

When you run the **dmp_joint_trajectory_action_server.py** python node above, it creat a trajectory server.<br />
If you want read more details please move this website : [Joint Trajectory Playback Example](http://sdk.rethinkrobotics.com/wiki/Joint_Trajectory_Playback_Example).<br />
When you run Baxter_DMP_RL_joint_trajectory_learn_client.py python node, it will creat a DMP instance and reinforcement learning agent, DMP instance will running a DMP trajectory point, but the trajectory is randomly genarated by reinforcement learning agent, and the RL agent will give a reward to each trajectory and learn a optimal trajectory.<br />
 If you want learn more about the ROS DMP package please go to this website [ROS DMP package](http://wiki.ros.org/dmp), move to this RL off-policy wiki to learn more about [RL Q-learning](https://en.wikipedia.org/wiki/Q-learning).

<br /><br /><br />


<a name="FAQ"></a>
### FAQ

***

**Q1**. Can I genarating the DMP trajectory by myself ?<br />
**A1**. Ofcouse you can! <br />
1. Before you genarating a DMP trajectory, you should learn this example:
[Joint Trajectory Playback Example](http://sdk.rethinkrobotics.com/wiki/Joint_Trajectory_Playback_Example).<br /> This example wil tell you how to genarate a joint trajectory, which is input data of DMP.<br />
2. And then run:
```
rosrun dmp dmp_server
rosrun dmp baxter_r_arm_dmp.py
```
[Tip]: please make sure your joint trajectory are input to the dmp baxter_r_arm_dmp.py file.<br />

<br />
<a name="Q2"></a>
**Q2**. I don't have the **Robotiq force torque sensor**, What shoule I do?<br />
**A2**. If you don't have Robotiq force touque sensor, which means you can't run this deno on the real Baxter robot, but I still have a sulotion, you can run this node in your gazebo simulator:
 1. I will update this sulotion next time.

<br />
## !!! Warning !!!
 If you hit the **Emergency** button, **please remmber kill the  dmp_server node first**, and then release the Emergency button. Otherwise, the robot arm will move super fast.
