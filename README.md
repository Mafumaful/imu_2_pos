# imu_2_pos
将（sensor_msgs/Imu）IMU的数据转化成为位置数据。接收/data/imu数据，并将这个数据的加速度通过积分得出位置信息。

imu.launch以及imu_2.launch 是支持不同imu设置的launch文件，这里可以不用管它。只要编译并运行imu_2_pos.launch就可以将相应的/imu/data数据转化成为位置信息。

## 缺点

由于单IMU定位自身会存在缺陷，在长时间定位的情况下。速度，位置都会有累计误差。但是可以通过卡尔曼滤波融合位置信息来解决。

## 改进措施（占坑）
 https://github.com/shenshikexmu/Gait-Tracking-With-x-IMU
 将四元数不再变动作为速度归零的依据，也许可以缓解累计误差的影响。
 
