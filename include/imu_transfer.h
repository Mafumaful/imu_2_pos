#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <iostream>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace Eigen;

class ImuTransfer
{
private:
    ros::NodeHandle nh_;
    Eigen::Vector3d position_;    // 当前位置
    Eigen::Vector3d gravity_;     // 重力
    Eigen::Vector3d velocity_;    // 速度
    Eigen::Matrix3d rotate_;      // 当前旋转角度（旋转矩阵）
    Eigen::Quaterniond rotate_q_; // 当前旋转角度（四元数）
    Eigen::Vector3d acc_;         // 加速度
    ros::Time time_;              // 时间

    double delta_t_ = 0;            // 间隔时间
    bool is_the_first_time_ = true; // 判断是否是第一次进入

    /** 用来测试重力平均值的**/
    int cnt = 0; // 数进入程序的次数
    double out;  // 重力输出

    ros::Subscriber imu_data_sub_; // IMU的订阅者

    void imu_cb(const sensor_msgs::Imu::ConstPtr &msg);
    void init_Gravity(void); // 重力设置初始化
    void init_draw(void);    // 画图初始化
    void calculateP(void);

    // 新建一个画图的节点
    ros::Publisher line_pub;
    // 下面是画图相关的参数
    visualization_msgs::Marker path;
    // 更新路径
    void update_path(const Eigen::Vector3d &);

public:
    ImuTransfer(const ros::NodeHandle &nh);
    ~ImuTransfer();
};

/**
 * @brief 接收IMU 数据，并将这个数据转化成rpy
 *
 * @param msg
 */
void ImuTransfer::imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    // 创建一个四原数
    tf::Quaternion quat;
    // 将msg文件转化成tf能读懂的内容
    tf::quaternionMsgToTF(msg->orientation, quat);
    // 将tf四原数转化成eigen四元数
    tf::quaternionTFToEigen(quat, rotate_q_);
    rotate_ = rotate_q_.toRotationMatrix();

    // 加速度赋值
    acc_[0] = msg->linear_acceleration.x;
    acc_[1] = msg->linear_acceleration.y;
    acc_[2] = msg->linear_acceleration.z;

    //   计算重力平均值
    if (false)
    {
        cnt++;
        out += acc_[1];
        cout << out / cnt << endl;
    }

    if (is_the_first_time_)
    {
        time_ = msg->header.stamp;
        init_Gravity();
        is_the_first_time_ = false;
    }
    else
    {
        delta_t_ = (msg->header.stamp - time_).toSec();
        time_ = msg->header.stamp;
        calculateP();
        update_path(position_);
        line_pub.publish(path);
    }
}

/**
 * @brief Construct a new Imu Transfer:: Imu Transfer object
 *
 * @param nh ROS节点句柄
 */
ImuTransfer::ImuTransfer(const ros::NodeHandle &nh) : nh_(nh)
{
    position_[0] = 0;
    position_[1] = 0;
    position_[2] = 0;
    imu_data_sub_ = nh_.subscribe("/IMU_data", 1, &ImuTransfer::imu_cb, this);
    velocity_[0] = 0;
    velocity_[1] = 0;
    velocity_[2] = 0;
    line_pub = nh_.advertise<visualization_msgs::Marker>("Imu_path", 1000);

    init_draw();
}

/**
 * @brief 设置重力参数
 *
 */
void ImuTransfer::init_Gravity()
{
    gravity_[0] = 0;
    gravity_[1] = 0;
    gravity_[2] = 9.775;
}

/**
 * @brief 计算位置
 *
 */
void ImuTransfer::calculateP()
{
    Eigen::Vector3d acc_g = rotate_ * acc_;
    velocity_ = velocity_ + delta_t_ * (acc_g - gravity_);
    position_ = position_ + delta_t_ * velocity_;
    // cout << velocity_ << endl;
}

/**
 * @brief 画图初始化函数
 *
 */
void ImuTransfer::init_draw()
{
    path.color.b = 1.0;
    path.color.a = 1.0;
    path.type = visualization_msgs::Marker::LINE_STRIP;
    path.header.frame_id = "/map";
    path.ns = "points_and_lines";
    path.action = visualization_msgs::Marker::ADD;
    path.pose.orientation.w = 1.0;
    path.scale.x = 0.2;
    geometry_msgs::Point p;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    path.points.push_back(p);
}

/**
 * @brief 更新路径
 *
 * @param p 新的点
 */
void ImuTransfer::update_path(const Eigen::Vector3d &msg)
{
    geometry_msgs::Point p;
    p.x = msg[0];
    p.y = msg[1];
    p.z = msg[2];
    path.points.push_back(p);
}

/**
 * @brief Destroy the Imu Transfer:: Imu Transfer object
 *
 */
ImuTransfer::~ImuTransfer()
{
}
