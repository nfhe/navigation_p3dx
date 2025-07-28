#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <vector>
#include <string>

class TrajectoryVisualizer
{
public:
    TrajectoryVisualizer() : nh_("~")
    {
        // 获取参数
        std::string odom_topic, goal_topic;
        nh_.param<std::string>("odom_topic", odom_topic, "/RosAria/odom");
        nh_.param<std::string>("goal_topic", goal_topic, "/move_base_simple/goal");
        nh_.param<std::string>("frame_id", frame_id_, "map");
        nh_.param<double>("trajectory_duration", trajectory_duration_, 60.0);
        nh_.param<double>("marker_lifetime", marker_lifetime_, 0.5);
        
        // 创建发布者
        trajectory_pub_ = nh_.advertise<visualization_msgs::Marker>("trajectory_marker", 1);
        goal_pub_ = nh_.advertise<visualization_msgs::Marker>("goal_marker", 1);
        
        // 创建订阅者
        odom_sub_ = nh_.subscribe(odom_topic, 1, &TrajectoryVisualizer::odomCallback, this);
        goal_sub_ = nh_.subscribe(goal_topic, 1, &TrajectoryVisualizer::goalCallback, this);
        
        // 初始化变量
        last_update_time_ = ros::Time::now();
        
        // 创建可视化定时器
        viz_timer_ = nh_.createTimer(ros::Duration(0.1), &TrajectoryVisualizer::vizTimerCallback, this);
        
        ROS_INFO("轨迹可视化节点已启动");
        ROS_INFO("订阅里程计话题: %s", odom_topic.c_str());
        ROS_INFO("订阅目标点话题: %s", goal_topic.c_str());
    }
    
private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // 将当前位置添加到轨迹中
        geometry_msgs::Point point;
        point.x = msg->pose.pose.position.x;
        point.y = msg->pose.pose.position.y;
        point.z = 0.1; // 稍微抬高一点，以便在RViz中更好地可视化
        
        trajectory_points_.push_back(point);
        trajectory_timestamps_.push_back(ros::Time::now());
        
        // 删除超过持续时间的点
        while (!trajectory_timestamps_.empty() &&
               (ros::Time::now() - trajectory_timestamps_.front()).toSec() > trajectory_duration_) {
            trajectory_points_.erase(trajectory_points_.begin());
            trajectory_timestamps_.erase(trajectory_timestamps_.begin());
        }
    }
    
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        current_goal_ = *msg;
        has_goal_ = true;
        
        // 立即发布目标点标记
        publishGoalMarker();
    }
    
    void vizTimerCallback(const ros::TimerEvent& event)
    {
        // 发布轨迹标记
        publishTrajectoryMarker();
        
        // 如果有目标点，重新发布目标点标记（保持其可见性）
        if (has_goal_) {
            publishGoalMarker();
        }
    }
    
    void publishTrajectoryMarker()
    {
        if (trajectory_points_.empty()) {
            return;
        }
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "robot_trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        
        // 设置线条属性
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05; // 线宽
        marker.color.r = 0.76;
        marker.color.g = 0.32;
        marker.color.b = 0.31;
        marker.color.a = 0.7;
        marker.lifetime = ros::Duration(marker_lifetime_);
        
        // 添加轨迹点
        marker.points = trajectory_points_;
        
        // 发布标记
        trajectory_pub_.publish(marker);
    }
    
    void publishGoalMarker()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "goal_marker";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        
        // 设置位置和方向
        marker.pose = current_goal_.pose;
        
        // 设置箭头大小
        marker.scale.x = 0.5; // 箭头长度
        marker.scale.y = 0.1; // 箭头宽度
        marker.scale.z = 0.1; // 箭头高度
        
        // 设置颜色（绿色）
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(marker_lifetime_);
        
        // 发布标记
        goal_pub_.publish(marker);
    }
    
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher trajectory_pub_;
    ros::Publisher goal_pub_;
    ros::Timer viz_timer_;
    
    std::string frame_id_;
    std::vector<geometry_msgs::Point> trajectory_points_;
    std::vector<ros::Time> trajectory_timestamps_;
    geometry_msgs::PoseStamped current_goal_;
    bool has_goal_ = false;
    
    ros::Time last_update_time_;
    double trajectory_duration_; // 轨迹持续时间（秒）
    double marker_lifetime_; // 标记存活时间（秒）
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_visualizer");
    TrajectoryVisualizer visualizer;
    ros::spin();
    return 0;
} 