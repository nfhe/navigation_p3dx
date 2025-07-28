#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <ros/package.h>
#include <fstream>
#include <string>
#include <chrono>
#include <cmath>
#include <limits>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <sys/stat.h>

class NavigationDataLogger
{
public:
    NavigationDataLogger() : nh_("~")
    {
        // 从参数服务器加载参数
        std::string log_base_dir;
        nh_.param<std::string>("log_dir", log_base_dir, "$(find dwa_planner)/logs");
        
        // 解析ROS包路径
        if (log_base_dir.find("$(find") == 0) {
            size_t end_pos = log_base_dir.find(")");
            if (end_pos != std::string::npos) {
                std::string package_name = log_base_dir.substr(7, end_pos - 7);
                std::string remaining_path = log_base_dir.substr(end_pos + 1);
                
                std::string package_path = ros::package::getPath(package_name);
                if (!package_path.empty()) {
                    log_base_dir = package_path + remaining_path;
                }
            }
        }
        
        // 确保日志目录存在
        ensureDirExists(log_base_dir);
        
        // 生成基于时间的文件名
        std::string log_filename = generateTimestampedFilename(log_base_dir);
        
        // 创建日志文件（使用默认模式，不覆盖原有文件）
        log_file_.open(log_filename.c_str());
        if (!log_file_.is_open())
        {
            ROS_ERROR("Failed to open log file: %s", log_filename.c_str());
            return;
        }
        ROS_INFO("Navigation data will be logged to: %s", log_filename.c_str());
        
        // 订阅必要的话题
        odom_sub_ = nh_.subscribe("/RosAria/odom", 1, &NavigationDataLogger::odomCallback, this);
        cmd_vel_sub_ = nh_.subscribe("/RosAria/cmd_vel", 1, &NavigationDataLogger::cmdVelCallback, this);
        scan_sub_ = nh_.subscribe("/scan", 1, &NavigationDataLogger::scanCallback, this);
        
        // 初始化定时器
        log_timer_ = nh_.createTimer(ros::Duration(0.1), &NavigationDataLogger::logTimerCallback, this);
        
        // 初始化变量
        odom_received_ = false;
        cmd_vel_received_ = false;
        scan_received_ = false;
        closest_obstacle_dist_ = 10.0;  // 默认10米
        
        // 写入日志文件头部，解释每列数据的含义
        log_file_ << "# 时间戳 X坐标 Y坐标 朝向角度 线速度 角速度 障碍物距离 计算时间\n";
        log_file_ << "# 单位:  秒   米    米    弧度   米/秒  弧度/秒   米      秒\n";
    }
    
    ~NavigationDataLogger()
    {
        if (log_file_.is_open())
        {
            log_file_.close();
        }
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        odom_msg_ = *msg;
        odom_received_ = true;
    }
    
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        cmd_vel_msg_ = *msg;
        cmd_vel_received_ = true;
    }
    
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        // 计算最近的障碍物距离
        closest_obstacle_dist_ = std::numeric_limits<float>::max();
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float range = msg->ranges[i];
            if (range >= msg->range_min && range <= msg->range_max)
            {
                if (range < closest_obstacle_dist_)
                {
                    closest_obstacle_dist_ = range;
                }
            }
        }
        scan_received_ = true;
    }
    
    void logTimerCallback(const ros::TimerEvent& event)
    {
        if (odom_received_ && cmd_vel_received_ && scan_received_)
        {
            // 记录当前时间
            ros::WallTime now = ros::WallTime::now();
            
            // 记录机器人位置
            double x = odom_msg_.pose.pose.position.x;
            double y = odom_msg_.pose.pose.position.y;
            double yaw = tf::getYaw(odom_msg_.pose.pose.orientation);
            
            // 记录机器人速度
            double linear_vel = cmd_vel_msg_.linear.x;
            double angular_vel = cmd_vel_msg_.angular.z;
            
            // 记录障碍物距离
            double obs_dist = closest_obstacle_dist_;
            
            // 计算计算时间（在此简化，记录定时器周期）
            double compute_time = 0.1;  // 假设为定时器周期
            
            // 记录数据到日志文件
            log_file_ << std::fixed
                      << now.toSec() << " "
                      << x << " "
                      << y << " "
                      << yaw << " "
                      << linear_vel << " "
                      << angular_vel << " "
                      << obs_dist << " "
                      << compute_time << std::endl;
                      
            ROS_DEBUG_THROTTLE(1.0, "Logging navigation data: pos=[%.2f, %.2f, %.2f] vel=[%.2f, %.2f] obs_dist=%.2f",
                              x, y, yaw, linear_vel, angular_vel, obs_dist);
        }
    }
    
    // 生成基于时间戳的文件名
    std::string generateTimestampedFilename(const std::string& base_dir)
    {
        // 获取当前时间
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        
        // 格式化时间为字符串：年月日_时分秒
        std::stringstream ss;
        ss << base_dir;
        // 确保目录以斜杠结尾
        if (!base_dir.empty() && base_dir.back() != '/') {
            ss << '/';
        }
        ss << "dwa_nav_log_";
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
        ss << ".txt";
        
        return ss.str();
    }
    
    // 确保目录存在
    void ensureDirExists(const std::string& dir_path)
    {
        if (dir_path.empty()) return;
        
        // 创建嵌套目录
        size_t pos = 0;
        std::string current_path;
        
        while ((pos = dir_path.find('/', pos + 1)) != std::string::npos) {
            current_path = dir_path.substr(0, pos);
            if (!current_path.empty()) {
                // 创建目录，忽略已存在的情况
                mkdir(current_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
            }
        }
        
        // 创建最终目录
        mkdir(dir_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }

    ros::NodeHandle nh_;
    std::ofstream log_file_;

    ros::Subscriber odom_sub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber scan_sub_;
    ros::Timer log_timer_;
    
    nav_msgs::Odometry odom_msg_;
    geometry_msgs::Twist cmd_vel_msg_;
    
    bool odom_received_;
    bool cmd_vel_received_;
    bool scan_received_;
    float closest_obstacle_dist_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_data_logger");
    NavigationDataLogger logger;
    ros::spin();
    return 0;
} 