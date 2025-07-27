#include <string>
#include <stdio.h>
#include <math.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {
  class ObstacleController : public ModelPlugin {
    public:
      ObstacleController() {}

      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
        // 存储模型指针
        this->model = _model;
        
        // 获取插件参数
        if (_sdf->HasElement("trajectory"))
          this->trajectory_type = _sdf->GetElement("trajectory")->Get<std::string>();
        else
          this->trajectory_type = "circle"; // 默认轨迹类型
        
        // 圆形轨迹参数
        if (_sdf->HasElement("radius"))
          this->radius = _sdf->GetElement("radius")->Get<double>();
        else
          this->radius = 1.0;
        
        // 线性轨迹参数
        if (_sdf->HasElement("direction")) {
          sdf::ElementPtr dirElem = _sdf->GetElement("direction");
          this->direction = ignition::math::Vector3d(
            dirElem->Get<ignition::math::Vector3d>()
          );
          this->direction.Normalize();
        } else {
          this->direction = ignition::math::Vector3d(1, 0, 0);
        }
        
        // 边界参数（用于线性轨迹）
        if (_sdf->HasElement("boundary_x")) {
          sdf::ElementPtr boundX = _sdf->GetElement("boundary_x");
          this->boundary_x_max = boundX->Get<ignition::math::Vector2d>()[0];
          this->boundary_x_min = boundX->Get<ignition::math::Vector2d>()[1];
        } else {
          this->boundary_x_max = 5.0;
          this->boundary_x_min = -5.0;
        }
        
        if (_sdf->HasElement("boundary_y")) {
          sdf::ElementPtr boundY = _sdf->GetElement("boundary_y");
          this->boundary_y_max = boundY->Get<ignition::math::Vector2d>()[0];
          this->boundary_y_min = boundY->Get<ignition::math::Vector2d>()[1];
        } else {
          this->boundary_y_max = 5.0;
          this->boundary_y_min = -5.0;
        }
        
        // 速度参数
        if (_sdf->HasElement("speed"))
          this->speed = _sdf->GetElement("speed")->Get<double>();
        else
          this->speed = 0.5;
        
        // 初始位置
        this->initial_pose = this->model->WorldPose();
        
        // 连接更新事件
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&ObstacleController::OnUpdate, this));
        
        // 初始化计时器和状态
        this->angle = 0.0;
        this->last_time = this->model->GetWorld()->SimTime();
        this->reverse_direction = false;
      }

      void OnUpdate() {
        // 计算时间步长
        gazebo::common::Time current_time = this->model->GetWorld()->SimTime();
        double dt = (current_time - this->last_time).Double();
        this->last_time = current_time;
        
        if (this->trajectory_type == "circle") {
          // 圆形轨迹运动
          this->angle += this->speed * dt;
          if (this->angle > 2.0 * M_PI) {
            this->angle -= 2.0 * M_PI;
          }
          
          double x = this->initial_pose.Pos().X() + this->radius * cos(this->angle);
          double y = this->initial_pose.Pos().Y() + this->radius * sin(this->angle);
          
          ignition::math::Pose3d pose = this->model->WorldPose();
          pose.Pos().X(x);
          pose.Pos().Y(y);
          
          this->model->SetWorldPose(pose);
        } else if (this->trajectory_type == "linear") {
          // 线性轨迹运动
          ignition::math::Pose3d pose = this->model->WorldPose();
          ignition::math::Vector3d delta;
          
          if (!this->reverse_direction) {
            delta = this->direction * this->speed * dt;
          } else {
            delta = -this->direction * this->speed * dt;
          }
          
          pose.Pos() += delta;
          
          // 检查是否超出边界
          if (pose.Pos().X() > this->boundary_x_max || 
              pose.Pos().X() < this->boundary_x_min || 
              pose.Pos().Y() > this->boundary_y_max || 
              pose.Pos().Y() < this->boundary_y_min) {
            // 反转方向
            this->reverse_direction = !this->reverse_direction;
            // 调整位置使其在边界内
            if (pose.Pos().X() > this->boundary_x_max) pose.Pos().X(this->boundary_x_max);
            if (pose.Pos().X() < this->boundary_x_min) pose.Pos().X(this->boundary_x_min);
            if (pose.Pos().Y() > this->boundary_y_max) pose.Pos().Y(this->boundary_y_max);
            if (pose.Pos().Y() < this->boundary_y_min) pose.Pos().Y(this->boundary_y_min);
          }
          
          this->model->SetWorldPose(pose);
        }
      }

    private:
      physics::ModelPtr model;
      event::ConnectionPtr updateConnection;
      ignition::math::Pose3d initial_pose;
      
      std::string trajectory_type;
      double radius;
      ignition::math::Vector3d direction;
      double speed;
      double angle;
      
      double boundary_x_max;
      double boundary_x_min;
      double boundary_y_max;
      double boundary_y_min;
      
      gazebo::common::Time last_time;
      bool reverse_direction;
  };

  // 注册插件
  GZ_REGISTER_MODEL_PLUGIN(ObstacleController)
} 