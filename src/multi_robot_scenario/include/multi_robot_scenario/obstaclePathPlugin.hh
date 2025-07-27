#ifndef OBSTACLEPATHPLUGIN_HH
#define OBSTACLEPATHPLUGIN_HH
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <math.h>

namespace gazebo
{
  class DynamicObstacle : public ModelPlugin{
  private:
  	// Gazebo相关指针
  	physics::ModelPtr model; // 指向模型的指针
  	event::ConnectionPtr updateConnection; // 指向更新事件连接的指针

  	// 自定义属性
  	sdf::ElementPtr sdf;
  	double velocity;
  	bool orientation;
    bool loop;
    double angularVelocity;

  	std::vector<ignition::math::Vector3d> path;
  	std::vector<std::vector<double>> pathWithAngle;
  	std::vector<double> timeKnot;

  public: 
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    std::vector<double>& interpolateAngle(double start, double end, double dx);
  };

  // 在模拟器中注册该插件
  GZ_REGISTER_MODEL_PLUGIN(DynamicObstacle)
}

#endif 