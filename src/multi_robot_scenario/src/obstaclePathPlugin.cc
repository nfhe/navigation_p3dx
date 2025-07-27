#include <multi_robot_scenario/obstaclePathPlugin.hh>
#include <gazebo/common/Animation.hh>
#include <gazebo/common/Console.hh>

namespace gazebo
{
  void DynamicObstacle::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){
      // 存储指向模型的指针
      this->model = _parent;
      this->sdf = _sdf;

      // 重要：禁用重力，防止障碍物下沉
      this->model->SetGravityMode(false);

      // 读取sdf文件中的参数
      if (this->sdf->HasElement("velocity")){
        this->velocity = _sdf->Get<double>("velocity");
      }
      else{
        this->velocity = 1.0;
      }

      if (this->sdf->HasElement("orientation")){
        this->orientation = _sdf->Get<bool>("orientation");
      }
      else{
        this->orientation = true;
      }

      if (this->orientation){
        if (this->sdf->HasElement("angular_velocity")){
          this->angularVelocity = _sdf->Get<double>("angular_velocity");
        }
        else{
          this->angularVelocity = 0.8;
        }
      }

      if (this->sdf->HasElement("loop")){
        this->loop = _sdf->Get<bool>("loop");
      }
      else{
        this->loop = false;
      }

      // 读取路径
      this->path.clear();
      if (this->sdf->HasElement("path")){
        sdf::ElementPtr waypointElem = _sdf->GetElement("path")->GetElement("waypoint");
        while (waypointElem){
          ignition::math::Vector3d wp = waypointElem->Get<ignition::math::Vector3d>();
          this->path.push_back(wp);
          waypointElem = waypointElem->GetNextElement("waypoint");
        }
      }

      // 确保路径中至少有两个点
      if (this->path.size() < 2) {
        gzerr << "DynamicObstacle插件需要至少两个路径点！" << std::endl;
        return;
      }
      
      if (this->loop){
        this->path.push_back(this->path[0]); // 形成循环
      }
      else{
        // 往返路径
        std::vector<ignition::math::Vector3d> temp = this->path;
        for (int i=temp.size()-2; i>=0; --i){
          this->path.push_back(temp[i]);
        }
      }

      if (this->orientation){
        // 修改路径，加入角度
        this->pathWithAngle.clear();
        double yawCurr;
        double yawLast;
        for (int i=0; i<this->path.size(); ++i){
          if (i == 0){
            double xCurr, yCurr, zCurr, yawStart;
            double xNext, yNext, zNext;

            xCurr = this->path[i].X();
            yCurr = this->path[i].Y();
            zCurr = this->path[i].Z();

            xNext = this->path[i+1].X();
            yNext = this->path[i+1].Y();
            zNext = this->path[i+1].Z();
            
            yawStart = atan2(yNext-yCurr, xNext-xCurr);
            std::vector<double> pose {xCurr, yCurr, zCurr, yawStart};
            this->pathWithAngle.push_back(pose);
            yawLast = yawStart;
          }
          else{
            // 如果不是第一个点，需要添加两个姿态来处理不同的偏航角
            double xCurr, yCurr, zCurr;
            double xNext, yNext, zNext;
            double xPrev, yPrev, zPrev;

            xCurr = this->path[i].X();
            yCurr = this->path[i].Y();
            zCurr = this->path[i].Z();

            if (i+1 < this->path.size()){
              xNext = this->path[i+1].X();
              yNext = this->path[i+1].Y();
              zNext = this->path[i+1].Z();
            }
            else{
              xNext = this->path[1].X();
              yNext = this->path[1].Y();
              zNext = this->path[1].Z();
            }

            xPrev = this->path[i-1].X();
            yPrev = this->path[i-1].Y();
            zPrev = this->path[i-1].Z();

            yawCurr = atan2(yNext-yCurr, xNext-xCurr);

            // 添加第一个点
            std::vector<double> pose1 {xCurr, yCurr, zCurr, yawLast};
            this->pathWithAngle.push_back(pose1);

            // 添加第二个点
            std::vector<double> pose2 {xCurr, yCurr, zCurr, yawCurr};
            this->pathWithAngle.push_back(pose2);

            yawLast = yawCurr;
          }
        }
      }
      else{
        this->pathWithAngle.clear();
        for (int i=0; i<this->path.size(); ++i){
          ignition::math::Vector3d wp = this->path[i];
          std::vector<double> pose {wp.X(), wp.Y(), wp.Z(), 0};
          this->pathWithAngle.push_back(pose);
        }
      }

      // 计算总时间
      this->timeKnot.clear();
      double totalTime = 0.0;
      this->timeKnot.push_back(totalTime);
      for (int i=0; i<this->pathWithAngle.size()-1; ++i){
        std::vector<double> poseCurr = this->pathWithAngle[i];
        std::vector<double> poseNext = this->pathWithAngle[i+1];

        bool rotation = ((poseCurr[0] == poseNext[0]) && (poseCurr[1] == poseNext[1]) && (poseCurr[2] == poseNext[2]));

        if (!rotation){ // 前进运动
          double xCurr, yCurr, zCurr;
          double xNext, yNext, zNext;

          xCurr = poseCurr[0];
          yCurr = poseCurr[1];
          zCurr = poseCurr[2];

          xNext = poseNext[0];
          yNext = poseNext[1];
          zNext = poseNext[2];

          double distance = sqrt(pow(xNext-xCurr, 2) + pow(yNext-yCurr, 2) + pow(zNext-zCurr, 2));
          double duration = distance/this->velocity;
          totalTime += duration;
          this->timeKnot.push_back(totalTime);
        }
        else{ // 旋转
          double yawCurr, yawNext;
          yawCurr = poseCurr[3];
          yawNext = poseNext[3];
          double angleABSDiff = std::abs(atan2(sin(yawNext-yawCurr), cos(yawNext-yawCurr)));
          double duration = angleABSDiff/this->angularVelocity;
          totalTime += duration;
          this->timeKnot.push_back(totalTime);
        }
      }

      // 确保动画总时间大于0
      if (totalTime <= 0) {
        gzerr << "动态障碍物动画总时间必须大于0！" << std::endl;
        return;
      }

      gazebo::common::PoseAnimationPtr anim(new gazebo::common::PoseAnimation("obstaclePath", totalTime, this->loop));
      gazebo::common::PoseKeyFrame *key;
      
      // 完全重写关键帧生成逻辑，使用大量中间点实现匀速运动
      gzdbg << "使用密集关键帧生成匀速动画..." << std::endl;
      
      // 为路径中的每个线段生成更多插值点
      double timeStep = 0.05;  // 每0.05秒生成一个关键帧，确保超平滑运动
      
      for (unsigned int i=0; i<this->pathWithAngle.size()-1; ++i) {
        // 获取当前段的起点和终点
        double t1 = this->timeKnot[i];
        double t2 = this->timeKnot[i+1];
        std::vector<double> pose1 = this->pathWithAngle[i];
        std::vector<double> pose2 = this->pathWithAngle[i+1];
        
        double x1 = pose1[0];
        double y1 = pose1[1];
        double z1 = pose1[2];
        double yaw1 = pose1[3];
        
        double x2 = pose2[0];
        double y2 = pose2[1];
        double z2 = pose2[2];
        double yaw2 = pose2[3];
        
        // 检查是旋转还是平移动作
        bool isRotation = ((x1 == x2) && (y1 == y2) && (z1 == z2));
        
        // 为当前时间段生成密集的关键帧
        double segmentDuration = t2 - t1;
        int numSteps = std::max(2, static_cast<int>(segmentDuration / timeStep) + 1);
        
        // 处理偏航角变化，确保选择最短路径
        double yawDiff;
        if (isRotation) {
          // 处理偏航角变化超过±PI的情况
          yawDiff = yaw2 - yaw1;
          double absYawDiff = std::abs(yawDiff);
          
          if (absYawDiff > M_PI) {
            // 选择最短路径
            if (yawDiff > 0) {
              yawDiff = yawDiff - 2.0 * M_PI;
            } else {
              yawDiff = yawDiff + 2.0 * M_PI;
            }
          }
        }

        gzdbg << "线段" << i << ": 从(" << x1 << "," << y1 << "," << z1 << ") 偏航=" << yaw1 
               << " 到(" << x2 << "," << y2 << "," << z2 << ") 偏航=" << yaw2 
               << ", 时间: " << t1 << "-" << t2 << ", 步数: " << numSteps << std::endl;
               
        for (int step = 0; step <= numSteps; ++step) {
          double ratio = static_cast<double>(step) / numSteps;
          double t = t1 + ratio * (t2 - t1);
          
          // 对位置进行线性插值
          double x = x1 + ratio * (x2 - x1);
          double y = y1 + ratio * (y2 - y1);
          double z = z1 + ratio * (z2 - z1);
          
          // 对角度进行插值
          double yaw;
          if (isRotation) {
            // 旋转运动，特殊处理偏航角
            yaw = yaw1 + ratio * yawDiff;
          } else {
            // 移动过程中的偏航角，直接线性插值即可
            if (std::abs(yaw2 - yaw1) <= M_PI) {
              yaw = yaw1 + ratio * (yaw2 - yaw1);
            } else {
              // 处理跨越-pi到pi边界的情况
              double yawDiff = std::fmod(yaw2 - yaw1 + 3*M_PI, 2*M_PI) - M_PI;
              yaw = yaw1 + ratio * yawDiff;
            }
          }
          
          // 创建关键帧
          key = anim->CreateKeyFrame(t);
          key->Translation(ignition::math::Vector3d(x, y, z));
          key->Rotation(ignition::math::Quaterniond(0, 0, yaw));
          
          if (step == 0 || step == numSteps || step % 10 == 0) {
            gzdbg << "  帧" << step << ": t=" << t << ", 位置=(" << x << "," << y << "," << z << "), 偏航=" << yaw << std::endl;
          }
        }
      }

      // 设置动画并启动
      _parent->SetAnimation(anim);
      
      // 添加日志输出以确认动画已设置
      gzdbg << "为模型 " << _parent->GetName() << " 设置了动画，总时长: " << totalTime << "秒" << std::endl;
      gzdbg << "总关键帧数: " << anim->GetLength() << "，确保匀速平滑运动" << std::endl;
  }

  std::vector<double>& DynamicObstacle::interpolateAngle(double start, double end, double dx){
    static std::vector<double> interpolation;
    double angleDiff = end - start;
    double angleDiffABS = std::abs(angleDiff);

    if (angleDiff >= 0 && angleDiffABS <= M_PI){
      for (double mid=start+dx; mid<end; mid+=dx){
        interpolation.push_back(mid);
      }
    }
    else if (angleDiff >= 0 && angleDiffABS > M_PI){
      // 减少到-PI
      double mid = start-dx;
      while (mid >= -M_PI){
        interpolation.push_back(mid);
        mid -= dx;
      }

      // 从PI减少到end
      mid = M_PI;
      while (mid > end){
        interpolation.push_back(mid);
        mid -= dx;
      }
    }
    else if (angleDiff < 0 && angleDiffABS <= M_PI){
      for (double mid=start-dx; mid>end; mid-=dx){
        interpolation.push_back(mid);
      }
    }
    else if (angleDiff < 0 && angleDiffABS > M_PI){
      // 增加到PI
      double mid = start+dx;
      while (mid <= M_PI){
        interpolation.push_back(mid);
        mid += dx;
      }

      // 从-PI增加到end
      mid = -M_PI;
      while (mid < end){
        interpolation.push_back(mid);
        mid += dx;
      }
    }
    return interpolation;
  }
} 