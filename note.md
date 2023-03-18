# DLO代码笔记

> 摘要：直接法激光里程计，基于关键帧构建子图，nanoGICP配准   

[toc]

关键数据类型：
std::vector<std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, pcl::PointCloud::Ptr>> keyframes;

## 点云回调函数icpCB

preprocessPoints：预处理点云，去nan值，降采样
computeMetrics：计算点云统计数据，Compute Spaciousness of Current Scan，单开线程进入computeSpaciousness函数
setAdaptiveParams：设置自适应参数，Set Keyframe Thresh from Spaciousness Metric，目前理解是通过计算的点云范围设置自适应关键帧选取阈值。
initializeInputTarget：第一帧初始化
setInputSources：设置当前输入帧

### getNextPose：点云配准计算位姿，分为帧间配准与帧子图配准两步

帧间配准：先结合IMU给出估计位移（取上一帧时间戳到现在的IMU数据叠加），好像只有对旋转的估计。直接调用nanogicp配准函数，获得配准后结果。得到的帧间变换叠加到前一帧s2s变换上（propagateS2S）。

帧-子图配准：获得当前子图关键帧 getSubmapKeyframes，如果子图改变，修改gicp输入。以s2s变换作为基础变换进行帧-子图配准，结果更新T_s2s_prev，

### getSubmapKeyframes 计算子图关键帧

- 关键变量：
  1. submap_kf_idx_curr 当前子图应包含的关键帧序号

- 步骤

  取当前位置（由s2s位置给出）附近距离最近TopK关键帧，存入submap_kf_idx_curr。计算所有关键帧凹包凸包，分别得到距离最近TopK关键帧，存入 submap_kf_idx_curr。

  子图关键帧去重，比较和前帧关键帧是否相同，赋值 submap_hasChanged。

  构建当前子图点云 submap_cloud 和 submap_normals

### updateKeyframes

先变换当前点云，计算临近关键帧。条件判断是否新增关键帧。增加关键帧：体素滤波，拼接关键帧点云，计算协方差并储存，发布关键帧。

### 发布，更新各类参数

icpCB结束

## imu回调函数imuCB

imu标定bias，采样存入buffer