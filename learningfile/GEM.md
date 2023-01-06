# **GEM文档V1.0**

作者：LDY				联系方式：308756172@qq.com				github地址：JACKLiuDay				编辑时间：2022.11.01				版权：SUCRO仅供学习参考

https://maverickpeter.github.io/2022/03/15/GEM-tutorial/ 原版GEM git地址与使用教程

代码流程：

<img src="/home/tuboshu/.config/Typora/typora-user-images/codeflow.png" alt="image-20221213155256384" style="zoom: 200%;" />

- **Notes:** 

  FAST_LIO发布的激光里程计，坐标变换"camera_init"和"body"之间，相当于cameara_init是原点，body是os_sensor正在变化的坐标系

  ```c++
  br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body" ) );
  ```

- #### **1.主要运行fxz_demo.launch，文件加载部分参数，param等yaml文件**

  ````yaml
  <node pkg="tf2_ros" type="static_transform_publisher" name="map_to_camera_init"  args="0 0 0 0 0 0  map camera_init"/>
  <node pkg="tf2_ros" type="static_transform_publisher" name="map_to_robot0map"  args="0 0 0 0 0 0  map robot0/map"/>
  <node pkg="tf2_ros" type="static_transform_publisher" name="body_to_os_sensor"  args="0 0 0 0 0 0  body os_sensor"/>
  <node pkg="tf2_ros" type="static_transform_publisher" name="realsense_tf"  args="0.07863858 -0.01679587 -0.09286361 0.518882 -0.5127513 0.4824561 -0.4848543 body camera_color_optical_frame"/> 
  ````

  上述tf变换发布将map和camera_init和robot0/map绑定在一起，将os_sensor和body绑定在一起，需要标定的是相机的彩色camera_color_optical_frame到body之间的外参数

  官方文档中两个yaml文件：

  simple_demo_map.yaml

  simple_demo_robot.yaml

  GEM程序加载的实际yaml参数为fxz_lio.yaml      fxz_robot.yaml

  launch文件最后执行四个tf变换发布：

  主函数入口在elevation_mapping_node.cpp中

  ````c++
  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "elevation_mapping", ros::init_options::AnonymousName);
  
    ros::NodeHandle nodeHandle("~");
  
    string robot_id;
    string robot_name;
  
    nodeHandle.param("robot_id", robot_id, string("1"));
    nodeHandle.param("robot_name", robot_name, string("robot1"));
    ROS_INFO("get robot_id: %s", robot_id.c_str());
    ROS_INFO("get robot_name: %s", robot_name.c_str());
  
    checkFormat(robot_name);
    // 1.实例化
    elevation_mapping::ElevationMapping elevationMap(nodeHandle, robot_name);
    // 2.Run（）函数，四信号接受
    boost::thread MapPublisherThread(&elevation_mapping::ElevationMapping::Run, &elevationMap);
  
    ros::MultiThreadedSpinner spinner(4);
    // 3.话题同步，数据处理
    elevationMap.startsync();
    
    spinner.spin();
    
    return 0;
  }
  ````

- #### **2.程序运行主要在ElevationMapping这个头文件中的类**，定位到ElevationMapping.cpp和hpp

   在ElevationMapping.cpp文件中，用到的GPU函数的接口

  ````c++
  // 地图处理
  void Map_optmove(float *opt_p, float height_update, float resolution,  int length, float *opt_alignedPosition);
  void Move(float *current_Position, float resolution, int length, float *h_central_coordinate, int *h_start_indice, float *position_shift);
  // 计算点云属性
  void Map_feature(int length, float *elevation, float *var, int *point_colorR, int *point_colorG, int *point_colorB, float *rough, float *slope, float *traver, float *intensity);
  void Init_GPU_elevationmap(int length, float resolution, float h_mahalanobisDistanceThreshold_, float h_obstacle_threshold);
  void Map_closeloop(float *update_position, float height_update, int length, float resolution);
  // 光线追踪处理
  void Raytracing(int length_);
  void Fuse(int length, int point_num, int *point_index, int *point_colorR, int *point_colorG, int *point_colorB, float *point_intensity, float *point_height, float *point_var);
  ````
  
  广播信息，读取参数
  
  ````c++
  pointMapPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(robotName + "/history_point", 1);
  subMapPublisher_ = nodeHandle_.advertise<dislam_msgs::SubMap>(robotName + "/submap", 1);
  globalMapPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(robotName + "/global_point", 1);
  lastmapPublisher_ =  nodeHandle_.advertise<grid_map_msgs::GridMap>(robotName + "/opt_map", 1);
  keyFramePCPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(robotName + "/keyframe_pc", 1);
  octomapPublisher_ = nodeHandle_.advertise<octomap_msgs::Octomap>(robotName + "/local_octomap", 1);
  roadOctomapPublisher_ = nodeHandle_.advertise<octomap_msgs::Octomap>(robotName + "/road_octomap", 1);
  obsOctomapPublisher_ = nodeHandle_.advertise<octomap_msgs::Octomap>(robotName + "/obs_octomap", 1);
  ````
  
  - readParameters()       
  - initialize()
  
  将launch文件中加载的yaml参数读取到程序中
  
- #### **3.initialize() 函数**

  初始化函数

  resetMapUpdateTimer()  

  对各部分标志位进行重置

  以上为GEM初始化部分程序执行过程，后续进行主回调函数调用循环

----------------------------------------------------------------------------------------------------------分解线----------------------------------------------------------------------------------------------------------

- #### **4.主运行函数** ElevationMapping::Run()

  开启线程boost::thread 运行 elevation_mapping::ElevationMapping::Run()函数

  ````c++
  void ElevationMapping::Run()
  ​{
    	// subscriber for map build signal
    	denseSubmapSignalSub_ = nodeHandle_.subscribe(robotName + "/dense_mapping", 1, &**ElevationMapping::denseMappingSignal**, this);
    	savingSignalSub_ = nodeHandle_.subscribe(robotName + "/map_saving", 1, &**ElevationMapping::mapSavingSignal,** this);
    	optKeyframeSub_ = nodeHandle_.subscribe(robotName + "/opt_keyframes", 1, &**ElevationMapping::optKeyframeCallback**, this);
    	keyFrameSignalSub_ = nodeHandle_.subscribe(robotName + "/new_keyframe", 1, &**ElevationMapping::newKeyframeSignal**, this);
  ​}
  ````
  
  ​	四个订阅函数
  
  - ##### 4.1 ElevationMapping::denseMappingSignal(const std_msgs::Bool::ConstPtr& denseSignal)
  
     接受稠密地图建图信号
  
     denseSubmap = true;
  
  - ##### **4.2 ElevationMapping::mapSavingSignal**(const std_msgs::Bool::ConstPtr& savingSignal）
  
     接收保存地图信号，激活保存地图功能 savingMap()
  
     savingMap(); //函数执行保存全局点云
  
  - **4.3 ElevationMapping::optKeyframeCallback(const slam_msg::Keyframes::ConstPtr& optKeyFrame)**
  
    接收闭环检测信号
    
    optFlag = 1;
    
    JumpOdomFlag = 1;
    
    optGlobalMapLoc_.clear();
  
    保存optKeyFrame中的四元数与位置信息
  
  - **4.4 ElevationMapping::newKeyframeSignal(const nav_msgs::Odometry::ConstPtr& newKeyframeSignal)**
  
    newLocalMapFlag = 1;
    

----------------------------------------------------------------------------------------------------------分解线----------------------------------------------------------------------------------------------------------

- #### **5.同步激光和相机话题 ElevationMapping::startsync()**

  connection = sync.registerCallback(boost::bind(&ElevationMapping::Callback, this, _1, _2));

  主回调函数 

  ````c++
  ElevationMapping::Callback(const sensor_msgs::PointCloud2ConstPtr& rawPointCloud, const sensor_msgs::Image::ConstPtr& image) 
  ````

  总体架构流程

  1】转换图像与点云数据

  2】计算相机与激光的安装外参数

  3】将相机彩色信息投影到激光点云
  
  4】更新信息
  
  5】两个主处理线程
  
  6】计算点云属性
  
  7】获取图像信息
  
  8】可视化局部地图与点云
  
  9】光纤追踪处理动态障碍物
  
  - **5.1 将图像数据转换为cv::Mat矩阵** 
  
    激光点云转换为pcl::Pointcloud2形式，image转换cv::Mat矩阵
  
  - **5.2 计算相机和Lidar之间的外参数**
  
    ````c++
    // calculate extrinsic parameters
    float P_x, P_y;
    float width, height;
    Eigen::Vector4d P_lidar;
    Eigen::Vector3d P_img;
    Eigen::Vector3d P_XY;
    Eigen::MatrixXd P_lidar2img(3, 4);
    Eigen::MatrixXd Tcamera(3, 4);
    Eigen::MatrixXd TLidar(4, 4);
    cv::Mat TCAM, TV;
    
    // Read camera instrinsic and camera-lidar extrinsics parameters
    cv::FileStorage fsSettings(cameraParamsFile, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
    ​  cerr << "ERROR: Wrong path to settings" << endl;
    }
    
    fsSettings["T.camera"] >> TCAM;
    fsSettings["T.lidar"] >> TV;
    
    Tcamera = toMatrix34(TCAM);
    TLidar = toMatrix44(TV);
    
    // calculate transform camera to lidar
    P_lidar2img = Tcamera * TLidar;
    ````
  
  - **5.3 将图像彩色信息投影到Lidar的pointcloud上**
  
    ````c++
    // Project image to point cloud
    for(int i = 0; i < pointCloud->points.size(); i++)
    {
        P_lidar << pointCloud->points[i].x, pointCloud->points[i].y, pointCloud->points[i].z, 1;
        P_img = P_lidar2img * P_lidar;
    
        P_x = P_img.x() / P_img.z();  // NCLT need to flip x, y
        P_y = P_img.y() / P_img.z();
    
        cv::Point midPoint;
    
        midPoint.x = P_x;
        midPoint.y = P_y;
    
        // 根据激光点云和相机点云重叠的部分，对lidar的点云进行染色
        if(midPoint.x > 0 && midPoint.x < img.size().width && midPoint.y > 0 && midPoint.y < img.size().height && P_img.z() > 0)
        { // NCLT need to flip height, width  
            int b = img.at<cv::Vec3b>(midPoint.y,midPoint.x)[0];
            int g = img.at<cv::Vec3b>(midPoint.y,midPoint.x)[1];
            int r = img.at<cv::Vec3b>(midPoint.y,midPoint.x)[2];
            cv::circle(img, midPoint, 1, cv::Scalar(b, g, r));
            pointCloud->points[i].b = b;
            pointCloud->points[i].g = g;
            pointCloud->points[i].r = r;
        }
        else{
            pointCloud->points[i].b = 0;
            pointCloud->points[i].g = 0;
            pointCloud->points[i].r = 0;
            pointCloud->points[i].intensity = 0;
        }
    }
    ````
  
  - **5.4 更新信息**
  
    ````c++
    // 更新坐标变换
    sensorProcessor_->updateTransformations(timeStamp);
    // 更新点定位
    updatepointsMapLocation(timeStamp);
    // 更新grid地图定位
    updateMapLocation();
    ````
  
    1】监听TF变换，根据更新时间timeStamp监听，相当于接收FAST_LIO或者其他SLAM系统发来的里程计信息，即os_sensor相对于map起点的变换T
  
    ````c++
    bool SensorProcessorBase::updateTransformations(const ros::Time& timeStamp)
    {
        try {
            transformListener_.waitForTransform(sensorFrameId_, mapFrameId_, timeStamp, ros::Duration(1.0));
    
            // 1.获取map和sensor之间的tf变换, "robot0/map"与"os_sensor"之间的坐标变换
            tf::StampedTransform transformTf;
            transformListener_.lookupTransform(mapFrameId_, sensorFrameId_, ros::Time(0), M2StransformTf);  // ros::Time(0)
            poseTFToEigen(M2StransformTf, transformationSensorToMap_);
    
            // 2.获取sensor和robot之间的坐标变换,"os_sensor"与"os_sensor"之间的坐标变换？
            transformListener_.lookupTransform(robotBaseFrameId_, sensorFrameId_, ros::Time(0), transformTf); // TODO Why wrong direction?
            Eigen::Affine3d transform;
            poseTFToEigen(transformTf, transform);
            rotationBaseToSensor_.setMatrix(transform.rotation().matrix());
            translationBaseToSensorInBaseFrame_.toImplementation() = transform.translation();
    
            // 3.获取map和robot之间的坐标变换，"robot0/map"与"os_sensor"之间的坐标变换
            transformListener_.lookupTransform(mapFrameId_, robotBaseFrameId_, ros::Time(0), transformTf); // TODO Why wrong direction?
            poseTFToEigen(transformTf, transform);
            rotationMapToBase_.setMatrix(transform.rotation().matrix());
            translationMapToBaseInMapFrame_.toImplementation() = transform.translation();
            return true;
    
            } catch (tf::TransformException &ex) 
            {
                ROS_ERROR("%s", ex.what());
                return false;
            }
    }
    ````
  
    
  
    2】更新updatepointsMapLocation(timeStamp) ;关于tf::transformlistener 参考https://blog.csdn.net/YiYeZhiNian/article/details/124355687
  
    ````c++
    // 该函数的作用是？
    bool ElevationMapping::updatepointsMapLocation(const ros::Time& timeStamp)
    {
        ROS_DEBUG("Elevation map is checked for relocalization.");
    
        geometry_msgs::PointStamped trackPoint;
        trackPoint.header.frame_id = trackPointFrameId_;
        trackPoint.header.stamp = timeStamp;
        // kindr::Position3D trackPoint_为高程地图追踪的点
        convertToRosGeometryMsg(trackPoint_, trackPoint.point);
        geometry_msgs::PointStamped trackPointTransformed;
    
        // 1.将trackPoint转换到map坐标系下，相当于获取"robot0/map"和高程图追踪的点（x，y，z）之间的坐标变换
        try {
            transformListener_.transformPoint(map_.getFrameId(), trackPoint, trackPointTransformed);
        } catch (TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            return false;
        }
    
        // 2.监听map和trackPointTransformed之间的坐标变换,trackPointFrameId_ = "os_sensor",获取“os_sensor”和"robot0/map"之间坐标变换
        try{
            transformListener_.lookupTransform(mapFrameId, trackPointFrameId_, timeStamp, trackPoseTransformed_);
        }catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    
        trackPointTransformed_x = trackPointTransformed.point.x;
        trackPointTransformed_y = trackPointTransformed.point.y;
        trackPointTransformed_z = trackPointTransformed.point.z;
    
        // 3.JumpOdom is not sync to the tf in front-end odometry 这里调里程计的设置怎么理解？
        if(JumpOdomFlag == 1 && abs(trackPointTransformed_z - later_trackPointTransformed_z) <= 0.02){
            JumpCount ++;
        }else if(JumpCount >= 3){
            JumpOdomFlag = 0;
            JumpFlag = 1;
            JumpCount = 0;
        }
        return false;
    }
    ````
  
    
  
    3】更新地图定位updateMapLocation();
  
    ````c++
    bool ElevationMapping::updateMapLocation()
    {
        float current_p[3];
        current_p[0] = trackPointTransformed_x;
        current_p[1] = trackPointTransformed_y;
        current_p[2] = trackPointTransformed_z;
        grid_map::Index M_startindex;
        grid_map::Position M_position;
    
        // 1.优化完成时处理里程计跳帧现象
        if(JumpOdomFlag == 1)
        {    
            float opt_position[2];
            float opt_alignedPosition[2];
            float height_update = trackPointTransformed_z - later_trackPointTransformed_z;
    
            opt_position[0] = trackPointTransformed_x;
            opt_position[1] = trackPointTransformed_y;
            // 根据opt_position, height_update高度更新差，分辨率，长度，计算得到opt_alignedPosition
            Map_optmove(opt_position, height_update, resolution_,  length_, opt_alignedPosition);
            // 将更新后的opt_alignedPosition赋值
            M_position.x() = opt_alignedPosition[0];
            M_position.y() = opt_alignedPosition[1];
            // map_.opt_move(M_position, height_update);
            prevMap_ = map_.visualMap_;
    
        }else
        {
            // 2.如果没有跳帧标志位
            int d_startindex[2];
            // 将栅格地图移动 当前位置，分辨率，长度，
            Move(current_p , resolution_,  length_, current_position, d_startindex, position_shift);
    
            M_startindex.x() = d_startindex[0];
            M_startindex.y() = d_startindex[1];
            M_position.x() = current_position[0];
            M_position.y() = current_position[1];
    
            map_.move(M_startindex, M_position);
        }
    
        // 3.更新高度记录
        later_trackPointTransformed_z = trackPointTransformed_z;
        return true;
    }
    ````
    
    - **Map_optmove(opt_position, height_update, resolution_,  length_, opt_alignedPosition)**; 传入当前位置opt_position，高度变化height_update，分辨率resolution_(0.04)，计算
    
      ````c++
      {
          float last_p[2];
          float d_central_coordinate[2];
          // 1.通过central_coordinate初始化last_p
          cudaMemcpyFromSymbol(&last_p, central_coordinate, sizeof(float) * 2);
          // 2.根据当前位置opt_p，和last_p，计算得到opt_alignedPosition
          alignedPosition(opt_p, last_p, resolution, opt_alignedPosition);
          d_central_coordinate[0] = opt_alignedPosition[0];
          d_central_coordinate[1] = opt_alignedPosition[1];
          // 3.使用d_central_coordinate更新central_coordinate
          cudaMemcpyToSymbol(central_coordinate, &d_central_coordinate, sizeof(float) * 2);
          int threadsPerBlock = 256; 
          int blocksPerGrid =(length * length + threadsPerBlock - 1) / threadsPerBlock; 
          // 4.更新计算地图高度
          G_update_mapheight<<<blocksPerGrid, threadsPerBlock>>>(height_update);
      }
      ````
    
      alignedPosition，当前位置，上一次的位置，分辨率，处理后的位置数据
    
      ````c
      void alignedPosition(float *current_p, float *last_p,  float  resolution, float *opt_alignedPosition)
      {
          int indexShift[2];
          float positionShift[2];
          positionShift[0] = current_p[0] - last_p[0];
          positionShift[1] = current_p[1] - last_p[1];
          // 这里计算的什么？
          for (int i = 0; i < 2; i++) 
          {
              // 按照分辨率格子来调整 跳帧？
              indexShift[i] = static_cast<int>(positionShift[i] / resolution + 0.5 * (positionShift[i] > 0 ? 1 : -1));
              opt_alignedPosition[i] = last_p[i] + resolution * indexShift[i];
          }
      }
      ````
    
      调用GPU计算更新地图高度
    
      ````c
      __global__ void G_update_mapheight(float height_update)
      {    
          int i = blockDim.x * blockIdx.x + threadIdx.x;
          // 判断GPU线程能否支持运算
      	if (i < Length * Length && map_elevation[i] != -10) 
          {
              map_elevation[i] += height_update;    
          }
      }
      ````
    
      
    
    - **Move(current_p , resolution_, length, current_position, d_startindex, position_shift);** 传递当前位置current_p，分辨率resolution_，长度length，计算当前
    
      ````c++
      void Move(float *current_Position, float resolution, int length, float *Central_coordinate, int *Start_indice, float *alignedPositionShift)
      {
          int indexShift[2];
          float positionShift[2];
      	
          float h_central_coordinate[2];
          int h_start_indice[2];
          float robot_height = current_Position[2];
          
          cudaMemcpyToSymbol(sensorZatLowestScan, &robot_height, sizeof(float));
      
          cudaMemcpyFromSymbol(&h_central_coordinate, central_coordinate, sizeof(float) * 2);
          cudaMemcpyFromSymbol(&h_start_indice, start_indice, sizeof(int) * 2);
      
          positionShift[0] = current_Position[0] - h_central_coordinate[0];
          positionShift[1] = current_Position[1] - h_central_coordinate[1];
      	
          getIndexShiftFromPositionShift(indexShift, positionShift, resolution);
          // float alignedPositionShift[2];
          getPositionShiftFromIndexShift(alignedPositionShift, indexShift, resolution);
      	
          for(int i = 0; i < 2; i++)
          {
              if(indexShift[i] != 0){
                  if(indexShift[i] >= length)
                  {
                      int threadsPerBlock = 128; 
                      int blocksPerGrid =(length * length + threadsPerBlock - 1) / threadsPerBlock; 
                      G_Clear_allmap<<<blocksPerGrid, threadsPerBlock>>>();
                  }
                  else
                  {
                      int sign = (indexShift[i] > 0 ? 1 : -1);
                      int startIndex = h_start_indice[i] - ( sign > 0 ? 1:0 );
                      int endIndex = startIndex + sign - indexShift[i];
                      int nCells = abs(indexShift[i]);
                      int index = (sign < 0 ? startIndex : endIndex);
                      index = IndexToRange(index, length);
                      if(index + nCells <= length)
                      {
                          if(i == 0)
                              Clear_regionrow(index, nCells, length);
                          else
                              Clear_regioncol(index, nCells, length); 
                      }
                      else
                      {
                          int firstIndex = index;
                          int firstnCells = length - firstIndex;
                          if(i == 0)
                              Clear_regionrow(firstIndex, firstnCells, length);
                          else
                              Clear_regioncol(firstIndex, firstnCells, length);
      
                          int secondIndex = 0;
                          int secondnCells = nCells - firstnCells;
                          if(i == 0)
                              Clear_regionrow(secondIndex, secondnCells, length);
                          else
                              Clear_regioncol(secondIndex, secondnCells, length);
                      }
                  }
              }
              h_start_indice[i] -= indexShift[i];
              h_start_indice[i] = IndexToRange(h_start_indice[i], length);
              h_central_coordinate[i] = PositionToRange(h_central_coordinate[i], alignedPositionShift[i], resolution);
          }   
      	
          cudaMemcpyToSymbol(start_indice, &h_start_indice, sizeof(int) * 2);
          cudaMemcpyToSymbol(central_coordinate, &h_central_coordinate, sizeof(float) * 2);
      
          Central_coordinate[0] = h_central_coordinate[0];
          Central_coordinate[1] = h_central_coordinate[1];
          Start_indice[0] = h_start_indice[0];
          Start_indice[1] = h_start_indice[1];
          //cudaDeviceSynchronize();
      }
      ````
      
      更新函数
    
    
    
  - **5.5 处理线程，两个处理线程**
  
    ````c++
    std::thread processPointThread(&ElevationMapping::processpoints, this, pointCloud);
    std::thread processMapCellThread(&ElevationMapping::processmapcells, this);
    processPointThread.join();// join()函数，该子线程执行完才能向下执行
    processMapCellThread.join();
    ````
  
    处理线程 两个：分别是processpoints和processmapcells
  
    
  
    - processpoints(pcl::PointCloud<Anypoint>::ConstPtr pointCloud)
  
      ````c++
      void ElevationMapping::processpoints(pcl::PointCloud<Anypoint>::ConstPtr pointCloud)
      { 
          ros::Time begin_time = ros::Time::now ();
      
          // 点云属性 获取点云大小
          int point_num = pointCloud->size();
          
          int point_index[point_num];            // 点下标
          float point_height[point_num];         // 点高度
          float point_var[point_num];            // 变量
      
          int point_colorR[point_num];           // 彩色信息R
          int point_colorG[point_num];           // 彩色信息G
          int point_colorB[point_num];           // 彩色信息B
          float point_intensity[point_num];      // 点反射强度
      
          PointCloud<Anypoint>::Ptr pointProcessed(new PointCloud<Anypoint>);
      	// 调用sensorProcessor处理点云，传入原始点云，处理后的点云，获取点云的各项属性
          if (!this->sensorProcessor_->process(pointCloud, pointProcessed, point_colorR, point_colorG, point_colorB, point_index, point_intensity, point_height, point_var)) 
          {
              ROS_ERROR("Point cloud could not be processed.");
              this->resetMapUpdateTimer();
          }
      
          boost::recursive_mutex::scoped_lock lock(MapMutex_);
      
          // GPU functions for fusing multi-frame point cloud 处理多个坐标系下的点云
          Fuse(length_, point_num, point_index, point_colorR, point_colorG, point_colorB, point_intensity, point_height, point_var);
      
          lock.unlock();
      }
      ````
  
      1】处理点云函数 bool SensorProcessorBase::process，位于SensorProcessorBase.cpp
  
      ````c++
      bool SensorProcessorBase::process(
      		const pcl::PointCloud<Anypoint>::ConstPtr pointCloudInput, // 原始点云
      		const pcl::PointCloud<Anypoint>::Ptr pointCloudMapFrame,   // 存储处理过后的点云
              int *point_colorR,
              int *point_colorG,
              int *point_colorB,
              int *point_index,
              float* point_intensity,
              float *point_height,
              float *point_var)
      {
          ros::Time timeStamp;
          timeStamp.fromNSec(1000 * pointCloudInput->header.stamp);
          ros::Time begin_time = ros::Time::now ();                      // 获取当前点云的时间戳
      
          pcl::PointCloud<Anypoint> Points;
          Points = *pointCloudInput;
          pcl::PointCloud<Anypoint>::Ptr pointCloudSensorFrame(new pcl::PointCloud<Anypoint>);
          pointCloudSensorFrame = Points.makeShared();                   // 将原始点云转存到pointCloudSensorFrame
      
        	cleanPointCloud(pointCloudSensorFrame);                        // 除去NAN值
       
          // 调用GPU处理点云，mapFrameId_为robot0/map 传入要处理的原始点云
          if (!GPUPointCloudprocess(pointCloudSensorFrame, pointCloudMapFrame, mapFrameId_, point_colorR, point_colorG, point_colorB, point_index, point_intensity, point_height, point_var)) 
              return false; //TF变化
      
      	return true;
      }
      ````
  
      2】GPUPointCloudprocess函数，位于SensorProcessorBase.cpp中
  
      ````c++
      bool SensorProcessorBase::GPUPointCloudprocess(
      		pcl::PointCloud<Anypoint>::Ptr pointCloudSensorframe,
      		pcl::PointCloud<Anypoint>::Ptr pointCloudTransformed,
      		const std::string& targetFrame, // targetFrame为robot0/map
          int *point_colorR,
          int *point_colorG,
          int *point_colorB,
          int *point_index,
          float* point_intensity,
          float *point_height,
          float *point_var)
      {
          // 获取时间和点云的frame_id
          ros::Time timeStamp;
          timeStamp.fromNSec(1000 * pointCloudSensorframe->header.stamp);
          const std::string inputFrameId(pointCloudSensorframe->header.frame_id);
      
          tf::StampedTransform transformTf;
          try {
              // 监听tf变换 查看map和传感器的点云frame之间的变换，即robot0/map和os_sensor之间的变换
              transformListener_.waitForTransform(targetFrame, inputFrameId, timeStamp, ros::Duration(0.5));
              transformListener_.lookupTransform(targetFrame, inputFrameId, timeStamp, transformTf);
          } catch (tf::TransformException &ex) 
          {
              ROS_ERROR("%s", ex.what());
              return false;
          }
      	
          ros::Time begin_time = ros::Time::now ();
          // 获取传入点云的大小
          int point_num = pointCloudSensorframe->size();
      
          float point_x[point_num];
          float point_y[point_num];
          float point_z[point_num];
          float point_x_ts[point_num];
          float point_y_ts[point_num];
      	
          // 将传入点云的数据进行转存，包括点的位置信息，彩色信息，强度等
          for (size_t i = 0; i < point_num; ++i) {
              auto& point = pointCloudSensorframe->points[i];
              point_x[i] = point.x;
              point_y[i] = point.y;
              point_z[i] = point.z;
              point_colorR[i] = point.r;
              point_colorG[i] = point.g;
              point_colorB[i] = point.b;
              point_intensity[i] = point.intensity;
          } 
      	
          // 将map与sensor之间的tf变换转存到transformTf中
          Eigen::Affine3d transform;	
          poseTFToEigen(transformTf, transform);
          Eigen::Matrix4f Transform;
          for (int i = 0; i < 4; i ++)
              for (int j = 0; j < 4; j ++)
                  Transform(i, j) = transform(i, j);
      
          double t1;
          // kindr::Position3D，map和base之间的变换，计算垂直高度上的上下限
          double relativeLowerThreshold = translationMapToBaseInMapFrame_.z() + ignorePointsLowerThreshold_;
          double relativeUpperThreshold = translationMapToBaseInMapFrame_.z() + ignorePointsUpperThreshold_;
          pcl::PointCloud<Anypoint> Ps;
          pcl::PointCloud<Anypoint> Ps_ts;
      
          float min_r;
          float beam_a; 
          float beam_c; 
          Eigen::RowVector3f sensorJacobian; 
          Eigen::Matrix3f rotationVariance;
          Eigen::Matrix3f C_SB_transpose;
          Eigen::RowVector3f P_mul_C_BM_transpose; 
          Eigen::Matrix3f B_r_BS_skew; 
      
          rotationVariance << 0, 0, 0,
          					0, 0, 0,
          					0, 0, 0;
      	// 读取计算参数
          readcomputerparam(&min_r, &beam_a, &beam_c, &sensorJacobian, &C_SB_transpose, &P_mul_C_BM_transpose, &B_r_BS_skew);
      
          // Process_points位于gpu_process.cu中
          Process_points(point_index, point_x, point_y, point_z, point_var, point_x_ts, point_y_ts, point_height,  Transform, point_num, relativeLowerThreshold, relativeUpperThreshold, min_r, beam_a, beam_c, sensorJacobian, rotationVariance, C_SB_transpose, P_mul_C_BM_transpose, B_r_BS_skew);
      
          return true;
      }
      ````
  
      3】readcomputerparam(&min_r, &beam_a, &beam_c, &sensorJacobian, &C_SB_transpose, &P_mul_C_BM_transpose, &B_r_BS_skew);函数实现功能为？计算sensor的雅克比，
  
      ````c++
      void SensorProcessorBase::readcomputerparam(float *min_r, float *beam_a, float *beam_c, Eigen::RowVector3f *sensorJacobian, Eigen::Matrix3f *C_SB_transpose, Eigen::RowVector3f *P_mul_C_BM_transpose, Eigen::Matrix3f *B_r_BS_skew)
      {
      	const Eigen::RowVector3f projectionVector = Eigen::RowVector3f::UnitZ();
      
      	// Sensor Jacobian (J_s).
      	*sensorJacobian = projectionVector * (rotationMapToBase_.transposed() * rotationBaseToSensor_.transposed()).toImplementation().cast<float>();
      
      	// Preparations for robot rotation Jacobian (J_q) to minimize computation for every point in point cloud.
      	const Eigen::Matrix3f C_BM_transpose = rotationMapToBase_.transposed().toImplementation().cast<float>();
          
      	*P_mul_C_BM_transpose = projectionVector * C_BM_transpose;
          
      	*C_SB_transpose = rotationBaseToSensor_.transposed().toImplementation().cast<float>();
          
      	*B_r_BS_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(translationBaseToSensorInBaseFrame_.toImplementation().cast<float>()));
      
      	*min_r = sensorParameters_.at("min_radius");
      	*beam_c = sensorParameters_.at("beam_constant");
      	*beam_a = sensorParameters_.at("beam_angle");
      }
      ````
  
      4】Process_points函数，Process_points位于gpu_process.cu中，点的下表，点的x,y,z属性，每个点的协方差，x_ts，y_ts，点的高度，robot0/map和os_sensor之间的变换Transform，点的数量point_num，相对阈值，min_radius(最小半径)，beam_c(常数)，sensorJacobian(传感器的雅克比)，rotationVariance(旋转方差)，C_SB_transpose()，P_mul_C_BM_transpose()，B_r_BS_skew()。
  
      ````c++
      int Process_points(int *map_index, float *point_x, float *point_y, float *point_z, float *point_var, float *point_x_ts, float *point_y_ts, float *point_z_ts, Eigen::Matrix4f transform, int point_num, double relativeLowerThreshold, double relativeUpperThreshold, float min_r, float beam_a, float beam_c, Eigen::RowVector3f sensorJacobian, Eigen::Matrix3f rotationVariance, Eigen::Matrix3f C_SB_transpose, Eigen::RowVector3f P_mul_C_BM_transpose, Eigen::Matrix3f B_r_BS_skew)
      {
      	float* dev_x; 
      	float* dev_y; 
      	float* dev_z; 
      	float* dev_result;
      	float* dev_x_ts; 
      	float* dev_y_ts; 
          float* dev_z_ts; 
          int* dev_mapindex;
      	
      	cudaMalloc((void**)&dev_x, point_num * sizeof(float)); 
      	cudaMalloc((void**)&dev_y, point_num * sizeof(float)); 
          cudaMalloc((void**)&dev_z, point_num * sizeof(float));
          
      	cudaMalloc((void**)&dev_result, point_num * sizeof(float));
      	cudaMalloc((void**)&dev_x_ts, point_num * sizeof(float)); 
      	cudaMalloc((void**)&dev_y_ts, point_num * sizeof(float)); 
      	cudaMalloc((void**)&dev_z_ts, point_num * sizeof(float));
          cudaMalloc((void**)&dev_mapindex, point_num * sizeof(int));
      
      	cudaMemcpy(dev_x, point_x, point_num * sizeof(float), cudaMemcpyHostToDevice); 
      	cudaMemcpy(dev_y, point_y, point_num * sizeof(float), cudaMemcpyHostToDevice);
      	cudaMemcpy(dev_z, point_z, point_num * sizeof(float), cudaMemcpyHostToDevice);
      
      	cudaMemcpyToSymbol(C_relativeLowerThreshold, &relativeLowerThreshold, sizeof(double));
      	cudaMemcpyToSymbol(C_relativeUpperThreshold, &relativeUpperThreshold, sizeof(double));
      	
      	cudaMemcpyToSymbol(C_min_r, &min_r, sizeof(float));
      	cudaMemcpyToSymbol(C_beam_a, &beam_a, sizeof(float));
      	cudaMemcpyToSymbol(C_beam_c, &beam_c, sizeof(float));
          
          int threadsPerBlock = 256;
          int point_blocksPerGrid =(point_num + threadsPerBlock - 1) / threadsPerBlock; 
      	G_pointsprocess<<<point_blocksPerGrid, threadsPerBlock>>>(dev_mapindex, dev_x, dev_y, dev_z, dev_result, dev_x_ts, dev_y_ts, dev_z_ts, transform, point_num, sensorJacobian, rotationVariance, C_SB_transpose, P_mul_C_BM_transpose, B_r_BS_skew); 
          
      	cudaMemcpy(point_var, dev_result, point_num * sizeof(float), cudaMemcpyDeviceToHost);
          cudaMemcpy(point_z_ts, dev_z_ts, point_num * sizeof(float), cudaMemcpyDeviceToHost);
          cudaMemcpy(map_index, dev_mapindex, point_num * sizeof(int), cudaMemcpyDeviceToHost);
          cudaMemcpy(point_x_ts, dev_x_ts, point_num * sizeof(float), cudaMemcpyDeviceToHost);
          cudaMemcpy(point_y_ts, dev_y_ts, point_num * sizeof(float), cudaMemcpyDeviceToHost);
          
          cudaError_t cudaStatus = cudaGetLastError();
          if (cudaStatus != cudaSuccess) 
          {
              fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
              //goto Error;
          }
      
      	cudaFree(dev_x); 	
      	cudaFree(dev_y); 
      	cudaFree(dev_z); 
      	cudaFree(dev_result);
      	cudaFree(dev_x_ts); 	
      	cudaFree(dev_y_ts); 
          cudaFree(dev_z_ts);
          cudaFree(dev_mapindex); 
      
      	return 0;
      }
      ````
  
      5】cuda内部的处理函数global void G_pointsprocess
  
      ````c++
      __global__ void G_pointsprocess(int* map_index, float *point_x, float *point_y, float *point_z, float *result_var, float *point_x_ts, float *point_y_ts, float *point_z_ts, Eigen::Matrix4f transform, int point_num, Eigen::RowVector3f C_sensorJacobian, Eigen::Matrix3f C_rotationVariance, Eigen::Matrix3f C_C_SB_transpose, Eigen::RowVector3f C_P_mul_C_BM_transpose, Eigen::Matrix3f C_B_r_BS_skew)
      {
          int i = blockDim.x * blockIdx.x + threadIdx.x; 
      	if(i < point_num)
      	{
      		float point_height =  transform(2, 0) * point_x[i] + transform(2, 1) * point_y[i] + transform(2, 2) * point_z[i] + transform(2, 3);
      		
      		int flag = 0;
      
      		if((point_height > C_relativeLowerThreshold && point_height < C_relativeUpperThreshold) && flag == 0)
      		{
      			point_x_ts[i] = transform(0, 0) * point_x[i] + transform(0, 1) * point_y[i] + transform(0, 2) * point_z[i] + transform(0, 3) ;
      			point_y_ts[i] = transform(1, 0) * point_x[i] + transform(1, 1) * point_y[i] + transform(1, 2) * point_z[i] + transform(1, 3) ;
      			point_z_ts[i] = point_height;
      
      			Eigen::Vector3f pointVector(point_x[i], point_y[i], point_z[i]); // S_r_SP
      			float heightVariance = 0.0; // sigma_p
      
      			// Measurement distance.
      			float measurementDistance = pointVector.norm();
      
      			// Compute sensor covariance matrix (Sigma_S) with sensor model.
      			float varianceNormal = pow(C_min_r, 2);
      			float varianceLateral = pow(C_beam_c + C_beam_a * measurementDistance, 2);
      
      			Eigen::Matrix3f sensorVariance = Eigen::Matrix3f::Zero();
      			sensorVariance.diagonal() << varianceLateral, varianceLateral, varianceNormal;
      
      			// Robot rotation Jacobian (J_q).
      			const Eigen::Matrix3f C_SB_transpose_times_S_r_SP_skew = SkewMatrixFromVector(Eigen::Vector3f(C_C_SB_transpose * pointVector));
      			Eigen::RowVector3f rotationJacobian = C_P_mul_C_BM_transpose * (C_SB_transpose_times_S_r_SP_skew + C_B_r_BS_skew);
      		
      			// Measurement variance for map (error propagation law).
      			Eigen::Vector3f rotationJacobian_T = cuda_Transpose(rotationJacobian);
      			heightVariance = cuda_computer(rotationJacobian, C_rotationVariance, rotationJacobian_T);
      		
      			Eigen::Vector3f C_sensorJacobian_T = cuda_Transpose(C_sensorJacobian);
      			heightVariance += cuda_computer(C_sensorJacobian, sensorVariance, C_sensorJacobian_T);
      	
      			// Copy to list.
                  result_var[i] = heightVariance;
                  
                  int grid_index = PointsToIndex(point_x_ts[i], point_y_ts[i]);
                  map_index[i] = PointsToMapIndex(point_x_ts[i], point_y_ts[i]);
                  if(grid_index != -1)
                  {
                      atomicMin(&map_lowest[grid_index], point_height);
                      if(point_height == map_lowest[grid_index])
                      {
                          map_lowest[grid_index] = map_lowest[grid_index] + 3 * heightVariance; 
                      }
                  }
      		}
      		else
      		{
                  map_index[i] = -1;
      			point_x[i] = -1;
      			point_y[i] = -1;
      			point_z[i] = -1;
      			point_x_ts[i] = -1;
      			point_y_ts[i] = -1;
      			point_z_ts[i] = -1;
      			result_var[i] = -1;
      		}
          }
      }
      ````
  
      
  
    - processmapcells()
  
      ````c++
      void ElevationMapping::processmapcells()
      {
          ros::Time begin_time = ros::Time::now ();
          boost::recursive_mutex::scoped_lock lock(MapMutex_);
          // 根据点云时间更新预测
          if (!this->updatePrediction(this->lastPointCloudUpdateTime_)) 
          {
              ROS_ERROR("Updating process noise failed.");
              this->resetMapUpdateTimer();
              return;
          }
          lock.unlock();   
      }
      ````
      
      处理地图格函数，调用updatePrediction(this->lastPointCloudUpdateTime_)函数
      
      ````c++
      bool ElevationMapping::updatePrediction(const ros::Time& time)
      {
          if (ignoreRobotMotionUpdates_) 
              return true;
      
          ROS_DEBUG("Updating map with latest prediction from time %f.", robotPoseCache_.getLatestTime().toSec());
      
          if (time + timeTolerance_ < map_.getTimeOfLastUpdate()) 
          {
              ROS_ERROR("Requested update with time stamp %f, but time of last update was %f.", time.toSec(), map_.getTimeOfLastUpdate().toSec());
              return false;
          } else if (time < map_.getTimeOfLastUpdate()) 
          {
              ROS_DEBUG("Requested update with time stamp %f, but time of last update was %f. Ignoring update.", time.toSec(), map_.getTimeOfLastUpdate().toSec());
              return true;
          }
      
          HomTransformQuatD robotPose;
          geometry_msgs::Pose Tf_robotpose;
      
          Tf_robotpose.position.x = sensorProcessor_->M2StransformTf.getOrigin().x();
          Tf_robotpose.position.y = sensorProcessor_->M2StransformTf.getOrigin().y();
          Tf_robotpose.position.z = sensorProcessor_->M2StransformTf.getOrigin().z();
          Tf_robotpose.orientation.x = sensorProcessor_->M2StransformTf.getRotation().getX();
          Tf_robotpose.orientation.y = sensorProcessor_->M2StransformTf.getRotation().getY();
          Tf_robotpose.orientation.z = sensorProcessor_->M2StransformTf.getRotation().getZ();
          Tf_robotpose.orientation.w = sensorProcessor_->M2StransformTf.getRotation().getW();
          // std::cout << "Tf_robotpose " << Tf_robotpose.position.x << "  "<< Tf_robotpose.position.y << std::endl; 
          convertFromRosGeometryMsg(Tf_robotpose, robotPose);
      
          if(Tf_robotpose.position.x == 0 && Tf_robotpose.position.y == 0)
              return true;
          // Covariance is stored in row-major in ROS: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovariance.html
          Eigen::Matrix<double, 6, 6> robotPoseCovariance;
          robotPoseCovariance.setZero();
      
          // Compute map variance update from motio n prediction.
          robotMotionMapUpdater_.update(map_, robotPose, robotPoseCovariance, time);
      
          return true;
      }
      ````
      
      
  
  - **5.6 计算点云属性**
  
    ````c++
    Map_feature(length_, elevation, var, point_colorR, point_colorG, point_colorB, rough, slope, traver, intensity);
    ````
  
    计算点云属性
  
    ````c
    void Map_feature(int length, float *elevation, float *var, int *colorR, int *colorG, int *colorB, float *rough, float *slope, float *traver, float *intensity)
    {
        float *d_elevation;
        float *d_var;
        float *d_rough = 0;
        float *d_slope = 0;
        float *d_traver = 0;
        float *d_intensity = 0;
        int *d_colorR;
        int *d_colorG;
        int *d_colorB;
    
        cudaMalloc((void**)&d_colorR, length * length * sizeof(int)); 
        cudaMalloc((void**)&d_colorG, length * length * sizeof(int)); 
        cudaMalloc((void**)&d_colorB, length * length * sizeof(int)); 
        cudaMalloc((void**)&d_elevation, length * length * sizeof(float)); 
        cudaMalloc((void**)&d_var, length * length * sizeof(float)); 
        cudaMalloc((void**)&d_rough, length * length * sizeof(float)); 
        cudaMalloc((void**)&d_slope, length * length * sizeof(float)); 
        cudaMalloc((void**)&d_traver, length * length * sizeof(float)); 
        cudaMalloc((void**)&d_intensity, length * length * sizeof(float)); 
    
        int cell_num = length * length;
    	int threadsPerBlock = 256; 
    	int blocksPerGrid =(cell_num + threadsPerBlock - 1) / threadsPerBlock; 
        G_Mapfeature<<<blocksPerGrid, threadsPerBlock>>>(d_colorR, d_colorG, d_colorB, d_elevation, d_var, d_rough, d_slope, d_traver, d_intensity); 
    
        cudaMemcpy(colorR, d_colorR, cell_num * sizeof(int), cudaMemcpyDeviceToHost); 
        cudaMemcpy(colorG, d_colorG, cell_num * sizeof(int), cudaMemcpyDeviceToHost); 
        cudaMemcpy(colorB, d_colorB, cell_num * sizeof(int), cudaMemcpyDeviceToHost); 
        cudaMemcpy(elevation, d_elevation, cell_num * sizeof(float), cudaMemcpyDeviceToHost); 
        cudaMemcpy(var, d_var, cell_num * sizeof(float), cudaMemcpyDeviceToHost);
        cudaMemcpy(rough, d_rough, cell_num * sizeof(float), cudaMemcpyDeviceToHost); 
        cudaMemcpy(slope, d_slope, cell_num * sizeof(float), cudaMemcpyDeviceToHost); 
        cudaMemcpy(traver, d_traver, cell_num * sizeof(float), cudaMemcpyDeviceToHost); 
        cudaMemcpy(intensity, d_intensity, cell_num * sizeof(float), cudaMemcpyDeviceToHost); 
    
        cudaFree(d_colorR);
        cudaFree(d_colorG);
        cudaFree(d_colorB);
        cudaFree(d_elevation);
        cudaFree(d_var);
        cudaFree(d_rough);
        cudaFree(d_slope);
        cudaFree(d_traver);
        cudaFree(d_intensity);
    }
    ````
  
    计算点云属性
  
    ````c
    __global__ void G_Mapfeature(int *d_colorR, int *d_colorG, int *d_colorB, float *d_elevation, float *d_var, float *d_rough, float *d_slope, float *d_traver, float *d_intensity)
    {
        
        int idx = threadIdx.x + blockDim.x * blockIdx.x;
        if(idx >= Length * Length) return;
    
        Pos3 point[25];
    
        float s_z;
        float px_mean = 0;
        float py_mean = 0;
        float pz_mean = 0;
        
        int cell_x = idx / Length;
        int cell_y = idx % Length;
        int point_x;
        int point_y;
    
        int Ele_x;
        int Ele_y;
    
        // Debug
        //slope[idx] = map_height[idx];
        //printf("X:%d,Y:%d,height:%f", cell_x, cell_y, map_height[idx]);
    
        d_elevation[idx] = map_elevation[idx];
        d_colorR[idx] = map_colorR[idx];
        d_colorG[idx] = map_colorG[idx];
        d_colorB[idx] = map_colorB[idx];
        d_intensity[idx] = map_intensity[idx];
    
        d_var[idx] = map_variance[idx];
        if(map_elevation[idx] == -10)
            return ;
        
        int p_n = 0;    
        for (int i = -2; i < 3 ;i ++)
        {
          for(int j = -2; j < 3; j++)
          {
                Ele_x = (cell_x + Length - start_indice[0]) % Length;
                Ele_y = (cell_y + Length - start_indice[1]) % Length;
    
                Ele_x = Ele_x + i;
                Ele_y = Ele_y + j;
    
                if( Ele_x >= 0 && Ele_x < Length && Ele_y >= 0 && Ele_y < Length)
                {
                    point_x = cell_x + i;
                    point_y = cell_y + j;
    
                    point_x = (point_x + Length) % Length;
                    point_y = (point_y + Length) % Length;
                    s_z = map_elevation[point_x * Length + point_y];
                    if(s_z != -10)
                    {
                        point[p_n].x = point_x * Resolution;
                        point[p_n].y = point_y * Resolution;
                        point[p_n].z = s_z;
    
                        px_mean = px_mean + point[p_n].x;
                        py_mean = py_mean + point[p_n].y;
                        pz_mean = pz_mean + point[p_n].z;
                        p_n++;
                    }
                }
          	}
        }
        
        if(p_n > 7)
        {
            px_mean = px_mean / p_n;
            py_mean = py_mean / p_n;
            pz_mean = pz_mean / p_n;
        
            float pMatrix[9] = {0};
            for(int i = 0; i < p_n; i ++)
            {
                pMatrix[0] = pMatrix[0] + (point[i].x - px_mean) * (point[i].x - px_mean);
                pMatrix[4] = pMatrix[4] + (point[i].y - py_mean) * (point[i].y - py_mean);
                pMatrix[8] = pMatrix[8] + (point[i].z - pz_mean) * (point[i].z - pz_mean);
                pMatrix[1] = pMatrix[1] + (point[i].x - px_mean) * (point[i].y - py_mean);
                pMatrix[2] = pMatrix[2] + (point[i].x - px_mean) * (point[i].z - pz_mean);
                pMatrix[5] = pMatrix[5] + (point[i].y - py_mean) * (point[i].z - pz_mean);
                pMatrix[3] = pMatrix[1];
                pMatrix[6] = pMatrix[2];
                pMatrix[7] = pMatrix[5];
            }
            
            float dbEps = 0.01;
            int nJt = 30;
            int nDim = 3;
            float normal_vec[3];
            float Slope;
            computerEigenvalue(pMatrix, nDim, normal_vec, dbEps, nJt);
            
            float height = map_elevation[idx];
            float smooth_height = pz_mean;
            
            if(normal_vec[2] > 0)
                Slope = acos(normal_vec[2]);
            else
                Slope = acos(-normal_vec[2]); 
          
            float Rough = fabs(height - smooth_height);
            float Traver = 0.5 * (1.0 - Slope / 0.6)+ 0.5 * (1.0 - (Rough / 0.2));
            d_slope[idx] = Slope; 
            d_rough[idx] = Rough;
    
            d_traver[idx] = Traver;
            map_traver[idx] = Traver;
        }
        else
        {
            d_slope[idx] = 0; 
            d_rough[idx] = 0;
            d_traver[idx] = -10;
            map_traver[idx] = -10;
        }  
    }
    ````
    
    
    
  - **5.7 获取图像** map为 ElevationMap map_;
  
    ````c++
    orthoImage = map_.show(timeStamp, robotName, trackPointTransformed_x, trackPointTransformed_y, length_, elevation, var, point_colorR, point_colorG, point_colorB, rough, slope, traver, intensity);
    ````
  
    显示调用ElevationMap的show函数
  
    ````c++
    sensor_msgs::ImagePtr ElevationMap::show(ros::Time timeStamp, string robot_name, float trackPointTransformed_x, float trackPointTransformed_y, int length, float *elevation, float *var, int *point_colorR, int *point_colorG, int *point_colorB, float *rough, float *slope, float *traver, float* intensity)
    {
        cv::Mat image(length, length, CV_8UC3, cv::Scalar(0,0,0));
        cv::Mat ele_image(length, length, CV_32FC1, cv::Scalar(-10.0f));
    
        visualMap_.clearAll();
    
        pcl::PointCloud<pcl::PointXYZRGB> show_pointCloud;
        pcl::PointXYZRGB show_point;
    
        int index, index_x, index_y;
        Index start_index = visualMap_.getStartIndex();
    
        for (GridMapIterator iterator(visualMap_); !iterator.isPastEnd(); ++iterator) 
        {
            index_x = (*iterator).transpose().x();
            index_y = (*iterator).transpose().y();
            index = index_x * length + index_y;
    
            if(elevation[index] != -10 && traver[index] != -10 && !std::isnan(traver[index]))
            {
                visualMap_.at("elevation", *iterator) = elevation[index];
                visualMap_.at("variance", *iterator) = var[index];
                visualMap_.at("rough", *iterator) = rough[index];
                visualMap_.at("slope", *iterator) = slope[index];
                visualMap_.at("traver", *iterator) = traver[index];
                visualMap_.at("color_r", *iterator) = point_colorR[index];
                visualMap_.at("color_g", *iterator) = point_colorG[index];
                visualMap_.at("color_b", *iterator) = point_colorB[index];
                visualMap_.at("intensity", *iterator) = intensity[index];
    
                Position point;
                visualMap_.getPosition(*iterator, point);
    
                show_point.x = point.x();
                show_point.y = point.y();
                show_point.z = visualMap_.at("elevation", *iterator);
                show_point.r = visualMap_.at("color_r", *iterator);
                show_point.g = visualMap_.at("color_g", *iterator);
                show_point.b = visualMap_.at("color_b", *iterator);
    
                show_pointCloud.push_back(show_point);
                image.at<cv::Vec3b>((index_x + length - start_index[0]) % length, (index_y + length - start_index[1]) % length)[0] = visualMap_.at("color_b", *iterator);
                image.at<cv::Vec3b>((index_x + length - start_index[0]) % length, (index_y + length - start_index[1]) % length)[1] = visualMap_.at("color_g", *iterator);
                image.at<cv::Vec3b>((index_x + length - start_index[0]) % length, (index_y + length - start_index[1]) % length)[2] = visualMap_.at("color_r", *iterator);
    
                ele_image.at<float>((index_x + length - start_index[0]) % length, (index_y + length - start_index[1]) % length) = visualMap_.at("elevation", *iterator);
            }
        }
    
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        msg->header.stamp = timeStamp;
        orthomosaicPublisher_.publish(msg);
    
        sensor_msgs::ImagePtr msg_ele = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_32FC1, ele_image).toImageMsg();
        msg_ele->header.stamp = timeStamp;
        elevationimgPublisher_.publish(msg_ele);
    
        pcl_conversions::toPCL(timeStamp, show_pointCloud.header.stamp);
    
        show_pointCloud.header.frame_id = frame_id;
        // ROS_WARN("visualpoints: %d", show_pointCloud.size());
        if(show_pointCloud.size() > 0)
        {
            sensor_msgs::PointCloud2 pub_pointcloud;
            pcl::toROSMsg(show_pointCloud, pub_pointcloud);
            VpointsPublisher_.publish(pub_pointcloud); 
        }
    
        grid_map_msgs::GridMap message;
        GridMapRosConverter::toMessage(visualMap_, message);
        visualMapPublisher_.publish(message);
        return msg;
    }
    ````
  
    
  
  - **5.8 更新局部地图并可视化局部点云**
  
    ````c++
    // 更新局部地图
    updateLocalMap(rawPointCloud);
    // 可视化点地图
    visualPointMap();
    ````
  
    - 更新局部地图 updateLocalMap(rawPointCloud);
  
      ````c++
      void ElevationMapping::updateLocalMap(const sensor_msgs::PointCloud2ConstPtr& rawPointCloud)
      {
          int index, index_x, index_y;
          float delta_x, delta_y;
      
          float current_x = current_position[0];
          float current_y = current_position[1];
      
          delta_x = position_shift[0];
          delta_y = position_shift[1];
      
          if(initFlag == 1)
              prevMap_ = map_.visualMap_;
      
          // Should adapt to the SLAM. Check if a new local map is needed
          if(initFlag == 0 && sqrt(pow((trackPoseTransformed_.getOrigin().x() - trajectory_.back().translation().x()),2) + pow((trackPoseTransformed_.getOrigin().y() - trajectory_.back().translation().y()),2)) >= localMapSize_){
              newLocalMapFlag = 1;
              keyFramePCPublisher_.publish(rawPointCloud);
          }
      
          // main if for generate local map
          if(newLocalMapFlag == 1 && (JumpFlag == 1 || optFlag == 0))
          {
              if(!localMap_.empty() && initFlag == 0)
              { // Not the init state
      
                  // Get keyframe pose
                  Eigen::Quaternionf q(trackPoseTransformed_.getRotation().w(), trackPoseTransformed_.getRotation().x(), trackPoseTransformed_.getRotation().y(), trackPoseTransformed_.getRotation().z());
                  Eigen::Isometry3f prev_center(q);
                  prev_center.pretranslate(Eigen::Vector3f(trackPoseTransformed_.getOrigin().x(), trackPoseTransformed_.getOrigin().y(), trackPoseTransformed_.getOrigin().z()));
                  trajectory_.push_back(prev_center);
      
                  // Save the center of every keyframe
                  PointXY localMapCenter_;
                  localMapCenter_.x = trajectory_.back().translation().x();
                  localMapCenter_.y = trajectory_.back().translation().y();
                  localMapLoc_.push_back(localMapCenter_);
      
                  // Debug only
                  ROS_WARN("newLocalMapFlag %d, Push new keyframe! x = %lf, y = %lf", newLocalMapFlag, trajectory_.back().translation().x(), trajectory_.back().translation().y());
      
                  // Save local hash to point cloud
                  pointCloud::Ptr out_pc, grid_pc;
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr dense_output;
      
                  // Save local map into global map stack
                  gridMaptoPointCloud(map_.visualMap_, grid_pc);
                  localHashtoPointCloud(localMap_, out_pc);
      
                  if(denseSubmap)
                      pointcloudinterpolation(out_pc);
      
                  denseSubmap = false;
                  unique_lock<mutex> lock(GlobalMapMutex_);
                  globalMap_.push_back(*out_pc + *grid_pc);
                  lock.unlock();
      
                  // Publish submap
                  dislam_msgs::SubMap output;
                  sensor_msgs::PointCloud2 pc;
                  pcl::toROSMsg(*out_pc, pc);
                  pc.header.frame_id = mapFrameId;
                  pc.header.stamp = ros::Time(0);
                  output.submap = pc;
                  output.orthoImage = *orthoImage;
                  output.keyframePC = *rawPointCloud;
                  output.pose.position.x = trackPoseTransformed_.getOrigin().x();
                  output.pose.position.y = trackPoseTransformed_.getOrigin().y();
                  output.pose.position.z = trackPoseTransformed_.getOrigin().z();
                  output.pose.orientation.x = trackPoseTransformed_.getRotation().x();
                  output.pose.orientation.y = trackPoseTransformed_.getRotation().y();
                  output.pose.orientation.z = trackPoseTransformed_.getRotation().z();
                  output.pose.orientation.w = trackPoseTransformed_.getRotation().w();
      
                  subMapPublisher_.publish(output);
      
                  // Quick clear unordered_map
                  umap tmp;
                  localMap_.swap(tmp);
                  newLocalMapFlag = 0;
      
              }else if(initFlag == 1)
              {  // Init process
                  Eigen::Isometry3f prev_center = Eigen::Isometry3f::Identity();  // Initial map center
                  prev_center.pretranslate(Eigen::Vector3f(0.0, 0.0, 0.0));
                  trajectory_.push_back(prev_center);
      
                  PointXY localMapCenter_;
                  localMapCenter_.x = trajectory_.back().translation().x();
                  localMapCenter_.y = trajectory_.back().translation().y();
                  localMapLoc_.push_back(localMapCenter_);
                  umap tmp;
                  localMap_.swap(tmp);
      
                  optFlag = 0;
                  initFlag = 0;
                  JumpOdomFlag = 0;
                  JumpFlag = 0;
      
                  newLocalMapFlag = 0;
                  ROS_INFO("Map Stack Initialized.");
                  return;
              }  
          }
      
          // ROS_INFO("Move_x: %lf, Move_y: %lf ",delta_x, delta_y);
          int count = 0;
      
          if (1)
          {
              for (GridMapIterator iterator(prevMap_); !iterator.isPastEnd(); ++iterator) 
              {   // Add L shape infomation of the previous grid map into the local map
      
                  grid_map::Position position;
                  prevMap_.getPosition(*iterator, position);
                  index_x = (*iterator).transpose().x();
                  index_y = (*iterator).transpose().y();
                  index = index_x * length_ + index_y;
      
                  if(prevMap_.at("elevation", *iterator) != -10 && prevMap_.at("traver", *iterator) >= 0.0)
                  {
                      if (position.x() > (current_x + delta_x + length_ * resolution_ / 2.0f - 2.0f*resolution_) ||
                          position.x() < (current_x + delta_x - length_ * resolution_ / 2.0f + 2.0f*resolution_) ||
                          position.y() > (current_y + delta_y + length_ * resolution_ / 2.0f - 2.0f*resolution_) ||
                          position.y() < (current_y + delta_y - length_ * resolution_ / 2.0f + 2.0f*resolution_) ||
                          0)
                      { 
                          GridPoint save_pos(position.x(), position.y());
                          GridPointData save_data(prevMap_.at("elevation", *iterator), prevMap_.at("variance", *iterator), prevMap_.at("color_r", *iterator), prevMap_.at("color_g", *iterator), prevMap_.at("color_b", *iterator), prevMap_.at("intensity", *iterator), prevMap_.at("traver", *iterator));
      
                          // Update the grid information within local map
                          auto got = localMap_.find(save_pos);
                          if(got == localMap_.end())
                          {
                              localMap_.insert(make_pair(save_pos, save_data));
                          }else{
                              localMap_.erase(got);
                              localMap_.insert(make_pair(save_pos, save_data));
                              count++;
                          }
      
                          // Add information to real-time visualization
                          Anypoint pt;
                          pt.x = position.x();
                          pt.y = position.y();
                          pt.z = prevMap_.at("elevation", *iterator);
                          pt.r = prevMap_.at("color_r", *iterator);
                          pt.g = prevMap_.at("color_g", *iterator);
                          pt.b = prevMap_.at("color_b", *iterator);
                          pt.travers = prevMap_.at("traver", *iterator);
                          pt.intensity = prevMap_.at("intensity", *iterator);
                          pt.covariance = prevMap_.at("variance", *iterator);
                          visualCloud_.push_back(pt);
                      }
                  }
              }
          }
          JumpFlag = 0;
      }
      ````
      
    - 可视化点地图
  
      ````c++
      void ElevationMapping::visualPointMap()
      {
          if(!optFlag)
          {
              pointCloud::Ptr grid_pc;
              gridMaptoPointCloud(map_.visualMap_, grid_pc);
              sensor_msgs::PointCloud2 output;
              pcl::toROSMsg(visualCloud_+*grid_pc, output);
              // pcl::toROSMsg(visualCloud_, output);
              output.header.frame_id = mapFrameId;
              pointMapPublisher_.publish(output);
              // save the pointmap
              // pcl::io::savePCDFile("/home/nickle/Downloads/pointmap.pcd", visualCloud_+*grid_pc);
          }
      }
      ````
    
      
  
  - **5.9 光线追踪去除障碍物**
  
    ````c++
    // 光纤追踪处理
    Raytracing(length_);
    prevMap_ = map_.visualMap_;
    preMapAvail = true;
    ````
    
    - 光线追踪处理技术，调用.cu中的函数
    
      ````c++
      void Raytracing(int length)
      {
          int cell_num = length * length;
      	int threadsPerBlock = 256; 
          int blocksPerGrid =(cell_num + threadsPerBlock - 1) / threadsPerBlock; 
          
          G_Raytracing<<<blocksPerGrid, threadsPerBlock>>>(); 
          G_Clear_maplowest<<<blocksPerGrid, threadsPerBlock>>>(); 
          cudaError_t cudaStatus = cudaDeviceSynchronize();
          if (cudaStatus != cudaSuccess) 
          {
              fprintf(stderr, "addKernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
              //goto Error;
          }
      }
      ````
    
      调用.cu中的函数
    
      ````c++
      __global__ void G_Raytracing()
      {
          int i = blockDim.x * blockIdx.x + threadIdx.x; 
          if (i < Length * Length){
              if(map_traver[i] < obstacle_threshold && map_elevation[i] != -10)
              {
                  // Debug
                  // printf("i:%d,map_traver:%f\n", i, map_traver[i]);
                  // d_map_clean[i] = -1;
                  int cell_x = i / Length;
                  int cell_y = i % Length;
      
                  int robot_index;
                  int obstacle_indice[2];
                  StorageP2geoP(cell_x, cell_y, obstacle_indice);
      
                  float obstacle_ele = map_elevation[i];
                  int current_indice[2];
                  float increment[2];
                  int increment_x, increment_y;
                  current_indice[0] = obstacle_indice[0]; 
                  current_indice[1] = obstacle_indice[1]; 
      
                  if(Length % 2 == 0)
                  {
                      robot_index = (float)(Length / 2 - 0.5);
                      increment[0] = obstacle_indice[0] - robot_index;
                      increment[1] = obstacle_indice[1] - robot_index;
                  }
                  else
                  {
                      robot_index = (float)(Length / 2);
                      increment[0] = obstacle_indice[0] - robot_index;
                      increment[1] = obstacle_indice[1] - robot_index;
                  }
      
                  if(increment[0] > 0)
                      increment_x = 1;
                  else if(increment[0] == 0)
                      increment_x = 0;
                  else
                      increment_x = -1;    
      
                  if(increment[1] > 0)
                      increment_y = 1;
                  else if(increment[1] == 0)
                      increment_y = 0;
                  else
                      increment_y = -1;   
      
                  float obstacle_max_ele;
                  float obstacle_restrict_ele = obstacle_ele;
                  if(increment_x == 0 && increment_y == 0)
                      return ;
                  else if(increment_x == 0)
                  {
                      current_indice[1] += increment_y;
                      while(current_indice[1] >= 0 && current_indice[1] < Length)
                      {
                          if(P_isVaild(current_indice[0], current_indice[1]))
                          {
                              obstacle_max_ele = d_min_elevation(current_indice[0], current_indice[1], obstacle_indice[0], robot_index); 
                              //std::cout << "x:"<<current_indice[0] << " y:" << current_indice[1]<< std::endl;   
                              if(obstacle_max_ele < obstacle_restrict_ele)
                                  obstacle_restrict_ele = obstacle_max_ele;
                          }
                          current_indice[1] += increment_y;
                      }
                      return ;
                  }
                  else if(increment_y == 0)
                  {
                      current_indice[0] += increment_x;
                      while(current_indice[0] >= 0 && current_indice[0] < Length)
                      {
                          if(P_isVaild(current_indice[0], current_indice[1]))
                          {
                              obstacle_max_ele = d_min_elevation(current_indice[0], current_indice[1], obstacle_indice[0], robot_index);    
                              //std::cout << "x:"<<current_indice[0] << " y:" << current_indice[1]<< std::endl;  
                              if(obstacle_max_ele < obstacle_restrict_ele)
                                  obstacle_restrict_ele = obstacle_max_ele; 
                          }    
                          current_indice[0] += increment_x;
                      }
                      return ;
                  }
      
                  float dis = sqrt(increment[0] * increment[0] + increment[1] * increment[1]);
                  float dir[2];
                  dir[0] = increment[0] /dis;
                  dir[1] = increment[1] /dis;
      
                  float threshold;
                  if(fabs(increment[0]) > fabs(increment[1]))
                      threshold = sqrt(0.5 * 0.5 + pow(0.5/increment[0]*increment[1], 2));
                  else
                      threshold = sqrt(0.5 * 0.5 + pow(0.5/increment[1]*increment[0], 2));
      
      
                  float dir_num_x;
                  float dir_num_y;
      
                  float bound_increment_x = (float)increment_x/2;
                  float bound_increment_y = (float)increment_y/2;
      
                  dir_num_x = bound_increment_x / dir[0];
                  dir_num_y = bound_increment_y / dir[1];
      
                  float dir_num_later = 0;
      
                  // Debug
                  // std::cout << "threshold" << threshold << std::endl;
                  // std::cout << "x:"<<current_indice[0] << " y:" << current_indice[1]<< std::endl; 
                  while(current_indice[0] >= 0 && current_indice[0] < Length && current_indice[1] >= 0 && current_indice[1] < Length)
                  {
                      if(dir_num_x > dir_num_y)
                      {
                          if(dir_num_y - dir_num_later > threshold && current_indice[0] != obstacle_indice[0] && current_indice[1] != obstacle_indice[1])
                          {
                              if(P_isVaild(current_indice[0], current_indice[1]))
                              {
                                  obstacle_max_ele = d_min_elevation(current_indice[0], current_indice[1], obstacle_indice[0], robot_index);    
                                  if(obstacle_max_ele < obstacle_restrict_ele)
                                      obstacle_restrict_ele = obstacle_max_ele;
                                  //std::cout << "x:"<<current_indice[0] << " y:" << current_indice[1]<< std::endl;
                              }  
                          }
      
                          current_indice[1] += increment_y;
                          bound_increment_y += (float)increment_y;
                          dir_num_later = dir_num_y;
                          dir_num_y = bound_increment_y / dir[1];         
                      }
                      else if(dir_num_x < dir_num_y)
                      {
                          if(dir_num_x - dir_num_later > threshold && current_indice[0] != obstacle_indice[0] && current_indice[1] != obstacle_indice[1])
                          {
                              if(P_isVaild(current_indice[0], current_indice[1]))
                              {
                                  obstacle_max_ele = d_min_elevation(current_indice[0], current_indice[1], obstacle_indice[0], robot_index);    
                                  if(obstacle_max_ele < obstacle_restrict_ele)
                                      obstacle_restrict_ele = obstacle_max_ele; 
                                  // Debug 
                                  // std::cout << "x:"<<current_indice[0] << " y:" << current_indice[1]<< std::endl;
                              } 
                          }
                          current_indice[0] += increment_x;  
                          bound_increment_x += (float)increment_x;
                          dir_num_later = dir_num_x;
                          dir_num_x = bound_increment_x / dir[0];
                      }
                      else
                      {
                          if(dir_num_x - dir_num_later > threshold && current_indice[0] != obstacle_indice[0] && current_indice[1] != obstacle_indice[1])
                          {
                              if(P_isVaild(current_indice[0], current_indice[1]))
                              {
                                  obstacle_max_ele = d_min_elevation(current_indice[0], current_indice[1], obstacle_indice[0], robot_index);    
                                  if(obstacle_max_ele < obstacle_restrict_ele)
                                      obstacle_restrict_ele = obstacle_max_ele;  
                                  // Debug
                                  // std::cout << "x:"<<current_indice[0] << " y:" << current_indice[1]<< std::endl;
                              } 
                          }
                          current_indice[0] += increment_x;  
                          current_indice[1] += increment_y;  
                          bound_increment_x += (float)increment_x;
                          bound_increment_y += (float)increment_y;
                          dir_num_later = dir_num_x;
                          dir_num_x = bound_increment_x / dir[0];
                          dir_num_y = bound_increment_y / dir[1];
                      }
                  }
                  // Debug
                  // printf("height:%f,restrict_height:%f\n", obstacle_ele, obstacle_restrict_ele);
                  // d_map_clean[i] = obstacle_restrict_ele;
                  if(obstacle_ele - 8 * sqrt(map_variance[i])> obstacle_restrict_ele)
                      map_elevation[i] = -10;
              }
              //else
              //    d_map_clean[i] = 1;
          }
      }
      ````

----------------------------------------------------------------------------------------------------------分解线----------------------------------------------------------------------------------------------------------

- **6.汇总梳理**

  经过以上拆解，确认GEM功能的主要实现函数

  -  **一条线程执行Run()函数，负责四项功能**，分别是

    - 稠密地图构建触发信号，ROS_WARN("Starting dense submap build.");

    - 保存地图触发信号，ROS_WARN("Saving Global Map at: ....") ; 

    - 闭环检测触发信号，ROS_WARN("Start optimizing global map with submap num ...."); 


    - 新关键帧到达触发信号，ROS_WARN("NEW KEYFRAME! ....");

  - **同步lidar和camera话题，主回调函数 Callback()**

    sensor_msgs::PointCloud2ConstPtr

    sensor_msgs::Image::ConstPtr

    将激光点云转换为PCL pointcloud点云， 将image图像转换为cv::Mat矩阵

    

    读取相机内参数和激光外参数 T.camera * T.lidar, 将图像投影到点云上

    根据时间timeStamp更新

    sensorProcessor_->updateTransformations(timeStamp)      sensorProcessor为ETH的程序

    updatepointsMapLocation(timeStamp)  //更新点定位

    updateMapLocation()  //更新地图定位

    - 两条线程：

      processpoints

      processmapcells

  - 结尾

​    

































































































