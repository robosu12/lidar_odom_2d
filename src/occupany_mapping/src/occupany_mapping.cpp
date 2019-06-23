#include "occupany_mapping.h"
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point32.h"

#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include "nav_msgs/Path.h"

#include "thread"
#include "mutex"

#include "gaussian_newton_method.h"

const double GN_PI = 3.14159;
map_t *map;

class OccupanyMapping
{
    public:
    OccupanyMapping()
    {
        m_laserscanSub = m_nh.subscribe("sick_scan",1,&OccupanyMapping::rosLaserScanCallback,this);

        m_LaserOdomPub = m_nh.advertise<nav_msgs::Odometry>("laser_odom", 10);

        m_MapPub = m_nh.advertise<nav_msgs::OccupancyGrid>("laser_map",10,true);

        m_laserCloudPub = m_nh.advertise<sensor_msgs::PointCloud>("laser_cloud",10,true);

        m_markerPublisher = m_nh.advertise<visualization_msgs::Marker>("marker_point", 1, true);
        m_markerArrayPublisher = m_nh.advertise<visualization_msgs::MarkerArray>("marker_array", 1, true);

        m_odomPub = m_nh.advertise<nav_msgs::Path>("odom_path",10,true);
        m_gaussianNewtonPub = m_nh.advertise<nav_msgs::Path>("gaussian_newton_path",10,true);

        //设置地图信息
        SetMapParams();

        setMapTransformation(Eigen::Vector2d(-map->origin_x, -map->origin_y),map->resolution);

        Mapping_thead = std::thread(&OccupanyMapping::Mapping_Task, this);
        Publish_thead = std::thread(&OccupanyMapping::Publish_Task, this);

        PublishMap();

        m_LaserPose.setZero();
        m_LastLaserPose.setZero();
    }

        /**
     * Increments all the grid cells from (x0, y0) to (x1, y1);
     * //不包含(x1,y1)
     * 2D画线算法　来进行计算两个点之间的grid cell
     * @param x0
     * @param y0
     * @param x1
     * @param y1
     */
    std::vector<MapCoords> TraceLine(int x0, int y0, int x1, int y1)
    {
        MapCoords tmpIndex;
        std::vector<MapCoords> gridIndexVector;

        bool steep = abs(y1 - y0) > abs(x1 - x0);
        if (steep)
        {
            std::swap(x0, y0);
            std::swap(x1, y1);
        }
        if (x0 > x1)
        {
            std::swap(x0, x1);
            std::swap(y0, y1);
        }

        int deltaX = x1 - x0;
        int deltaY = abs(y1 - y0);
        int error = 0;
        int ystep;
        int y = y0;

        if (y0 < y1)
        {
            ystep = 1;
        }
        else
        {
            ystep = -1;
        }

        int pointX;
        int pointY;
        for (int x = x0; x <= x1; x++)
        {
            if (steep)
            {
                pointX = y;
                pointY = x;
            }
            else
            {
                pointX = x;
                pointY = y;
            }

            error += deltaY;

            if (2 * error >= deltaX)
            {
                y += ystep;
                error -= deltaX;
            }

            //不包含最后一个点．
            if(pointX == x1 && pointY == y1) continue;

            //保存所有的点
            tmpIndex.SetIndex(pointX,pointY);

            gridIndexVector.push_back(tmpIndex);
        }

        return gridIndexVector;
    }

    //进行角度正则化．
    double NormalizationAngle(double angle)
    {
        if(angle > GN_PI)
            angle -= GN_PI*2;
        else if(angle < -GN_PI)
            angle += GN_PI*2;

        return angle;
    }
    
    // Create a new map
    void SetMapParams(void )
    {
        map = (map_t*) malloc(sizeof(map_t));

        map->width = 6000;
        map->height =6000;
        map->resolution = 0.025;

        //每次被集中的log变化值
        map->log_free = -1.0;
        map->log_occ = 2.0;

        //每个栅格的最大最小值．
        map->log_max = 50.0;
        map->log_min = -50.0;

        float totalMapSizeX = map->resolution * static_cast<float>(map->width);
        float mid_offset_x = totalMapSizeX * 0.4;

        float totalMapSizeY = map->resolution * static_cast<float>(map->height);
        float mid_offset_y = totalMapSizeY * 0.4;

        map->origin_x = -mid_offset_x;
        map->origin_y = -mid_offset_y;

        //地图的原点，在地图的正中间
        //    map->offset_x = 300; 
        //    map->offset_y = 600;

        map->offset_x = 0; 
        map->offset_y = 0;

        //    map->offset_x = 450; 
        //    map->offset_y = 450;

        // map_cell_t new_cells[map->width*map->height];

        map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->width*map->height);

        //初始化为50
        for(unsigned int i = 0; i < map->width * map->height;i++)
        {
            // map->cells[i] = 50;
            // map->cells[i].logodd_val = 50;
        }
                
    }

    //从世界坐标系转换到栅格坐标系
    MapCoords ConvertWorld2MapCoords(double x,double y)
    {
        MapCoords index;

        // index.x = std::ceil((x - map->origin_x) / map->resolution) + map->offset_x;
        // index.y = std::ceil((y - map->origin_y) / map->resolution) + map->offset_y;

        index.x = floor((x - map->origin_x) / map->resolution + 0.5) + map->offset_x;
        index.y = floor((y - map->origin_y) / map->resolution + 0.5) + map->offset_y;

        return index;
    }

    int MapCoordsToLinearIndex(MapCoords index)
    {
        int linear_index;
        linear_index = index.x + index.y * map->width;
    }


    //判断index是否有效
    bool isValidMapCoords(MapCoords index)
    {
        if(index.x >= 0 && index.x < map->width && index.y >= 0 && index.y < map->height)
            return true;

        return false;
    }

     bool isValidMapCoords(Eigen::Vector2i point)
    {
        if(point.x() >= 0 && point.x() < map->width && point.y() >= 0 && point.y() < map->height)
            return true;

        return false;
    }

    // Destroy a map
    void map_free(map_t *map)
    {
        free(map->cells);
        free(map);
        std::cout <<"Release map Memory!!"<<std::endl;
        return;
    }

    void creat_init_map(std::vector<Eigen::Vector2d> Laser_pts, Eigen::Vector3d robotPose)
    {
        Mapping(Laser_pts, robotPose);
    }

    //
    // void Mapping(const sensor_msgs::LaserScanConstPtr & scan, Eigen::Vector3d & robotPose)
    // {
    //     //机器人的下标
    //     MapCoords robotIndex = ConvertWorld2MapCoords(robotPose(0),robotPose(1));
    //     if(!isValidMapCoords(robotIndex)) return;
    //     //Get pose in map coordinates from pose in world coordinates
    //     Eigen::Vector3d mapPose(getMapCoordsPose(robotPose));
    //     //Get a 2D homogenous transform that can be left-multiplied to a robot coordinates vector to get world coordinates of that vector
    //     Eigen::Affine2f poseTransform((Eigen::Translation2f(mapPose[0], mapPose[1]) * Eigen::Rotation2Df(mapPose[2])));

    //     std::vector<Eigen::Vector2d> Laser_pts;
    //     // ConvertLaserScanToEigenPointCloud(scan,Laser_pts);
    //     ConvertLaserScanToMapPointCloud(msg, LaserPts, map->resolution );

    //     m_laserCloud.header.stamp = ros::Time::now();
    //     m_laserCloud.header.frame_id = "odom";
    //     m_laserCloud.points.resize(Laser_pts.size());

    //     m_laserCloud.channels.resize(1);
    //     m_laserCloud.channels[0].name = "distance";
    //     m_laserCloud.channels[0].values.resize(Laser_pts.size());

    //     for(int id = 0; id < Laser_pts.size();id++)
    //     {
    //         //计算得到该激光点的世界坐标系的坐标
    //         // double theta = -robotPose(2); // 激光雷达逆时针转，角度取反
    //         double theta = robotPose(2); // 激光雷达逆时针转，角度取反
    //         double world_x = cos(theta) * Laser_pts[id](0) - sin(theta) * Laser_pts[id](1) + robotPose(0);
    //         double world_y = sin(theta) * Laser_pts[id](0) + cos(theta) * Laser_pts[id](1) + robotPose(1);

    //         //start of TODO 对对应的map的cell进行更新．

    //         MapCoords temp_laser_index = ConvertWorld2MapCoords(world_x,world_y);
    //         if(!isValidMapCoords(temp_laser_index)) continue;

    //         //计算激光雷达光束经过的地图网格，并对这些地图网格进行free值增加；
    //         std::vector<MapCoords> trace_grid = TraceLine(robotIndex.x,robotIndex.y,temp_laser_index.x,temp_laser_index.y);

    //         for(int j=0; j<trace_grid.size(); j++)
    //         {
    //             int map_index = MapCoordsToLinearIndex(trace_grid[j]);
    //             map->cells[map_index].logodd_val += map->log_free;
    //             if(map->cells[map_index].logodd_val<map->log_min) map->cells[map_index].logodd_val = map->log_min;
    //         }

    //         //激光雷达光束击中的地图网格，并对这些地图网格进行occ值增加；
    //         int shot_point_index = MapCoordsToLinearIndex(temp_laser_index);
    //         map->cells[shot_point_index].logodd_val += map->log_occ;
    //         if(map->cells[shot_point_index].logodd_val >map->log_max) map->cells[shot_point_index].logodd_val = map->log_max;
            
    //         m_laserCloud.points[id].x = world_x;
    //         m_laserCloud.points[id].y = world_y;

    //         m_laserCloud.points[id].z = 0;
    //         m_laserCloud.channels[0].values[id] = 100;
            
            
    //         //end of TODO
    //     }
    // }

    Eigen::Matrix3d GN_V2T(Eigen::Vector3d vec)
    {
        Eigen::Matrix3d T;
        T  << cos(vec(2)),-sin(vec(2)),vec(0),
            sin(vec(2)), cos(vec(2)),vec(1),
                        0,           0,     1;

        return T;
    }

    //对某一个点进行转换．
    Eigen::Vector2d GN_TransPoint(Eigen::Vector2d pt,Eigen::Matrix3d T)
    {
        Eigen::Vector3d tmp_pt(pt(0),pt(1),1);
        tmp_pt = T * tmp_pt;
        return Eigen::Vector2d(tmp_pt(0),tmp_pt(1));
    }

    void Mapping(std::vector<Eigen::Vector2d> Laser_pts, Eigen::Vector3d robotPose)
    {
        // Eigen::Vector3d mapPose(getMapCoordsPose(robotPose));
        // Eigen::Affine2d poseTransform((Eigen::Translation2d(mapPose[0], mapPose[1]) * Eigen::Rotation2Dd(mapPose[2])));
        // mapPose.array() += (0.5f);
        // Eigen::Vector3i begain_point(mapPose.cast<int>());
        // if(!isValidMapCoords(begain_point.head<2>())) return;

        // // std::vector<Eigen::Vector2d> Laser_pts;
        // // Laser_pts = ConvertLaserScanToMapPointCloud(scan, map->resolution );

        // m_laserCloud.header.stamp = ros::Time::now();
        // m_laserCloud.header.frame_id = "robot_laser";
        // m_laserCloud.points.resize(Laser_pts.size());

        // m_laserCloud.channels.resize(1);
        // m_laserCloud.channels[0].name = "distance";
        // m_laserCloud.channels[0].values.resize(Laser_pts.size());

        // for(int id = 0; id < Laser_pts.size();id++)
        // {
        //     //计算得到该激光点的地图坐标系的坐标
        //     Eigen::Vector2d scanEndMapf(poseTransform * Laser_pts[id]);
        //     scanEndMapf.array() += (0.5f);
        //     Eigen::Vector2i end_point(scanEndMapf.cast<int>());

        //     if(!isValidMapCoords(end_point)) continue;
            
        //     //start of TODO 对对应的map的cell进行更新．

        //     //计算激光雷达光束经过的地图网格，并对这些地图网格进行free值增加；
        //     std::vector<MapCoords> trace_grid = TraceLine(begain_point.x(),begain_point.y(),end_point.x(),end_point.y());
    
        //     for(int j=0; j<trace_grid.size(); j++)
        //     {
        //         int map_index = MapCoordsToLinearIndex(trace_grid[j]);
        //         map->cells[map_index].logodd_val += map->log_free;
        //         if(map->cells[map_index].logodd_val<map->log_min) map->cells[map_index].logodd_val = map->log_min;
        //     }

        //     //激光雷达光束击中的地图网格，并对这些地图网格进行occ值增加；
        //     MapCoords temp_laser_index;
        //     temp_laser_index.x = end_point.x();
        //     temp_laser_index.y = end_point.y();
        //     int shot_point_index = MapCoordsToLinearIndex(temp_laser_index);
        //     map->cells[shot_point_index].logodd_val += map->log_occ;
        //     if(map->cells[shot_point_index].logodd_val >map->log_max) map->cells[shot_point_index].logodd_val = map->log_max;
            
        //     m_laserCloud.points[id].x = Laser_pts[id].x() ;
        //     m_laserCloud.points[id].y = Laser_pts[id].y() ;
        //     m_laserCloud.points[id].z = 0;
        //     m_laserCloud.channels[0].values[id] = 100;
            
        //     //end of TODO
        // }

        Eigen::Matrix3d Trans = GN_V2T(robotPose);
        Eigen::Vector3d mapPose(get_map_pose(robotPose));
        mapPose.array() += (0.5f);
        Eigen::Vector3i begain_point(mapPose.cast<int>());
        if(!isValidMapCoords(begain_point.head<2>())) return;

        m_laserCloud.header.stamp = ros::Time::now();
        m_laserCloud.header.frame_id = "robot_laser";
        m_laserCloud.points.resize(Laser_pts.size());

        m_laserCloud.channels.resize(1);
        m_laserCloud.channels[0].name = "distance";
        m_laserCloud.channels[0].values.resize(Laser_pts.size());

        for(int id = 0; id < Laser_pts.size();id++)
        {
            // 将激光点从雷达坐标系转化到世界坐标系；
            Eigen::Vector2d tmp_pt = GN_TransPoint(Laser_pts[id],Trans);
            // 从世界坐标转化到地图坐标
            Eigen::Vector2d map_point = get_map_coords(tmp_pt);
            Eigen::Vector2i end_point(map_point.cast<int>());

            if(!isValidMapCoords(end_point)) continue;
            
            //start of TODO 对对应的map的cell进行更新．

            //计算激光雷达光束经过的地图网格，并对这些地图网格进行free值增加；
            std::vector<MapCoords> trace_grid = TraceLine(begain_point.x(),begain_point.y(),end_point.x(),end_point.y());
    
            for(int j=0; j<trace_grid.size(); j++)
            {
                int map_index = MapCoordsToLinearIndex(trace_grid[j]);
                map->cells[map_index].logodd_val += map->log_free;
                if(map->cells[map_index].logodd_val<map->log_min) map->cells[map_index].logodd_val = map->log_min;
            }

            //激光雷达光束击中的地图网格，并对这些地图网格进行occ值增加；
            MapCoords temp_laser_index;
            temp_laser_index.x = end_point.x();
            temp_laser_index.y = end_point.y();
            int shot_point_index = MapCoordsToLinearIndex(temp_laser_index);
            map->cells[shot_point_index].logodd_val += map->log_occ;
            if(map->cells[shot_point_index].logodd_val >map->log_max) map->cells[shot_point_index].logodd_val = map->log_max;
            
            m_laserCloud.points[id].x = Laser_pts[id].x() ;
            m_laserCloud.points[id].y = Laser_pts[id].y() ;
            m_laserCloud.points[id].z = 0;
            m_laserCloud.channels[0].values[id] = 100;
            
            //end of TODO
        }
    }

    //单纯的数据类型转换，不进行坐标系转换．
    void ConvertLaserScanToEigenPointCloud(const sensor_msgs::LaserScanConstPtr& msg,
                                                std::vector<Eigen::Vector2d>& eigen_pts)
    {
        //转换到矫正需要的数据
        std::vector<double> angles,ranges;
        ros::Time startTime, endTime;

        //得到最终点的时间
        startTime = msg->header.stamp;
        int beamNum = msg->ranges.size();
        endTime = startTime + ros::Duration(msg->time_increment * beamNum);
        
        double tem_angle;
        eigen_pts.clear();
        for(int i = 0; i < msg->ranges.size();i++)
        {
            if(msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max)
                continue;
            if(std::isinf(msg->ranges[i]) || std::isnan(msg->ranges[i])) 
                continue;

            tem_angle = msg->angle_min + msg->angle_increment * i;

            // ranges.push_back(msg->ranges[i]);
            // angles.push_back(tem_angle);

            double lx = msg->ranges[i] * std::cos(tem_angle);
            double ly = msg->ranges[i] * std::sin(tem_angle);

            if(std::isnan(lx) || std::isinf(ly) || std::isnan(ly) || std::isinf(ly))
                continue;

            eigen_pts.push_back(Eigen::Vector2d(lx,ly));
        }
    }

     //将原始激光数据，转换到车体局部地图坐标系的点云数据；
    std::vector<Eigen::Vector2d> ConvertLaserScanToMapPointCloud(const sensor_msgs::LaserScanConstPtr& msg, double factor)
    {
        double tem_angle,tem_range;

        std::vector<Eigen::Vector2d> pts;
        for(int i = 0; i < msg->ranges.size();i++)
        {
            if(msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max)
                continue;
            if(std::isinf(msg->ranges[i]) || std::isnan(msg->ranges[i])) 
                continue;

            tem_angle = msg->angle_min + msg->angle_increment * i;
            tem_range = msg->ranges[i] / factor;
            // tem_range = msg->ranges[i];

            double lx = tem_range * cos(tem_angle);
            double ly = tem_range * sin(tem_angle);

            if(std::isnan(lx) || std::isinf(ly) || std::isnan(ly) || std::isinf(ly))
                continue;

            pts.push_back(Eigen::Vector2d(lx,ly));
        }
        return pts;
    }

    void transform_test(void)
    {
        static Eigen::Vector3d pose1, pose2;
        pose1(0) = MAP_WXtoMX(map,m_OdomPose(0));
        pose1(1) = MAP_WYtoMY(map,m_OdomPose(1));
        pose1(2) = m_OdomPose(2);

        pose2 = getMapCoordsPose(m_OdomPose);

        std::cout <<"pose1: " << pose1.transpose() <<" , pose2: "<< pose2.transpose() <<std::endl;
    }

    void rosLaserScanCallback(const sensor_msgs::LaserScanConstPtr& msg)
    {
        static bool init_map_flag=false;
        static char init_count=0;
        
        if(getOdomPose(msg->header.stamp,m_OdomPose) == false)
        {
            std::cout <<"Failed to get Odom Pose"<<std::endl;
            return ;
        }

        std::cout <<"getOdomPose : " << m_OdomPose.transpose() <<std::endl;
       
        // 数据类型转换．
        ConvertLaserScanToEigenPointCloud(msg, LaserPts);

        if(init_map_flag == false)
        {
            m_LaserPose = m_OdomPose;
            creat_init_map(LaserPts, m_LaserPose);

            if(init_count++ > 20)
            {
                init_map_flag=true;
                std::cout <<" finish creat_init_map..."<<std::endl;
            }
        }
        else
        {
            PridictPose(m_LastOdomPose, m_OdomPose, m_LastLaserPose, m_LastLaserPose);

            std::cout <<"PridictPose : " << m_LastLaserPose.transpose() <<std::endl;

            m_LaserPose = GN_optimizer.GaussianNewtonOptimization(map, m_LastLaserPose, LaserPts);

            Mapping(LaserPts, m_LaserPose);
            // transform_test();
        }

        //保存路径．
        m_odomPath.push_back(m_OdomPose);
        m_gaussianNewtonPath.push_back(m_LaserPose);
       
        m_LastLaserPose = m_LaserPose;
        m_LastOdomPose = m_OdomPose;

        //更新数据．
        m_LaserOdomProcessFlag = 1;

        PublishMap();
        publishLaserOdom(m_LaserPose);
        DrawMarkerPoint(m_LaserPose);
        m_laserCloudPub.publish(m_laserCloud);
        PublishPath(m_odomPub,m_odomPath);
        PublishPath(m_gaussianNewtonPub,m_gaussianNewtonPath);

        //std::cout <<"pub map!!!"<<std::endl;
    }

    void Mapping_Task(void)
    {
        while(true)
        {
            if(m_LaserOdomProcessFlag  == 1)
            {
                mutex_process2.lock();

                // Mapping(LaserPts, m_LaserPose);

                m_LaserOdomProcessFlag = 0;
                m_LaserMappingFlag = 1;

                mutex_process2.unlock();
            }
            std::chrono::milliseconds dura(5);
            std::this_thread::sleep_for(dura);
        }
    }

    void Publish_Task(void)
    {
        while(true)
        {
            if(m_LaserMappingFlag  == 1)
            {
                mutex_process1.lock();

                // PublishMap();
                // publishLaserOdom(m_LaserPose);
                // DrawMarkerPoint(m_LaserPose);
                // m_laserCloudPub.publish(m_laserCloud);
                // PublishPath(m_odomPub,m_odomPath);
                // PublishPath(m_gaussianNewtonPub,m_gaussianNewtonPath);

                m_LaserMappingFlag = 0;

                mutex_process1.unlock();
            }

            std::chrono::milliseconds dura(5);
            std::this_thread::sleep_for(dura);
        }
        
    }

    void PridictPose(Eigen::Vector3d LastOdomPose,  Eigen::Vector3d NowOdomPose, 
                     Eigen::Vector3d LastRobotPose, Eigen::Vector3d & PridictPose )
    {
        //初始解为上一帧激光位姿+运动增量
        Eigen::Vector3d deltaPose = NowOdomPose - LastOdomPose;
        deltaPose(2) = NormalizationAngle(deltaPose(2));

        Eigen::Matrix3d R_robot = Eigen::Matrix3d::Zero();
        double theta = LastRobotPose(2);
        R_robot << cos(theta), -sin(theta), 0, 
                   sin(theta),  cos(theta), 0,
                        0,          0,      1;

        Eigen::Matrix3d R_odom = Eigen::Matrix3d::Zero();
        theta = LastOdomPose(2);
        R_odom << cos(theta), -sin(theta), 0, 
                  sin(theta),  cos(theta), 0,
                       0,          0,      1;
        //将odom下的增量转换到激光里程计坐标下；增量只是相对于各自坐标系原点起的一个线段，是一个相对值，知道长度就够了，不需要对齐；
        PridictPose = LastRobotPose + R_robot * R_odom.transpose() * deltaPose;
        PridictPose(2) = NormalizationAngle(PridictPose(2));
    }

    void PublishPath(ros::Publisher& puber, std::vector<Eigen::Vector3d>& path)
    {
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "/odom";

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "/odom";
        for(int i = 0; i < path.size();i++)
        {
            Eigen::Vector3d traj_node = path[i];
            pose.pose.position.x = traj_node(0);
            pose.pose.position.y = traj_node(1);
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(traj_node(2));
            path_msg.poses.push_back(pose);
        }

        puber.publish(path_msg);
    }

    bool getLaserOdomPose(ros::Time t, Eigen::Vector3d& pose)
    {
        // Get the robot's pose
        tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                                tf::Vector3(0,0,0)), t, "/base_laser");
        tf::Stamped<tf::Transform> odom_pose;
        try
        {
            m_tfListener.transformPose("/LaserOdom", ident, odom_pose);
        }
        catch(tf::TransformException e)
        {
            ROS_WARN("Failed to compute LaserOdom pose, skipping scan (%s)", e.what());
            return false;
        }

        double yaw = tf::getYaw(odom_pose.getRotation());
        pose << odom_pose.getOrigin().x(),odom_pose.getOrigin().y(),yaw;

        return true;
    }

    bool getOdomPose(ros::Time t,Eigen::Vector3d& pose)
    {
        // Get the robot's pose
        tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                                   tf::Vector3(0,0,0)), t, "/base_laser");
        tf::Stamped<tf::Transform> odom_pose;
        try
        {
            m_tfListener.transformPose("/odom", ident, odom_pose);
        }
        catch(tf::TransformException e)
        {
            ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
            return false;
        }

        double yaw = tf::getYaw(odom_pose.getRotation());
        pose << odom_pose.getOrigin().x(),odom_pose.getOrigin().y(),yaw;

        return true;
    }

    //发布地图．
    void PublishMap(void)
    {
        nav_msgs::OccupancyGrid rosMap;

        rosMap.info.resolution = map->resolution;
        rosMap.info.origin.position.x = map->origin_x;
        rosMap.info.origin.position.y = map->origin_y;
        rosMap.info.origin.position.z = 0.0;
        rosMap.info.origin.orientation.x = 0.0;
        rosMap.info.origin.orientation.y = 0.0;
        rosMap.info.origin.orientation.z = 0.0;
        rosMap.info.origin.orientation.w = 1.0;

        rosMap.info.width = map->width;
        rosMap.info.height = map->height;
        rosMap.data.resize(rosMap.info.width * rosMap.info.height);

        int sizeX = map->width;
        int sizeY = map->height;
        unsigned int size = sizeX * sizeY;

        std::vector<int8_t>& data = rosMap.data;

        //std::vector contents are guaranteed to be contiguous, use memset to set all to unknown to save time in loop
        memset(&data[0], -1, sizeof(int8_t) * size);

        //0~100
        for(unsigned int i = 0; i < map->width * map->height;i++)
        {
            if(fabs(map->cells[i].logodd_val) > 1 )
            {
                rosMap.data[i] = map->cells[i].logodd_val + 50 ;
            }
        }

        rosMap.header.stamp = ros::Time::now();
        rosMap.header.frame_id = "map";

        m_MapPub.publish(rosMap);
    }

    void publishLaserOdom(const Eigen::Vector3d& tempLaserPose)
    {
        static Eigen::Vector3d LastLaserPose;
        geometry_msgs::TransformStamped OdomTf;
        tf::TransformBroadcaster laserOdomTf_broadcaster;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(m_LaserPose(2));

        ros::Time current_time = ros::Time::now();

        OdomTf.header.stamp = current_time;
        OdomTf.header.frame_id = "laser_odom";
        OdomTf.child_frame_id = "robot_laser";

        OdomTf.transform.translation.x = tempLaserPose(0);
        OdomTf.transform.translation.y = tempLaserPose(1);
        OdomTf.transform.translation.z = 0.0;
        OdomTf.transform.rotation = odom_quat;
        //send the transform
        laserOdomTf_broadcaster.sendTransform(OdomTf);

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "laser_odom";
        odom.child_frame_id = "robot_laser";

        //set the position
        odom.pose.pose.position.x = tempLaserPose(0);
        odom.pose.pose.position.y = tempLaserPose(1);
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        //set the velocity
        odom.twist.twist.linear.x = m_LastLaserPose(0) - tempLaserPose(0);
        odom.twist.twist.linear.y = m_LastLaserPose(1) - tempLaserPose(1);
        odom.twist.twist.angular.z = m_LastLaserPose(2) - tempLaserPose(2);
        LastLaserPose = tempLaserPose;
        //publish the message
        m_LaserOdomPub.publish(odom);

    }

    void DrawMarkerPoint(const Eigen::Vector3d& poseWorldFrame)
    {
        visualization_msgs::Marker tempMarker;

        // uint8 ARROW=0
        // uint8 CUBE=1
        // uint8 SPHERE=2
        // uint8 CYLINDER=3
        // uint8 LINE_STRIP=4
        // uint8 LINE_LIST=5
        // uint8 CUBE_LIST=6
        // uint8 SPHERE_LIST=7
        // uint8 POINTS=8
        // uint8 TEXT_VIEW_FACING=9
        // uint8 MESH_RESOURCE=10
        // uint8 TRIANGLE_LIST=11

        // tempMarker.id = m_markerIdCounter++;
        tempMarker.id = 0;

        tempMarker.header.frame_id = "odom";
        tempMarker.header.stamp = ros::Time::now();
        tempMarker.id = 0;
        tempMarker.ns = "robot_position";
        tempMarker.type = visualization_msgs::Marker::CUBE;
        tempMarker.action = visualization_msgs::Marker::ADD;

        tempMarker.pose.position.x = poseWorldFrame(0);
        tempMarker.pose.position.y = poseWorldFrame(1);
        tempMarker.pose.position.z = 0.0;

        tempMarker.pose.orientation = tf::createQuaternionMsgFromYaw(poseWorldFrame(2));

        tempMarker.scale.x = 1.0;
        tempMarker.scale.y = 1.0;
        tempMarker.scale.z = 1.0;
 
        tempMarker.color.r = 0.0f;
        tempMarker.color.g = 0.0f;
        tempMarker.color.b = 1.0f;
        tempMarker.color.a = 1.0;
 
        tempMarker.lifetime = ros::Duration();

        m_markerPublisher.publish(tempMarker);
    }

    void AddMarkerDrawPoint(const Eigen::Vector3d& poseWorldFrame)
    {
        visualization_msgs::Marker tempMarker;

        tempMarker.id = m_markerIdCounter++;

        tempMarker.pose.position.x = poseWorldFrame.x();
        tempMarker.pose.position.y = poseWorldFrame.y();
        tempMarker.pose.position.z = 0.0;

        tempMarker.pose.orientation = tf::createQuaternionMsgFromYaw(poseWorldFrame.z());
        tempMarker.type = visualization_msgs::Marker::CUBE;

        //m_markerPublisher.publish(tempMarker);

        m_markerArray.markers.push_back(tempMarker);
    }

    void sendAndResetMarkerArrayData(void)
    {
        m_markerArrayPublisher.publish(m_markerArray);
        m_markerArray.markers.clear();
        m_markerIdCounter = 0;
    }

    void setMapTransformation(const Eigen::Vector2d& Offset, double cellLength)
    {
        double scaleToMap = 1.0 / cellLength;

        mapTworld = Eigen::AlignedScaling2d(scaleToMap, scaleToMap) * Eigen::Translation2d(Offset[0], Offset[1]);

        worldTmap = mapTworld.inverse();
    }

    // Returns the world coordinates for the given map coords.
    inline Eigen::Vector2d getWorldCoords(const Eigen::Vector2d& mapCoords) const
    {
        return worldTmap * mapCoords;
    }
    
    // Returns the map coordinates for the given world coords.
    inline Eigen::Vector2d getMapCoords(const Eigen::Vector2d& worldCoords) const
    {
        return mapTworld * worldCoords;
    }
    
    // Returns the world pose for the given map pose.
    inline Eigen::Vector3d getWorldCoordsPose(const Eigen::Vector3d& mapPose) const
    {
        Eigen::Vector2d worldCoords (worldTmap * mapPose.head<2>());
        return Eigen::Vector3d(worldCoords[0], worldCoords[1], mapPose[2]);
    }
    
    // Returns the map pose for the given world pose.
    inline Eigen::Vector3d getMapCoordsPose(const Eigen::Vector3d& worldPose) const
    {
        Eigen::Vector2d mapCoords (mapTworld * worldPose.head<2>());
        return Eigen::Vector3d(mapCoords[0], mapCoords[1], worldPose[2]);
    }

    inline Eigen::Vector2d get_map_coords(const Eigen::Vector2d& worldCoords) const
    {
        Eigen::Vector2d map_coords;
        map_coords(0) = MAP_WXtoMX(map,worldCoords(0));
        map_coords(1) = MAP_WYtoMY(map,worldCoords(1));
        return map_coords;
    }

    inline Eigen::Vector3d get_map_pose(const Eigen::Vector3d& worldPose) const
    {
        Eigen::Vector2d mapCoords;
        mapCoords = get_map_coords(worldPose.head<2>());
        return Eigen::Vector3d(mapCoords[0], mapCoords[1], worldPose[2]);
    }

    inline Eigen::Vector2d get_world_coords(const Eigen::Vector2d& mapCoords) const
    {
        Eigen::Vector2d world_coords;
        world_coords(0) = MAP_MXtoWX(map,mapCoords(0));
        world_coords(1) = MAP_MYtoWY(map,mapCoords(1));
        return world_coords;
    }

    inline Eigen::Vector3d get_world_pose(const Eigen::Vector3d& mapPose) const
    {
        Eigen::Vector2d worldCoords;
        worldCoords = get_world_coords(mapPose.head<2>());
        return Eigen::Vector3d(worldCoords[0], worldCoords[1], mapPose[2]);
    }

    //  Get the probability value represented by the grid cell.
    double getGridProbability(MapCoords index) 
    {
        int linear_index = index.x + index.y * map->width;
        double log_val = map->cells[linear_index].logodd_val ;
        double odds = exp(log_val);
        return odds / (odds + 1.0f);
    }

    double getGridProbability(int index) 
    {
        double log_val = map->cells[index].logodd_val ;
        double odds = exp(log_val);
        return odds / (odds + 1.0f);
    }

    ros::NodeHandle m_nh;

    std::vector<Eigen::Vector2d> LaserPts;
    Eigen::Vector3d m_LaserPose,m_LastLaserPose;
    Eigen::Vector3d m_OdomPose,m_LastOdomPose;
    Eigen::Vector3d m_RobotPose,m_LastRobotPose;

    std::vector<Eigen::Vector2d> m_LaserPoints,m_prevLaserPoints;

    ros::Subscriber m_laserscanSub;

    ros::Publisher m_LaserOdomPub;
    tf::TransformListener m_tfListener;
    tf::TransformBroadcaster m_LaserodomTfBroadcaster;

    tf::TransformBroadcaster m_mapOdomTfPub;

    ros::Publisher m_MapPub;
    ros::Publisher m_laserCloudPub;
    sensor_msgs::PointCloud m_laserCloud;

    std::vector<Eigen::Vector3d> robotPoses;
    std::vector<GeneralLaserScan> generalLaserScans;

    ros::Publisher m_odomPub;
    ros::Publisher m_gaussianNewtonPub;
    std::vector<Eigen::Vector3d> m_odomPath;
    std::vector<Eigen::Vector3d> m_gaussianNewtonPath;

    Eigen::Affine2d mapTworld;     ///< Homogenous 2D transform from world to map coordinates.
    Eigen::Affine2d worldTmap;     ///< Homogenous 2D transform from map to world coordinates.

    ros::Publisher m_markerPublisher;
    ros::Publisher m_markerArrayPublisher;
    visualization_msgs::MarkerArray m_markerArray;
    visualization_msgs::Marker m_tempMarker;
    int m_markerIdCounter;

    char m_LaserOdomProcessFlag, m_LaserMappingFlag;
    std::thread Publish_thead;
    std::thread Mapping_thead;
    std::mutex mutex_process1;
    std::mutex mutex_process2;
    
    gaussian_newton_optimize GN_optimizer;

};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_odom_and_mapping");

    OccupanyMapping  Mapper;

    std::cout <<"OccupanyMapping begain, wating for data !!!"<<std::endl;

    ros::spin();

    Mapper.map_free(map);

    std::cout <<"OccupanyMapping end !!!"<<std::endl;

    return 0;

}




