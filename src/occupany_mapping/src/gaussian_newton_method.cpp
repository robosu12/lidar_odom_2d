#include "gaussian_newton_method.h"
#include "occupany_mapping.h"

const double GN_PI = 3.1415926;

//进行角度正则化．
double  gaussian_newton_optimize::GN_NormalizationAngle(double angle)
{
    if(angle > GN_PI)
        angle -= 2*GN_PI;
    else if(angle < -GN_PI)
        angle += 2*GN_PI;

    return angle;
}

Eigen::Matrix3d gaussian_newton_optimize::GN_V2T(Eigen::Vector3d vec)
{
    Eigen::Matrix3d T;
    T  << cos(vec(2)),-sin(vec(2)),vec(0),
        sin(vec(2)), cos(vec(2)),vec(1),
                    0,           0,     1;

    return T;
}

Eigen::Affine2d gaussian_newton_optimize::getTransformForState(const Eigen::Vector3d& transVector)
{
    return Eigen::Translation2d(transVector(0), transVector(1)) * Eigen::Rotation2Dd(transVector(2));
}

//对某一个点进行转换．
Eigen::Vector2d gaussian_newton_optimize::GN_TransPoint(Eigen::Vector2d pt,Eigen::Matrix3d T)
{
    Eigen::Vector3d tmp_pt(pt(0),pt(1),1);
    tmp_pt = T * tmp_pt;
    return Eigen::Vector2d(tmp_pt(0),tmp_pt(1));
}

Eigen::Vector2d gaussian_newton_optimize::get_map_coords(const Eigen::Vector2d& worldCoords)
{
    Eigen::Vector2d map_coords;
    map_coords(0) = MAP_WXtoMX(map,worldCoords(0));
    map_coords(1) = MAP_WYtoMY(map,worldCoords(1));
    return map_coords;
}

Eigen::Vector3d gaussian_newton_optimize::get_map_pose(const Eigen::Vector3d& worldPose)
{
    Eigen::Vector2d mapCoords;
    mapCoords = get_map_coords(worldPose.head<2>());
    return Eigen::Vector3d(mapCoords(0), mapCoords(1), worldPose(2));
}

Eigen::Vector2d gaussian_newton_optimize::get_world_coords(const Eigen::Vector2d& mapCoords)
{
    Eigen::Vector2d world_coords;
    world_coords(0) = MAP_MXtoWX(map,mapCoords(0));
    world_coords(1) = MAP_MYtoWY(map,mapCoords(1));
    return world_coords;
}

Eigen::Vector3d gaussian_newton_optimize::get_world_pose(const Eigen::Vector3d& mapPose)
{
    Eigen::Vector2d worldCoords;
    worldCoords = get_world_coords(mapPose.head<2>());
    return Eigen::Vector3d(worldCoords(0), worldCoords(1), mapPose(2));
}


/**
 * @brief InterpMapValueWithDerivatives
 * 在地图上的进行插值，得到coords处的势场值和对应的关于位置的梯度．
 * 返回值为Eigen::Vector3d ans
 * ans(0)表示市场值
 * ans(1:2)表示梯度
 * @param map
 * @param coords
 * @return
 */
Eigen::Vector3d gaussian_newton_optimize::InterpMapValueWithDerivatives(map_t* map, const Eigen::Vector2d& coords)
{
    // Eigen::Vector3d ans;
    // //TODO
    // //检查坐标点是否超出地图范围
    // if (!MAP_VALID(map, coords.x(), coords.y()))
    // {
    //     return Eigen::Vector3d(0,0,0) ;
    //     std::cout <<"GaussianNewton--out of map bound:"<<std::endl;
    // }
    // //向下取整 
    // Eigen::Vector2i indMin(coords.cast<int>());
    // //这个向量是用于双线性差值用的
    // Eigen::Vector2d factors(coords - indMin.cast<double>());
    // int map_sizeX = map->width;
    
    // //地图是以一维数组的形式存储的，每个元素有2个值，val(概率)，index(元素索引，初始化为-1)
    // int index = indMin(1) * map_sizeX + indMin(0);

    // //获取当前实数坐标点相邻的四个栅格坐标，并获取该栅格的占用概率
    // double score_around[4];
    // score_around[0] = map->getCellProbability(index) ;
    // index++;
    // score_around[1] = map->getCellProbability(index) ;
    // index += map_sizeX - 1;
    // score_around[2] = map->getCellProbability(index) ;
    // index++;
    // score_around[3] = map->getCellProbability(index) ;


    // //双线性差值，具体的对照论文中公式4、5、6
    // double dx1 = score_around[0] - score_around[1] ;
    // double dx2 = score_around[2] - score_around[3] ;
    // double dy1 = score_around[0] - score_around[2] ;
    // double dy2 = score_around[1] - score_around[3] ;

    // double xFac_inverse = (1.0 - factors(0));
    // double yFac_inverse = (1.0 - factors(1));

    // //函数的返回值是占用概率加偏导数
    // // ans =  Eigen::Vector3d(
    // // ((score_around[0] * xFac_inverse + score_around[1] * factors[0]) * (yFac_inverse)) +
    // // ((score_around[2] * xFac_inverse + score_around[3] * factors[0]) * (factors[1])),
    // // -((dx1 * yFac_inverse) + (dx2 * factors[1])),
    // // -((dy1 * xFac_inverse) + (dy2 * factors[0]))
    // // );

    // ans =  Eigen::Vector3d(
    //   ((score_around[0] * xFac_inverse + score_around[1] * factors(0)) * (yFac_inverse)) +
    //   ((score_around[2] * xFac_inverse + score_around[3] * factors(0)) * (factors(1))),
    //   -((dx1 * xFac_inverse) + (dx2 * factors(0))),
    //   -((dy1 * yFac_inverse) + (dy2 * factors(1)))
    // );

    // //END OF TODO

    // return ans;


    Eigen::Vector3d ans;
    //TODO
    double x = coords(0);
    double y = coords(1);
    double x_0 = x - 0.0125;
    double y_0 = y - 0.0125;
    double x_1 = x_0 + 0.025;
    double y_1 = y_0 + 0.025;
   
    int p00_x,p00_y;
    p00_x = MAP_WXtoMX(map, x_0);
    p00_y = MAP_WYtoMY(map, y_0);

    if(!MAP_VALID(map, p00_x, p00_y))
    {
        std::cout << "point out map : " << p00_x << " , "<< p00_y << " world : "<< coords(0) << " , " << coords(1) << std::endl;
        return Eigen::Vector3d(0,0,0);
    }
    //地图是以一维数组的形式存储的，每个元素有2个值，val(概率)，index(元素索引，初始化为-1)
    int index = p00_y * map->width + p00_x;

    //获取当前实数坐标点相邻的四个栅格坐标，并获取该栅格的占用概率
    double score_00,score_01,score_10,score_11;

    score_00 = map->getCellProbability(index) ;
    index++;
    score_10 = map->getCellProbability(index) ;
    index += (map->width - 1);
    score_01 = map->getCellProbability(index) ;
    index++;
    score_11 = map->getCellProbability(index) ;

    //双线性差值，具体的对照论文中公式4、5、6
    double dx1 = score_10 - score_00 ;
    double dx2 = score_11 - score_01 ;
    double dy1 = score_01 - score_00 ;
    double dy2 = score_11 - score_10 ;

    x_0 = MAP_MXtoWX(map, p00_x);
    y_0 = MAP_MYtoWY(map, p00_y);
    x_1 = MAP_MXtoWX(map, p00_x+1);
    y_1 = MAP_MYtoWY(map, p00_y+1);

    //函数的返回值是占用概率加偏导数
    ans(0) = ((y-y_0)/(y_1-y_0))*((x-x_0)*score_11/(x_1-x_0)+(x_1-x)*score_01/(x_1-x_0))
            +((y_1-y)/(y_1-y_0))*((x-x_0)*score_10/(x_1-x_0)+(x_1-x)*score_00/(x_1-x_0));

    ans(1) = ((y-y_0)*(dx2)/(y_1-y_0) + (y_1-y)*(dx1)/(y_1-y_0))/(x_1-x_0);
    ans(2) = ((x-x_0)*(dy2)/(x_1-x_0) + (x_1-x)*(dy1)/(x_1-x_0))/(y_1-y_0);

    return ans;
}


/**
 * @brief ComputeCompleteHessianAndb
 * 计算H*dx = b中的H和b
 * @param map
 * @param now_pose
 * @param laser_pts
 * @param H
 * @param b
 */
void gaussian_newton_optimize::ComputeHessianAndb(map_t* map, Eigen::Vector3d now_pose,
                                std::vector<Eigen::Vector2d>& laser_pts,
                                Eigen::Matrix3d& H, Eigen::Vector3d& b)
{
    // H = Eigen::Matrix3d::Zero();
    // b = Eigen::Vector3d::Zero();
    // //TODO

    // // Eigen::Vector3d map_pose = get_map_pose(now_pose);
    // Eigen::Affine2d transform  =  getTransformForState(now_pose);

    // Eigen::Matrix3d Trans = GN_V2T(now_pose);
    // Eigen::Vector3d mapvalue_dx_dy;

    // double sinRot = sin(now_pose(2));
    // double cosRot = cos(now_pose(2));

    // for(int i = 0; i < laser_pts.size();i++)
    // {
    //     // //将激光点从雷达坐标系转化到世界坐标系；
    //     // Eigen::Vector2d tmp_pt = GN_TransPoint(laser_pts[i],Trans);
    //     // //从世界坐标转化到地图坐标
    //     // Eigen::Vector2d map_point;
    //     // map_point(0) = (tmp_pt(0) - map->origin_x) / map->resolution + map->offset_x ;
    //     // map_point(1) = (tmp_pt(1) - map->origin_y) / map->resolution + map->offset_y ;

    //     //取地图的值，以及偏导数，对应论文中公式（4）、（5）、（6）、（14）
    //     mapvalue_dx_dy = InterpMapValueWithDerivatives( map, transform * laser_pts[i]);
    //     // mapvalue_dx_dy = InterpMapValueWithDerivatives( map, map_point);

    //     double f_error = 1.0 - mapvalue_dx_dy(0);

    //     b(0) += mapvalue_dx_dy(1) * f_error;
    //     b(1) += mapvalue_dx_dy(2) * f_error;
    //     double map_rotDeriv = ((-sinRot * laser_pts[i].x() - cosRot * laser_pts[i].y()) * mapvalue_dx_dy(1) + 
    //                             (cosRot * laser_pts[i].x() - sinRot * laser_pts[i].y()) * mapvalue_dx_dy(2));
    //     b(2) += map_rotDeriv * f_error;

    //     H(0, 0) += mapvalue_dx_dy(1) * mapvalue_dx_dy(1);
    //     H(1, 1) += mapvalue_dx_dy(2) * mapvalue_dx_dy(2);
    //     H(2, 2) += map_rotDeriv * map_rotDeriv;

    //     H(0, 1) += mapvalue_dx_dy(1) * mapvalue_dx_dy(2);
    //     H(0, 2) += mapvalue_dx_dy(1) * map_rotDeriv;
    //     H(1, 2) += mapvalue_dx_dy(2) * map_rotDeriv;
    // }

    // H(1, 0) = H(0, 1);
    // H(2, 0) = H(0, 2);
    // H(2, 1) = H(1, 2);
    // //END OF TODO

    H = Eigen::Matrix3d::Zero();
    b = Eigen::Vector3d::Zero();
    //TODO
    Eigen::Matrix3d Trans = GN_V2T(now_pose);
    Eigen::Vector3d mapvalue_dx_dy;

    double sinRot = sin(now_pose(2));
    double cosRot = cos(now_pose(2));

    for(int i = 0; i < laser_pts.size();i++)
    {
        //将激光点从雷达坐标系转化到世界坐标系；
        Eigen::Vector2d tmp_pt = GN_TransPoint(laser_pts[i],Trans);
        // //从世界坐标转化到地图坐标
        // Eigen::Vector2d map_point;
        // map_point.x() = (tmp_pt.x() - map->origin_x) / map->resolution + map->size_x/2.0 ;
        // map_point.y() = (tmp_pt.y() - map->origin_y) / map->resolution + map->size_y/2.0 ;

        //取地图的值，以及偏导数，对应论文中公式（4）、（5）、（6）、（14）
        mapvalue_dx_dy = InterpMapValueWithDerivatives( map, tmp_pt);

        double f_error = 1.0 - mapvalue_dx_dy(0);

        b(0) += mapvalue_dx_dy(1) * f_error;
        b(1) += mapvalue_dx_dy(2) * f_error;
        double map_rotDeriv = ((-sinRot * laser_pts[i].x() - cosRot * laser_pts[i].y()) * mapvalue_dx_dy(1) + 
                                (cosRot * laser_pts[i].x() - sinRot * laser_pts[i].y()) * mapvalue_dx_dy(2));
        b(2) += map_rotDeriv * f_error;

        H(0, 0) += mapvalue_dx_dy(1) * mapvalue_dx_dy(1);
        H(1, 1) += mapvalue_dx_dy(2) * mapvalue_dx_dy(2);
        H(2, 2) += map_rotDeriv * map_rotDeriv;

        H(0, 1) += mapvalue_dx_dy(1) * mapvalue_dx_dy(2);
        H(0, 2) += mapvalue_dx_dy(1) * map_rotDeriv;
        H(1, 2) += mapvalue_dx_dy(2) * map_rotDeriv;
    }

    H(1, 0) = H(0, 1);
    H(2, 0) = H(0, 2);
    H(2, 1) = H(1, 2);
    //END OF TODO
}


/**
 * @brief GaussianNewtonOptimization
 * 进行高斯牛顿优化．
 * @param map
 * @param init_pose
 * @param laser_pts
 */
Eigen::Vector3d gaussian_newton_optimize::GaussianNewtonOptimization(map_t* map, Eigen::Vector3d& init_pose, std::vector<Eigen::Vector2d>& laser_pts)
{
    // int maxIteration =10;
    // Eigen::Vector3d map_pose , world_pose, optimized_pose;
    // world_pose = init_pose;
    // map_pose = get_map_pose(init_pose);

    // static int count=0;

    // std::cout <<"GaussianNewton--init_pose: "<< count << ", "<<map_pose(0)<<", "<<map_pose(1)<<", "<<map_pose(2)*57.295<<std::endl;

    // for(int i = 0; i < maxIteration;i++)
    // {
    //     //TODO

    //     Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    //     Eigen::Vector3d b = Eigen::Vector3d::Zero();
    //     ComputeHessianAndb(map, map_pose, laser_pts, H, b);

    //     if ((H(0, 0) != 0.0) && (H(1, 1) != 0.0))
    //     {
    //         Eigen::Vector3d delta_kesi = H.inverse() * b ;
    
    //         if(delta_kesi(2)>0.2) delta_kesi(2) = 0.2;
    //         else if(delta_kesi(2)<-0.2) delta_kesi(2) = -0.2;

    //         map_pose += delta_kesi;
    //         std::cout <<"GN--iter: "<< i << " , "<<delta_kesi(0)<<","<<delta_kesi(1)<<","<<delta_kesi(2)*57.295<<std::endl;
    //     }
        
    //     //END OF TODO
    // }
    // map_pose(2) = GN_NormalizationAngle(map_pose(2));
    // std::cout <<"GaussianNewton--map_pose: "<< count << ", "<<map_pose(0)<<", "<<map_pose(1)<<", "<<map_pose(2)*57.295<<std::endl;
    // optimized_pose = get_world_pose(map_pose);

    // std::cout <<"GaussianNewton--end_pose: "<<count++ << ", "<<optimized_pose(0)<<", "<<optimized_pose(1)<<", "<<optimized_pose(2)*57.295<<std::endl <<std::endl;
    // return optimized_pose;

    int maxIteration = 20;
    Eigen::Vector3d map_pose;
    Eigen::Vector3d now_pose = init_pose;

    std::cout <<"GaussianNewton--init_pose:"<<init_pose(0)<<","<<init_pose(1)<<","<<init_pose(2)*57.295<<std::endl;

    for(int i = 0; i < maxIteration;i++)
    {
        //TODO

        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();

        ComputeHessianAndb(map, now_pose, laser_pts, H, b);

        if ((H(0, 0) != 0.0) && (H(1, 1) != 0.0))
        {
            Eigen::Vector3d delta_kesi(H.inverse() * b);
            if(delta_kesi(0)>2) delta_kesi(0) = 2;
            else if(delta_kesi(0)<-2) delta_kesi(0) = -2;
            if(delta_kesi(1)>2) delta_kesi(1) = 2;
            else if(delta_kesi(1)<-2) delta_kesi(1) = -2;
            if(delta_kesi(2)>0.2) delta_kesi(2) = 0.2;
            else if(delta_kesi(2)<-0.2) delta_kesi(2) = -0.2;

            //Eigen::Vector3d delta_kesi =  H.colPivHouseholderQr().solve(b);
            delta_kesi(2) = GN_NormalizationAngle(delta_kesi(2));

            now_pose += delta_kesi;
            now_pose(2) = GN_NormalizationAngle(now_pose(2));

            std::cout <<"GaussianNewton--delta_kesi:"<<delta_kesi(0)<<","<<delta_kesi(1)<<","<<delta_kesi(2)*57.295<<std::endl;
        }
        
        //END OF TODO
    }
    
    init_pose = now_pose;

    std::cout <<"GaussianNewton--end_pose:"<<init_pose(0)<<","<<init_pose(1)<<","<<init_pose(2)*57.295<<std::endl;

    return init_pose;
}






