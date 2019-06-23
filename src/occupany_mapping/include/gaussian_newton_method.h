#ifndef GAUSSIAN_NEWTON_METHOD_H
#define GAUSSIAN_NEWTON_METHOD_H

#include <cstdio>
#include <vector>
#include <cmath>
#include <iostream>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

#include "occupany_mapping.h"



// //用高斯牛顿的方法来进行优化
// void GaussianNewtonOptimization(map_t* map, Eigen::Vector3d& init_pose, std::vector<Eigen::Vector2d>& laser_pts);

// //进行角度正则化．
// double GN_NormalizationAngle(double angle);

class gaussian_newton_optimize
{
    public:
    gaussian_newton_optimize()
    {
    
    }

    //进行角度正则化．
    double GN_NormalizationAngle(double angle);

    Eigen::Matrix3d GN_V2T(Eigen::Vector3d vec);

    Eigen::Affine2d getTransformForState(const Eigen::Vector3d& transVector);

    //对某一个点进行转换．
    Eigen::Vector2d GN_TransPoint(Eigen::Vector2d pt,Eigen::Matrix3d T);

    Eigen::Vector2d get_map_coords(const Eigen::Vector2d& worldCoords);

    Eigen::Vector3d get_map_pose(const Eigen::Vector3d& worldPose);

    Eigen::Vector2d get_world_coords(const Eigen::Vector2d& mapCoords);

    Eigen::Vector3d get_world_pose(const Eigen::Vector3d& mapPose);
   
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
    Eigen::Vector3d InterpMapValueWithDerivatives(map_t* map, const Eigen::Vector2d& coords);

    /**
     * @brief ComputeCompleteHessianAndb
     * 计算H*dx = b中的H和b
     * @param map
     * @param now_pose
     * @param laser_pts
     * @param H
     * @param b
     */
    void ComputeHessianAndb(map_t* map, Eigen::Vector3d now_pose,
                                    std::vector<Eigen::Vector2d>& laser_pts,
                                    Eigen::Matrix3d& H, Eigen::Vector3d& b);

    /**
     * @brief GaussianNewtonOptimization
     * 进行高斯牛顿优化．
     * @param map
     * @param init_pose
     * @param laser_pts
     */
    Eigen::Vector3d GaussianNewtonOptimization(map_t* map, Eigen::Vector3d& init_pose, std::vector<Eigen::Vector2d>& laser_pts);

    
};


#endif // GAUSSIAN_NEWTON_METHOD_H
