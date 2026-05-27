//
// Created by gaoxiang on 19-5-4.
//

#ifndef MYSLAM_ALGORITHM_H
#define MYSLAM_ALGORITHM_H

// algorithms used in myslam
#include "myslam/common_include.h"

namespace myslam {

/**
 * linear triangulation with SVD
 * @param poses     poses,
 * @param points    points in normalized plane
 * @param pt_world  triangulated point in the world
 * @return true if success
 */

 /*
  tj : 投影方程简化为：λ * [u, v, 1]^T = [R|t] * [X, Y, Z, 1]^T

注意这里 [u, v, 1] 是归一化坐标，不是像素坐标。

相机坐标系 (3D)
    |
    | 除以 Z_cam (深度)
    ↓
归一化坐标系 (2D)  ← 投影到 Z=1 平面上
    |
    | 乘以内参 K
    ↓
像素坐标系 (2D)


 [ X_cam ]   [ m00 m01 m02 m03 ] [ X ]
 [ Y_cam ] = [ m10 m11 m12 m13 ] [ Y ]
 [ Z_cam ]   [ m20 m21 m22 m23 ] [ Z ]
                                 [ 1 ]


u = X_cam / Z_cam
v = Y_cam / Z_cam

u * Z_cam = X_cam

u * (m20·X + m21·Y + m22·Z + m23) = (m00·X + m01·Y + m02·Z + m03)
u * (m20·X + m21·Y + m22·Z + m23) - (m00·X + m01·Y + m02·Z + m03) = 0

(u*m20 - m00)·X + (u*m21 - m01)·Y + (u*m22 - m02)·Z + (u*m23 - m03) = 0


 */
inline bool triangulation(const std::vector<SE3> &poses,
                   const std::vector<Vec3> points, Vec3 &pt_world) {
    MatXX A(2 * poses.size(), 4);
    VecX b(2 * poses.size());
    b.setZero();
    for (size_t i = 0; i < poses.size(); ++i) {
        Mat34 m = poses[i].matrix3x4();
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }
    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
        return true;
    }
    return false;
}

// converters
inline Vec2 toVec2(const cv::Point2f p) { return Vec2(p.x, p.y); }

}  // namespace myslam

#endif  // MYSLAM_ALGORITHM_H
