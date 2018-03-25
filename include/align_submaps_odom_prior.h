#ifndef ALIGN_SUBMAPS_ODOM_PRIOR_H
#define ALIGN_SUBMAPS_ODOM_PRIOR_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/feature.h>
#include <pcl/cloud_iterator.h>
#include "transformation_estimation_point_to_plane_lls_odomprior.h"

// using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGBNormal PointTNormal;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef typename pcl::Feature<PointT, PointT>::PointCloudConstPtr PointCloudConstPtr;

int align_submaps_odom_prior(const typename pcl::PointCloud<PointT>::Ptr &source,
                  const typename pcl::PointCloud<PointT>::Ptr &target,
                  const typename pcl::PointCloud<PointTNormal>::Ptr &final,
                  Matrix6d &ATA,
                  Vector6d &ATb,
                  const double &max_corresp_distance,
                  const Eigen::Matrix4f &initial_guess,
                  Eigen::Matrix4f &transform, double &score,
                  float odom_init_[], float odom_final_[]);

#endif /* ALIGN_SUBMAPS_ODOM_PRIOR_H */