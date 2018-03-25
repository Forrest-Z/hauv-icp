#ifndef ALIGN_SUBMAPS_TWOD_H
#define ALIGN_SUBMAPS_TWOD_H

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

// using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGBNormal PointTNormal;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef typename pcl::Feature<PointT, PointT>::PointCloudConstPtr PointCloudConstPtr;

int align_submaps_twod(const typename pcl::PointCloud<PointT>::Ptr &source,
                  const typename pcl::PointCloud<PointT>::Ptr &target,
                  const typename pcl::PointCloud<PointT>::Ptr &final,
                  Matrix6d &ATA,
                  Vector6d &ATb,
                  const double &max_corresp_distance,
                  const Eigen::Matrix4f &initial_guess,
                  Eigen::Matrix4f &transform, double &score);

#endif /* ALIGN_SUBMAPS_TWOD_H */