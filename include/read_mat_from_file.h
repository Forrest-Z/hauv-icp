#ifndef READ_MAT_FROM_FILE_H
#define READ_MAT_FROM_FILE_H

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

Eigen::MatrixXf read_mat_from_file(std::ifstream &fin, int start, int stop, int rows, int cols);

#endif /* READ_MAT_FROM_FILE_H */