#ifndef GET_EXT_DIR_H
#define GET_EXT_DIR_H

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

void get_ext_dir(const ::boost::filesystem::path& root, const std::string& ext, std::vector<::boost::filesystem::path>& ret);

#endif /* GET_EXT_DIR_H */