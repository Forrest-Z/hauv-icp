#include "align_submaps_twod.h"

int align_submaps_twod(const typename pcl::PointCloud<PointT>::Ptr &source,
                  const typename pcl::PointCloud<PointT>::Ptr &target,
                  const typename pcl::PointCloud<PointT>::Ptr &final,
                  Matrix6d &ATA,
                  Vector6d &ATb,
                  const double &max_corresp_distance,
                  const Eigen::Matrix4f &initial_guess,
                  Eigen::Matrix4f &transform, double &score) {

  

  /******************** ESTIMATE NORMALS HERE, BEFORE CALLING icp.align() ********************/
  
  
  
  
  // pcl::IterativeClosestPointWithNormals<PointTNormal, PointTNormal> icp;

  // Comment out if using SVD
  // const typename pcl::registration::TransformationEstimationPointToPlaneLLSOdomPrior<PointTNormal, PointTNormal, float>::Ptr tf_ptp( new pcl::registration::TransformationEstimationPointToPlaneLLSOdomPrior<PointTNormal, PointTNormal, float>);
  
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  const typename pcl::registration::TransformationEstimation2D<PointT, PointT, float>::Ptr tf_2D( new pcl::registration::TransformationEstimation2D<PointT, PointT, float>);

  // for (int i = 0; i < 6; i++) {
  //     tf_ptp -> odom_init[i]  = odom_init_[i];
  // }

  // for (int i = 0; i < 6; i++) {
  //     tf_ptp -> odom_final[i]  = odom_final_[i];
  // }

  icp.setTransformationEstimation(tf_2D); 

  icp.setMaxCorrespondenceDistance(max_corresp_distance); // assume significant
                                                          // drift; 0.1 seems to
                                                          // work but that's for
                                                          // small amounts of
                                                          // drift
  icp.setTransformationEpsilon(1e-8);

  // icp.setInputSource(source_with_normals);   // Comment out if using SVD
  // icp.setInputTarget(target_with_normals);   // Comment out if using SVD

  icp.setInputSource(source); // Comment out if using PointToPlaneOdomPrior
  icp.setInputTarget(target); // Comment out if using PointToPlaneOdomPrior

  /************************ ICP RUNS AND CONVERGES ************************/
  icp.align(*final, initial_guess);



  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne_target;
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne_source;
  ne_target.setInputCloud (target);
  ne_source.setInputCloud (final);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne_target.setSearchMethod (tree);
  ne_source.setSearchMethod (tree);

  
  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_target_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_final_normals (new pcl::PointCloud<pcl::Normal>);
  
  // Use all neighbors in a sphere of radius 3cm
  ne_target.setRadiusSearch (0.03);
  ne_source.setRadiusSearch (0.03);
  
  // Compute the features
  ne_target.compute (*cloud_target_normals);
  ne_source.compute (*cloud_final_normals);
  
  const pcl::PointCloud<PointTNormal>::Ptr target_with_normals (new pcl::PointCloud<PointTNormal>);
  const pcl::PointCloud<PointTNormal>::Ptr final_with_normals (new pcl::PointCloud<PointTNormal>);

  /******************** ADD NORMALS FIELD WITH TARGET CLOUD *****************/
  // add normals to the target poin cloud by concatenating fields of cloud_normals and target 
  // and creating target_with_normals
  pcl::concatenateFields (*target, *cloud_target_normals, *target_with_normals);
  pcl::concatenateFields (*final, *cloud_final_normals, *final_with_normals);

  pcl::ConstCloudIterator<PointTNormal> source_it (*final_with_normals);
  pcl::ConstCloudIterator<PointTNormal> target_it (*target_with_normals);

  /****** SET ATA AND ATb TO ZERO ******/
  ATA.setZero ();
  ATb.setZero ();

  /******************** APPROXIMATE AS A LLS PROBLEM AND POPULATE ATA AND ATb **********************/
  while (source_it.isValid () && target_it.isValid ())
  {
    if (!pcl_isfinite (source_it->x) ||
        !pcl_isfinite (source_it->y) ||
        !pcl_isfinite (source_it->z) ||
        !pcl_isfinite (target_it->x) ||
        !pcl_isfinite (target_it->y) ||
        !pcl_isfinite (target_it->z) ||
        !pcl_isfinite (target_it->normal_x) ||
        !pcl_isfinite (target_it->normal_y) ||
        !pcl_isfinite (target_it->normal_z))
    {
      ++target_it;
      ++source_it;    
      continue;
    }

    const float & sx = source_it->x;
    const float & sy = source_it->y;
    const float & sz = source_it->z;
    const float & dx = target_it->x;
    const float & dy = target_it->y;
    const float & dz = target_it->z;
    const float & nx = target_it->normal[0];
    const float & ny = target_it->normal[1];
    const float & nz = target_it->normal[2];

    double a = nz*sy - ny*sz;
    double b = nx*sz - nz*sx; 
    double c = ny*sx - nx*sy;
   
    //    0  1  2  3  4  5
    //    6  7  8  9 10 11
    //   12 13 14 15 16 17
    //   18 19 20 21 22 23
    //   24 25 26 27 28 29
    //   30 31 32 33 34 35
   
    ATA.coeffRef (0) += a * a;
    ATA.coeffRef (1) += a * b;
    ATA.coeffRef (2) += a * c;
    ATA.coeffRef (3) += a * nx;
    ATA.coeffRef (4) += a * ny;
    ATA.coeffRef (5) += a * nz;
    ATA.coeffRef (7) += b * b;
    ATA.coeffRef (8) += b * c;
    ATA.coeffRef (9) += b * nx;
    ATA.coeffRef (10) += b * ny;
    ATA.coeffRef (11) += b * nz;
    ATA.coeffRef (14) += c * c;
    ATA.coeffRef (15) += c * nx;
    ATA.coeffRef (16) += c * ny;
    ATA.coeffRef (17) += c * nz;
    ATA.coeffRef (21) += nx * nx;
    ATA.coeffRef (22) += nx * ny;
    ATA.coeffRef (23) += nx * nz;
    ATA.coeffRef (28) += ny * ny;
    ATA.coeffRef (29) += ny * nz;
    ATA.coeffRef (35) += nz * nz;

    double d = nx*dx + ny*dy + nz*dz - nx*sx - ny*sy - nz*sz;
    ATb.coeffRef (0) += a * d;
    ATb.coeffRef (1) += b * d;
    ATb.coeffRef (2) += c * d;
    ATb.coeffRef (3) += nx * d;
    ATb.coeffRef (4) += ny * d;
    ATb.coeffRef (5) += nz * d;

    ++target_it;
    ++source_it;    
  }

    ATA.coeffRef (6) = ATA.coeff (1);
    ATA.coeffRef (12) = ATA.coeff (2);
    ATA.coeffRef (13) = ATA.coeff (8);
    ATA.coeffRef (18) = ATA.coeff (3);
    ATA.coeffRef (19) = ATA.coeff (9);
    ATA.coeffRef (20) = ATA.coeff (15);
    ATA.coeffRef (24) = ATA.coeff (4);
    ATA.coeffRef (25) = ATA.coeff (10);
    ATA.coeffRef (26) = ATA.coeff (16);
    ATA.coeffRef (27) = ATA.coeff (22);
    ATA.coeffRef (30) = ATA.coeff (5);
    ATA.coeffRef (31) = ATA.coeff (11);
    ATA.coeffRef (32) = ATA.coeff (17);
    ATA.coeffRef (33) = ATA.coeff (23);
    ATA.coeffRef (34) = ATA.coeff (29);

  score = icp.getFitnessScore();
  transform = icp.getFinalTransformation();

  return (icp.hasConverged());
}