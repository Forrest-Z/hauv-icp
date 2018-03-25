#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_2D.h>

//#include "transformation_estimation_point_to_plane_lls_odomprior.h"

#include <pcl/visualization/cloud_viewer.h>

// includes for normal estimation
#include <pcl/features/normal_3d.h>
#include <pcl/features/feature.h>

// includes for point to plane formulation
#include <pcl/cloud_iterator.h>

//#include "transformation_estimation_point_to_plane_lls_odomprior.h"

/*
 * Add new header files for better coding and readability   
*/

// get external directory
#include "get_ext_dir.h"

// read mat from file
#include "read_mat_from_file.h"

// get odometry params from file
#include "get_odom_from_file.h"

// align submaps with odometry prior in point-to-plane
#include "align_submaps_odom_prior.h"

// align submaps with 2D ICP
#include "align_submaps_twod.h"

// align submaps with point-to-plane only
#include "align_submaps_ptp.h"

#include <getopt.h>

/* Example Usage
./main.cpp ../data/dataset_shiphull/loopclosures/pair_2-3
*/
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGBNormal PointTNormal;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef typename pcl::Feature<PointT, PointT>::PointCloudConstPtr PointCloudConstPtr;

void print_usage() {
    printf("Usage: ./main -o ../data/dataset_shiphull/loopclosures/... \n");
}

int main (int argc, char** argv)
{
  int option = 0;
  bool odom_prior_icp = false;
  bool twod_icp = false;
  bool ptp_icp = false;

  while ((option = getopt(argc, argv,"otp")) != -1) {
        switch (option) {
             case 'o' : {odom_prior_icp = true; /*std::string src_dir = atoi(optarg);*/}
                 break;
             case 't' : {twod_icp = true; /*std::string src_dir = atoi(optarg);*/}
                 break;
             case 'p' : {ptp_icp = true; /*std::string src_dir = atoi(optarg);*/}
                 break;
             default: print_usage(); 
                 exit(EXIT_FAILURE);
        }
  }


	pcl::PointCloud<PointT>::Ptr cloud_src (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_tgt (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_final_ (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointTNormal>::Ptr cloud_final (new pcl::PointCloud<PointTNormal>);

  // Read clouds from directory
  std::string src_dir = argv[2];
  std::vector<::boost::filesystem::path> src_files;
  std::stringstream ss_file;
  get_ext_dir(src_dir, ".pcd", src_files);

  ss_file.str(""); ss_file << src_dir << "/" << src_files[0].string();
  std::cout << "Reading Cloud : " << ss_file.str() << std::endl;
  if (pcl::io::loadPCDFile(ss_file.str(), *cloud_src) < 0 )
    cerr << "Cloud Reading Failed" << endl;

  ss_file.str(""); ss_file << src_dir << "/" << src_files[1].string();
  std::cout << "Reading Cloud : " << ss_file.str() << std::endl;
  if (pcl::io::loadPCDFile(ss_file.str(), *cloud_tgt) < 0 )
    cerr << "Cloud Reading Failed" << endl;

  // Read param file from directory
  Eigen::Matrix4f T_odom;

  ss_file.str(""); ss_file << src_dir << "/params_icp.txt";
  std::cout << "Reading Param File : " << ss_file.str() << std::endl;

  std::ifstream fin;
  fin.open(ss_file.str());
  T_odom = read_mat_from_file(fin, 2, 16+2, 4, 4);
  std::cout << "T_odom : " << std::endl << T_odom << std::endl;
  fin.close();
    
  float odom_init_[6];
  float odom_final_[6];

  fin.open(ss_file.str());
  get_odom_from_file(fin, odom_init_, odom_final_);
  fin.close();
  // for (int i=0; i<6; i++){
  //     std::cout << "odom_init_[" << i << "]: " << odom_init_[i] << std::endl;
  //   }

  //   for (int j=0; j<6; j++){
  //     std::cout << "odom_final_[" << j << "]: " << odom_final_[j] << std::endl;
  //   }

	// std::string src_cloud_file = argv[1];
	// std::string tgt_cloud_file = argv[2];

	// if (pcl::io::loadPCDFile(src_cloud_file, *cloud_src) < 0 )
	// 	cerr << "Cloud Reading Failed" << endl;
	// if (pcl::io::loadPCDFile(tgt_cloud_file, *cloud_tgt) < 0 )
	// 	cerr << "Cloud Reading Failed" << endl;

	float mcd = 10;
	double score;
	// Eigen::Matrix4f initial_guess;
	// initial_guess << 1, 0, 0, 0, // initial guess set as identity, but should be derived 
	// 				 0, 1, 0, 0,
	// 				 0, 0, 1, 0,
	// 				 0, 0, 0, 1;
	Eigen::Matrix4f transform_final;

	/********** Declare ATA and ATb **********/
	Matrix6d ATA;
	Vector6d ATb;

	/********** Pass in empty ATA and ATb vectors as input parameters to align_submaps(...) **************/

  if (odom_prior_icp) {
      if(align_submaps_odom_prior(cloud_src, cloud_tgt, cloud_final, ATA, ATb, mcd, T_odom, transform_final, score, odom_init_, odom_final_))
          std::cout << "has converged with score: " << score << std::endl;
  }

  else if (twod_icp) {
      if(align_submaps_twod(cloud_src, cloud_tgt, cloud_final_, ATA, ATb, mcd, T_odom, transform_final, score))
          std::cout << "has converged with score: " << score << std::endl;
  }

  else if (ptp_icp) {
      if(align_submaps_ptp(cloud_src, cloud_tgt, cloud_final, ATA, ATb, mcd, T_odom, transform_final, score))
          std::cout << "has converged with score: " << score << std::endl;
  }

	/******* OUTPUT ATA AND ATb ********/
	// std::cout << "ATA Matrix: " << std::endl << ATA << std::endl;
  // std::cout << "Atb Matrix:" << std::endl << ATb << std::endl;
  std::cout << "transform_final: " << std::endl << transform_final << std::endl;

	/******* CALCULATE EIGENVALUES AND EIGENVECTORS OF ATA **********/
	Eigen::EigenSolver<Matrix6d> es(ATA);
    cout << "The eigenvalues of ATA are:" << endl << es.eigenvalues() << endl;
    cout << "The matrix of eigenvectors, V, is:" << endl << es.eigenvectors() << endl << endl;

	std::cout << transform_final << std::endl;

	// Visualization
	pcl::visualization::PCLVisualizer viewer1("icp");
	viewer1.setBackgroundColor (1.0, 1.0, 1.0);
	viewer1.addCoordinateSystem ();
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> color_handler_tgt(cloud_final, 255, 0, 0);

	viewer1.addPointCloud(cloud_tgt, "cloud_tgt");
	viewer1.addPointCloud(cloud_final, color_handler_tgt, "cloud_tf_final");

	while (!(viewer1.wasStopped()))
		viewer1.spinOnce();

	return 0;
}