#include <iostream>
#include <fstream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_2D.h>

#include <pcl/visualization/cloud_viewer.h>

using namespace std;

typedef pcl::PointXYZRGB PointT;


void get_ext_dir(const ::boost::filesystem::path& root, const string& ext, vector<::boost::filesystem::path>& ret)
{
    if(!::boost::filesystem::exists(root) || !::boost::filesystem::is_directory(root)) return;
    ::boost::filesystem::recursive_directory_iterator it(root);
    ::boost::filesystem::recursive_directory_iterator endit;
    while(it != endit)
    {
        if(::boost::filesystem::is_regular_file(*it) && it->path().extension() == ext) ret.push_back(it->path().filename());
        ++it;
    }
    std::sort(ret.begin(), ret.end());
}

Eigen::MatrixXf read_mat_from_file(std::ifstream &fin, int start, int stop, int rows, int cols)
{
	// first line considered to be at index 0

	Eigen::MatrixXf T(rows,cols);
	int line_count = -1;
	float T_arr[rows*cols];
	std::string string_read;

	while(!fin.eof()) {
		line_count = line_count + 1;

		if (line_count < start) {
			std::getline(fin, string_read);
		}
		else if ( (line_count >= start) && (line_count < stop) ) {
			fin >> T_arr[line_count-start];
			// std::cout << T_arr[line_count+1-line_start] << std::endl;
		}
		else {
			break;
		}
	}

	for (int i = 0; i < rows; i++) { 
		for (int j = 0; j < cols; j++) {
			T(i,j) = T_arr[i*cols+j];
		}	
	}

	return T;
}

/* Usage
./visualize_odom_icp_tf ../data/dataset_shiphull/loopclosures/pair_6-16
*/

int main (int argc, char** argv)
{

	pcl::PointCloud<PointT>::Ptr cloud_src (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_tgt (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_src_tf_icp (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_src_tf_odom (new pcl::PointCloud<PointT>);

	// Read clouds from directory
	std::string src_dir = argv[1];
	vector<::boost::filesystem::path> src_files;
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
	double score;
	Eigen::Matrix4f T_odom, T_icp;

	ss_file.str(""); ss_file << src_dir << "/params_icp.txt";
	std::cout << "Reading Param File : " << ss_file.str() << std::endl;

	std::ifstream fin;
	fin.open(ss_file.str());
	T_odom = read_mat_from_file(fin, 2, 16+2, 4, 4);
	std::cout << "T_odom : " << std::endl << T_odom << std::endl;
	fin.close();

	fin.open(ss_file.str());
	T_icp = read_mat_from_file(fin, 7, 16+7, 4, 4);
	std::cout << "T_icp : " << std::endl << T_icp << std::endl;
	fin.close();

	// Transform clouds using initial odom and final icp params read from file
	pcl::transformPointCloud(*cloud_src, *cloud_src_tf_odom, T_odom);
	pcl::transformPointCloud(*cloud_src, *cloud_src_tf_icp, T_icp);

	// Visualization
	pcl::visualization::PCLVisualizer viewer1("loopclosure_pairs");
	int vp_1, vp_2;

	// set viewer properties
	viewer1.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  	viewer1.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
	viewer1.setBackgroundColor (1.0, 1.0, 1.0);
	viewer1.addCoordinateSystem ();
	pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler_src_tf(cloud_src_tf_icp, 255, 0, 0);

	viewer1.addPointCloud(cloud_tgt, "cloud_tgt_odom", vp_1);
	viewer1.addPointCloud(cloud_src_tf_odom, color_handler_src_tf, "cloud_src_tf_odom", vp_1);
	viewer1.addPointCloud(cloud_tgt, "cloud_tgt_icp", vp_2);
	viewer1.addPointCloud(cloud_src_tf_icp, color_handler_src_tf, "cloud_src_tf_icp", vp_2);

	while (!(viewer1.wasStopped()))
		viewer1.spinOnce();

	return 0;
}