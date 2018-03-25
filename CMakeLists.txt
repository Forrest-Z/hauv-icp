cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

set (CMAKE_CXX_STANDARD 11)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/include")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_BUILD_TYPE Debug)
set(PCL_BUILD_TYPE Debug)

set(srcs
	"src/main.cpp"
	"src/transformation_estimation_point_to_plane_lls_odomprior.cpp"
	"src/get_ext_dir.cpp"
	"src/read_mat_from_file.cpp"
	"src/get_odom_from_file.cpp"
	"src/align_submaps_odom_prior.cpp"
	"src/align_submaps_twod.cpp"
	"src/align_submaps_ptp.cpp")

# set(srcs2
#	"src/main_2.cpp")

# set(srcs3
#	"src/main_3.cpp")

add_executable(main ${srcs})
target_link_libraries(main ${PCL_LIBRARIES})

# add_executable(main_2 ${srcs2})
# target_link_libraries(main_2 ${PCL_LIBRARIES})

# add_executable(main_3 ${srcs3})
# target_link_libraries(main_3 ${PCL_LIBRARIES})

# add_executable(visualize_odom_icp_tf src/visualize_odom_icp_tf.cpp)
# target_link_libraries(visualize_odom_icp_tf ${PCL_LIBRARIES})
