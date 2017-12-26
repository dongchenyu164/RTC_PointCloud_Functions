#pragma once
#include <iostream>
#include <math.h>
#include <string>// 注意是<string>，不是<string.h>，带.h的是C语言中的头文件
#include <thread>

using std::string;
using std::wstring;

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/registration/icp.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>

//For plane extraction.
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//For Euclidean Cluster Extraction
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

//For PCA
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>

#define END_INDEX 19
#define NUM_OF_DATA 19
#define START_INDEX 0
//#define NEED_TO_SMALL //是否需要降低点数
#define TO_RAD (3.14159265359/180.0)
#define TO_DEG (180.0/3.14159265359)

typedef Eigen::Matrix4f Mat4f;
typedef pcl::PointXY PointXY;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointXYZRGB PointXYZRGB;
typedef pcl::PointCloud<pcl::PointXY>::Ptr PCXY_Ptr;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCXYZ_Ptr;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCXYZRGB_Ptr;
typedef pcl::PointCloud<pcl::PointXYZ> PCXYZ;
typedef pcl::PointCloud<pcl::PointXY> PCXY;
typedef pcl::PointCloud<pcl::PointXYZRGB> PCXYZRGB;
typedef  pcl::PointCloud<pcl::Normal> PC_Normal;
typedef  pcl::PointCloud<pcl::Normal>::Ptr PC_Normal_Ptr;

typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> PCXYZ_ColorHandler;