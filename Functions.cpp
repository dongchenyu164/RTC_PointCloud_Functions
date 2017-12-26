#include "Functions.h"
#include <queue>

void CovertTo_OrgnizedPointCloud(PCXYZ_Ptr &Source, double Width, double Height);
void CovertTo_UnOrgnizedPointCloud(PCXYZ_Ptr &Source);

//滤波处理
enum PointCloudProcessMode{Capture, Process};
//先是数据读入状态，接收【当前机械臂位姿】信号
bool isBusy = false;//是否正在处理点云。
PointCloudProcessMode SystemMode = Capture;

PCXYZ_Ptr PointsOfTable(new PCXYZ);//合成后的桌子的点云。
std::queue<PCXYZ_Ptr> queue_PointsOfCapture;//被捕获的点云的存放队列。
std::queue<Eigen::Matrix4f> queue_TransformData;//被捕获的点云的位姿矩阵。
// 作为主动触发的函数
std::string Capture_PointClould(double TransformData[4][4])/****RTM****/
{
	PCXYZ_Ptr DataIn = PCXYZ_Ptr(new PCXYZ());//到时候换成RTM点云输入变量。

	if (isBusy)
		return "Processing!";
	while (isBusy);
	isBusy = true;

	queue_PointsOfCapture.push(DataIn);
	queue_TransformData.push(MakeTransformMatrix(TransformData));

	isBusy = false;

	return "Capture_PointClould() Success!";
}

//应该在OnExecute函数内，在 拍摄 模式时，每周期调用。
void Transform_PointCloud()
{
	if (SystemMode != Capture)
		return;

	if (isBusy)
		return;
	isBusy = true;//防止队列（在RTM中）的多线程调用冲突。

	if (queue_PointsOfCapture.empty() || queue_TransformData.empty())
	{
		isBusy = false;
		return;
	}

	PCXYZ_Ptr tmp(new PCXYZ);
	PCXYZ_Ptr tmp2(new PCXYZ);
	PCXYZ_Ptr tmp3(new PCXYZ);

	pcl::transformPointCloud(*queue_PointsOfCapture.front(), *tmp, queue_TransformData.front());

	//调用完后弹出队列
	queue_PointsOfCapture.pop();
	queue_TransformData.pop();

	isBusy = false;//关闭忙标志，使能队列操作。

	Filters(tmp, tmp2);

	//ICP
	if (PointsOfTable.size() != 0)
		ICP_Single(tmp2, PointsOfTable, tmp3);

	*PointsOfTable += *tmp2;//累加点云
}

//主动触发
std::string Clear_QueueAndPoints()
{
	if (isBusy)
		return "Processing!";
	isBusy = true;

	(*PointsOfTable).clear();

	while (!queue_PointsOfCapture.empty())
		queue_PointsOfCapture.pop();	
	while (!queue_TransformData.empty())
		queue_TransformData.pop();

	isBusy = false;

	return "Clear_Queue_Points() Success!";
}

//主动触发
std::string SwitchSysMode(std::string ModeStr)
{
	switch (SystemMode)
	{
		case Capture://当前模式
			switch (ModeStr)//目标模式
			{
				case "CaptureMode":
					return "Now CaptureMode. No need to switch!"
					break;
				case "ProcessMode":
					if (!queue_PointsOfCapture.empty())
						return "Now CaptureMode. Processing......!";
					SystemMode = Process;
					break;
				default:
					break;
			}
			break;
		case Process:
			switch (ModeStr)//目标模式
			{
				case "CaptureMode":
					SystemMode = Capture;
					Clear_QueueAndPoints();
					return "Switch to Capture mode and clear Queue and GlobePointCloud successfully!";
					break;
				case "ProcessMode":
					return "Now ProcessMode. No need to switch!"
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
	
}



Eigen::Matrix4f MakeTransformMatrix(double Data[4][4])
{
	Eigen::Matrix4f tmp;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			tmp(i, j) = Data[i][j];
	return tmp;
}

PCXYZ_Ptr Filters(PCXYZ_Ptr Source, PCXYZ_Ptr Output)
{
#pragma region 各个滤波器变量声明，以及滤波器设置
	//
	pcl::PassThrough<pcl::PointXYZ> PassFilter;
	PassFilter.setFilterFieldName("z");
	PassFilter.setFilterLimits(0.020, 0.50);

	//
	pcl::VoxelGrid<pcl::PointXYZ> VoxelGrid_sor;//新声明体素网格对象
	VoxelGrid_sor.setLeafSize(0.005f, 0.005f, 0.005f);//设定体素网格大小
	//
	pcl::FastBilateralFilter<pcl::PointXYZ> fbf;
	fbf.setSigmaS(1);
	fbf.setSigmaR(0.02);
	//
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;//Removing outliers using a StatisticalOutlierRemoval filter
	sor.setMeanK(50);
	sor.setStddevMulThresh(0.6);
#pragma endregion

	/*===============各个滤波器之间的临时变量================*/
	PCXYZ_Ptr tmp(new PCXYZ);
	PCXYZ_Ptr tmp2(new PCXYZ);
	PCXYZ_Ptr tmp3(new PCXYZ);
	PCXYZ_Ptr tmp4(new PCXYZ);
	/*===============Z轴限定范围================*/

	/*============<<<体素网格滤波================*/
	VoxelGrid_sor.setInputCloud(Source);
	VoxelGrid_sor.filter(*tmp);//过滤点云
	/*===============体素网格滤波>>>=============*/


	/*============<<<带通滤波器================*/ //Source->tmp
	PassFilter.setInputCloud(tmp);
	PassFilter.filter(*tmp2);
	/*======END======带通滤波器>>>====END======*/

#pragma region /*======双边滤波器FastBilateralFilter======*/不用tmp -> tmp2
	
	CovertTo_OrgnizedPointCloud(tmp2, 640, 480);
	fbf.setInputCloud(tmp2);
	fbf.filter(*tmp3);
#pragma endregion

#pragma region /*===============去除极值================*/不用tmp2->tmp4
	CovertTo_UnOrgnizedPointCloud(tmp3);
	sor.setInputCloud(tmp3);
	sor.filter(*Output);
#pragma endregion

	return Output;
}

void CovertTo_OrgnizedPointCloud(PCXYZ_Ptr &Source, double Width, double Height)
{
	/**/////原始点云大小(Size)=Height*Width S=H*W Scale=W/H
		//S=H*H*Scale
		//H=sqrt(S/Scale)
	double Scale = Width / Height;
	double RealHeight = sqrt((*Source).size() / Scale);

	(*Source).height = round(RealHeight);///注意，不能让新算的总点数超过真实点数。
	(*Source).width = RealHeight * Scale;
	if ((*Source).height * (*Source).width > (*Source).size())//如果新算的大小大于原始的，则将宽度减少1；
		(*Source).width--;
	(*Source).isOrgnized = true;
}

void CovertTo_UnOrgnizedPointCloud(PCXYZ_Ptr &Source)
{
	(*Source).height = 1;
	(*Source).width = (*Source).size();
	(*Source).isOrgnized = false;
}

void ExtractPlane(PCXYZ_Ptr Source, PCXYZ_Ptr Plane, PCXYZ_Ptr Rest)//抽出平面，Rest去除平面后的点云；返回值是抽出的平面。
{
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.02);

	seg.setInputCloud(Source);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
		PCL_ERROR("Could not estimate a planar model for the given dataset.");

	//输出处理
	pcl::ExtractIndices<pcl::PointXYZ> extract;//由于SACSegmentation抽出来的只是点的数值，需要将数值转化为点云。这个类便是专门来干这个的。
	extract.setInputCloud(Source);
	extract.setIndices(inliers);
	extract.setNegative(false);
	// Get the points associated with the planar surface
	extract.filter(*Plane);
	// Remove the planar inliers, extract the rest
	extract.setNegative(true);
	extract.filter(*Rest);
}
int ExtractEuclideanCluster(PCXYZ_Ptr Source, PCXYZ_Ptr Clusters[])
{
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(Source);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.01); // 2cm
	ec.setMinClusterSize(300);
	ec.setMaxClusterSize(450000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(Source);
	ec.extract(cluster_indices);

	int i = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(Source->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		Clusters[i++] = cloud_cluster;
	}
	return cluster_indices.size();
}
Mat4f ICP_Single(PCXYZ_Ptr Source, PCXYZ_Ptr Target, PCXYZ_Ptr Output)
{
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	icp.setMaxCorrespondenceDistance(1);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.001);

	icp.setMaximumIterations(300);
	icp.setInputSource(Source);
	icp.setInputTarget(Target);//这个点云不动，其他点云跟Target配合。ptr_To_ICP_cloud[0]
	icp.align(*Output);

	if (icp.hasConverged())
	{
		std::cout << std::endl << i << "ICP has converged, score is " << icp.getFitnessScore();
		pcl::transformPointCloud(*Source, *Output, icp.getFinalTransformation());
		return icp.getFinalTransformation();
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		return Mat4f().setZero();
	}
}