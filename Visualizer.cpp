#include "Visualizer.h"
#include "ThreadPool.h"

std::thread VisualizationThread = std::thread();//声明线程对象
std::ThreadPool VisualizationThreadPool(5);
std::mutex mtx;

void VisualizationProcess_PCXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr Data);
void VisualizationProcess_PCXYZ_Array(pcl::PointCloud<pcl::PointXYZ>::Ptr Data[], int NumOfData);

//视觉化-多线程版本。
void PointCloud_Visualizator(pcl::PointCloud<pcl::PointXYZ>::Ptr Data)
{
	VisualizationThreadPool.commit(VisualizationProcess_PCXYZ, Data);
}
void PointCloud_Visualizator(pcl::PointCloud<pcl::PointXYZ> Data)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp = Data.makeShared();
	VisualizationThreadPool.commit(VisualizationProcess_PCXYZ, tmp);
}
//视觉化-点云数组-多线程版本。
void PointCloud_Visualizator(pcl::PointCloud<pcl::PointXYZ>::Ptr Data[], int NumOfData)
{
	VisualizationThreadPool.commit(VisualizationProcess_PCXYZ_Array, Data, NumOfData);
}
void PointCloud_Visualizator(pcl::PointCloud<pcl::PointXYZ> Data[], int NumOfData)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp[256];
	for (int i = 0; i < NumOfData; i++)
		tmp[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(Data[i].makeShared());
	VisualizationThreadPool.commit(VisualizationProcess_PCXYZ_Array, tmp, NumOfData);
}

//视觉化-阻塞主线程版本。
void PointCloud_Visualizator_Stuck(pcl::PointCloud<pcl::PointXYZ>::Ptr Data)
{
	VisualizationProcess_PCXYZ(Data);
}
void PointCloud_Visualizator_Stuck(pcl::PointCloud<pcl::PointXYZ> Data)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp = Data.makeShared();
	VisualizationProcess_PCXYZ(tmp);
}
//视觉化-点云数组-阻塞主线程版本。
void PointCloud_Visualizator_Stuck(pcl::PointCloud<pcl::PointXYZ>::Ptr Data[], int NumOfData)
{
	VisualizationProcess_PCXYZ_Array(Data, NumOfData);
}
void PointCloud_Visualizator_Stuck(pcl::PointCloud<pcl::PointXYZ> Data[], int NumOfData)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp[256];
	for (int i = 0; i < NumOfData; i++)
		tmp[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(Data[i].makeShared());
	VisualizationProcess_PCXYZ_Array(tmp, NumOfData);
}

void VisualizationProcess_PCXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr Data)
{
	pcl::visualization::PCLVisualizer viewertest("3D Viewer Test");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandler(Data, 255, 0, 0);//orange
	viewertest.addPointCloud(Data, ColorHandler, "Cloud");

	viewertest.setBackgroundColor(0.1, 0.1, 0.1, 0);

	while (!viewertest.wasStopped())
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(13));
		if (mtx.try_lock())
		{
			viewertest.spinOnce(5);
			mtx.unlock();
		}
	}
	viewertest.close();

}
void VisualizationProcess_PCXYZ_Array(pcl::PointCloud<pcl::PointXYZ>::Ptr Data[], int NumOfData)
{
	pcl::visualization::PCLVisualizer viewertest("3D Viewer Test");

	for (int i = 0; i < NumOfData; i++)
	{
		pcl::PointXYZRGB RGB = HSV2RGB(i * (360 / NumOfData), 255, 255);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandler(Data[i], RGB.r, RGB.g, RGB.b);
		viewertest.addPointCloud(Data[i], ColorHandler, "Cloud" + std::to_string(i));
	}

	viewertest.setBackgroundColor(0.1, 0.1, 0.1, 0);

	while (!viewertest.wasStopped())
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(13));
		if (mtx.try_lock())
		{
			viewertest.spinOnce(5);
			mtx.unlock();
		}
	}
	viewertest.close();

}

pcl::PointXYZRGB HSV2RGB(double H, double S, double V, double RangeH, double RangeS, double RangeV,
	double RangeR, double RangeG, double RangeB)
{
	pcl::PointXYZRGB Res;
	int Hi = 0.0;
	int R = 0, G = 0, B = 0;
	if (S == 0)
		R = G = B = V;
	else
	{
		H /= 60.0;
		Hi = (int)H % 6;
	}

	double f = H - Hi;
	double a = V / RangeV * (1 - S / RangeS);
	double b = V / RangeV * (1 - S / RangeS * f);
	double c = V / RangeV * (1 - S / RangeS * (1 - f));

	switch (Hi)
	{
	case 0: Res.r = V;			Res.g = c * RangeG; Res.b = a * RangeB; break;
	case 1: Res.r = b * RangeR; Res.g = V;			Res.b = a * RangeB; break;
	case 2: Res.r = a * RangeR; Res.g = V;			Res.b = c * RangeB; break;
	case 3: Res.r = a * RangeR; Res.g = b * RangeG; Res.b = V;			break;
	case 4: Res.r = c * RangeR; Res.g = a * RangeG; Res.b = V;			break;
	case 5: Res.r = V;			Res.g = a * RangeG; Res.b = b * RangeB; break;
	}
	return Res;
}
