#pragma once

#ifndef _VISUALIZER_H_
#define _VISUALIZER_H_

#include <pcl/visualization/pcl_visualizer.h>
#include <thread>   // std::thread
#include <mutex>
#include <chrono>
extern std::mutex mtx;

pcl::PointXYZRGB HSV2RGB(double H, double S, double V, double RangeH = 360, double RangeS = 255, double RangeV = 255, double RangeR = 255, double RangeG = 255, double RangeB = 255);

//视觉化-多线程版本。
void PointCloud_Visualizator(pcl::PointCloud<pcl::PointXYZ>::Ptr Data);
void PointCloud_Visualizator(pcl::PointCloud<pcl::PointXYZ> Data);

void PointCloud_Visualizator(pcl::PointCloud<pcl::PointXYZ>::Ptr Data[], int NumOfData);
void PointCloud_Visualizator(pcl::PointCloud<pcl::PointXYZ> Data[], int NumOfData);

//视觉化-阻塞主线程版本。
void PointCloud_Visualizator_Stuck(pcl::PointCloud<pcl::PointXYZ>::Ptr Data);
void PointCloud_Visualizator_Stuck(pcl::PointCloud<pcl::PointXYZ> Data);

void PointCloud_Visualizator_Stuck(pcl::PointCloud<pcl::PointXYZ>::Ptr Data[], int NumOfData);
void PointCloud_Visualizator_Stuck(pcl::PointCloud<pcl::PointXYZ> Data[], int NumOfData);

#endif