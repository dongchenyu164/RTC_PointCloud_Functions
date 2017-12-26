#pragma once

#include "define.h"

int ReadPreProcessData(PCXYZ_Ptr Output);//读取数据，并进行滤波及合成。
int PreProcessData();
Eigen::Matrix4f MakeTransformMatrix(double Data[4][4]);

PCXYZ_Ptr Filters(PCXYZ_Ptr Source, PCXYZ_Ptr Output);
void ExtractPlane(PCXYZ_Ptr Source, PCXYZ_Ptr Plane = PCXYZ_Ptr(new PCXYZ), PCXYZ_Ptr Rest = PCXYZ_Ptr(new PCXYZ));//抽出平面
void ExtractEuclideanCluster(PCXYZ_Ptr Source, PCXYZ_Ptr Clusters[]);
void Filters(PCXYZ_Ptr Source[], int SourceCount, PCXYZ_Ptr Output[], int StartIndex = 0);
Mat4f Useless = Mat4f::Identity();
bool ICP(PCXYZ_Ptr Source[], int SourceCount, PCXYZ_Ptr Output[], int StartIndex = 0, Mat4f& OutputTrans = Useless);
void Segment(PCXYZ_Ptr Source, PCXYZ_Ptr Output[]);
void Segment(PCXYZ_Ptr Source[], int CalCount, PCXYZ_Ptr Output[], int StartIndex = 0);