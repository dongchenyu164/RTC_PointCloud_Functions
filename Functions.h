#pragma once

#include "define.h"

void CovertTo_OrgnizedPointCloud(PCXYZ_Ptr &Source, double Width, double Height);
void CovertTo_UnOrgnizedPointCloud(PCXYZ_Ptr &Source);
Eigen::Matrix4f MakeTransformMatrix(double Data[4][4]);

PCXYZ_Ptr Filters(PCXYZ_Ptr Source, PCXYZ_Ptr Output);
void ExtractPlane(PCXYZ_Ptr Source, PCXYZ_Ptr Plane = PCXYZ_Ptr(new PCXYZ), PCXYZ_Ptr Rest = PCXYZ_Ptr(new PCXYZ));
int ExtractEuclideanCluster(PCXYZ_Ptr Source, PCXYZ_Ptr Clusters[]);
Mat4f ICP_Single(PCXYZ_Ptr Source, PCXYZ_Ptr Target, PCXYZ_Ptr Output);