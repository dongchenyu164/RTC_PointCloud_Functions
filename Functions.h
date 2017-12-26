#pragma once

#include "define.h"

Eigen::Matrix4f MakeTransformMatrix(double Data[4][4]);

PCXYZ_Ptr Filters(PCXYZ_Ptr Source, PCXYZ_Ptr Output);
void ExtractPlane(PCXYZ_Ptr Source, PCXYZ_Ptr Plane = PCXYZ_Ptr(new PCXYZ), PCXYZ_Ptr Rest = PCXYZ_Ptr(new PCXYZ));
int ExtractEuclideanCluster(PCXYZ_Ptr Source, PCXYZ_Ptr Clusters[]);
bool ICP(PCXYZ_Ptr Source[], int SourceCount, PCXYZ_Ptr Output[], int StartIndex = 0, Mat4f& OutputTrans = Useless);