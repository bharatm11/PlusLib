#ifndef POSE_MATH_H
#define POSE_MATH_H

#include "vtkMatrix4x4.h"
#include "Pose.h"
#include "vtkSmartPointer.h"


 /**
 * @brief Convert Pose to 4x4 Homogenous Transformation Matrix
 * @param pose Pose object.
 * @return vtkSmartPointer<vtkMatrix4x4> Homogenous Transformation Matrix.
 */

void PoseToTransMat(Pose pose, vtkSmartPointer<vtkMatrix4x4> mToolToTracker);

#endif //POSE_MATH_H