#include "pose_math.h"
#include "vtkMatrix4x4.h"
#include "Pose.h"
#include "vtkSmartPointer.h"
#include <cmath>

void normalizeQuaternion(double& qw, double& qx, double& qy, double& qz) {
    double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm != 0) {
        qw /= norm;
        qx /= norm;
        qy /= norm;
        qz /= norm;
    }
}

// Function to convert a quaternion to a rotation matrix
void quaternionToRotationMatrix(const Pose& pose, double R[3][3]) {
    // Normalize quaternion
    double qw = pose.quatW, qx = pose.quatX, qy = pose.quatY, qz = pose.quatZ;
    normalizeQuaternion(qw, qx, qy, qz);

    // Compute rotation matrix elements
    R[0][0] = 1 - 2 * (qy * qy + qz * qz);
    R[0][1] = 2 * (qx * qy - qz * qw);
    R[0][2] = 2 * (qx * qz + qy * qw);

    R[1][0] = 2 * (qx * qy + qz * qw);
    R[1][1] = 1 - 2 * (qx * qx + qz * qz);
    R[1][2] = 2 * (qy * qz - qx * qw);

    R[2][0] = 2 * (qx * qz - qy * qw);
    R[2][1] = 2 * (qy * qz + qx * qw);
    R[2][2] = 1 - 2 * (qx * qx + qy * qy);
}


void PoseToTransMat(Pose pose, vtkSmartPointer<vtkMatrix4x4> mToolToTracker)
{

    double R[3][3];
    quaternionToRotationMatrix(pose, R);

    mToolToTracker->Identity();

    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            mToolToTracker->SetElement(row, col, R[row][col]);
        }
    }
    mToolToTracker->SetElement(0, 3, 10*pose.x);
    mToolToTracker->SetElement(1, 3, 10*pose.y);
    mToolToTracker->SetElement(2, 3, 10*pose.z);

}

