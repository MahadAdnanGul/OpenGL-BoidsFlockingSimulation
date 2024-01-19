#include <cmath>
#include "Quaternion.h"



Quaternion* eulerToQuaternion(double roll, double pitch, double yaw) {
    Quaternion* q;

    // Calculate half angles
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);


    double w = cr * cp * cy + sr * sp * sy;
    double x = sr * cp * cy - cr * sp * sy;
    double y = cr * sp * cy + sr * cp * sy;
    double z = cr * cp * sy - sr * sp * cy;
    q = new Quaternion(w,x,y,z);

    return q;
}

GLfloat* eulerAndTranslationToMatrix(double roll, double pitch, double yaw, double x, double y, double z)
{
    // Precompute trigonometric values for efficiency
    double cos_roll = cos(roll);
    double sin_roll = sin(roll);
    double cos_pitch = cos(pitch);
    double sin_pitch = sin(pitch);
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);

    Matrix4x4 g_rotationMatrix;

    // Fill the transformation matrix
    g_rotationMatrix.m[0][0] = cos_yaw * cos_pitch;
    g_rotationMatrix.m[0][1] = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
    g_rotationMatrix.m[0][2] = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
    g_rotationMatrix.m[0][3] = 0;

    g_rotationMatrix.m[1][0] = sin_yaw * cos_pitch;
    g_rotationMatrix.m[1][1] = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
    g_rotationMatrix.m[1][2] = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;
    g_rotationMatrix.m[1][3] = 0;

    g_rotationMatrix.m[2][0] = -sin_pitch;
    g_rotationMatrix.m[2][1] = cos_pitch * sin_roll;
    g_rotationMatrix.m[2][2] = cos_pitch * cos_roll;
    g_rotationMatrix.m[2][3] = 0;

    g_rotationMatrix.m[3][0] = x;
    g_rotationMatrix.m[3][1] = y;
    g_rotationMatrix.m[3][2] = z;
    g_rotationMatrix.m[3][3] = 1.0;

    GLfloat rotation[16];

    int index = 0;
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            rotation[index] = g_rotationMatrix.m[row][col];
            index++;
        }
    }

    return rotation;
}



GLfloat* quaternionToMatrix(Quaternion* q)
{
    double w = q->w;
    double x = q->x;
    double y = q->y;
    double z = q->z;

    double xx = x * x;
    double xy = x * y;
    double xz = x * z;
    double xw = x * w;

    double yy = y * y;
    double yz = y * z;
    double yw = y * w;

    double zz = z * z;
    double zw = z * w;

    Matrix4x4 g_rotationMatrix;

    g_rotationMatrix.m[0][0] = 1 - 2 * (yy + zz);
    g_rotationMatrix.m[0][1] = 2 * (xy - zw);
    g_rotationMatrix.m[0][2] = 2 * (xz + yw);
    g_rotationMatrix.m[0][3] = 0;

    g_rotationMatrix.m[1][0] = 2 * (xy + zw);
    g_rotationMatrix.m[1][1] = 1 - 2 * (xx + zz);
    g_rotationMatrix.m[1][2] = 2 * (yz - xw);
    g_rotationMatrix.m[1][3] = 0;

    g_rotationMatrix.m[2][0] = 2 * (xz - yw);
    g_rotationMatrix.m[2][1] = 2 * (yz + xw);
    g_rotationMatrix.m[2][2] = 1 - 2 * (xx + yy);
    g_rotationMatrix.m[2][3] = 0;

    g_rotationMatrix.m[3][0] = 0;
    g_rotationMatrix.m[3][1] = 0;
    g_rotationMatrix.m[3][2] = 0;
    g_rotationMatrix.m[3][3] = 1;

    GLfloat rotation[16];

    int index = 0;
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            rotation[index] = g_rotationMatrix.m[row][col];
            index++;
        }
    }

    return rotation;
}

GLfloat* quaternionAndTranslationToMatrix(Quaternion* q,double xCor, double yCor, double zCor)
{
    double w = q->w;
    double x = q->x;
    double y = q->y;
    double z = q->z;

    double xx = x * x;
    double xy = x * y;
    double xz = x * z;
    double xw = x * w;

    double yy = y * y;
    double yz = y * z;
    double yw = y * w;

    double zz = z * z;
    double zw = z * w;

    Matrix4x4 g_rotationMatrix;

    g_rotationMatrix.m[0][0] = 1 - 2 * (yy + zz);
    g_rotationMatrix.m[0][1] = 2 * (xy - zw);
    g_rotationMatrix.m[0][2] = 2 * (xz + yw);
    g_rotationMatrix.m[0][3] = 0;

    g_rotationMatrix.m[1][0] = 2 * (xy + zw);
    g_rotationMatrix.m[1][1] = 1 - 2 * (xx + zz);
    g_rotationMatrix.m[1][2] = 2 * (yz - xw);
    g_rotationMatrix.m[1][3] = 0;

    g_rotationMatrix.m[2][0] = 2 * (xz - yw);
    g_rotationMatrix.m[2][1] = 2 * (yz + xw);
    g_rotationMatrix.m[2][2] = 1 - 2 * (xx + yy);
    g_rotationMatrix.m[2][3] = 0;

    g_rotationMatrix.m[3][0] = xCor;
    g_rotationMatrix.m[3][1] = yCor;
    g_rotationMatrix.m[3][2] = zCor;
    g_rotationMatrix.m[3][3] = 1;

    GLfloat rotation[16];

    int index = 0;
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            rotation[index] = g_rotationMatrix.m[row][col];
            index++;
        }
    }

    return rotation;
}



GLfloat* eulerToMatrix(double roll, double pitch, double yaw)
{
    // Precompute trigonometric values for efficiency
    double cos_roll = cos(roll);
    double sin_roll = sin(roll);
    double cos_pitch = cos(pitch);
    double sin_pitch = sin(pitch);
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);

    Matrix4x4 g_rotationMatrix;

    // Fill the transformation matrix
    g_rotationMatrix.m[0][0] = cos_yaw * cos_pitch;
    g_rotationMatrix.m[0][1] = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
    g_rotationMatrix.m[0][2] = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
    g_rotationMatrix.m[0][3] = 0;

    g_rotationMatrix.m[1][0] = sin_yaw * cos_pitch;
    g_rotationMatrix.m[1][1] = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
    g_rotationMatrix.m[1][2] = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;
    g_rotationMatrix.m[1][3] = 0;

    g_rotationMatrix.m[2][0] = -sin_pitch;
    g_rotationMatrix.m[2][1] = cos_pitch * sin_roll;
    g_rotationMatrix.m[2][2] = cos_pitch * cos_roll;
    g_rotationMatrix.m[2][3] = 0;

    g_rotationMatrix.m[3][0] = 0.0;
    g_rotationMatrix.m[3][1] = 0.0;
    g_rotationMatrix.m[3][2] = 0.0;
    g_rotationMatrix.m[3][3] = 1.0;

    GLfloat rotation[16];

    int index = 0;
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            rotation[index] = g_rotationMatrix.m[row][col];
            index++;
        }
    }

    return rotation;
}
