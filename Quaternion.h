#pragma once
#include <GL/glut.h>

struct Quaternion
{
    double x, y, z, w;

    Quaternion(double a, double b, double c, double d)
    {
        w = a;
        x = b;
        y = c;
        z = d;
    }

    static Quaternion* Identity()
    {
        return new Quaternion(1,0,0,0);
    }
    
    Quaternion* operator*(const Quaternion* other) const {
        return new Quaternion(
            w * other->w - x * other->x - y * other->y - z * other->z,
            w * other->x + x * other->w + y * other->z - z * other->y,
            w * other->y - x * other->z + y * other->w + z * other->x,
            w * other->z + x * other->y - y * other->x + z * other->w
        );
    }
};

struct Matrix4x4 {
    double m[4][4];
};


Quaternion* eulerToQuaternion(double roll, double pitch, double yaw);



GLfloat* quaternionToMatrix(Quaternion* q);
GLfloat* quaternionAndTranslationToMatrix(Quaternion* q,double xCor, double yCor, double zCor);

GLfloat* eulerToMatrix(double roll, double pitch, double yaw);