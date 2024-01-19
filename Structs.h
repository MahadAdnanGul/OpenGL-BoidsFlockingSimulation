#pragma once
#include <cmath>
#include <sal.h>

struct Vector3
{
    __readonly float x;
    __readonly float y;
    __readonly float z;
    __readonly float magnitude;

    
    Vector3(float a,float b, float c)
    {
        x = a;
        y = b;
        z = c;
        CalculateMagnitude();
    }
private:
    void CalculateMagnitude() 
    {
        magnitude = sqrt(pow(x,2)+pow(y,2)+pow(z,2));
    }

public:
    float Magnitude()
    {
        return magnitude;
    }

    Vector3* Normalized()
    {
        if(Magnitude()==0)
        {
            return zero();
        }
        return new Vector3(x/Magnitude(),y/Magnitude(),z/Magnitude());
    }

    float Dot(Vector3* other) {
        return x * other->x + y * other->y + z * other->z;
    }

    Vector3* operator+(const Vector3* other) const {
        return new Vector3(x + other->x, y + other->y, z + other->z);
    }

    Vector3* operator*(const float other) const {
        return new Vector3(x * other, y * other, z * other);
    }
    
    Vector3* operator-(const Vector3* other) const {
        return new Vector3(x - other->x, y - other->y, z - other->z);
    }

   

    static Vector3* back()
    {
        return new Vector3(0,0,-1);
    }
    static Vector3* forward()
    {
        return new Vector3(0,0,1);
    }
    static Vector3* down()
    {
        return new Vector3(0,-1,0);
    }
    static Vector3* left()
    {
        return new Vector3(-1,0,0);
    }
    static Vector3* right()
    {
        return new Vector3(1,0,0);
    }
    static Vector3* up()
    {
        return new Vector3(0,1,0);
    }
    static Vector3* one()
    {
        return new Vector3(1,1,1);
    }
    static Vector3* zero()
    {
        return new Vector3(0,0,0);
    }
};


struct UserInput
{
    double x;
    double y;
    double z;

    double xR;
    double yR;
    double zR;

    double qX;
    double qY;
    double qZ;
    double qW;
};
struct Texture
{
    unsigned char* texture;
    int width;
    int height;
    int channels;
};

