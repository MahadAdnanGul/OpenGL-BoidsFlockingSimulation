#pragma once
#include "Quaternion.h"
#include "Structs.h"

class Transform
{
public:
    Vector3* position;
    Quaternion* rotation;
    Vector3* scale;
    Transform()
    {
        position = Vector3::zero();
        rotation = Quaternion::Identity();
        scale = Vector3::one();
    }

    
};
