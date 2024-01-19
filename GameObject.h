#pragma once
#include <stdlib.h>

#include "Collider.h"
#include "Rigidbody.h"
#include "Transform.h"
#include "Utils.h"
#include "SOIL/SOIL.h"

class GameObject
{
public:
    Transform* transform;
    Rigidbody* rigidbody;
    Collider* collider;
    GLUquadric* quad;
    const float searchRadius = 8;
    const float separationFactor = 1;
    const float alignmentFactor = 2;
    const float cohesionFactor = 1;
    const float maxVelocity = 5;
    std::vector<int> neighbors;
    

    GameObject()
    {
        transform = new Transform;
        rigidbody = new Rigidbody;
        collider = new Collider(transform->scale->Magnitude()/2);
        quad = gluNewQuadric();
    }
    
    void Render()
    {
        glPushMatrix();
        //glTranslatef (transform->position->x, transform->position->y, transform->position->z);
        glMultMatrixf(quaternionAndTranslationToMatrix(transform->rotation,transform->position->x, transform->position->y, transform->position->z));
        gluQuadricTexture(quad,1);
        gluSphere(quad,transform->scale->x,20,20);
        glPopMatrix();
    }

    void CheckForCollisions(GameObject* collision)
    {
        float distance = Distance(transform->position,collision->transform->position);
        if(distance < collision->collider->radius + collider->radius)
        {
            ResolveCollision(collision);
        }
    }

    void CheckForNeighbors(const std::vector<GameObject>& gameObjects, int exclude)
    {
        neighbors.clear();
        for(int i = 0;i<gameObjects.size();i++)
        {
            if(i!=exclude)
            {
                Vector3* direction = (*gameObjects[i].transform->position - transform->position)->Normalized();
                float distance = (*gameObjects[i].transform->position - transform->position)->Magnitude();

                //Only detect neighbor if within LOS
                if(distance<=searchRadius && (rigidbody->velocity->Normalized()->Dot(direction))>0)
                {
                    neighbors.push_back(i);
                }
                delete(direction);
            }
        }
    }
    
    Vector3* GetSeparation(const std::vector<GameObject>& gameObjects)
    {
        Vector3* direction = Vector3::zero();

        for (int i = 0; i < neighbors.size(); i++)
        {
            Vector3* diff = *gameObjects[neighbors[i]].transform->position - transform->position;
            float dist = std::max(diff->Magnitude()-(collider->radius*3), 0.1f);
            
            
            diff = new Vector3(diff->x/(dist*dist),diff->y/(dist*dist),diff->z/(dist*dist)); 
            //diff = *diff / (dist * dist);

            // Accumulate the modified difference vector to the direction
            direction = *direction - diff;
            
            delete diff;
        }
        
        direction = *direction * separationFactor;

        return direction;
    }

    /*Vector3* GetSeparation(const std::vector<GameObject>& gameObjects)
    {
        Vector3* direction = Vector3::zero();
        for(int i = 0; i<neighbors.size();i++)
        {
            Vector3* diff = (*gameObjects[neighbors[i]].transform->position - transform->position);
            float dist = std::max(diff->Magnitude(),collider->radius*2);
            diff = new Vector3(diff->x * (dist*dist), diff->y*(dist*dist), diff->z*(dist*dist));
            direction = *direction - diff;
        }
        direction = direction->Normalized();
        return new Vector3(direction->x*separationFactor,direction->y*separationFactor,direction->z*separationFactor);
    }*/

    Vector3* GetAlignment(const std::vector<GameObject>& gameObjects)
    {
        Vector3* averageDirection = Vector3::zero();

        for (int i = 0; i < neighbors.size(); i++)
        {
            // Accumulate the direction of the neighbor
            averageDirection = *averageDirection + (gameObjects[neighbors[i]].rigidbody->velocity->Normalized());
        }

        // Calculate the average direction
        if (neighbors.size() > 0)
        {
            float size = static_cast<float>(neighbors.size());
            averageDirection = new Vector3(averageDirection->x/size, averageDirection->y/size,averageDirection->z/size);
        }

        Vector3* alignment = *averageDirection - rigidbody->velocity;
        

        return new Vector3 (alignment->x*alignmentFactor,alignment->y*alignmentFactor,alignment->z*alignmentFactor);
    }

    Vector3* GetCohesion(const std::vector<GameObject>& gameObjects)
    {
        Vector3* centerOfMass = Vector3::zero();

        for (int i = 0; i < neighbors.size(); i++)
        {
            // Accumulate the position of the neighbor
            centerOfMass = *centerOfMass + gameObjects[neighbors[i]].transform->position;
        }

        // Calculate the average position (center of mass)
        if (neighbors.size() > 0)
        {
            float size = static_cast<float>(neighbors.size());
            centerOfMass = new Vector3(centerOfMass->x/size, centerOfMass->y/size,centerOfMass->z/size);
        }

        // Calculate the vector pointing towards the center of mass
        Vector3* cohesionVector = *centerOfMass - transform->position;

        
        delete centerOfMass;

        return new Vector3(cohesionVector->x*cohesionFactor,cohesionVector->y*cohesionFactor,cohesionVector->z*cohesionFactor);
    }

    Vector3* GetAttractionForce(const std::vector<Vector3>& targets, int index)
    {
        if(targets.size()==0)
        {
            return Vector3::zero();
        }
        return (targets[index] - transform->position);
    }
    
    float MomentOfInertia()
    {
        return rigidbody->mass * pow(collider->radius, 2); 
    }

    void ResolveCollision(GameObject* collision)
    {
        // Calculate the direction from this object to the colliding object
        Vector3* collisionDirection = *collision->transform->position - transform->position;

        // Calculate the normal of our direction
        Vector3* collisionNormal = collisionDirection->Normalized();

        // Calculate the overlap distance
        float overlap = (collider->radius + collision->collider->radius) - collisionDirection->Magnitude();

        // calculate the separation vector
        Vector3* separation = *collisionDirection*(0.25f*overlap);

        // Adjust the positions of the colliding objects using the separation vector so that they are no longer overlapping
        transform->position = *transform->position - (separation);
        collision->transform->position = *collision->transform->position + (separation);


        //Handling Translation
        //Calculate relative velocity
        /*Vector3* relativeVelocity = *rigidbody->velocity-collision->rigidbody->velocity;

        // Calculate impulse magnitude based of conservation of momentum and kinetic energy. We take dot product with our normal to account for the angle of collision
        float impulseMagnitude = -(1.0f + rigidbody->bounceReduction) * relativeVelocity->Dot(collisionNormal);
        impulseMagnitude /= 1.0f / rigidbody->mass + 1.0f / collision->rigidbody->mass;

        //adjust resultant velocity addition based on mass
        Vector3* resultant = *collisionNormal*(impulseMagnitude/rigidbody->mass);
        Vector3* resultantB = *collisionNormal*(impulseMagnitude/collision->rigidbody->mass);
        
        rigidbody->velocity = *rigidbody->velocity + resultant;
        collision->rigidbody->velocity = *collision->rigidbody->velocity - resultantB;

        Vector3* relativeAngularVelocity = *rigidbody->angularVelocity - collision->rigidbody->angularVelocity;
        
        float momentOfInertiaA = MomentOfInertia();
        float momentOfInertiaB = collision->MomentOfInertia();

        // Calculate the angular impulse similarly using conservation of angular momentum
        float angularImpulseMagnitude = -(1.0f + rigidbody->bounceReduction) * relativeAngularVelocity->Dot(collisionNormal);
        angularImpulseMagnitude /= 1.0f / momentOfInertiaA + 1.0f / momentOfInertiaB;

        // Apply angular impulse to adjust angular velocities
        Vector3* angularResultA = *collisionNormal*(angularImpulseMagnitude / momentOfInertiaA) ;
        Vector3* angularResultB = *collisionNormal*(angularImpulseMagnitude / momentOfInertiaB) ;
        rigidbody->angularVelocity = *rigidbody->angularVelocity + angularResultA;
        collision->rigidbody->angularVelocity = *collision->rigidbody->angularVelocity - angularResultB;*/
    }

    void CheckForCollisionsWithBounds(Vector3* negBounds, Vector3* posBounds)
    {
        // Simply reflect velocity with bounds and reduce angular momentum based on bounce reduction
        if(transform->position->y - collider->radius <= negBounds->y)
        {
            transform->position = new Vector3(transform->position->x,negBounds->y + collider->radius,transform->position->z);
            rigidbody->velocity = new Vector3(rigidbody->velocity->x*(rigidbody->bounceReduction),-rigidbody->velocity->y*(rigidbody->bounceReduction),rigidbody->velocity->z*(rigidbody->bounceReduction));
            rigidbody->angularVelocity = new Vector3(rigidbody->angularVelocity->x*(rigidbody->bounceReduction),rigidbody->angularVelocity->y*(rigidbody->bounceReduction),rigidbody->angularVelocity->z*(rigidbody->bounceReduction));
        }
        if(transform->position->y + collider->radius >= posBounds->y)
        {
            transform->position = new Vector3(transform->position->x,posBounds->y - collider->radius,transform->position->z);
            rigidbody->velocity = new Vector3(rigidbody->velocity->x*(rigidbody->bounceReduction),-rigidbody->velocity->y*(rigidbody->bounceReduction),rigidbody->velocity->z*(rigidbody->bounceReduction));
            rigidbody->angularVelocity = new Vector3(rigidbody->angularVelocity->x*(rigidbody->bounceReduction),rigidbody->angularVelocity->y*(rigidbody->bounceReduction),rigidbody->angularVelocity->z*(rigidbody->bounceReduction));
        }

        if(transform->position->x - collider->radius <= negBounds->x)
        {
            transform->position = new Vector3(negBounds->x + collider->radius,transform->position->y,transform->position->z);
            rigidbody->velocity = new Vector3(-rigidbody->velocity->x*(rigidbody->bounceReduction),rigidbody->velocity->y*(rigidbody->bounceReduction),rigidbody->velocity->z*(rigidbody->bounceReduction));
            rigidbody->angularVelocity = new Vector3(rigidbody->angularVelocity->x*(rigidbody->bounceReduction),rigidbody->angularVelocity->y*(rigidbody->bounceReduction),rigidbody->angularVelocity->z*(rigidbody->bounceReduction));
        }
        if(transform->position->x + collider->radius >= posBounds->x)
        {
            transform->position = new Vector3(posBounds->x - collider->radius,transform->position->y,transform->position->z);
            rigidbody->velocity = new Vector3(-rigidbody->velocity->x*(rigidbody->bounceReduction),rigidbody->velocity->y*(rigidbody->bounceReduction),rigidbody->velocity->z*(rigidbody->bounceReduction));
            rigidbody->angularVelocity = new Vector3(rigidbody->angularVelocity->x*(rigidbody->bounceReduction),rigidbody->angularVelocity->y*(rigidbody->bounceReduction),rigidbody->angularVelocity->z*(rigidbody->bounceReduction));
        }

        if(transform->position->z - collider->radius <= negBounds->z)
        {
            transform->position = new Vector3(transform->position->x,transform->position->y,negBounds->z + collider->radius);
            rigidbody->velocity = new Vector3(rigidbody->velocity->x*(rigidbody->bounceReduction),rigidbody->velocity->y*(rigidbody->bounceReduction),-rigidbody->velocity->z*(rigidbody->bounceReduction));
            rigidbody->angularVelocity = new Vector3(rigidbody->angularVelocity->x*(rigidbody->bounceReduction),rigidbody->angularVelocity->y*(rigidbody->bounceReduction),rigidbody->angularVelocity->z*(rigidbody->bounceReduction));
        }
        if(transform->position->z + collider->radius >= posBounds->z)
        {
            transform->position = new Vector3(transform->position->x,transform->position->y,posBounds->z - collider->radius);
            rigidbody->velocity = new Vector3(rigidbody->velocity->x*(rigidbody->bounceReduction),rigidbody->velocity->y*(rigidbody->bounceReduction),-rigidbody->velocity->z*(rigidbody->bounceReduction));
            rigidbody->angularVelocity = new Vector3(rigidbody->angularVelocity->x*(rigidbody->bounceReduction),rigidbody->angularVelocity->y*(rigidbody->bounceReduction),rigidbody->angularVelocity->z*(rigidbody->bounceReduction));
        }
    }
    void ResolveForces(float dt)
    {
        //Record changes in velocity
        float x = rigidbody->acceleration->x * dt - (rigidbody->velocity->x * rigidbody->drag * dt);
        float y = rigidbody->acceleration->y * dt - (rigidbody->velocity->y * rigidbody->drag * dt);
        float z = rigidbody->acceleration->z * dt - (rigidbody->velocity->z * rigidbody->drag * dt);

        //Add gravitational force if gravity enabled
        if(rigidbody->useGravity)
        {
            y += Rigidbody::GRAVITY*dt;
        }

        //Convert angular velocity change into a quaternion
        Quaternion* q = eulerToQuaternion(rigidbody->angularVelocity->x*dt,rigidbody->angularVelocity->y*dt,rigidbody->angularVelocity->z*dt);

        //Calculate new rotation
        Quaternion* newRotation = *q * transform->rotation;
        

        //Normalize
        double magnitude = std::sqrt(newRotation->x * newRotation->x +
                              newRotation->y * newRotation->y +
                              newRotation->z * newRotation->z +
                              newRotation->w * newRotation->w);
        
        newRotation->x /= magnitude;
        newRotation->y /= magnitude;
        newRotation->z /= magnitude;
        newRotation->w /= magnitude;

        //Update rotation
        transform->rotation = newRotation;
        
        // update velocity
        rigidbody->velocity = new Vector3(rigidbody->velocity->x+x,rigidbody->velocity->y+y,rigidbody->velocity->z+z);
        if(rigidbody->velocity->Magnitude()>maxVelocity)
        {
            Vector3* velocity = rigidbody->velocity->Normalized();
            velocity = *velocity*maxVelocity;
            rigidbody->velocity=velocity;
        }

        //Apply angular drag
        float xA = (rigidbody->angularVelocity->x * rigidbody->angularDrag * dt);
        float yA = (rigidbody->angularVelocity->y * rigidbody->angularDrag * dt);
        float zA = (rigidbody->angularVelocity->z * rigidbody->angularDrag * dt);

        rigidbody->angularVelocity = new Vector3(rigidbody->angularVelocity->x - xA,rigidbody->angularVelocity->y-yA,rigidbody->angularVelocity->z-zA);
        
        //update position based on velocity
        transform->position = new Vector3(transform->position->x+(rigidbody->velocity->x*dt),transform->position->y+(rigidbody->velocity->y*dt),transform->position->z+(rigidbody->velocity->z*dt));

        delete(q);
    }

    void HandleFlocking(const std::vector<GameObject>& gameObjects,float dt, Vector3 pos,const std::vector<Vector3>& targets, int targetIndex)
    {
        Vector3* steeringForce = Vector3::zero();
        if(neighbors.size()>0)
        {
            Vector3* separation = GetSeparation(gameObjects);
            //std::cout<<separation->x<<std::endl;
            //std::cout<<separation->y<<std::endl;
            //std::cout<<separation->z<<std::endl<<std::endl;
            Vector3* alignment = GetAlignment(gameObjects);
            Vector3* cohesion = GetCohesion(gameObjects);
            Vector3* attraction = GetAttractionForce(targets,targetIndex);
            steeringForce = *(*separation+(*alignment+cohesion))+attraction;
            delete(separation);
            delete(alignment);
            delete(cohesion);
            delete(attraction);
        }
        else
        {
            steeringForce = GetAttractionForce(targets,targetIndex);
        }
        
        
        //Vector3* steeringForce = *separation + cohesion;
        rigidbody->AddImpulseForce(new Vector3(steeringForce->x*dt,steeringForce->y*dt,steeringForce->z*dt));
        rigidbody->angularVelocity = new Vector3(-rigidbody->velocity->z,0,rigidbody->velocity->x);
        delete(steeringForce);
        
    }

    void HandleSeparation(const std::vector<GameObject>& gameObjects,float dt)
    {
        Vector3* steeringForce = GetSeparation(gameObjects);
        //rigidbody->velocity = steeringForce;
        rigidbody->AddImpulseForce(new Vector3(steeringForce->x*dt,steeringForce->y*dt,steeringForce->z*dt));
        //transform->position = new Vector3(transform->position->x+(steeringForce->x*dt), transform->position->y+(steeringForce->y*dt), transform->position->z+(steeringForce->z*dt));
    }

    void HandleAlignment(const std::vector<GameObject>& gameObjects,float dt)
    {
        Vector3* steeringForce = GetAlignment(gameObjects);
        //rigidbody->velocity = steeringForce;
        rigidbody->AddImpulseForce(new Vector3(steeringForce->x*dt,steeringForce->y*dt,steeringForce->z*dt));
    }

    void HandleCohesion(const std::vector<GameObject>& gameObjects,float dt)
    {
        Vector3* steeringForce = GetCohesion(gameObjects);
        //rigidbody->velocity = steeringForce;
        rigidbody->AddImpulseForce(new Vector3(steeringForce->x*dt,steeringForce->y*dt,steeringForce->z*dt));
    }

    
    
    
};
