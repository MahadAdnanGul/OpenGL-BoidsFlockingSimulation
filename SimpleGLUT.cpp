#include "stdafx.h"
#include "Interpolation.h"
#include "Structs.h"
// standard


#include <fstream>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

// glut
#include <iostream>
#include <string>
#include <vector>


#include "GameObject.h"
#include "Quaternion.h"
#include "Utils.h"
#include "SOIL/SOIL.h"
#include "OpenFBX/ofbx.h"



//================================
// global variables
//================================
// screen size
int i = 1;

int g_screenWidth  = 100;
int g_screenHeight = 100;

// frame index
int g_frameIndex = 0;

// angle for rotation
int g_angle = 0;

double g_positionX = 0;
double g_positionY = 0;
double g_positionZ = 0;

double g_rotationX = 0;
double g_rotationY = 0;
double g_rotationZ = 0;

double dt;
int framesPerKeyframe = 60;

Vector3* position = new Vector3(1, 2, 3);

UserInput* g_userInput;
int inputSize;


GLfloat* g_rotation;

int const legAngleFactor = 45;
double legTraverseFactor = 0.5;


double legAngleA = 0;
int legAngleFactorA = legAngleFactor;
double legAngleB = 0;
int legAngleFactorB = -legAngleFactor;

double legTraverseA = 0;
double legTraverseFactorA = legTraverseFactor;

double legTraverseB  = 0;
double legTraverseFactorB = -legTraverseFactor;

double movementDelta = 0;

SplineType g_splineType;
AngleInput g_angleInput;


Texture skyboxTextures[6];
Texture ground;

float separationDistance = 4;

std::vector<Vector3> targets;
int currentTargetIndex = 0;

//GameObject** gameObject;
std::vector<GameObject> gameObject;
int numberOfGameObjects = 100;
//GameObject* collisionObject;

Vector3* negBounds = new Vector3(-15,-15,-15);
Vector3* posBounds = new Vector3(15,15,15);

struct PhysicsInput
{
	int numberOfObjects;
	float mass;
	float bounceReduction;
	float drag;
	float angularDrag;
	float minVelocity;
	float maxVelocity;
	
	//IN RADIANS
	float minAngular;
	float maxAngular;
};

PhysicsInput pInput;

float skyboxVertices[] =
{
	//   Coordinates
	-1.0f, -1.0f,  1.0f,//        7--------6
	 1.0f, -1.0f,  1.0f,//       /|       /|
	 1.0f, -1.0f, -1.0f,//      4--------5 |
	-1.0f, -1.0f, -1.0f,//      | |      | |
	-1.0f,  1.0f,  1.0f,//      | 3------|-2
	 1.0f,  1.0f,  1.0f,//      |/       |/
	 1.0f,  1.0f, -1.0f,//      0--------1
	-1.0f,  1.0f, -1.0f
};

unsigned int skyboxIndices[] =
{
	// Right
	1, 2, 6,
	6, 5, 1,
	// Left
	0, 4, 7,
	7, 3, 0,
	// Top
	4, 5, 6,
	6, 7, 4,
	// Bottom
	0, 3, 2,
	2, 1, 0,
	// Back
	0, 1, 5,
	5, 4, 0,
	// Front
	3, 7, 6,
	6, 2, 3
};

int generateRandomNumber(int min, int max) {
	return rand() % (max - min + 1) + min;
}



void drawSkybox()
{
	//Not exactly a sky box but since our camera did not need to rotate a backdrop seemed enough for now.
	glPushMatrix();
	glTranslatef(0, 1, -300);
	glScalef(500, 500, 1);
	GLuint textureID;
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(
	GL_TEXTURE_2D,  // Target
0,              // Level of detail (0 for base level)
GL_RGBA,        // Internal format (depends on your image format)
skyboxTextures[0].width,          // Width of the texture
skyboxTextures[0].height,         // Height of the texture
0,              // Border (always 0)
GL_RGBA,        // Format of the pixel data in your image (depends on your image format)
GL_UNSIGNED_BYTE, // Data type of the pixel data
skyboxTextures[0].texture          // Pointer to the image data
);
	glEnable(GL_TEXTURE_2D);
	glBegin(GL_QUADS);
	// Front face vertices and texture coordinates
	glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f, -1.0f, 1.0f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(1.0f, -1.0f,  1.0f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(1.0f,  1.0f,  1.0f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  1.0f,  1.0f);
	glEnd();
	glDisable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 0);
	glPopMatrix();

}

void applyGroundTexture()
{
	glPushMatrix();
	GLuint textureID;
	LoadTexture(textureID,ground);
	glEnable(GL_TEXTURE_2D);
	int numTilesX = 1; // Adjust as needed
	int numTilesY = 1; // Adjust as needed
	//glBindTexture(GL_TEXTURE_2D,textureID);
	glBegin(GL_QUADS);
	// Define the vertices and texture coordinates
	for (int i = 0; i < numTilesX; i++) {
		for (int j = 0; j < numTilesY; j++) {
			float tileSizeX = 10.0f / numTilesX;
			float tileSizeY = 10.0f / numTilesY;

			float x0 = i * tileSizeX;
			float x1 = (i + 1) * tileSizeX;
			float y0 = j * tileSizeY;
			float y1 = (j + 1) * tileSizeY;

			// Vertex 1
			glTexCoord2f(x0, y0);
			glVertex3f(-100.0f, -15.0f, -100.0f);

			// Vertex 2
			glTexCoord2f(x1, y0);
			glVertex3f(100.0f, -15.0f, -100.0f);

			// Vertex 3
			glTexCoord2f(x1, y1);
			glVertex3f(100.0f, -15.0f, 100.0f);

			// Vertex 4
			glTexCoord2f(x0, y1);
			glVertex3f(-100.0f, -15.0f, 100.0f);
		}
	}

	glEnd();
	
	glDisable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D,0);

	glPopMatrix();
	
}

void getUserInputForPhysicsSimulation(const std::string& filename)
{
	std::ifstream inputFile(filename);

	if (!inputFile.is_open()) {
		//std::cerr << "Error: Failed to open the input file." << std::endl;
		return;
	}
	inputFile>>pInput.numberOfObjects;
	inputFile>>pInput.mass;
	inputFile>>pInput.bounceReduction;
	inputFile>>pInput.drag;
	inputFile>>pInput.angularDrag;
	inputFile>>pInput.minVelocity;
	inputFile>>pInput.maxVelocity;
	inputFile>>pInput.minAngular;
	inputFile>>pInput.maxAngular;
	
	
	
}

void getUserInputForFlocking(const std::string& filename)
{
	std::ifstream inputFile(filename);

	if (!inputFile.is_open()) {
		//std::cerr << "Error: Failed to open the input file." << std::endl;
		return;
	}

	Vector3 target(0,0,0);
	float number;
	while (inputFile >> number) {
		float x,y,z;
		x = number;
		inputFile>>y;
		inputFile>>z;
		target = Vector3(x,y,z);
		targets.push_back(target);
	}

	inputFile.close();
	
	
	
}

void SpawnRandomBoid()
{
	GameObject newObject = GameObject();

	newObject.transform->position = new Vector3(generateRandomNumber(-14,14),generateRandomNumber(-14,14),generateRandomNumber(-14,14));
	newObject.rigidbody->useGravity = false;

	/*if(targets.size()>0)
	{
		newObject.target = targets[currentTargetIndex];
	}*/
	newObject.rigidbody->AddImpulseForce(new Vector3(generateRandomNumber(-1,1),generateRandomNumber(-1,1),generateRandomNumber(-1,1)));
	gameObject.push_back(newObject);
}

void SpawnBoid(Vector3* position, Vector3* velocity)
{
	GameObject newObject = GameObject();

	newObject.transform->position = position;
	newObject.rigidbody->useGravity = false;

	/*if(targets.size()>0)
	{
		newObject.target = targets[0];
	}
	*/

	newObject.rigidbody->AddImpulseForce(velocity);
	newObject.rigidbody->AddAngularImpulse(new Vector3(velocity->z,0,velocity->x));

	gameObject.push_back(newObject);
}

void SpawnGameObject(bool gravity = true)
{
	GameObject newObject = GameObject();  // Create a new GameObject
	/*if(targets.size()>0)
	{
		newObject.target = targets[0];
	}*/
	// Spawn objects at random positions within bounds
	newObject.transform->position = new Vector3(generateRandomNumber(-15,15), generateRandomNumber(-15,15), generateRandomNumber(-15,15));
	newObject.rigidbody->useGravity = gravity;

	// Add a random velocity and angular velocity impulse
	newObject.rigidbody->AddImpulseForce(new Vector3(generateRandomNumber(-1,1),
													  generateRandomNumber(-1,1),
													  generateRandomNumber(-1,1)));

	newObject.rigidbody->AddAngularImpulse(new Vector3(newObject.rigidbody->velocity->z,
													  newObject.rigidbody->velocity->y,
													  newObject.rigidbody->velocity->x));

	gameObject.push_back(newObject);
}



void getUserInputFromFile(const std::string& filename) {
	std::ifstream inputFile(filename);

	if (!inputFile.is_open()) {
		//std::cerr << "Error: Failed to open the input file." << std::endl;
		return;
	}

	inputFile>>dt;
	inputFile>>inputSize;

	framesPerKeyframe = static_cast<int>(1000.0)/dt;

	g_userInput = new UserInput[inputSize];

	std::string splineType;
	inputFile>>splineType;
	if(splineType=="Bezier")
	{
		g_splineType = Bezier;
	}
	else
	{
		g_splineType = CatMullRom;
	}

	std::string angleType;
	inputFile>>angleType;
	if(angleType=="Quaternion")
	{
		g_angleInput = Quaternions;
	}
	else
	{
		g_angleInput = Fixed;
	}

	for (int i = 0; i < inputSize; i++) {
		inputFile >> g_userInput[i].x >> g_userInput[i].y >> g_userInput[i].z;
		if(g_angleInput==Fixed)
		{
			inputFile >> g_userInput[i].xR >> g_userInput[i].yR >> g_userInput[i].zR;
			// Convert rotation angles to radians
			g_userInput[i].xR = g_userInput[i].xR * M_PI / 180.0;
			g_userInput[i].yR = g_userInput[i].yR * M_PI / 180.0;
			g_userInput[i].zR = g_userInput[i].zR * M_PI / 180.0;
		}
		else
		{
			inputFile >> g_userInput[i].qX >> g_userInput[i].qY >> g_userInput[i].qZ >> g_userInput[i].qW;
		}
	}
	
	inputFile.close();
}


void init( void ) {
	// init something before main loop...
}

void CatMullRomHandling(double t, int currentKeyframeIndex, int nextKeyframeIndex)
{
	// Perform Catmull-Rom interpolation for position
	double interpolatedX = Interpolation::CatmullRomInterpolation(t,
	                                               g_userInput[(currentKeyframeIndex - 1 + inputSize) % inputSize].x,
	                                               g_userInput[currentKeyframeIndex].x,
	                                               g_userInput[nextKeyframeIndex].x,
	                                               g_userInput[(nextKeyframeIndex + 1) % inputSize].x);

	double interpolatedY = Interpolation::CatmullRomInterpolation(t,
	                                               g_userInput[(currentKeyframeIndex - 1 + inputSize) % inputSize].y,
	                                               g_userInput[currentKeyframeIndex].y,
	                                               g_userInput[nextKeyframeIndex].y,
	                                               g_userInput[(nextKeyframeIndex + 1) % inputSize].y);

	double interpolatedZ = Interpolation::CatmullRomInterpolation(t,
	                                               g_userInput[(currentKeyframeIndex - 1 + inputSize) % inputSize].z,
	                                               g_userInput[currentKeyframeIndex].z,
	                                               g_userInput[nextKeyframeIndex].z,
	                                               g_userInput[(nextKeyframeIndex + 1) % inputSize].z);

	// Perform Catmull-Rom interpolation for rotation (in radians)
	if(g_angleInput==Fixed)
	{
		double interpolatedXRotation = Interpolation::CatmullRomAngleInterpolation(t,
		                                                            g_userInput[(currentKeyframeIndex - 1 + inputSize) % inputSize].xR,
		                                                            g_userInput[currentKeyframeIndex].xR,
		                                                            g_userInput[nextKeyframeIndex].xR,
		                                                            g_userInput[(nextKeyframeIndex + 1) % inputSize].xR);

		double interpolatedYRotation = Interpolation::CatmullRomAngleInterpolation(t,
		                                                            g_userInput[(currentKeyframeIndex - 1 + inputSize) % inputSize].yR,
		                                                            g_userInput[currentKeyframeIndex].yR,
		                                                            g_userInput[nextKeyframeIndex].yR,
		                                                            g_userInput[(nextKeyframeIndex + 1) % inputSize].yR);

		double interpolatedZRotation = Interpolation::CatmullRomAngleInterpolation(t,
		                                                            g_userInput[(currentKeyframeIndex - 1 + inputSize) % inputSize].zR,
		                                                            g_userInput[currentKeyframeIndex].zR,
		                                                            g_userInput[nextKeyframeIndex].zR,
		                                                            g_userInput[(nextKeyframeIndex + 1) % inputSize].zR);

		g_rotationX = interpolatedXRotation;
		g_rotationY = interpolatedYRotation;
		g_rotationZ = interpolatedZRotation;

		Quaternion* q = eulerToQuaternion(g_rotationX, g_rotationY, g_rotationZ);
		g_rotation = quaternionToMatrix(q);
	}
	else
	{
		double interpolatedXRotation = Interpolation::CatmullRomAngleInterpolation(t,
		                                                            g_userInput[(currentKeyframeIndex - 1 + inputSize) % inputSize].qX,
		                                                            g_userInput[currentKeyframeIndex].qX,
		                                                            g_userInput[nextKeyframeIndex].qX,
		                                                            g_userInput[(nextKeyframeIndex + 1) % inputSize].qX);

		double interpolatedYRotation = Interpolation::CatmullRomAngleInterpolation(t,
		                                                            g_userInput[(currentKeyframeIndex - 1 + inputSize) % inputSize].qY,
		                                                            g_userInput[currentKeyframeIndex].qY,
		                                                            g_userInput[nextKeyframeIndex].qY,
		                                                            g_userInput[(nextKeyframeIndex + 1) % inputSize].qY);

		double interpolatedZRotation = Interpolation::CatmullRomAngleInterpolation(t,
		                                                            g_userInput[(currentKeyframeIndex - 1 + inputSize) % inputSize].qZ,
		                                                            g_userInput[currentKeyframeIndex].qZ,
		                                                            g_userInput[nextKeyframeIndex].qZ,
		                                                            g_userInput[(nextKeyframeIndex + 1) % inputSize].qZ);

		double interpolatedWRotation = Interpolation::CatmullRomAngleInterpolation(t,
		                                                            g_userInput[(currentKeyframeIndex - 1 + inputSize) % inputSize].qW,
		                                                            g_userInput[currentKeyframeIndex].qW,
		                                                            g_userInput[nextKeyframeIndex].qW,
		                                                            g_userInput[(nextKeyframeIndex + 1) % inputSize].qW);

		Quaternion* q = new Quaternion(interpolatedWRotation,interpolatedXRotation,interpolatedYRotation,interpolatedZRotation);
		g_rotation = quaternionToMatrix(q);
	}
    
	movementDelta = abs(((abs(interpolatedX-g_positionX)+abs(interpolatedZ-g_positionZ))/2.0));
	// Update the current position and rotation
	g_positionX = interpolatedX;
	g_positionY = interpolatedY;
	g_positionZ = interpolatedZ;
}

void BezierHandling(double t, int currentKeyframeIndex, int nextKeyframeIndex)
{
	double interpolatedX = Interpolation::BSplinePositionInterpolation(t,
	                                                    g_userInput[(currentKeyframeIndex - 1 + inputSize) % inputSize].x,
	                                                    g_userInput[currentKeyframeIndex].x,
	                                                    g_userInput[nextKeyframeIndex].x,
	                                                    g_userInput[(nextKeyframeIndex + 1) % inputSize].x);

	double interpolatedY = Interpolation::BSplinePositionInterpolation(t,
	                                                    g_userInput[(currentKeyframeIndex - 1 + inputSize) % inputSize].y,
	                                                    g_userInput[currentKeyframeIndex].y,
	                                                    g_userInput[nextKeyframeIndex].y,
	                                                    g_userInput[(nextKeyframeIndex + 1) % inputSize].y);

	double interpolatedZ = Interpolation::BSplinePositionInterpolation(t,
	                                                    g_userInput[(currentKeyframeIndex - 1 + inputSize) % inputSize].z,
	                                                    g_userInput[currentKeyframeIndex].z,
	                                                    g_userInput[nextKeyframeIndex].z,
	                                                    g_userInput[(nextKeyframeIndex + 1) % inputSize].z);
	
	if(g_angleInput==Fixed)
	{
		double interpolatedXRotation = Interpolation::BSplineAngleInterpolation(t,
		                                                         g_userInput[(currentKeyframeIndex - 1 + inputSize) % inputSize].xR,
		                                                         g_userInput[currentKeyframeIndex].xR,
		                                                         g_userInput[nextKeyframeIndex].xR,
		                                                         g_userInput[(nextKeyframeIndex + 1) % inputSize].xR);

		double interpolatedYRotation = Interpolation::BSplineAngleInterpolation(t,
		                                                         g_userInput[(currentKeyframeIndex - 1 + inputSize) % inputSize].yR,
		                                                         g_userInput[currentKeyframeIndex].yR,
		                                                         g_userInput[nextKeyframeIndex].yR,
		                                                         g_userInput[(nextKeyframeIndex + 1) % inputSize].yR);

		double interpolatedZRotation = Interpolation::BSplineAngleInterpolation(t,
		                                                         g_userInput[(currentKeyframeIndex - 1 + inputSize) % inputSize].zR,
		                                                         g_userInput[currentKeyframeIndex].zR,
		                                                         g_userInput[nextKeyframeIndex].zR,
		                                                         g_userInput[(nextKeyframeIndex + 1) % inputSize].zR);

		g_rotationX = interpolatedXRotation;
		g_rotationY = interpolatedYRotation;
		g_rotationZ = interpolatedZRotation;
		

		Quaternion* q = eulerToQuaternion(g_rotationX, g_rotationY, g_rotationZ);
		g_rotation = quaternionToMatrix(q);
	}
	else
	{
		double interpolatedXRotation = Interpolation::BSplineAngleInterpolation(t,
		                                                         g_userInput[(currentKeyframeIndex - 1 + inputSize) % inputSize].qX,
		                                                         g_userInput[currentKeyframeIndex].qX,
		                                                         g_userInput[nextKeyframeIndex].qX,
		                                                         g_userInput[(nextKeyframeIndex + 1) % inputSize].qX);

		double interpolatedYRotation = Interpolation::BSplineAngleInterpolation(t,
		                                                         g_userInput[(currentKeyframeIndex - 1 + inputSize) % inputSize].qY,
		                                                         g_userInput[currentKeyframeIndex].qY,
		                                                         g_userInput[nextKeyframeIndex].qY,
		                                                         g_userInput[(nextKeyframeIndex + 1) % inputSize].qY);

		double interpolatedZRotation = Interpolation::BSplineAngleInterpolation(t,
		                                                         g_userInput[(currentKeyframeIndex - 1 + inputSize) % inputSize].qZ,
		                                                         g_userInput[currentKeyframeIndex].qZ,
		                                                         g_userInput[nextKeyframeIndex].qZ,
		                                                         g_userInput[(nextKeyframeIndex + 1) % inputSize].qZ);

		double interpolatedWRotation = Interpolation::BSplineAngleInterpolation(t,
		                                                         g_userInput[(currentKeyframeIndex - 1 + inputSize) % inputSize].qW,
		                                                         g_userInput[currentKeyframeIndex].qW,
		                                                         g_userInput[nextKeyframeIndex].qW,
		                                                         g_userInput[(nextKeyframeIndex + 1) % inputSize].qW);

		Quaternion* q = new Quaternion(interpolatedWRotation,interpolatedXRotation,interpolatedYRotation,interpolatedZRotation);
		g_rotation = quaternionToMatrix(q);
	}
	
	movementDelta = abs(((abs(interpolatedX-g_positionX)+abs(interpolatedZ-g_positionZ))/2.0));
	g_positionX = interpolatedX;
	g_positionY = interpolatedY;
	g_positionZ = interpolatedZ;
}

void LegsHandling(int currentKeyframeIndex)
{
	if(currentKeyframeIndex == 1)
	{
		if(g_frameIndex==1)
		{
			legAngleA=0;
			legAngleFactorA=legAngleFactor;

			legAngleB=0;
			legAngleFactorB=-legAngleFactor;

			legTraverseA = 0;
			legTraverseB = 0;

			legTraverseFactorA = legTraverseFactor;
			legTraverseFactorB = -legTraverseFactor;
		}
	}
	
	legAngleA += movementDelta*legAngleFactorA;
	if (abs(legAngleA) >= legAngleFactor) {
		if(legAngleA>legAngleFactor)
		{
			legAngleA = legAngleFactor;
		}
		else
		{
			legAngleA = -legAngleFactor;
		}
		
		legAngleFactorA*=-1;
	}

	legTraverseA += movementDelta*legTraverseFactorA;
	if (abs(legTraverseA) >= legTraverseFactor) {
		if(legTraverseA>legTraverseFactor)
		{
			legTraverseA = legTraverseFactor;
		}
		else
		{
			legTraverseA = -legTraverseFactor;
		}
		legTraverseFactorA*=-1;
	}

	legAngleB += movementDelta*legAngleFactorB;
	if (abs(legAngleB) >= legAngleFactor) {
		if(legAngleB>legAngleFactor)
		{
			legAngleB = legAngleFactor;
		}
		else
		{
			legAngleB = -legAngleFactor;
		}
		legAngleFactorB*=-1;
	}

	legTraverseB += movementDelta*legTraverseFactorB;
	if (abs(legTraverseB) >= legTraverseFactor) {
		if(legTraverseB>legTraverseFactor)
		{
			legTraverseB = legTraverseFactor;
		}
		else
		{
			legTraverseB = -legTraverseFactor;
		}
		legTraverseFactorB*=-1;
	}
}

void update( void ) {
	
    double t = static_cast<double>(g_frameIndex) / framesPerKeyframe; // Assuming 60 frames per second

    // Ensure t is in the range [0, 1]
    t = std::min(1.0, std::max(0.0, t));
	

	for(int i = 0; i<gameObject.size();i++)
	{
		gameObject[i].CheckForNeighbors(gameObject,i);
	}

	/*for(int i = 0; i<gameObject.size();i++)
	{
		for (int j = i+1; j<gameObject.size();j++)
		{
			if(i!=j)
			{
				//gameObject[i].CheckForCollisions(&gameObject[j]);
			}
			
		}
	}*/

	float dt = static_cast<float>(1)/60;
	int count = 0;
	for(int i = 0;i<gameObject.size();i++)
	{
		//gameObject[i].CheckForCollisionsWithBounds(negBounds,posBounds);
		Vector3 pos(0,0,0);
		if(!targets.empty())
		{
			pos = targets[currentTargetIndex];
			if((*gameObject[i].transform->position-&pos)->Magnitude()<=5)
			{
				count++;
				if(count>=std::min(std::max(static_cast<int>(gameObject.size() / 4),1),4))
				{
					if(!targets.empty())
					{
						currentTargetIndex = (currentTargetIndex + 1)%targets.size();
						count=0;
					}
				}
			}
			if(!targets.empty())
			{
				pos=targets[currentTargetIndex];
			}
			
		}
		gameObject[i].HandleFlocking(gameObject,dt,pos,targets,currentTargetIndex);
		gameObject[i].ResolveForces(dt);
		std::cout<<currentTargetIndex<<std::endl;
	}
	
	
	
    //std::cout<<gameObject->rigidbody->acceleration->y<<std::endl;

    // Determine the current and next keyframes
    /*int currentKeyframeIndex = i;
    int nextKeyframeIndex = (i + 1) % inputSize; // Wrap around to the first keyframe if needed

	if(g_splineType==CatMullRom)
	{
		CatMullRomHandling(t, currentKeyframeIndex, nextKeyframeIndex);
	}
	else
	{
		BezierHandling(t, currentKeyframeIndex, nextKeyframeIndex);
	}*/
	
	//LegsHandling(currentKeyframeIndex);

    // Increment the frame index and wrap around if needed
	// We are treating the first and last frame as a phantom frame so we dont actually use that for the trajectory
    g_frameIndex++;
    if (g_frameIndex >= framesPerKeyframe) {
        g_frameIndex = 0;
    	i = i+1;
    	if(i==inputSize-2)
    	{
    		i=1;
    	}
    }
}

void RenderArticulatedFigure()
{
	//Render Torso
	glPushMatrix();
	glTranslatef (g_positionX, g_positionY, g_positionZ);
	glMultMatrixf(g_rotation);
	glScalef(4,4,4);
	drawCube(new Vector3(0.5,0.5,0.5));

	//Setting up parent child hierarchy
	glPushMatrix();

	//Thigh1
	glTranslatef (0, -0.75+std::max(legTraverseA,0.0), -0.4);
	glRotatef(legAngleA,0,0,1);
	glScalef(0.4,1,0.5);
	drawCube(new Vector3(0.5,0.5,0.5));

	//Calf1
	glTranslatef (0, -0.9, 0);
	glRotatef(std::min(0.0,-legAngleA),0,0,1);
	glScalef(1,1,1);
	drawCube(new Vector3(0.5,0.5,0.5));
	glPopMatrix();

	//Thigh2
	glPushMatrix();
	glTranslatef (0, -0.75+std::max(legTraverseB,0.0), 0.4);
	glRotatef(legAngleB,0,0,1);
	glScalef(0.4,1,0.5);
	drawCube(new Vector3(0.5,0.5,0.5));

	//Calf2
	glTranslatef (0, -0.9, 0);
	glRotatef(std::min(0.0,-legAngleB),0,0,1);
	glScalef(1,1,1);
	drawCube(new Vector3(0.5,0.5,0.5));
	glPopMatrix();

	glPopMatrix();
}

//================================
// render
//================================
void render( void ) {

	// clear buffer
	glClearColor (0.0, 0.0, 0.0, 0.0);
	glClearDepth (1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	
	// render state
	glEnable(GL_DEPTH_TEST);

	glShadeModel(GL_SMOOTH);

	// enable lighting
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	// light source attributes
	GLfloat LightAmbient[]	= { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightDiffuse[]	= { 0.3f, 0.3f, 0.3f, 1.0f };
	GLfloat LightSpecular[]	= { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f };

	glLightfv(GL_LIGHT0, GL_AMBIENT , LightAmbient );
	glLightfv(GL_LIGHT0, GL_DIFFUSE , LightDiffuse );
	glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);

	// surface material attributes
	GLfloat material_Ka[]	= { 0.11f, 0.06f, 0.11f, 1.0f };
	GLfloat material_Kd[]	= { 0.43f, 0.47f, 0.54f, 1.0f };
	GLfloat material_Ks[]	= { 0.33f, 0.33f, 0.52f, 1.0f };
	GLfloat material_Ke[]	= { 0.1f , 0.0f , 0.1f , 1.0f };
	GLfloat material_Se		= 10;

	glMaterialfv(GL_FRONT, GL_AMBIENT	, material_Ka);
	glMaterialfv(GL_FRONT, GL_DIFFUSE	, material_Kd);
	glMaterialfv(GL_FRONT, GL_SPECULAR	, material_Ks);
	glMaterialfv(GL_FRONT, GL_EMISSION	, material_Ke);
	glMaterialf (GL_FRONT, GL_SHININESS	, material_Se);


	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();

	//Applying camera transforms
	glTranslatef(0,-5,-75);
	glRotated(15.0,1,0,0);

	//Wireframe for out bounds
	//glutWireCube(30);

	if(!targets.empty())
	{
		glPushMatrix();
		glTranslatef(targets[currentTargetIndex].x, targets[currentTargetIndex].y, targets[currentTargetIndex].z);
		glutSolidTeapot(2);
		glPopMatrix();
	}

	
	
	//collisionObject->Render();
	//drawCube(Vector3::one());
	//RenderArticulatedFigure();
	
	// disable lighting
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);
	//Apply Texture and render gameobjects
	GLuint texID;
	LoadTexture(texID,skyboxTextures[0]);
	glEnable(GL_TEXTURE_2D);
	for(int i = 0; i<gameObject.size(); i++)
	{
		gameObject[i].Render();
	}
	glDisable(GL_TEXTURE_2D);

	//Render Ground and skybox
	//applyGroundTexture();
	//drawSkybox();
	// swap back and front buffers
	glutSwapBuffers();
}
//================================
// keyboard input
//================================

// Press x to launch a large, fast, high mass ball upwards from a random starting position on the ground


void keyboard( unsigned char key, int x, int y ) {
	if(key=='x')
	{
		SpawnRandomBoid();
	}
}

//================================
// reshape : update viewport and projection matrix when the window is resized
//================================
void reshape( int w, int h ) {
	// screen size
	g_screenWidth  = w;
	g_screenHeight = h;	
	
	// viewport
	glViewport( 0, 0, (GLsizei)w, (GLsizei)h );

	// projection matrix
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluPerspective(45.0, (GLfloat)w/(GLfloat)h, 1.0, 2000.0);
}


//================================
// timer : triggered every 16ms ( about 60 frames per second )
//================================
void timer( int value ) {	
	// increase frame index
	g_frameIndex++;

	update();
	
	// render
	glutPostRedisplay();

	// reset timer
	// 16 ms per frame ( about 60 frames per second )
	glutTimerFunc( 16, timer, 0 );
}

void loadTextures()
{
	skyboxTextures[0].texture = SOIL_load_image(
	R"(Skybox\test1.png)",  // Replace with the path to your image file
	&skyboxTextures[0].width, &skyboxTextures[0].height, &skyboxTextures[0].channels,
	SOIL_LOAD_RGBA    // Specify the desired image format (e.g., SOIL_LOAD_RGBA)
);

	if (!skyboxTextures[0].texture) {
		printf("SOIL loading error: %s\n", SOIL_last_result());
		// Handle the error (e.g., exit or display a message)
	}


	ground.texture = SOIL_load_image(
		R"(Skybox\dirt.png)",  // Replace with the path to your image file
		&ground.width, &ground.height, &ground.channels,
		SOIL_LOAD_RGBA    // Specify the desired image format (e.g., SOIL_LOAD_RGBA)
	);

	if (!ground.texture) {
		printf("SOIL loading error: %s\n", SOIL_last_result());
		// Handle the error (e.g., exit or display a message)
	}
}


//================================
// main
//================================
int main( int argc, char** argv ) {
	//Import fbx data using OpenFBX (We're using a simple cube fbx for the torso and legs and scaling it accordingly).
	initOpenFBX("Skybox/Cube.fbx");
	//std::cout << position->Magnitude();

	//Load all textures using SOIL
	loadTextures();
	/*targets.push_back(Vector3(-14,-14,-14));
	targets.push_back(Vector3(-14,-14,14));
	targets.push_back(Vector3(14,-14,14));
	targets.push_back(Vector3(14,-14,-14));
	
	targets.push_back(Vector3(-14,14,-14));
	targets.push_back(Vector3(-14,14,14));
	targets.push_back(Vector3(14,14,14));
	targets.push_back(Vector3(14,14,-14));*/
	getUserInputForFlocking("flocking.txt");


	for(int i = 0;i<10;i++)
	{
		SpawnRandomBoid();
	}

	
	//Get Input from file
	//getUserInputFromFile("keyframes.txt");

	//Generate 
	/*getUserInputForPhysicsSimulation("PhysicsSim.txt");
	numberOfGameObjects = pInput.numberOfObjects;
	//gameObject = new GameObject*[pInput.numberOfObjects];
	for(int i = 0; i<pInput.numberOfObjects;i++)
	{
		gameObject.push_back(GameObject());
		//gameObject[i] = new GameObject;
		gameObject[i].rigidbody->mass = pInput.mass;
		gameObject[i].rigidbody->drag = pInput.drag;
		gameObject[i].rigidbody->angularDrag = pInput.angularDrag;
		gameObject[i].rigidbody->bounceReduction = pInput.bounceReduction;
		//Spawn objects at random positions within bounds
		gameObject[i].transform->position = new Vector3(std::rand() % 31 - 15,std::rand() % 31 - 15,std::rand() % 31 - 15);

		//Add a random velocity and angular velocity impulse
		gameObject[i].rigidbody->AddImpulseForce(new Vector3(generateRandomNumber(pInput.minVelocity,pInput.maxVelocity),generateRandomNumber(pInput.minVelocity,pInput.maxVelocity),generateRandomNumber(pInput.minVelocity,pInput.maxVelocity)));
		gameObject[i].rigidbody->AddAngularImpulse(new Vector3(generateRandomNumber(pInput.minAngular,pInput.maxAngular),generateRandomNumber(pInput.minAngular,pInput.maxAngular),generateRandomNumber(pInput.minAngular,pInput.maxAngular)));
	}*/
	//gameObject[0]->transform->position = new Vector3(-10,0,0);
	//gameObject[0]->rigidbody->AddImpulseForce(new Vector3(5,0,0));
	//gameObject[0]->rigidbody->AddAngularImpulse(new Vector3(0,1,0));
	//gameObject[0]->rigidbody->mass = 5;
	
	//gameObject[1]->transform->position = new Vector3(10,0,0);
	//gameObject[1]->rigidbody->AddImpulseForce(new Vector3(-30,0,0));
	
	//collisionObject = new GameObject;
	//collisionObject->rigidbody->useGravity = false;
	//collisionObject->transform->position = new Vector3(0,-5,0);

	/*SpawnBoid(new Vector3(-15,-15,0),new Vector3(2,0,0));
	SpawnBoid(new Vector3(15,-15,0),new Vector3(-2,0,0));

	SpawnBoid(new Vector3(0,-15,15),new Vector3(0,0,-2));
	SpawnBoid(new Vector3(0,-15, -15),new Vector3(0,0,2));*/
	
	glutInit( &argc, argv );
	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH );
	glutInitWindowSize( 600, 600 ); 
	glutInitWindowPosition( 100, 100 );
	glutCreateWindow( argv[0] );

	if(glewInit()!=GLEW_OK)
	{
		printf("NotOK");
	}

	//g_positionX = g_userInput[1].x;
	//g_positionY = g_userInput[1].y;
	//g_positionZ = g_userInput[1].z;

	
	// init
	init();
	
	// set callback functions
	glutDisplayFunc( render );
	glutReshapeFunc( reshape );
	glutKeyboardFunc( keyboard );
	glutTimerFunc( 16, timer, 0 );
	
	// main loop
	glutMainLoop();
	
	return 0;
}