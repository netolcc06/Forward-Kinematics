#ifndef _FORWARDKINEMATIC_H_
#define _FORWARDKINEMATIC_H_

#include <stack>
#include <cassert>  
#include <cstdio>  
#include <cstdlib>  
#include <iostream>
#include <vector>

#include <GL\freeglut.h>
#include "Vectors.h"

using namespace std;

class Joint;
class ForwardKinematic
{
public:
	ForwardKinematic() :currentNode(nullptr), p(nullptr), root(nullptr), frameCount(0), frameData(nullptr), pFrame(nullptr)
		, frameTime(1), jointCount(1), currentFrame(0){}
	~ForwardKinematic() {clear();}
	void clear();
	bool load_BVH(const char * pfile); //load the bvh file
	inline unsigned getFrameCount() { return frameCount; }; //return total # of frames
	inline void addFrame() { currentFrame++; if (currentFrame == frameCount) currentFrame = 0; } //load a new frame data to pFrame
	void print() { printRecursive(root, 0); } //print the loaded skeleton structure

	void draw(); //draw a skeleton with one frame data

	/*clear buffers and call calculateJointPosRecursivelyWithQuaternion(Joint* joint)*/
	void calculateJointPos() {

		//clear display buffers
		Points.clear(); LocalCoorSys.clear(); Bones.clear();

		//traverse down the entire skeleton tree and recursively calculate each joint's global position
		calculateJointPosRecursivelyWithQuaternion(root);
	};

private:

	/*------------------------variables and functions you may need to check----------------------------*/
	Joint * root;//root node   
	unsigned currentFrame;//current frame  
	float **frameData; //all the frame data
	float frameTime; //total frame time
	float *pFrame;//current frame data
	std::vector<Vector4> Points; //Points' display buffer
	std::vector<std::vector<Vector4>> LocalCoorSys; //LocalCoorSys's display buffer
	std::vector<std::vector<Vector4>> Bones; //Bones' display buffer


	//calculate every joint's global position with a root joint and quaternions
	void calculateJointPosRecursivelyWithQuaternion(Joint* joint);

	//coding part 1) get local rotation quaternion with frame data and rotation order (euler angle order)
	Vector4 computeLocalQuaternion(Joint* joint);

	//coding part 2) get global rotation quaternion with local rotation quaternion
	Vector4 computeGlobalQuaternion(Joint* quat1, Vector4 quat);

	//coding part 3) get global rotation quaternion with local rotation quaternion
	Vector4 computeGlobalPosition(Joint* joint);

	//calculate quaternion quat1*quat2
	Vector4 quaternionMultiplication(Vector4 quat1, Vector4 quat2)
	{
		Vector3 v1 = Vector3(quat1.x, quat1.y, quat1.z);
		Vector3 v2 = Vector3(quat2.x, quat2.y, quat2.z);
		float x1 = quat1.x; float y1 = quat1.y; float z1 = quat1.z;
		float x2 = quat2.x; float y2 = quat2.y; float z2 = quat2.z;

		float w1 = quat1.w;
		float w2 = quat2.w;

		Vector3 v3 = w1 * v2 + w2 * v1 + v1.cross(v2);
		float w3 = w1 * w2 - v1.dot(v2);
		Vector4 result = Vector4(v3.x, v3.y, v3.z, w3);

		return result;
	}

	/*return a quaternion with a rotaion: angle='angle', axis=(x,y,z)*/
	Vector4 buildQuaternionRotation(float angle, float x, float y, float z)
	{
		float degreetorad = 3.141592658f / 180.0f;
		float c = cosf((angle / 2.0f) * degreetorad);    // cosine
		float s = sinf((angle / 2.0f) * degreetorad);    // sine

		Vector4 rotationQuat = Vector4(s*x, s*y, s*z, c);
		return rotationQuat;
	}


	/*------------------Varialbles and functions for loading bvh file-----------------------*/
	unsigned jointCount;//joint num. including root  
	unsigned frameCount;// frame num. 
	unsigned char* p;//current buffer index 
	stack<Joint*> father; //stack structure for buiding the tree skeleton tree structure
	Joint* currentNode; //current node

	Joint * createJoint(); //create a new joint
	void printRecursive(Joint* r, int n); //print the skeleton
	void calculateLocalCoorSys(Joint* root, Vector4 GlobalPosition);
	bool loadHiarachy(unsigned char * buffer); //load the skeleton structure as a tree
	bool loadFrameData(unsigned char * buffer); //load the frame data
	void deleteRecursive(Joint* r); //delete the tree structure
	unsigned char getRotationOrder(); //get current node's rotation order
};

enum { CHILDSIZE = 16, NAMESIZE = 24 };
enum { NONE = 0, ZYX = 1, YZX = 2, ZXY = 3, XZY = 5, YXZ = 6, XYZ = 7 };


class Joint
{
public:
	friend class ForwardKinematic;
	Joint() :childNum(0), rotationOrder(0)
	{
		name[0] = 0;
		parent = nullptr;
		LocalPos = Vector4(0.0f, 0.0f, 0.0f, 0.0f);
		Globalquat = Vector4(0.0f, 0.0f, 0.0f, 1.0f);
		Xaxis = Vector4(1.0f, 0.0f, 0.0f, 0.0f);
		Yaxis = Vector4(0.0f, 1.0f, 0.0f, 0.0f);
		Zaxis = Vector4(0.0f, 0.0f, 1.0f, 0.0f);
	}
	bool addChild(Joint* pc)
	{
		assert(childNum != NAMESIZE);
		child[childNum] = pc;
		childNum++;
		return true;
	}
	Joint* getChild(int i)
	{
		assert(i >= 0 && i<CHILDSIZE);
		return child[i];
	}
private:

	Vector4 LocalPos; //local position of the joint
	Vector4 Xaxis, Yaxis, Zaxis; //local coordinate system
	Vector4 Globalquat; //global rotation quaternion accumulated from parent
	Vector4 GlobalPos; //global position of the joint
	Joint* parent; //parent node
	Joint * child[CHILDSIZE]; //pointers to child nodes
	int childNum;
	unsigned char name[NAMESIZE]; //joint name
	unsigned char rotationOrder; //rotation order in euler angle
};
#endif // !_FK_H_
