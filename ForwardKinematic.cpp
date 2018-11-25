#pragma warning(disable : 4996)

#include "ForwardKinematic.h"



/*recursively calculate each joint's global position from the root (in-order traversal)*/
void ForwardKinematic::calculateJointPosRecursivelyWithQuaternion(Joint* joint)
{
	//check if joint is root. If yes, set the first three elements as root joint's global translation
	if (joint->parent == nullptr)
	{
		pFrame = frameData[currentFrame];
		joint->LocalPos.x = pFrame[0];
		joint->LocalPos.y = pFrame[1];
		joint->LocalPos.z = pFrame[2];
		pFrame += 3; // pFrame points to joints' Euler angles by adding by 3.
	}

	/*------------------------------coding part start------------------------------------------*/
	/*Please modify the code accordingly inside these sub-functions                            */
	/*Coding Part: 1) calculate local rotation in quaternion from euler angles for current node*/
	Vector4 localQuat = computeLocalQuaternion(joint);

	/*Coding Part: 2) calculate global rotation quaternion for child nodes*/
	joint->Globalquat = computeGlobalQuaternion(joint, localQuat);

	/*Coding Part: 3) calculate current node's global position*/
	Vector4 GlobalPosition = computeGlobalPosition(joint);
	/*------------------------------coding part end--------------------------------------------*/

	//store the result local coordinate system into the display buffer
	calculateLocalCoorSys(joint, GlobalPosition);

	//store the result globalPosition into the display buffer
	Points.push_back(GlobalPosition);

	//calculate and store the bones into the display buffer
	if (joint->parent != nullptr)
	{
		std::vector<Vector4> bone;
		Vector4 start = joint->parent->GlobalPos;
		Vector4 end = GlobalPosition;
		bone.push_back(start);
		bone.push_back(end);
		Bones.push_back(bone);
	}

	//recursively call self for position computation
	for (int i = 0; i < joint->childNum; i++)
	{
		calculateJointPosRecursivelyWithQuaternion(joint->getChild(i));
	}
}

/*compute local rotation quaternion with specific rotation angles (angle1, angle2, angle3) and a euler rotation order*/
Vector4 ForwardKinematic::computeLocalQuaternion(Joint* joint)
{
	/*add/edit your code here for part 1*/
	float angles[3] = {0.0f,0.0f,0.0f};

	Vector3 Xaxis(1.0f, 0.0f, 0.0f), Yaxis(0.0f, 1.0f, 0.0f),
		Zaxis(0.0f, 0.0f, 1.0f), axis[3];
	Vector4	out(0.0f, 0.0f, 0.0f, 1.0f);

	switch (joint->rotationOrder)
	{
	case 1:
		//printf("zyx");
		
		axis[0] = Zaxis;
		axis[1] = Yaxis;
		axis[2] = Xaxis;

		break;
	case 2:
		//printf("yzx");

		axis[0] = Yaxis;
		axis[1] = Zaxis;
		axis[2] = Xaxis;

		break;
	case 3:
		//printf("zxy");

		axis[0] = Zaxis;
		axis[1] = Xaxis;
		axis[2] = Yaxis;

		break;
	case 5:
		//printf("xzy");

		axis[0] = Xaxis;
		axis[1] = Zaxis;
		axis[2] = Yaxis;

		break;
	case 6:
		//printf("yxz");

		axis[0] = Yaxis;
		axis[1] = Xaxis;
		axis[2] = Zaxis;

		break;
	case 7:
		//printf("xyz");

		axis[0] = Xaxis;
		axis[1] = Yaxis;
		axis[2] = Zaxis;
		
		break;
	default:
		return out;
		break;
	}

	angles[0] = pFrame[0];
	angles[1] = pFrame[1];
	angles[2] = pFrame[2];

	if(joint->rotationOrder != NONE)
		pFrame += 3;

	Vector4 rot2, rot1, rot0;
	rot2 = buildQuaternionRotation(angles[2], axis[2][0], axis[2][1], axis[2][2]);
	rot1 = buildQuaternionRotation(angles[1], axis[1][0], axis[1][1], axis[1][2]);
	rot0 = buildQuaternionRotation(angles[0], axis[0][0], axis[0][1], axis[0][2]);

	out = quaternionMultiplication(rot0,quaternionMultiplication(rot1, rot2));
	
	if (out[3] < 0)
		out *= -1.0f;
		
	return out;
}

/*compute global rotation quaternion accumulated from root joint for a joint*/
Vector4 ForwardKinematic::computeGlobalQuaternion(Joint* joint, Vector4 localQuat)
{
	if (joint->parent == NULL) {
		joint->Globalquat = localQuat;		
	}
	else {
		joint->Globalquat = quaternionMultiplication(joint->parent->Globalquat, localQuat);	
	}
	
	return joint->Globalquat;
}

//based on global quaternion and local position, compute global position for a joint
Vector4 ForwardKinematic::computeGlobalPosition(Joint* joint)
{
	/*add/edit your code here for part 3*/
	Vector4 q, rot, rot_in, r;
	
	if (joint->parent == NULL) {
		joint->GlobalPos = joint->LocalPos;
		joint->GlobalPos[3] = 0;
		return joint->GlobalPos;
	}
	/** The global position is calculated by parent’s global position + rotated 
	 ** current joint’s local position. **/
	rot = joint->parent->Globalquat;
	q = joint->LocalPos;

	rot_in = rot; 
	rot_in[0] *= -1.0f; rot_in[1] *= -1.0f; rot_in[2] *= -1.0f;
	
	//local position rotation = q * localp * q^(-1)
	r = quaternionMultiplication(rot,quaternionMultiplication(q, rot_in));
	
	joint->GlobalPos = joint->parent->GlobalPos + r;

	joint->GlobalPos[3] = 0;

	return joint->GlobalPos;
}

/*draw a skeleton with one frame data*/
void ForwardKinematic::draw()
{
	//draw joints
	glPointSize(8);
	glColor3f(1, 0, 1);
	glBegin(GL_POINTS);
	for (size_t i = 0; i < Points.size(); i++)
	{
		glVertex3f(Points[i].x, Points[i].y, Points[i].z);
	}
	glEnd();

	//draw local coordinate systems
	glLineWidth(1.0f);
	glBegin(GL_LINES);
	for (size_t i = 0; i < LocalCoorSys.size(); i++)
	{
		//display local x,y,z axis
		glColor3f(1.0f, 0.0f, 0.0f);
		glVertex3f(Points[i].x, Points[i].y, Points[i].z);
		glVertex3f(LocalCoorSys[i][0].x, LocalCoorSys[i][0].y, LocalCoorSys[i][0].z);

		glColor3f(0.0f, 1.0f, 0.0f);
		glVertex3f(Points[i].x, Points[i].y, Points[i].z);
		glVertex3f(LocalCoorSys[i][1].x, LocalCoorSys[i][1].y, LocalCoorSys[i][1].z);

		glColor3f(0.0f, 0.0f, 0.1f);
		glVertex3f(Points[i].x, Points[i].y, Points[i].z);
		glVertex3f(LocalCoorSys[i][2].x, LocalCoorSys[i][2].y, LocalCoorSys[i][2].z);
	}
	glEnd();

	//draw bones
	glColor3f(0.0f, 0.0f, 0.0f);
	glLineWidth(1.5f);
	glBegin(GL_LINES);
	for (size_t i = 0; i < Bones.size(); i++)
	{
		glVertex3f(Bones[i][0].x, Bones[i][0].y, Bones[i][0].z);
		glVertex3f(Bones[i][1].x, Bones[i][1].y, Bones[i][1].z);
	}
	glEnd();
}

/*create a node*/
Joint* ForwardKinematic::createJoint()
{
	Joint * node = new Joint();
	int i;
	for (i = 0; i < NAMESIZE && *p != '\n'; i++)
	{
		node->name[i] = *p;
		p++;
	}
	if (i == NAMESIZE) --i;
	node->name[i - 1] = 0;		//take care of '/r'£¡£¡  
	p++;

	return node;
}

/*identify the rotation order: ZYX = 1, YZX = 2,ZXY = 3,XZY = 5,YXZ = 6,XYZ = 7*/
unsigned char ForwardKinematic::getRotationOrder()
{
	return (unsigned char)((*p - 'X' + 1) * 1 + (*(p + 10) - 'X' + 1) * 2 + (*(p + 20) - 'X' + 1) * 4 - 10);
}

/*load bvh file: loadHiarachy()+loadFrameData()*/
bool ForwardKinematic::load_BVH(const char* pfile)
{
	if (pfile == 0)
		return false;

	FILE *f;

	if (!(f = fopen(pfile, "rb")))
	{
		printf("file load failed!\n");
		return false;
	}

	//get the length of the file  
	int iStart = ftell(f);
	fseek(f, 0, SEEK_END);
	int iEnd = ftell(f);
	rewind(f);
	int iFileSize = iEnd - iStart;

	//create the buffer for the bvh file 
	unsigned char *buffer = new unsigned char[iFileSize];

	if (!buffer)
	{
		printf("mem alloc failed!!\n");
		return false;
	}

	//load the file to the buffer
	if (fread(buffer, 1, iFileSize, f) != (unsigned)iFileSize)
	{
		printf("failed!!\n");
		delete[]buffer;
		return false;
	}

	//load the skeleton structure as a tree structure
	loadHiarachy(buffer);

	//load frame data
	loadFrameData(buffer);

	delete[]buffer;
	fclose(f);
	return true;
}

/*load the skeleton structure from the bvh file*/
bool ForwardKinematic::loadHiarachy(unsigned char * buffer)
{
	//check if the file is bvh 
	p = buffer;
	const char * fileheader = "HIERARCHY";
	for (int i = 0; i < 9; i++)
	{
		if (*p != fileheader[i])
		{
			delete[]buffer;
			return false;
		}
		p++;
	}


	//load the root name
	p += 7;
	root = createJoint();
	//push the root to the top of the stack
	father.push(root);
	currentNode = root;

	//Load root offset, it is different from other joints
	while (*p != 'O') p++;
	p += 7;
	root->LocalPos.x = (float)atof((char*)p);

	p += 5;
	if (*p == ' ') p++;
	root->LocalPos.y = (float)atof((char*)p);

	p += 5;
	if (*p == ' ') p++;
	root->LocalPos.z = (float)atof((char*)p);
	p += 5;

	//go to rotation information as root is usually XYZ order  
	while (*p != 'r') p++;
	p--;
	//get root rotation information 
	root->rotationOrder = getRotationOrder();

	p += 30;
	//end root information loading

	//use stack to build the tree structure skeleton    
	int temp = 0;
	int counter = 1; // count the number of brackets
	for (bool running = true; running; )
	{
		if (*p == 0)
		{
			delete[]buffer;
			clear();
			return 0;
		}

		//deal with buffer accordingly
		switch (*p)
		{
		case 13://Enter
		case '\n': {p++;  break; }//line break  
		case ' ': {p++;  break; }//space
		case '\t': {p++;  break; }//tab
		case '{': {
			father.push(currentNode);
			p++;
			counter++;
			break;
		}
		case '}': {
			father.pop();

			if (!father.empty())
				currentNode = father.top();
			p++;
			counter--;
			//check if the loading of hierarchy is done
			if (counter == 0) running = false;
			break;
		}
		case 'J': {
			jointCount++;
			p += 6;
			Joint *JointNode = createJoint();
			JointNode->parent = currentNode;
			currentNode->addChild(JointNode);
			currentNode = JointNode;
			break;
		}
		case 'O': {
			//Get Local Position's x value
			p += 7;
			currentNode->LocalPos.x = atof((const char *)p);

			//Get Local Position's y value
			p += 7;
			while (*p != ' ' && *p != '  ') p++;
			p++;
			currentNode->LocalPos.y = atof((const char *)p);

			//Get Local Position's z value
			p += 7;
			while (*p != ' ' && *p != '  ') p++;
			p++;
			currentNode->LocalPos.z = atof((const char *)p);

			while (*p++ != '\n');
			break;
		}
		case 'C': {
			p += 11;
			currentNode->rotationOrder = getRotationOrder();
			p += 30;
			break;
		}
		case 'E': {
			p += 4;
			Joint *Endnode = createJoint();
			Endnode->parent = currentNode;
			currentNode->addChild(Endnode);
			currentNode = Endnode;
			break;
		}
		default:
			printf("_%c_ _%d_ file format error!! \n", *p, *p);
			delete[]buffer;
			clear();
			return false;
		}
		temp++;
	}
	//end hierarchy's loading
}

/*load the frame data from the bvh file*/
bool ForwardKinematic::loadFrameData(unsigned char * buffer)
{
	while (*p != 'F') p++;
	p += 8;

	//number of frames
	frameCount = (unsigned)atoi((char *)p);

	while (*p != 'F') p++;
	p += 12;

	//total time length 
	frameTime = (float)atof((char*)p);


	while (*p++ != '\n');

	// allocate space for the framedata  
	frameData = new float*[frameCount];
	if (frameData == nullptr)
	{
		delete[]buffer;
		clear();
		return false;
	}
	//check if joint number is correct  
	if (jointCount == 1)
	{
		delete[]buffer;
		clear();
		return false;
	}


	//total data for one frame: root's translation + all joints' rotation information
	int dataCount = jointCount * 3 + 3;

	//allocate memory for each frame
	for (unsigned int i = 0; i < frameCount; i++)
	{
		frameData[i] = new float[dataCount];
		if (frameData == nullptr)
		{
			delete[]buffer;
			clear();
			return false;
		}
	}

	//load frame data, one frame by one frame	
	for (unsigned int i = 0; i < frameCount; i++)
	{
		//ignore space
		p += 1;
		for (int j = 0; j < dataCount; j++)
		{

			frameData[i][j] = (float)atof((char*)p);
			if (j == 77)
			{
				while (*p != '\r' && *p != '\n') p++;
				continue;
			}
			while (*p != ' ' && *p != '  ') p++;
			p++; //ignore tab
		}
		//ignore line breaks and enters 
		p += 2;
	}
}

/*calculate the local coordinate system and store them into display buffer for visualization*/
void ForwardKinematic::calculateLocalCoorSys(Joint* joint, Vector4 GlobalPosition)
{
	Vector4 quatInverse = Vector4(-joint->Globalquat.x, -joint->Globalquat.y, -joint->Globalquat.z, joint->Globalquat.w);

	Vector4 GlobalXaxis = quaternionMultiplication(Vector4(1.0f, 0.0f, 0.0f, 0.0f), quatInverse);
	Vector4 GlobalYaxis = quaternionMultiplication(Vector4(0.0f, 1.0f, 0.0f, 0.0f), quatInverse);
	Vector4 GlobalZaxis = quaternionMultiplication(Vector4(0.0f, 0.0f, 1.0f, 0.0f), quatInverse);
	GlobalXaxis = quaternionMultiplication(joint->Globalquat, GlobalXaxis);
	GlobalYaxis = quaternionMultiplication(joint->Globalquat, GlobalYaxis);
	GlobalZaxis = quaternionMultiplication(joint->Globalquat, GlobalZaxis);

	Vector4 xaxis = GlobalXaxis * 4.0f + GlobalPosition;
	Vector4 yaxis = GlobalYaxis * 4.0f + GlobalPosition;
	Vector4 zaxis = GlobalZaxis * 4.0f + GlobalPosition;

	std::vector<Vector4> coorSys;
	//coorSys.clear();
	coorSys.push_back(xaxis);
	coorSys.push_back(yaxis);
	coorSys.push_back(zaxis);
	LocalCoorSys.push_back(coorSys);
}

/*recursively delete joints */
void ForwardKinematic::deleteRecursive(Joint* r)
{
	for (int i = 0; i < r->childNum; i++)
	{
		deleteRecursive(r->getChild(i));
	}
	delete r;
}

/*clear all data*/
void ForwardKinematic::clear()
{
	currentNode = nullptr;
	p = nullptr;
	while (!father.empty()) father.pop();
	frameTime = 1;


	for (unsigned int i = 0; i < frameCount; i++)
		delete[]frameData[i];
	delete[]frameData;
	frameData = nullptr;
	pFrame = nullptr;
	frameCount = 0;
	currentFrame = 0;
	jointCount = 1;

	if (root != nullptr)
	{
		deleteRecursive(root);
	}
	root = nullptr;
}

//print the hierarchy of the skeleton
void ForwardKinematic::printRecursive(Joint* r, int n)
{
	for (int i = 0; i < n; i++) printf(" -");
	printf("%s", r->name);

	printf(" Local Position %f,%f,%f -%d- ", r->LocalPos.x, r->LocalPos.y, r->LocalPos.z, r->rotationOrder);
	switch (r->rotationOrder)
	{
	case 1:
		printf("zyx");
		break;
	case 2:
		printf("yzx");
		break;
	case 3:
		printf("zxy");
		break;
	case 5:
		printf("xzy");
		break;
	case 6:
		printf("yxz");
		break;
	case 7:
		printf("xyz");
		break;
	default:
		break;
	}
	printf("\n");
	for (int i = 0; i < r->childNum; i++)
	{
		printRecursive(r->getChild(i), n + 1);
	}
}

