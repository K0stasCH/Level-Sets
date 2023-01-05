#pragma once

#include <vector>
#include <chrono>
#include <fstream>
#include "MyMesh.h"

using namespace std;

typedef std::vector<std::vector<float>> SDF2D;		//signed distance field 2D
typedef std::vector<SDF2D> SDF3D;					//signed distance field 3D

typedef std::vector<std::vector<math::vec>> VFF2D;	//vector force field 2D
typedef std::vector<VFF2D> VFF3D;					//vector force field 3D


class MySDF3D {
public:
	// Constructors
	MySDF3D(MyMesh* mesh, int sizeX);
	MySDF3D() { return; };

	// Public Methods
	void writeSDF3D();					//write to a file the values of SDF
	void drawVFF3D(float density);
	float calcDistance(math::vec randomPoint, vec* force);
	bool collisionDetection(MyMesh* collidedMesh, bool showTime, vec* force);

private:
	// Private methods

	// Private members
	int mSizeX, mSizeY, mSizeZ;
	float mStepX, mStepY, mStepZ;
	MyMesh *mMesh;
	SDF3D mSDF3D;	//to access an element mSDF3D[Z][Y][X]
	VFF3D mVFF3D;
};
