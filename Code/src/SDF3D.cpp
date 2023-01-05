#include "SDF3D.h"

using namespace std;


MySDF3D::MySDF3D(MyMesh* mesh, int sizeX) {

	this->mMesh = mesh;

	math::AABB mAABB = mesh->getAABB();

	float sizeRatioXZ = (mAABB.MaxX() - mAABB.MinX()) / (mAABB.MaxZ() - mAABB.MinZ());  // ratio between lengh and depth of the mesh
	int layers = (sizeX / sizeRatioXZ);
	this->mStepZ = (mAABB.MaxZ() - mAABB.MinZ()) / layers;
	float sizeRatioXY = (mAABB.MaxX() - mAABB.MinX()) / (mAABB.MaxY() - mAABB.MinY());  // ratio between lengh and height of the mesh
	int cols = (sizeX / sizeRatioXY);
	this->mStepY = (mAABB.MaxY() - mAABB.MinY()) / cols;
	
	this->mStepX = (mAABB.MaxX() - mAABB.MinX()) / sizeX;

	auto start = chrono::steady_clock::now();

	for (float z = mAABB.MinZ(); z < mAABB.MaxZ(); z += this->mStepZ) {
		SDF2D mSDF2D = {};
		VFF2D mVFF2D = {};
		for (float y = mAABB.MaxY(); y > mAABB.MinY(); y -= this->mStepY) {
			vector<float> tempLineSDF = {};
			vector<vec> tempLineVFF = {};
			for (float x = mAABB.MinX(); x < mAABB.MaxX(); x += this->mStepX) {
				vec tempPoint = vec(x, y, z);
				float dist;
				const KDNode* nn = NULL;
				findNearest(tempPoint, this->mMesh->getKDTree()->root(), &nn, &dist);
				int sign = mMesh->isInside(tempPoint, nn);
				if (sign < 0) {
					vec dir = nn->split_point - tempPoint;		//dir show inside the mesh;
					tempLineVFF.push_back(dir.Normalized());
				}
				else {
					tempLineVFF.push_back( vec(NULL, NULL, NULL) );
				}

				tempLineSDF.push_back(sign * dist);
			}
			mSDF2D.push_back(tempLineSDF);
			mVFF2D.push_back(tempLineVFF);
		}
		this->mSDF3D.push_back(mSDF2D);
		this->mVFF3D.push_back(mVFF2D);
	}

	auto end = chrono::steady_clock::now();
	cout << "Elapsed time for computing whole SDF in seconds: "
		<< chrono::duration_cast<chrono::seconds>(end - start).count()
		<< " secs" << endl;

	this->mSizeZ = this->mSDF3D.size();				//number of layers  -> Z -> depth
	this->mSizeY = this->mSDF3D[0].size();			//number of lines   -> Y -> height
	this->mSizeX = this->mSDF3D[0][0].size();		//number of collums -> X -> lenght
}

//write the 3Dmatrix of SDF to a file *.m
void MySDF3D::writeSDF3D() {
	const string objDir = vvr::getBasePath() + "SDF_plot/";
	ofstream output_file(objDir + "SDF.m");
	ostream_iterator<float> output_iterator(output_file, "\t");
	int indexZ = 1;
	for (float z = 0; z < mSDF3D.size(); z++) {
		SDF2D mySDF2D = mSDF3D[z];
		output_file << "SDF3D(:,:," << indexZ << ")=[";
		for (int i = 0; i < mySDF2D.size(); i++) {
			copy(mySDF2D.at(i).begin(), mySDF2D.at(i).end(), output_iterator);
			output_file << '\n';
		}
		output_file << "];\n";
		indexZ++;
	}
	output_file.close();
	return;
}

void MySDF3D::drawVFF3D(float density) {
	math::AABB mAABB = this->mMesh->getAABB();
	vec init = vec(mAABB.MinX(), mAABB.MaxY(), mAABB.MinZ());
	for (int x = 0; x < this->mSizeX; x += round(1/density) ) {
		for (int y = 0; y < this->mSizeY; y += round(1 / density)) {
			for (int z = 0; z < this->mSizeZ; z += round(1 / density)) {
				if (this->mVFF3D[z][y][x].Equals( vec(NULL, NULL, NULL) ) ) {
					continue;
				}
				vec start = init + vec(x * this->mStepX, -y * this->mStepY, z * this->mStepZ);
				vec end = start + this->mVFF3D[z][y][x];
				vvr::Point3D(start.x, start.y, start.z, vvr::Colour::cyan).draw();
				vvr::Point3D(end.x, end.y, end.z, vvr::Colour::blue).draw();
				vvr::LineSeg3D(start.x, start.y, start.z, end.x, end.y, end.z).draw();
			}
		}
	}
	return;
}

float MySDF3D::calcDistance(math::vec randomPoint, vec* force) {
	float dist = 0;
	*force = vec(0, 0, 0);
	//outside the AABB, so we dont know the accurate distance
	if (this->mMesh->getAABB().MaxX() < randomPoint.x || this->mMesh->getAABB().MinX() > randomPoint.x) return 10E+5;
	if (this->mMesh->getAABB().MaxY() < randomPoint.y || this->mMesh->getAABB().MinY() > randomPoint.y) return 10E+5;
	if (this->mMesh->getAABB().MaxZ() < randomPoint.z || this->mMesh->getAABB().MinZ() > randomPoint.z) return 10E+5;

	vec bias = this->mMesh->getAABB().minPoint;
	//vec bias = vec(mAABB.MinX(), mAABB.MinY(), mAABB.MinZ());
	vec step = vec(this->mStepX, this->mStepY, this->mStepZ);
	vec transPoint = randomPoint - bias;

	int index_x_floor = floor(transPoint.x / this->mStepX);
	int index_x_ceil = ceil(transPoint.x / this->mStepX);

	int index_y_floor = floor(transPoint.y / this->mStepY);
	int index_y_ceil = ceil(transPoint.y / this->mStepY);

	int index_z_floor = floor(transPoint.z / this->mStepZ);
	int index_z_ceil = ceil(transPoint.z / this->mStepZ);


	// L -> left   R -> right
	// D ->down    U -> up   
	// B -> back   F -> front
	VecArray neighbors = {
		vec(index_x_floor, index_y_floor, index_z_floor),	// LDB
		vec(index_x_floor, index_y_floor, index_z_ceil),	// LDF
		vec(index_x_floor, index_y_ceil, index_z_floor),	// LUB
		vec(index_x_floor, index_y_ceil, index_z_ceil),		// LUF
		vec(index_x_ceil, index_y_floor, index_z_floor),	// RDB
		vec(index_x_ceil, index_y_floor, index_z_ceil),		// RDF
		vec(index_x_ceil, index_y_ceil, index_z_floor),		// RUB
		vec(index_x_ceil, index_y_ceil, index_z_ceil),		// RUF
	};

	float sumDist = 0;
	for (int i = 0; i < neighbors.size(); i++) {
		if (neighbors[i].x >= this->mSizeX) continue;
		if (neighbors[i].y >= this->mSizeY) continue;
		if (neighbors[i].z >= this->mSizeZ) continue;

		vec point = neighbors[i].Mul(step) + bias;

		dist += (1 / point.Distance(randomPoint)) * this->mSDF3D[neighbors[i].z][this->mSizeY - neighbors[i].y - 1][neighbors[i].x];
		sumDist += (1 / point.Distance(randomPoint));

		*force += dist * this->mVFF3D[neighbors[i].z][this->mSizeY - neighbors[i].y - 1][neighbors[i].x];
		//vvr::Point3D(point.x, point.y, point.z).draw();
	}
	dist /= sumDist;
	*force /= sumDist;
	
	return dist;
}

bool MySDF3D::collisionDetection(MyMesh* collidedMesh, bool showTime, vec* force) {

	auto time_start = chrono::steady_clock::now();

	vec start, finish;	//limits of overlapping AABB
	bool fastCheck = this->mMesh->collisionDetectionAABB(collidedMesh, &start, &finish);
	if (fastCheck == false)
		return false;

	bool collision = false;

	VecArray vertices = collidedMesh->getVertices();
	vec totalForce = vec(0, 0, 0);
	for (int i = 0; i < vertices.size(); i++) {
		vec tempforce = vec(0, 0, 0);
		float dist = this->calcDistance(vertices[i], &tempforce);
		if (dist < 0) {
			totalForce += tempforce;
			collision = true;
			vvr::Point3D(vertices[i].x, vertices[i].y, vertices[i].z, vvr::Colour::red).draw();
		}
		else {
			vvr::Point3D(vertices[i].x, vertices[i].y, vertices[i].z, vvr::Colour::green).draw();
		}
	}

	*force = totalForce;

	if (showTime == true) {
		auto time_end = chrono::steady_clock::now();
		cout << "Elapsed time for collision detection (SDF - no NN) in milliseconds: "
			<< chrono::duration_cast<chrono::nanoseconds>(time_end - time_start).count()
			<< " nsecs" << endl;
	}

	return collision;
}