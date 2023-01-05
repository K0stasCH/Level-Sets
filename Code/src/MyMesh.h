#pragma once

#include <VVRScene/mesh.h>
#include <unordered_set>
#include <iostream>
#include <utility>
#include <chrono>
#include "KDTree.h"


struct neighbors_Vertices_Triangles {
	std::unordered_set<int> neighbors_vertices = {};
	std::unordered_set<int> neighbors_triangles = {};
};

class MyMesh : public vvr::Mesh {
public:
	// Constructors
	
	MyMesh() : vvr::Mesh::Mesh() {}
	MyMesh(const std::string& objFile, const std::string& texFile = std::string(), bool ccw = true)
		: vvr::Mesh::Mesh(objFile, texFile, ccw) { 
		computeAllNeighbors(); 
	}
	MyMesh(const vvr::Mesh& original) : vvr::Mesh::Mesh(original) {}

	// Public Methods
	neighbors_Vertices_Triangles findNeighbors(const int vertex_index);
	math::VecArray intdex2VecArray(std::vector<int> indexes);
	void showNeighbors(std::vector<int> neighbors_index, int vertex_index);
	std::vector<math::float3> getDiffCordinates();
	void performTaubinShrinking(double lamda);
	void performTaubinInflation(double lamda, double mi);
	KDTree* getKDTree();
	VecArray getNormalVectors();
	int isInside(math::vec point);
	int isInside(math::vec point, const KDNode* nn);
	void drawnormals();
	void drawDiffCords();
	void mDraw(bool showSolid, bool showWire);
	bool collisionDetectionAABB(const MyMesh* collidedMesh, math::vec* start, math::vec* finish);
	bool collisionDetection(MyMesh* collidedMesh, bool showTime);

private:
	// Private methods
	void computeAllNeighbors();
	void computeDiffCoordinates();
	void computeKDTree();
	void computeNormalVectors();

	// Private members
	std::vector<math::float3> diffCoordinates;
	bool computedNeighbors = false;
	std::vector<std::vector<int>> neighbors_index;
	std::vector<std::vector<int>> triangle_index;

	bool meshChanged_KDTree = true;
	KDTree* mTree = nullptr;

	bool meshChanged_NormalVecs = true;
	VecArray mNormalVectors;
};