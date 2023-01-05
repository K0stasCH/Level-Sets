#include "MyMesh.h"

using namespace std;


neighbors_Vertices_Triangles MyMesh::findNeighbors(const int vertex_index) {
	vector<vvr::Triangle> all_triangles = this->getTriangles();
	neighbors_Vertices_Triangles _neighbors;

	for (int i = 0; i < all_triangles.size(); i++) {
		if (all_triangles[i].vi1 == vertex_index) {
			_neighbors.neighbors_vertices.insert(all_triangles[i].vi2);
			_neighbors.neighbors_vertices.insert(all_triangles[i].vi3);
			_neighbors.neighbors_triangles.insert(i);

		}
		if (all_triangles[i].vi2 == vertex_index) {
			_neighbors.neighbors_vertices.insert(all_triangles[i].vi1);
			_neighbors.neighbors_vertices.insert(all_triangles[i].vi3);
			_neighbors.neighbors_triangles.insert(i);
		}
		if (all_triangles[i].vi3 == vertex_index) {
			_neighbors.neighbors_vertices.insert(all_triangles[i].vi1);
			_neighbors.neighbors_vertices.insert(all_triangles[i].vi2);
			_neighbors.neighbors_triangles.insert(i);
		}
	}

	return _neighbors;
}

math::VecArray MyMesh::intdex2VecArray(std::vector<int> indexes) {
	math::VecArray& vertices = getVertices();
	math::VecArray points;
	for (int i = 0; i < indexes.size(); i++) {
			points.push_back( vertices[indexes[i]] );
	}
	return points;
}

// compute the neighbors for all vertexes and store them in the class member "neighbors_index"
// called in the constructor
void MyMesh::computeAllNeighbors() {
	if (computedNeighbors == true) return;
	else computedNeighbors = true;

	for (int i = 0; i < getVertices().size(); i++) {
		neighbors_Vertices_Triangles _neighbors = findNeighbors(i);

		vector<int> neigb_index(_neighbors.neighbors_vertices.begin(), _neighbors.neighbors_vertices.end());
		vector<int> tri_index(_neighbors.neighbors_triangles.begin(), _neighbors.neighbors_triangles.end());

		this->neighbors_index.push_back(neigb_index);
		this->triangle_index.push_back(tri_index);
	}
	return;
}

// work only inside -> Scene3D::draw() <-
void MyMesh::showNeighbors(vector<int> neighbors_index, int vertex_index) {
	math::VecArray vertices = this->getVertices();
	for (int i = 0; i < neighbors_index.size(); i++) {
		vec temp_point = vertices[neighbors_index[i]];
		vec center_point = vertices[vertex_index];
		vvr::Point3D(temp_point.x, temp_point.y, temp_point.z, vvr::Colour::blue).draw();
		vvr::Point3D(center_point.x, center_point.y, center_point.z, vvr::Colour::blue).draw();
	}
	return;
}


void MyMesh::computeDiffCoordinates() {
	math::VecArray vertices = getVertices();
	this->diffCoordinates.clear();
	for (int vertex = 0; vertex < vertices.size(); vertex++) {
		math::VecArray neighbors = intdex2VecArray( this->neighbors_index[vertex] );
		math::float3 diff_cord = float3(0, 0, 0);
		int size = 0;
		for (int i = 0; i < neighbors.size(); i++) {
				diff_cord += (vertices[vertex] - neighbors[i]);
				size += 1;
		}
		diff_cord /= size;
		this->diffCoordinates.push_back(diff_cord);
	}
	return;
}


std::vector<math::float3> MyMesh::getDiffCordinates() {
	computeDiffCoordinates();
	return this->diffCoordinates;
}

void MyMesh::performTaubinShrinking(double lamda) {
	math::VecArray& diffCord = getDiffCordinates();
	math::VecArray& vertices = getVertices();
	for (int vertex = 0; vertex < vertices.size(); vertex++){
		vertices[vertex] += lamda * diffCord[vertex];
	}
	this->meshChanged_KDTree = true;
	this->meshChanged_NormalVecs = true;
	//this->update(true);
	return;
}

void MyMesh::performTaubinInflation(double lamda, double mi) {

	math::VecArray& diffCord = getDiffCordinates();
	math::VecArray& vertices = getVertices();
	for (int vertex = 0; vertex < vertices.size(); vertex++) {
		vertices[vertex] += mi * diffCord[vertex];
	}

	performTaubinShrinking(lamda);
	//this->update(true);
	return;
}

void MyMesh::computeKDTree() {

	VecArray vertices = this->getVertices();
	vector<vvr::Triangle> triangles = this->getTriangles();
	VecArray diffCords = this->getDiffCordinates();

	vector<pair<vec, int>> points;
	for (int index = 0; index < vertices.size(); index++) {
		points.push_back({ vertices[index], index });

	}

	delete this->mTree;
	this->mTree = new KDTree(points);

}

KDTree* MyMesh::getKDTree() {
	if (meshChanged_KDTree == true) {
		computeKDTree();
		meshChanged_KDTree = false;
	}
	return this->mTree;
}

//return inside  -> -1
//return outside ->  1
int MyMesh::isInside(math::vec point) {
	vec dir = (vec(point.x, point.y, point.z + 1) - point);
	math::Ray mRay(point, dir);
	vector<vvr::Triangle> triangles = this->getTriangles();
	int counter = 0;
	for (int i = 0; i < triangles.size(); i++) {
		if ( mRay.Intersects( math::Triangle(triangles[i].v1(), triangles[i].v2(), triangles[i].v3()) ) ) {
			counter++;
		}
	}
	return (counter % 2 == 0) ? 1 : -1;
}

int MyMesh::isInside(math::vec point, const KDNode* nn) {
	vector<int> tri_index = this->triangle_index[nn->split_point_index];
	vector<vvr::Triangle> triangles = this->getTriangles();

	for (int i = 0; i < tri_index.size(); i++) {
		int index = tri_index[i];
		vec normal_triangle = triangles[index].getNormal();
		float dotProduct = (point - nn->split_point).Dot(normal_triangle);
		if (dotProduct < 0) {
			return 1;
		}
	}
	return -1;
}

void MyMesh::computeNormalVectors() {
	VecArray normalVecs = VecArray(this->getVertices().size(), vec(0, 0, 0));
	vector<vvr::Triangle> triangles = this->getTriangles();
	for (int i = 0; i < triangles.size(); i++) {
		vec tempNormal = triangles[i].getNormal();

		int vi1 = triangles[i].vi1;
		int vi2 = triangles[i].vi2;
		int vi3 = triangles[i].vi3;

		normalVecs[vi1] -= tempNormal;
		normalVecs[vi2] -= tempNormal;
		normalVecs[vi3] -= tempNormal;
	}

	for (int i = 0; i < normalVecs.size(); i++) {
		normalVecs[i] = normalVecs[i].Normalized();
	}

	this->mNormalVectors = normalVecs;

	return;
}


VecArray MyMesh::getNormalVectors() {
	if (meshChanged_NormalVecs == true) {
		computeNormalVectors();
		meshChanged_NormalVecs = false;
	}
	return this->mNormalVectors;
}

//use only inside Scene3D::draw()
void MyMesh::drawnormals() {
	
	math::VecArray vertices = this->getVertices();
	math::VecArray normals = this->getNormalVectors();
	for (int i = 0; i < normals.size(); i++) {
		vec start = vertices[i];
		vec end = vertices[i] + normals[i];
		vvr::LineSeg3D(start.x, start.y, start.z, end.x, end.y, end.z, vvr::Colour::red).draw();
	}
	return;
}

void MyMesh::drawDiffCords() {
	VecArray diffCords = this->getDiffCordinates();
	VecArray vertices = this->getVertices();

	vector<double> magnitude;
	for (int i = 0; i < diffCords.size(); i++) {
		magnitude.push_back(diffCords[i].Length());
	}
	double max = *max_element(magnitude.begin(), magnitude.end());
	double min = *min_element(magnitude.begin(), magnitude.end());

	for (int i = 0; i < diffCords.size(); i++) {
		//vec end = vertices[i] + diffCords[i];
		//vvr::LineSeg3D(vertices[i][0], vertices[i][1], vertices[i][2], end[0], end[1], end[2], vvr::Colour(0, 0, 254 * magnitude[i] / (max - min))).draw();
		vvr::Point3D(vertices[i][0], vertices[i][1], vertices[i][2], vvr::Colour(0, 0, 250 * magnitude[i] / (max - min))).draw();
	}
	return;
}

//use only inside Scene3D::draw() and only for the collided objects 
void MyMesh::mDraw(bool showSolid, bool showWire) {
	if (showSolid == true) this->draw(vvr::Colour::white, vvr::Style::SOLID);
	if (showWire == true) this->draw(vvr::Colour::black, vvr::Style::WIRE);
}

bool MyMesh::collisionDetectionAABB(const MyMesh* collidedMesh, math::vec *start, math::vec *finish) {
	math::AABB aabb1 = this->getAABB();
	math::AABB aabb2 = collidedMesh->getAABB();

	*finish = vec(min(aabb1.MaxX(), aabb2.MaxX()), min(aabb1.MaxY(), aabb2.MaxY()), min(aabb1.MaxZ(), aabb2.MaxZ()));
	*start = vec(max(aabb1.MinX(), aabb2.MinX()), max(aabb1.MinY(), aabb2.MinY()), max(aabb1.MinZ(), aabb2.MinZ()));

	if ((finish->x - start->x > 0) && (finish->y - start->y > 0) && (finish->z - start->z > 0)) {
		return true;
	}
	else {
		return false;
	}
}

bool MyMesh::collisionDetection(MyMesh* collidedMesh, bool showTime) {

	auto time_start = chrono::steady_clock::now();

	vec start, finish;
	bool fastCheck = this->collisionDetectionAABB(collidedMesh, &start, &finish);
	if (fastCheck == false)
		return false;

	bool collision = false;
	VecArray vertices1 = collidedMesh->getVertices();
	for (int i = 0; i < vertices1.size(); i++) {
		if (vertices1[i].x > finish.x || vertices1[i].x < start.x) continue;
		if (vertices1[i].y > finish.y || vertices1[i].y < start.y) continue;
		if (vertices1[i].z > finish.z || vertices1[i].z < start.z) continue;
		float dist;
		const KDNode* nn = NULL;
		findNearest(vertices1[i], this->getKDTree()->root(), &nn, &dist);
		int sign = this->isInside(vertices1[i], nn);
		if (sign == -1) {
			collision = true;
			vvr::Point3D(vertices1[i].x, vertices1[i].y, vertices1[i].z, vvr::Colour::green).draw();
		}
	}
	
	VecArray vertices2 = this->getVertices();
	for (int j = 0; j < vertices2.size(); j++) {
		if (vertices2[j].x > finish.x || vertices2[j].x < start.x) continue;
		if (vertices2[j].y > finish.y || vertices2[j].y < start.y) continue;
		if (vertices2[j].z > finish.z || vertices2[j].z < start.z) continue;
		int sign = collidedMesh->isInside(vertices2[j]);
		if (sign == -1) {
			collision = true;
			vvr::Point3D(vertices2[j].x, vertices2[j].y, vertices2[j].z, vvr::Colour::darkGreen).draw();
		}
	}

	if (showTime == true) {
		auto time_end = chrono::steady_clock::now();
		cout << "Elapsed time for collision detection (MESH) in milliseconds: "
			<< chrono::duration_cast<chrono::milliseconds>(time_end - time_start).count()
			<< " ms" << endl;
	}

	return collision;
}