#pragma once

#include <VVRScene/canvas.h>
#include <VVRScene/settings.h>
#include <VVRScene/utils.h>
#include <MathGeoLib.h>

#include <Windows.h>

#include "MyMesh.h"
#include "SDF3D.h"

#define FLAG_SHOW_AXES       1
#define FLAG_SHOW_WIRE       2
#define FLAG_SHOW_SOLID      4
#define FLAG_SHOW_NORMALS    8
#define FLAG_SHOW_PLANE     16
#define FLAG_SHOW_AABB      32


class Scene3D : public vvr::Scene
{
public:
	Scene3D();
	const char* getName() const { return "Level Sets 1066615"; }
	void keyEvent(unsigned char key, bool up, int modif) override;
	void arrowEvent(vvr::ArrowDir dir, int modif) override;
	void sliderChanged(int slider_id, float val);

private:
	void draw() override;
	void reset() override;
	void resize() override;
	void printKeyboardShortcuts();

private:
	int m_style_flag;
	int flag_show_diffCords = 0, flag_show_VFF = 0;
	int selectedMesh = 0;

	// coliosionMode = 0 -> no check
	// coliosionMode = 1 -> with Mesh
	// coliosionMode = 2 -> with SDF
	// coliosionMode = 3 -> with SDF of Neural Network
	int collisionMode = 0;
	bool isCollided = false;

	vvr::Canvas2D m_canvas;
	vvr::Colour m_obj_col;
	MyMesh m_model_original, m_model;
	MySDF3D m_model_SDF;

	float step = 1;
	float lamda = 0, mi = 0, dens = 1, speed = 1, k = 1;
	int counter = 0;

	vvr::Sphere3D mSphere;
	MyMesh mPolyhedron, mCube, mSimple_model, recovered;

	MyMesh* activeObject = &mPolyhedron;

	bool showTimeofCollision = false;
	bool showForce = true;

	bool showWireActiveObj = true, showSolidActiveObj = true;

	// memmbers for SDF of Neural Network
	math::VecArray trajectory = {};
	bool monitorTrajectory = false;
	int playAnimation = -1;
};
