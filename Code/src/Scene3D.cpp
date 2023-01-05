#include "Scene3D.h"

#define SDF_SIZE 100
//#define FILE_NAME "armadillo_low_low"
//#define FILE_NAME "dolphin"
//#define FILE_NAME "hand2"
#define FILE_NAME "unicorn_low"


using namespace std;
using namespace vvr;

Scene3D::Scene3D()
{
	//! Load settings.
	vvr::Shape::DEF_LINE_WIDTH = 4;
	vvr::Shape::DEF_POINT_SIZE = 10;
	m_perspective_proj = true;
	m_hide_log = false;
	m_hide_sliders = false;
	m_bg_col = Colour("768E77");
	m_obj_col = Colour("454545");
	const string objDir = getBasePath() + "resources/obj/";
	const string objFile = objDir + FILE_NAME + ".obj";
	m_model_original = MyMesh(objFile);

	//recovered = Mesh(objDir + FILE_NAME + "_recovered.obj");

	mSphere = vvr::Sphere3D(0, 0, 0, 0.2, vvr::Colour::white);

	mPolyhedron = MyMesh(objDir + "polyhedron.obj");
	mCube = MyMesh(objDir + "m_cube.obj");
	//mSimple_model = MyMesh(objDir + "torus_low.obj");
	mSimple_model = MyMesh(objDir + "suzanne.obj");
	reset();
}

void Scene3D::reset()
{
	Scene::reset();

	//! Define what will be vissible by default
	m_style_flag = 0;
	m_style_flag |= FLAG_SHOW_SOLID;
	m_style_flag |= FLAG_SHOW_WIRE;
	m_style_flag |= FLAG_SHOW_AXES;
}

void Scene3D::resize()
{
	//! By Making `first_pass` static and initializing it to true,
	//! we make sure that the if block will be executed only once.

	static bool first_pass = true;

	if (first_pass)
	{
		m_model_original.setBigSize(40);
		m_model_original.update(true);

		mPolyhedron.setBigSize(8);
		mPolyhedron.update(true);

		mCube.setBigSize(8);
		mCube.update(true);

		mSimple_model.setBigSize(14);
		mSimple_model.update(true);
		
		printKeyboardShortcuts();
		m_model = m_model_original;
		m_model_SDF = MySDF3D(&m_model, SDF_SIZE);

		first_pass = false;
	}
}

void Scene3D::arrowEvent(ArrowDir dir, int modif)
{
	vec newPos;
	if (dir == UP && modif == 0) newPos = vec(0, 1, 0);			//modif = 0 only the arroews
	else if (dir == DOWN && modif == 0) newPos = vec(0, -1, 0);
	else if (dir == LEFT) newPos = vec(-1, 0, 0);
	else if (dir == RIGHT) newPos = vec(1, 0, 0);
	else if (dir == UP && modif == 1) newPos = vec(0, 0, -1);	//modif = 1 press ctrl button with the arrows
	else if (dir == DOWN && modif == 1) newPos = vec(0, 0, 1);
	newPos *= (speed * step);
	
	if (selectedMesh == 1){
		mPolyhedron.move(newPos);
	}
	if (selectedMesh == 2) {
		mCube.move(newPos);
	}
	if (selectedMesh == 3) {
		mSimple_model.move(newPos);
	}
	if (selectedMesh == 4) {
		if (monitorTrajectory == true) {
			vec pos = vec(mSphere.x, mSphere.y, mSphere.z) + newPos;
			trajectory.push_back(pos);
		}
		mSphere.x += newPos.x;
		mSphere.y += newPos.y;
		mSphere.z += newPos.z;
	}
}

void Scene3D::keyEvent(unsigned char key, bool up, int modif)
{
	Scene::keyEvent(key, up, modif);
	key = tolower(key);

	switch (key)
	{
	case 's': if (modif == 0) m_style_flag ^= FLAG_SHOW_SOLID;
		      if (modif == 1) showSolidActiveObj = !showSolidActiveObj;
		      break;
	case 'w': if (modif == 0) m_style_flag ^= FLAG_SHOW_WIRE;
		      if (modif == 1) showWireActiveObj = !showWireActiveObj;
		      break;
	case 'n': m_style_flag ^= FLAG_SHOW_NORMALS; break;
	case 'd': flag_show_diffCords = ~flag_show_diffCords; break;
	case 'v': flag_show_VFF = ~flag_show_VFF; break;
	case 'a': m_style_flag ^= FLAG_SHOW_AXES; break;
	case 'r': {
		m_model = m_model_original;
		this->counter = 0;
		break;
	}
	case 'c': {
		collisionMode += 1;
		collisionMode %= 4;
		if (collisionMode == 0) cout << "mehtod of collision detection: NONE" << endl;
		if (collisionMode == 1) cout << "mehtod of collision detection: WITH MESH" << endl;
		if (collisionMode == 2) cout << "mehtod of collision detection: WITH SDF" << endl;
		if (collisionMode == 3) {
			cout << "mehtod of collision detection: WITH SDF of Neural Network" << endl;
			m_model = recovered;
			this->step = 0.1;
		}
		else {
			m_model = m_model_original;;
			this->step = 1;
		}

		break;
	}
	case 't': showTimeofCollision = !showTimeofCollision; break;
	case 'o': m_model_SDF.writeSDF3D(); break;
	case 'm': {
		if (monitorTrajectory == true) {
			monitorTrajectory = false;
			cout << "Monitor of the trajectory: DISABLED" << endl;

			// save the trajectory to a file;
			cout << "Save the trajectory to a file" << endl;
			const string objDir = vvr::getBasePath() + "Siren-DeepLearning/";
			ofstream output_file;
			output_file.open(objDir + "trajectory.txt", std::fstream::trunc);
			output_file << mSphere.rad << '\n';
			for (int i = 0; i < trajectory.size(); i++) {
				output_file << trajectory[i].x << '\t' << trajectory[i].y << '\t' << trajectory[i].z << '\n';
			}
			output_file.close();
		}
		else if (monitorTrajectory == false){
			trajectory.clear();
			monitorTrajectory = true;
			playAnimation = -1;
			cout << "Monitor of the trajectory: ENABLED" << endl;
		}
		break; 
	}
	case 'p': if (modif == 0) playAnimation += 1;
		      if (modif == 1) playAnimation = -1;
			  if (modif == 2) playAnimation -= 1;
			  break;
	case 'f': showForce = !showForce; break;
	case '-': {
		m_model.performTaubinShrinking(-lamda);
		this->counter--;
		cout << "counter = " << this->counter << endl;
		break;
	}
	case '+': {
		m_model.performTaubinInflation(-lamda, mi);
		this->counter++;
		cout << "counter = " << this->counter << endl;
		break;
	}
	case '0': selectedMesh = 0; break;
	case '1': selectedMesh = 1; break;
	case '2': selectedMesh = 2; break;
	case '3': selectedMesh = 3; break;
	case '4': selectedMesh = 4; break;
	}


	if (key == '?'){
		printKeyboardShortcuts();
	}

}

void Scene3D::draw()
{
	if (m_style_flag & FLAG_SHOW_SOLID) m_model.draw(m_obj_col, SOLID);
	if (m_style_flag & FLAG_SHOW_WIRE) m_model.draw(Colour::black, WIRE);
	//if (m_style_flag & FLAG_SHOW_NORMALS) m_model.draw(Colour::black, NORMALS);
	if (m_style_flag & FLAG_SHOW_NORMALS) m_model.drawnormals();
	if (m_style_flag & FLAG_SHOW_AXES) m_model.draw(Colour::black, AXES);
	if (flag_show_diffCords) m_model.drawDiffCords();
	if (flag_show_VFF) m_model_SDF.drawVFF3D(dens);

	if (selectedMesh == 1) {
		mPolyhedron.mDraw(showSolidActiveObj, showWireActiveObj);
		activeObject = &mPolyhedron;
	}
	if (selectedMesh == 2) {
		mCube.mDraw(showSolidActiveObj, showWireActiveObj);
		activeObject = &mCube;
	}
	if (selectedMesh == 3) {
		mSimple_model.mDraw(showSolidActiveObj, showWireActiveObj);
		activeObject = &mSimple_model;
	}
	if (selectedMesh == 4) {
		mSphere.draw();
		vvr::Point3D(mSphere.x, mSphere.y, mSphere.z, vvr::Colour::magenta).draw();
	}



	if (collisionMode == 1) {
		isCollided = m_model.collisionDetection(activeObject, showTimeofCollision);
	}
	if (collisionMode == 2) {
		vec forceDir;
		isCollided = m_model_SDF.collisionDetection(activeObject, showTimeofCollision, &forceDir);

		if (this->showForce == true) {
			vec startForce = activeObject->getAABB().CenterPoint();
			vec endForce = startForce - this->k * forceDir;

			vvr::Point3D(startForce.x, startForce.y, startForce.z, vvr::Colour::orange).draw();
			vvr::Point3D(endForce.x, endForce.y, endForce.z, vvr::Colour::darkOrange).draw();
			vvr::LineSeg3D(startForce.x, startForce.y, startForce.z, endForce.x, endForce.y, endForce.z).draw();
		}
	}
	if (collisionMode == 3 && selectedMesh == 4) {
		if (playAnimation > -1) {

			const string objDir = vvr::getBasePath() + "Siren-DeepLearning/";
			ifstream file(objDir + "results_SDF_SIREN_" + FILE_NAME + ".txt");
			string line;

			for (int i = 1; i <= playAnimation + 1; i++) {
				if (file.eof()) return;
				getline(file, line);
			}

			vector<float> results = {};
			string temp;
			line += '\t';
			for (int i = 0; i < (int)line.size(); i++) {
				if (line[i] == '\t') {
					results.push_back(atof(temp.c_str()));
					temp = "";
				}
				else {
					temp += line[i];
				}
			}

			mSphere.x = results[0];
			mSphere.y = results[1];
			mSphere.z = results[2];

			if (results[3] < mSphere.rad) isCollided = true;
			else isCollided = false;
		}
	}

	if (isCollided == true && collisionMode != 0) {
		m_bg_col = Colour("ab6f63");
	}
	else if (isCollided == false && collisionMode != 0) {
		m_bg_col = Colour("63ab66");
	}
	else {
		m_bg_col = Colour("768E77");
	}
}

void Scene3D::printKeyboardShortcuts()
{
	std::cout << "Keyboard shortcuts:"
		<< std::endl << "'?' => This shortcut list:"
		<< std::endl << "'s' => Show Solid for the main Mesh"
		<< std::endl << "'Ctrl+s' => Show Solid for moving Object"
		<< std::endl << "'w' => Show Wires/Edges for the main Mesh"
		<< std::endl << "'Ctrl+w' => Show Wire for moving Object"
		<< std::endl << "'n' => Show Normals for the main Mesh"
		<< std::endl << "'d' => Show Differential Cordinates for the main Mesh"
		<< std::endl << "'v' => Show (normal) Vector Force Field - VFF for the main Mesh"
		<< std::endl << "'a' => Show Axes"
		<< std::endl << "'r' => Reload the main model"
		<< std::endl << "'c' => Change the method of Collision Detection"
		<< std::endl << "'t' => Print the duration for checking if there is collision"
		<< std::endl << "'o' => Write the SDF to file a 'SDF.m'"
		<< std::endl << "'m' => Enable/Disable the monitoring of the trajectory (Only for the Sphere - Object 4)'"
		<< std::endl << "'p' => Press it once to go the next possition (Only for the sphere and NN approach)'"
		<< std::endl << "'Ctrl+p' => Reset the counter to start from the beginning of the trajectory'"
		<< std::endl << "'shift+p' => Press it once to go the previous possition (Only for the sphere and NN approach)'"
		<< std::endl << "'f' => Show the Force"
		<< std::endl << "'-' => Perform Taubin Inflation"
		<< std::endl << "'+' => Perform Taubin Srinking"
		<< std::endl << "'0' => Show none of the Objects"
		<< std::endl << "'1' => Show the Polyhedron - Sphere"
		<< std::endl << "'2' => Show the Cube"
		<< std::endl << "'3' => Show simple Mesh"
		<< std::endl << "'4' => Show a Sphere (Only for Neural Network approach)"
		<< std::endl << "1st slider: Change lamda (0-1)"
		<< std::endl << "2nd slider: Change mi (0-1)"
		<< std::endl << "3rd slider: Change the desnity of VFF (only for drawing)"
		<< std::endl << "4th slider: Change the moving step of the object"
		<< std::endl << "5th slider: Change the k (Elasticity of boby)"
		<< std::endl << std::endl;
}

void Scene3D::sliderChanged(int slider_id, float v) {
	switch (slider_id) {
	case 0: {
		lamda = v;
		cout << "lamda = " << lamda << endl;
		break;
	}
	case 1: {
		mi = v;
		cout << "mi = " << mi << endl;
		break;
	}
	case 2: {
		dens = v + 10E-5;
		cout << "Density of VFF = " << dens << endl;
		break;
	}
	case 3: {
		speed = this->step * v;
		cout << "Step of moving object = " << speed << endl;
		break;
	}
	case 4: {
		k = v;
		cout << "Elasticity k = " << k << endl;
		break;
	}
	}
}