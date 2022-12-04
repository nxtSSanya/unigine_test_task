#include "framework/engine.h"
#include "framework/utils.h"
#include "Train.h"
#include "SplineCalculateComponent.h"
#include "MeshGenerator.h"

#define GLM_ENABLE_EXPERIMENTAL

using namespace std;
using namespace glm;
using namespace SplineCalculateHelper;	// "MeshGenerator.h"
using namespace MeshGenerator;			// "SplineCalculateComponent.h"

const glm::vec3 rails_color_metallic(0.32, 0.30, 0.35);
const size_t train_size = 8;
const double initial_velocity = 1.9;

/*
* Coordinate system:
* x - right
* y - up
* z - backward
*/

int main()
{
	// initialization
	Engine *engine = Engine::get();
	engine->init(1600, 900, "UNIGINE Test Task");

	// set up camera
	Camera &cam = engine->getCamera();
	cam.Position = vec3(0.0f, 12.0f, 17.0f);
	cam.Yaw = -90.0f;
	cam.Pitch = -45.0f;
	cam.UpdateCameraVectors();

	// create shared meshes
	Mesh plane_mesh = createPlane();
	Mesh sphere_mesh = createSphere();
	Mesh cube_mesh = createCube();

	// create background objects
	Object *plane = engine->createObject(&plane_mesh);
	plane->setColor(0.2f, 0.37f, 0.2f); // green
	plane->setPosition(0, -0.5f, 0);
	plane->setRotation(-90.0f, 0.0f, 0.0f);
	plane->setScale(20.0f);

	// path
	const float path[] = {
		 0.0f, -0.375f,  7.0f, // 1
		-6.0f, -0.375f,  5.0f, // 2
		-8.0f, -0.375f,  1.0f, // 3
		-4.0f, -0.375f, -6.0f, // 4
		 0.0f, -0.375f, -7.0f, // 5
		 1.0f, -0.375f, -4.0f, // 6
		 4.0f, -0.375f, -3.0f, // 7
		 8.0f, -0.375f,  7.0f  // 8
	};
	vector<Object* > points;
	for (int i = 0; i < 8; i++)
	{
		Object *sphere = engine->createObject(&sphere_mesh);
		sphere->setColor(1, 0, 0);
		sphere->setPosition(path[i*3], path[i*3+1], path[i*3+2]);
		sphere->setScale(0.25f);
		points.push_back(sphere);
	}
	LineDrawer path_drawer(path, points.size(), true);

	std::vector<glm::vec3> _points;
	for (size_t i = 0; i < std::size(path); i+=3) {
		_points.push_back(glm::vec3(path[i], path[i + 1], path[i + 2]));
	}

	std::vector<Object*> railroad;
	MeshGenerator::drawSleepers(_points, engine, plane_mesh, railroad);

	// Sample for rails
	Mesh mesh = createCube();
	// Get coords from spline
	std::pair<std::vector<glm::vec3>, std::vector<glm::vec3> > tmp = MeshGenerator::generateRailsPts(_points, mesh);
	// draw left
	for (int i = 0; i < tmp.first.size(); ++i) { 
		Object* qweq = engine->createObject(&mesh);
		qweq->setPosition(tmp.first[i]);
		qweq->setScale(0.1);
		qweq->setColor(rails_color_metallic);
	}
	// draw right
	for (int i = 0; i < tmp.second.size(); ++i) { 
		Object* qweq = engine->createObject(&mesh);
		qweq->setPosition(tmp.second[i]);
		qweq->setScale(0.1);
		qweq->setColor(rails_color_metallic);
	}

	std::vector<double> curveLen;	
	double res_length = 0.0;
	SplineCalculateHelper::getSamples(_points, curveLen);
	for (float length : curveLen)
		res_length += length;

	Train train(train_size, cube_mesh, _points, curveLen, initial_velocity);

	// main loop
	while (!engine->isDone())
	{
		double deltaTime = engine->getDeltaTime();
		train.move(res_length, _points, deltaTime, curveLen);
		engine->update();
		engine->render();

		engine->swap();
	}
	// end main loop

	engine->shutdown();
	return 0;
}
