#include "MeshGenerator.h"
#include "SplineCalculateComponent.h"
#include <vector>

const double spacing = 0.5;
const glm::vec3 sleeper_size(0.1, 0.7, 1.2);
const glm::vec3 sleeper_color = { 0.5, 0.5, 0.5 };
constexpr double dist_rails = 0.125;

std::pair<std::vector<glm::vec3>, std::vector<glm::vec3> > MeshGenerator::generateRailsPts(std::vector<glm::vec3> _points, Mesh& mesh) {

	std::vector<glm::vec3> splinePts;
	std::vector<glm::vec3> splinePts1;
	std::pair<std::vector<glm::vec3>, std::vector<glm::vec3> > result;
	double delta = _points.size() - 1;

	while(delta < 2 * _points.size()) {
		glm::vec3 grad = SplineCalculateHelper::calculateGradient(_points, delta);
		glm::vec3 position = SplineCalculateHelper::catmull_rom_spline(_points, delta);
		
		splinePts.push_back(position);
		double delta_s = SplineCalculateHelper::getApproxVelocity(0.1, grad.x, grad.z);
		delta += delta_s;
	}
	std::copy(splinePts.begin(), splinePts.end(), std::back_inserter(splinePts1));

	for (int i = 1; i < splinePts.size() - 1; ++i) {
		glm::vec3 forwardVector = glm::vec3(0.0, 0.0, 0.0);
		forwardVector += splinePts[i + 1] - splinePts[i] + splinePts[i] - splinePts[i - 1];
		forwardVector = glm::normalize(forwardVector);
		glm::vec3 left = glm::vec3(-forwardVector.z, forwardVector.y, forwardVector.x);
		splinePts[i] += SplineCalculateHelper::multiply(SplineCalculateHelper::multiply(left, -1.0), dist_rails);
		splinePts1[i] += SplineCalculateHelper::multiply(SplineCalculateHelper::multiply(left, 1.0), dist_rails);
	}
	
	result = { splinePts, splinePts1 };

	return result;
}


void MeshGenerator::drawSleepers(const std::vector<glm::vec3>& _points, Engine* engine, Mesh& mesh, std::vector<Object*>& other_objects) {
	double delta = 0.0;
	while(delta < _points.size()) {

		glm::vec3 position = SplineCalculateHelper::catmull_rom_spline(_points, delta);
		glm::vec3 grad = SplineCalculateHelper::calculateGradient(_points, delta);
		double delta_s = SplineCalculateHelper::getApproxVelocity(spacing, grad.x, grad.z);

		double grad_angle = SplineCalculateHelper::getAngleFromGradient(grad);
		auto roadblock = MeshGenerator::drawSleeper(position, glm::vec3(-90.f, grad_angle, 0.f), sleeper_size, sleeper_color[0], sleeper_color[1], sleeper_color[2], engine, mesh);
		other_objects.push_back(roadblock);

		delta += delta_s;
	}
}

Object* MeshGenerator::drawSleeper(const glm::vec3& position, const glm::vec3& rotator, const glm::vec3 scale, const double r, const double g, const double b, Engine* _engine, Mesh& mesh) {
	Object* item = _engine->createObject(&mesh);
	item->setColor(r, g, b);
	item->setPosition(position.x, position.y, position.z);
	item->setRotation(rotator.x, rotator.y, rotator.z);
	item->setScale(scale.x, scale.y, scale.z);
	return item;
}

Object* MeshGenerator::createRailway(Engine* engine, Mesh& mesh) {
	Object* railway = engine->createObject(&mesh);
	return railway;
}

