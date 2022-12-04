#pragma once

#include "framework/engine.h"
#include "framework/utils.h"

namespace MeshGenerator {
	void drawSleepers(const std::vector<glm::vec3>& _points, Engine* engine, Mesh& mesh, std::vector<Object*>& other_objects);
	Object* drawSleeper(const glm::vec3& position, const glm::vec3& rotator, const glm::vec3 scale, const double r, const double g, const double b, Engine* _engine, Mesh& mesh);
	Object* createRailway(Engine* engine, Mesh& mesh);
	std::pair<std::vector<glm::vec3>, std::vector<glm::vec3> > generateRailsPts(std::vector<glm::vec3> _points, Mesh& mesh);
}