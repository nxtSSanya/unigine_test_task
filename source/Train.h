#pragma once
#include <vector>
#include "framework/engine.h"

#include "SplineCalculateComponent.h"
class Train
{
public:
	Train(const size_t& itemsCount, Mesh& m, const std::vector<glm::vec3>& _points, const std::vector<double>& samples, double velocity);
	void move(double _size, const std::vector<glm::vec3>& _points, double deltaTime, std::vector<double>& samples);
	~Train();
	Train& operator = (Train& other);
	Train(const Train& other);
private:
	std::vector<double> m_spacing;
	double m_velocity;
	std::vector<Object* > m_train;
};

