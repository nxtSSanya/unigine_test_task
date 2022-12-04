#include "Train.h"
#include "TrainItem.h"
#include "SplineCalculateComponent.h"
#include <memory>

Train::Train(const size_t& itemsCount, Mesh& m, const std::vector<glm::vec3>& _points, const std::vector<double>& samples, double velocity) {

	m_velocity = velocity;
	m_train.reserve(itemsCount);
	m_spacing.reserve(itemsCount);

	Engine* _engine = Engine::get();
	//std::unique_ptr<Engine> _engine = std::make_unique<Engine>(Engine::get());

	double distance = 0.0;
	double itemOffset = 1.0;
	const glm::vec3 trainItemSize = glm::vec3(1.0, 1.0, 1.0);

	for (size_t i = 0; i < itemsCount; ++i) {

		double t = SplineCalculateHelper::FindParameter(samples, distance, _points);
		glm::vec3 position = SplineCalculateHelper::catmull_rom_spline(_points, t);

		glm::vec3 grad = SplineCalculateHelper::calculateGradient(_points, t);
		double grad_angle = SplineCalculateHelper::getAngleFromGradient(grad);
		Object* trainItem = _engine->createObject(&m);
		trainItem->setColor(1, 0, 0);
		trainItem->setPosition(position.x, position.y, position.z);
		trainItem->setRotation(0.0f, grad_angle, 0.0f);
		trainItem->setScale(trainItemSize.x, trainItemSize.y, trainItemSize.z);
		m_train.push_back(trainItem);

		m_spacing.push_back(distance);
		distance += itemOffset;
	}

}

void Train::move(double _size, const std::vector<glm::vec3>& _points, double deltaTime, std::vector<double>& samples) {
	for (size_t i = 0; i < m_spacing.size(); ++i) {

		auto item = m_train[i];
		auto& dist = m_spacing[i];

		double t = SplineCalculateHelper::FindParameter(samples, dist, _points);
		glm::vec3 position = SplineCalculateHelper::catmull_rom_spline(_points, t);
		item->setPosition(position);

		// s = v * t
		dist += m_velocity * deltaTime;
		glm::vec3 grad = SplineCalculateHelper::calculateGradient(_points, t);
		double grad_angle = SplineCalculateHelper::getAngleFromGradient(grad);
		item->setRotation(0.0f, grad_angle, 0.0f);


		if (dist > _size)
		{
			dist -= _size;
		}
	}
}

Train::~Train() {
	Engine* engine = Engine::get();
	//std::unique_ptr<Engine> _engine = std::make_unique<Engine>(Engine::get());
	for (auto& item : m_train)
	{
		engine->deleteObject(item);
	}
}

Train& Train::operator=(Train& other)
{
	m_spacing = other.m_spacing;
	m_train = other.m_train;
	m_velocity = other.m_velocity;
	return *this;
}

Train::Train(const Train& other) : 
	m_spacing(other.m_spacing), m_train(other.m_train), m_velocity(other.m_velocity) 
{ }


