#pragma once

#include "glm/glm.hpp"
#include <vector>

namespace SplineCalculateHelper {

	struct quaternion {
		double qw = 1.0;
		double qx = 0.0;
		double qy = 0.0;
		double qz = 0.0;
	};

	void getSamples(const std::vector<glm::vec3>& cp, std::vector<double>& samples);
	glm::vec3 catmull_rom_spline(const std::vector<glm::vec3>& cp, double t);
	double lerp(double x, double y, double v);
	double inverseLerp(double x, double y, double v);
	glm::vec3 calculateGradient(const std::vector<glm::vec3>& cp, double t);
	inline glm::vec3 multiply(glm::vec3 vect, double t);
	double FindParameter(const std::vector<double>& samples, double distance, const std::vector<glm::vec3>& cp);
	double getAngleFromGradient(glm::vec3 grad);
	double getApproxVelocity(double curVelocity, double dx, double dy);
	glm::vec3 getPoint(double distance, const std::vector<glm::vec3>& cp, const std::vector<double>& samples);
	double Integrate(double lowerBound, double upperBound, const std::vector<glm::vec3>& cp);
}