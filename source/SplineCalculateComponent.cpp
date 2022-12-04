#include "SplineCalculateComponent.h"
#include "glm/glm.hpp"
#include "glm/gtx/spline.hpp"
#include <vector>
/* 
USED MATH
https://www.geometrictools.com/Documentation/MovingAlongCurveSpecifiedSpeed.pdf
https://habr.com/ru/post/512700/
https://stackoverflow.com/questions/37230747/how-can-i-generate-a-spline-curve-using-glm-gtx-splinecatmullrom
https://gamedev.ru/code/tip/catmull_rom
*/



std::vector<glm::vec3> m_vFactor(4);

const std::vector<std::pair<double, double> > gaussianQuad_3pts = 
{   {-0.7745966,    0.5555556},
    {0.0,           0.8888889},
    {0.7745966,     0.5555556}
};

void SplineCalculateHelper::getSamples(const std::vector<glm::vec3>& cp, std::vector<double>& samples) {
    for (size_t i = 0; i < cp.size(); ++i)
    {
        double _lower = i;
        double _upper = i + 1;
        samples.push_back(Integrate(_lower, _upper, cp));
    }
}

glm::vec3 SplineCalculateHelper::catmull_rom_spline(const std::vector<glm::vec3>& cp, double t)
{
    // indices of the relevant control points

    /*const int i0 = glm::clamp<int>(t - 1, 0, cp.size() - 1);
    const int i1 = glm::clamp<int>(t, 0, cp.size() - 1);
    const int i2 = glm::clamp<int>(t + 1, 0, cp.size() - 1);
    const int i3 = glm::clamp<int>(t + 2, 0, cp.size() - 1);*/

    const int i0 = t;
    const int i1 = t + 1;
    const int i2 = t + 2;
    const int i3 = t + 3;

    // parameter on the local curve interval
    double local_t = glm::fract(t);

    const glm::vec3& v0 = cp[i0 % cp.size()];
    const glm::vec3& v1 = cp[i1 % cp.size()];
    const glm::vec3& v2 = cp[i2 % cp.size()];
    const glm::vec3& v3 = cp[i3 % cp.size()];
    return glm::catmullRom(v0, v1, v2, v3, local_t);
}

inline double SplineCalculateHelper::lerp(double x, double y, double v) {
    return x * (1.0 - v) + (y * v);
}

inline double SplineCalculateHelper::inverseLerp(double x, double y, double v) {
    return (v - x) / (y - x);
}

glm::vec3 SplineCalculateHelper::calculateGradient(const std::vector<glm::vec3>& cp, double t) {
    // indices of the relevant control points
    const int i0 = t;
    const int i1 = t + 1;
    const int i2 = t + 2;
    const int i3 = t + 3;

    // parameter on the local curve interval
    double local_t = glm::fract(t);
    double square_t = local_t * local_t;

    const glm::vec3& dv0 = cp[i0 % cp.size()];
    const glm::vec3& dv1 = cp[i1 % cp.size()];
    const glm::vec3& dv2 = cp[i2 % cp.size()];
    const glm::vec3& dv3 = cp[i3 % cp.size()];

    double df0 = -3.0f * square_t + 4.0f * local_t - 1.0f;
    double df1 = 9.0f * square_t - 10.0f * local_t;
    double df2 = -9.0f * square_t + 8.0f * local_t + 1.0f;
    double df3 = 3.0f * square_t - 2.0f * local_t;

    return multiply((multiply(dv0, df0) + multiply(dv1, df1) + multiply(dv2, df2) + multiply(dv3, df3)), 0.5);

}

inline glm::vec3 SplineCalculateHelper::multiply(glm::vec3 vect, double t) {
    return vect *= t;
}

double SplineCalculateHelper::Integrate(double lowerBound, double upperBound, const std::vector<glm::vec3>& cp) {
    double weighted_sum = 0.0;

    for (auto& i: gaussianQuad_3pts) {
        double arg = i.first;
        double weight = i.second;
        double t = SplineCalculateHelper::lerp(lowerBound, upperBound, SplineCalculateHelper::inverseLerp(-1, 1, arg));
        weighted_sum += weight * glm::length(SplineCalculateHelper::calculateGradient(cp, t));
    }

    return weighted_sum * (upperBound - lowerBound) / 2;
}

double SplineCalculateHelper::FindParameter(const std::vector<double>& samples , double distance, const std::vector<glm::vec3>& cp) {
    double cur_len = samples[0];
    double local_offset = 0.0;

    for (auto& sample : samples)
    {
        if (distance < sample)
            break;

        cur_len = sample;
        local_offset += 1.0;
        distance -= sample;
    }

    double t = 0.0 + distance / cur_len;
    double lower_bound = 0.0;
    double upper_bound = 1.0;

    for (size_t i = 0; i < 100; ++i) {
        double f = SplineCalculateHelper::Integrate(local_offset, local_offset + t, cp) - distance;
        if (abs(f) < 0.01)
            break;
        double dy = glm::length(calculateGradient(cp, local_offset + t));
        double candidateT = t - f / dy;

        if (f > 0) {
            upper_bound = t;
            if (candidateT <= 0) {
                t = (upper_bound + lower_bound) / 2.0;
            }
            else {
                t = candidateT;
            }
        }
        else {
            lower_bound = t;
            if (candidateT >= 1) {
                t = (upper_bound + lower_bound) / 2.0;
            }
            else {
                t = candidateT;
            }
        }
    }
    return local_offset + t;
}

double SplineCalculateHelper::getAngleFromGradient(glm::vec3 grad) {
    return glm::degrees(atan2(-grad.z, grad.x));
}

double SplineCalculateHelper::getApproxVelocity(double curVelocity, double dx, double dy) {
    return curVelocity / glm::sqrt(dx * dx + dy * dy);
}

glm::vec3 SplineCalculateHelper::getPoint(double distance, const std::vector<glm::vec3>& cp, const std::vector<double>& samples) {
    double t = SplineCalculateHelper::FindParameter(samples, distance, cp);
    return SplineCalculateHelper::catmull_rom_spline(cp, t);
}
