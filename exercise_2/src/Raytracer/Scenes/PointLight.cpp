#include <Raytracer/Raytracer.h>

using namespace Raytracer;
using namespace Raytracer::Scenes;

PointLight::PointLight(const float3 &position, const float3 &intensity)
{
  this->position = position;
  this->intensity = intensity;
}

float3 PointLight::ComputeDirectContribution(const Intersection &intersection, const Scene &scene)
{
  // TODO: Implement the calculation of the diffuse and specular lighting according to the
  // Phong light modelat the point determined by the intersection calculation.
  // Also respect that the intensity of the light will decrease with increasing distance to the light source,
  // or by positioning them behind other objects (shadowing).
  return float3(0, 0, 0);
}
