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
	// Phong light model at the point determined by the intersection calculation.
	// Also respect that the intensity of the light will decrease with increasing distance to the light source,
	// or by positioning them behind other objects (shadowing).
	float dist2Light = length(position - intersection.position);
	float3 Il = intensity * (1 / powf(dist2Light, 2));
	float3 N = intersection.normal;
	float3 L = normalize(position - intersection.position);

	// Ambient term
	float3 I = intersection.material->GetAmbient() * Il;

	// Shadow ray
	Ray ray = Ray(intersection.position + 0.001f * intersection.normal, L);  // epsilon = 0.001f
	for (Raytracer::Scenes::IPrimitive* object: scene.GetObjects())
	{
		RayHit objectHit;
		object->HitTest(ray, objectHit);
		float t = objectHit.GetDistance();
		if (t > 0 && t < dist2Light)
		{
			return I;
		}
	}

	// Diffuse term
	float NdotL = dot(N, L);
	if (NdotL > 0)
	{
		I += intersection.material->GetDiffuse() * Il * NdotL;
		
		// Specular term
		float3 R = normalize(2.0f * N * dot(N, L) - L);
		float3 V = intersection.viewDirection;
		float RdotV = dot(R, V);
		if (RdotV > 0)
		{
			I += intersection.material->GetSpecular() * Il * powf(RdotV, intersection.material->GetShininess());
		}
	}

	// Emissive term is 0
	return I;
}
