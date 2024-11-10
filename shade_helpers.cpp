#include "iostream"
#include "parser.h"
#include "helpers.h"

parser::Vec3i clampColor(parser::Vec3f& color)
{
	parser::Vec3i result;

	result.x = std::min(255, std::max((int)color.x, 0));
	result.y = std::min(255, std::max((int)color.y, 0));
	result.z = std::min(255, std::max((int)color.z, 0));

	return result;
}

parser::Vec3f findIrradiance(const parser::PointLight& light,float lightDist)
{
	float lightDistSquare = lightDist * lightDist;

	if (lightDistSquare > 0.0)
	{
		return {
			light.intensity.x / lightDistSquare,
			light.intensity.y / lightDistSquare,
			light.intensity.z / lightDistSquare
		};
	}

	return { 0.0,0.0,0.0 };
}

parser::Vec3f findDiffuse(
	const parser::Vec3f& materialDiffuse,
	const parser::Vec3f& irradiance,
	const parser::Vec3f& normal,
	const parser::Vec3f& normalizedLightDir)
{
	float theta = dot(normal, normalizedLightDir);

	if (theta <= 0)
	{
		return { 0,0,0 };
	}
	
	return
	{
		materialDiffuse.x * irradiance.x * theta,
		materialDiffuse.y * irradiance.y * theta,
		materialDiffuse.z * irradiance.z * theta
	};
}

parser::Vec3f findSpacular(
	float phong,
	const parser::Vec3f& materialSpacular,
	const parser::Vec3f& irradiance,
	const parser::Vec3f& normalizedHalf,
	const parser::Vec3f& normal)
{
	float alpha = dot(normalizedHalf, normal);

	if (alpha < 0)
	{
		return { 0,0,0 };
	}

	float phongedAlpha = std::pow(alpha, phong);
	return {
		materialSpacular.x * irradiance.x * phongedAlpha,
		materialSpacular.y * irradiance.y * phongedAlpha,
		materialSpacular.z * irradiance.z * phongedAlpha
	};
}




// this function only executes when a intersection happens,
// therefore it assumes an intersection happened
parser::Vec3f applyShading(
	const parser::Scene& scene, 
	const Intersection& intersection,
	const Ray& r)
{
	
	parser::Vec3f finalColor = { 0,0,0 };
	parser::Material intersectionMaterial = scene.materials[intersection.materialID - 1];
	
	#pragma region AmbientLightCalculation

	parser::Vec3f ambientLightEffect = {
		scene.ambient_light.x * intersectionMaterial.ambient.x,
		scene.ambient_light.y * intersectionMaterial.ambient.y,
		scene.ambient_light.z * intersectionMaterial.ambient.z
	};

	finalColor = finalColor + ambientLightEffect;

	#pragma endregion

	#pragma region MirrorLightCalculation


    if (intersectionMaterial.is_mirror && r.depth < scene.max_recursion_depth) {
        Ray reflectionRay;
        parser::Vec3f w0 = normalize(r.direction * -1); // normalized
        reflectionRay.direction = normalize(intersection.normal * 2 * dot(intersection.normal, w0) - w0);
        reflectionRay.origin = intersection.point + (reflectionRay.direction * scene.shadow_ray_epsilon);
        reflectionRay.depth = r.depth + 1;

        // recursively calculate the color from the reflected ray
        parser::Vec3f mirrorColor = computeColor(reflectionRay, scene);
        finalColor = finalColor + elementwiseMultiply(mirrorColor, intersectionMaterial.mirror); //changed
    }

	#pragma endregion

	#pragma region SpecularDiffuseCalculation

	for (const parser::PointLight& light : scene.point_lights)
	{
		parser::Vec3f vecToLight = light.position - intersection.point;

		//back-face culling (get rid of it if does not work)
		if (dot(vecToLight, intersection.normal) <= 0) //not sure if only less or lessequal 
		{
			continue; // if the dot is negative, normal is not facing the light
		}

		parser::Vec3f normalizedVecToLight = normalize(vecToLight);

		Ray shadowRay;
		shadowRay.direction = normalizedVecToLight; //normalized
		shadowRay.origin = intersection.point + shadowRay.direction * scene.shadow_ray_epsilon;

		Intersection shadowInterseption = findFirstIntersection(shadowRay, scene);

		float lightDist = magnitude(vecToLight);

		if (shadowInterseption.t != -1)
		{
			float blockerDist = magnitude(shadowInterseption.point - intersection.point);

			if (lightDist > blockerDist)
			{
				continue; // if intersection is in shadow, does not get contribution from this light
			}
		}

		parser::Vec3f irradiance = findIrradiance(light, lightDist);
		parser::Vec3f normalizedHalf = normalize(normalizedVecToLight - r.direction);

		parser::Vec3f diffuse = findDiffuse(intersectionMaterial.diffuse, irradiance, intersection.normal, normalizedVecToLight);
		parser::Vec3f spacular = findSpacular(intersectionMaterial.phong_exponent, intersectionMaterial.specular, irradiance, normalizedHalf, intersection.normal);
		
		finalColor = finalColor + diffuse + spacular;
	}



	#pragma endregion
	

	return finalColor;
};