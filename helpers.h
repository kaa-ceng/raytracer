#ifndef RAYTRACER_H
#define RAYTRACER_H

#include <iostream>
#include <cmath>
#include "parser.h"

//direction is always normalised
struct Ray {
    parser::Vec3f origin = {0,0,0};
    parser::Vec3f direction = {0,0,0};
    int depth = 0;
};

struct Intersection {
    parser::Vec3f point = {0,0,0};
    parser::Vec3f normal= {0,0,0};
    int materialID = 0;
    float t = 0;
};

float dot(const parser::Vec3f& a, const parser::Vec3f& b);
float magnitude(const parser::Vec3f& v);

parser::Vec3f normalize(const parser::Vec3f& v);
parser::Vec3f cross(const parser::Vec3f& a, const parser::Vec3f& b);

parser::Vec3f operator*(const parser::Vec3f& v, float scalar);
parser::Vec3f operator+(const parser::Vec3f& a, const parser::Vec3f& b);
parser::Vec3f operator-(const parser::Vec3f& a, const parser::Vec3f& b);

parser::Vec3f elementwiseMultiply(const parser::Vec3f& a, const parser::Vec3f& b);

Ray generateRay(int x, int y, const parser::Camera& camera, int width, int height);

bool sphereIntersect(const Ray& ray, const parser::Sphere& sphere, const parser::Scene& scene, float& t);
bool faceIntersect(const Ray& ray, const parser::Face& face, const parser::Scene& scene, float& t);

parser::Vec3f computeColor(const Ray& ray, const parser::Scene& scene);

Intersection findFirstIntersection(const Ray& ray, const parser::Scene& scene);

parser::Vec3i clampColor(parser::Vec3f& color);
parser::Vec3f applyShading(
    const parser::Scene& scene,
    const Intersection& intersection,
    const Ray& ray
);

#endif 