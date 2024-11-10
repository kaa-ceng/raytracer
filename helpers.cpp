#include "iostream"
#include "parser.h"
#include "helpers.h"

#include <cfloat>


float dot(const parser::Vec3f& a, const parser::Vec3f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;       
}

float magnitude(const parser::Vec3f& v){
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

parser::Vec3f normalize(const parser::Vec3f& v) {                   
    float length = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return { v.x / length, v.y / length, v.z / length };
}

parser::Vec3f cross(const parser::Vec3f& a, const parser::Vec3f& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

parser::Vec3f operator*(const parser::Vec3f& v, float scalar) {      
    return { v.x * scalar, v.y * scalar, v.z * scalar };
}

parser::Vec3f operator+(const parser::Vec3f& a, const parser::Vec3f& b) {
    return { a.x + b.x, a.y + b.y, a.z + b.z };
}

parser::Vec3f operator-(const parser::Vec3f& a, const parser::Vec3f& b) {
    return { a.x - b.x, a.y - b.y, a.z - b.z };
}

parser::Vec3f elementwiseMultiply(const parser::Vec3f& a, const parser::Vec3f& b) {
    return { a.x * b.x, a.y * b.y, a.z * b.z };
}

//may need to normalize some vectors
Ray generateRay(int x, int y, const parser::Camera& camera, int width, int height) {
    
    parser::Vec3f e = camera.position;
    parser::Vec3f w = camera.gaze * -1;
    parser::Vec3f v = camera.up;
    parser::Vec3f u = cross(camera.gaze, camera.up);

    parser::Vec3f m = e - w * camera.near_distance;         //midpoint m of the image plane
    
    float l = camera.near_plane.x;                    
    float r = camera.near_plane.y;
    float b = camera.near_plane.z;
    float t = camera.near_plane.w;

    parser::Vec3f q = m + u * l + v * t;        
    
    float su = (x + 0.5f) * (r - l) / width;        
    float sv = (y + 0.5f) * (t - b) / height;

    parser::Vec3f s = q + (u * su) - (v * sv);      

    Ray ray;
    
    ray.origin = e;
    ray.direction = normalize(s - e);

    return ray;     
}


// Intersect functions also use t as an output to give the intersection time.
// These may also need the maximum t to control the intersection happened in a reasonable time limit



bool sphereIntersect(const Ray& ray, const parser::Sphere& sphere, const parser::Scene& scene, float& t) {

    parser::Vec3f center = scene.vertex_data[sphere.center_vertex_id-1];
    float radius = sphere.radius;

    parser::Vec3f oc = ray.origin - center;
    
    float a = dot(ray.direction, ray.direction);  // d.d
    float b = 2.0f * dot(ray.direction, oc);      // 2d.(o-c)
    float c = dot(oc, oc) - radius * radius;      // (o-c).(o-c) - R^2
    
    float discriminant = b * b - 4 * a * c;
    
    if (discriminant < 0) {
        return false;
    }
    
    float sqrDisc = sqrt(discriminant);
    float t1 = (-b - sqrDisc) / (2*a);
    float t2 = (-b + sqrDisc) / (2*a);
    
    // if both t values are negative, intersection is behind the ray
    if (t1 < scene.shadow_ray_epsilon && t2 < scene.shadow_ray_epsilon) {
        return false;
    }
    
    // nearest positive intersection point
    if (t1 > scene.shadow_ray_epsilon && t2 > scene.shadow_ray_epsilon) {
        t = std::min(t1, t2);
    } else {
        t = std::max(t1, t2);
    }
    
    return true;
}



bool faceIntersect(const Ray& ray, const parser::Face& face, const parser::Scene& scene, float& t) {
    parser::Vec3f a = scene.vertex_data[face.v0_id - 1];
    parser::Vec3f b = scene.vertex_data[face.v1_id - 1];
    parser::Vec3f c = scene.vertex_data[face.v2_id - 1];


    parser::Vec3f ab = b - a;
    parser::Vec3f ac = c - a;
    parser::Vec3f normal = cross(ab, ac);
    normal = normalize(normal);

    if ( dot(normal, ray.direction) == 0 ){
        return false;
    }

    float d = dot(normal, a);
    t = (d - dot(normal, ray.origin)) / dot(normal, ray.direction);

    if (t < 0) {
        // Intersection is behind the ray
        return false;
    }

    parser::Vec3f p = ray.origin + ray.direction * t;

    parser::Vec3f edge0 = b - a;
    parser::Vec3f vp0 = p - a;
    if (dot(normal, cross(edge0, vp0)) < 0) return false;

    parser::Vec3f edge1 = c - b;
    parser::Vec3f vp1 = p - b;
    if (dot(normal, cross(edge1, vp1)) < 0) return false;

    parser::Vec3f edge2 = a - c;
    parser::Vec3f vp2 = p - c;
    if (dot(normal, cross(edge2, vp2)) < 0) return false;

    // the point is inside the triangle
    return true;
}


parser::Vec3f computeColor(const Ray& ray, const parser::Scene& scene) {

    if (ray.depth > scene.max_recursion_depth) //max recursion depth
    {
        return { 0,0,0 };
    }

    Intersection intersection = findFirstIntersection(ray, scene);

    if (intersection.t != -1) //if hit an object
    {
        intersection.point = ray.origin + ray.direction * intersection.t;

        return applyShading(scene, intersection, ray);
    }
        
    else if (ray.depth == 0) //if not hit an object and ray is not a reflection ray
    {
        return {
            (float)scene.background_color.x,
            (float)scene.background_color.y,
            (float)scene.background_color.z
        };
    }

    else // ray is a reflection ray and not hit an object,
    {
        return { 0,0,0 };
    }

    
}



Intersection findFirstIntersection(const Ray& ray, const parser::Scene& scene) {
    Intersection intersection;
    float closestT = FLT_MAX;

    for (const parser::Sphere& sphere : scene.spheres) {
        float t;
        if (sphereIntersect(ray, sphere, scene, t) && t < closestT) {
            closestT = t;
            intersection.t = t;
            intersection.materialID = sphere.material_id;

            intersection.point = ray.origin + ray.direction * t;

            parser::Vec3f center = scene.vertex_data[sphere.center_vertex_id-1];
            intersection.normal = normalize(intersection.point - center);
        }
    }


    for (const parser::Triangle& triangle : scene.triangles) {
        float t;
        if (faceIntersect(ray, triangle.indices, scene, t) && t < closestT) {
            closestT = t;
            intersection.t = t;
            intersection.materialID = triangle.material_id;

            intersection.point = ray.origin + ray.direction * t;

            parser::Vec3f a = scene.vertex_data[triangle.indices.v0_id - 1];
            parser::Vec3f b = scene.vertex_data[triangle.indices.v1_id - 1];
            parser::Vec3f c = scene.vertex_data[triangle.indices.v2_id - 1];
            intersection.normal = normalize(cross(b - a, c - a));

        }
    }

    for (const parser::Mesh& mesh : scene.meshes) {
        for (const parser::Face& face : mesh.faces) {
            float t;
            if (faceIntersect(ray, face, scene, t) && t < closestT) {
                closestT = t;
                intersection.t = t;
                intersection.materialID = mesh.material_id;

                intersection.point = ray.origin + ray.direction * t;

                parser::Vec3f a = scene.vertex_data[face.v0_id - 1];
                parser::Vec3f b = scene.vertex_data[face.v1_id - 1];
                parser::Vec3f c = scene.vertex_data[face.v2_id - 1];
                intersection.normal = normalize(cross(b - a, c - a));
            }
        }
    }

    if (closestT == FLT_MAX) {
        intersection.t = -1;
    }

    return intersection;
}
