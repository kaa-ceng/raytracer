#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "parser.h"
#include "ppm.h"
#include "helpers.h"


void render(const parser::Camera& camera, parser::Scene& scene)
{
    int width = camera.image_width;
    int height = camera.image_height;
    unsigned char* image = new unsigned char[width * height * 3];

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            Ray ray = generateRay(x, y, camera, width, height);
            
            parser::Vec3f initColor = computeColor(ray, scene);
            parser::Vec3i color = clampColor(initColor);

            int index = (y * width + x) * 3;
            image[index] = color.x;
            image[index + 1] = color.y;
            image[index + 2] = color.z;
        }
    }

    write_ppm(camera.image_name.c_str(), image, width, height);
    delete[] image;
}



int main(int argc, char* argv[]) {
    
    parser::Scene scene;
    scene.loadFromXml(argv[1]);

    for (const parser::Camera camera : scene.cameras)
    {
        render(camera, scene);
    }
    

    return 0;
}
