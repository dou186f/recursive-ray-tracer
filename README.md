# Recursive Ray Tracer

A CPU-based recursive ray tracer written in C++. Built as part of CENG 477 - Introduction to Computer Graphics at METU.

## Overview

Given an XML scene, the program renders PPM images using Blinn-Phong shading, shadow rays, and recursive mirror reflections. The project uses a course-provided scaffold for XML parsing and PPM output; the ray tracing logic is implemented in `raytracer.cpp`.

## Features

- **Intersections** - Spheres, triangles, meshes, infinite planes, and capped cylinders
- **Shading** - Ambient, diffuse, and specular components using the Blinn-Phong model
- **Shadows** - Point light occlusion with an epsilon offset
- **Reflections** - Recursive mirror reflections with a configurable maximum depth
- **Multiple cameras** - A separate output image for each camera
- **Lighting** - Ambient light and multiple point lights with distance attenuation

## Build & Run

Requires `g++` and GNU Make.

```bash
git clone https://github.com/dou186f/recursive-ray-tracer.git
cd recursive-ray-tracer
make
./raytracer test_scenes/inputs/simple.xml
```

This example produces `simple.ppm` in the current working directory. Output filenames are defined by each camera's `ImageName` field.

To convert the output to PNG with ImageMagick:

```bash
magick simple.ppm simple.png
```

## Example Scenes and Outputs

XML scenes are in [`test_scenes/inputs`](test_scenes/inputs), and supplied PPM outputs are in [`test_scenes/outputs`](test_scenes/outputs).

| Scene | Input |
|---|---|
| Simple geometry | [simple.xml](test_scenes/inputs/simple.xml) |
| Shading | [simple_shading.xml](test_scenes/inputs/simple_shading.xml) |
| Reflectance | [simple_reflectance.xml](test_scenes/inputs/simple_reflectance.xml) |
| Mirror spheres | [mirror_spheres.xml](test_scenes/inputs/mirror_spheres.xml) |
| Cornell box | [cornellbox.xml](test_scenes/inputs/cornellbox.xml) |
| Cylinders | [cylinders.xml](test_scenes/inputs/cylinders.xml) |
| Gym | [gym.xml](test_scenes/inputs/gym.xml) |

## Tech Stack

- **Language:** C++11
- **Build:** GNU Make and g++
- **XML Parsing:** tinyxml2, included in the repository
- **Output Format:** PPM (ASCII P3)

## Author

**Doğu Erbaş**  
doguerbass@gmail.com  
METU Computer Engineering - CENG 477
