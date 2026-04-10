#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <complex>
#include "parser.h"
#include "ppm.h"


// Vector Class & Operations Part
struct Vec3
{
    float x, y, z;

    Vec3 () : x(0), y(0), z(0) {}
    Vec3 (float val) : x(val), y(val), z(val) {}
    Vec3 (float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 (const parser::Vec3f& val) : x(val.x), y(val.y), z(val.z) {}
    Vec3 (const parser::Vec3i& val) : x((float)val.x), y((float)val.y), z((float)val.z) {}

    Vec3 operator+ (const Vec3& other) const { return {x + other.x, y + other.y, z + other.z}; }
    Vec3 operator- (const Vec3& other) const { return {x - other.x, y - other.y, z - other.z}; }
    Vec3 operator* (const Vec3& other) const { return {x * other.x, y * other.y, z * other.z}; }
    Vec3 operator* (const float val) const { return {x * val, y * val, z * val}; }
    Vec3 operator/ (const float val) const { return {x / val, y / val, z / val}; }
    Vec3 operator-() const { return {-x, -y, -z}; }

    float length() const { return std::sqrt(x * x + y * y + z * z); }
    float length_squared() const { return x * x + y * y + z * z; }
};

inline Vec3 operator*(float val, const Vec3& vec) { return vec * val; }
inline Vec3 normalize (const Vec3& vec) {
    float len = vec.length();
    if (len < 1e-8f) {return Vec3(0.0f, 0.0f, 0.0f); }
    return vec / len;
}
inline float dot_product (const Vec3& vec1, const Vec3& vec2) { return vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z; }
inline Vec3 cross_product (const Vec3& vec1, const Vec3& vec2)
{
    return {
        vec1.y * vec2.z - vec1.z * vec2.y,
        vec1.z * vec2.x - vec1.x * vec2.z,
        vec1.x * vec2.y - vec1.y * vec2.x};
}

// Ray Class & Ops
struct Ray {
    Vec3 origin;
    Vec3 direction;
};

struct HitInfo {
    bool is_hit;
    float t;
    int material_id;
    Vec3 hit_point;
    Vec3 normal;
};

Ray generate_ray(const parser::Camera& camera, int x, int y) {
    Vec3 eye(camera.position);
    Vec3 w = -1.0f * Vec3(camera.gaze);
    Vec3 v = Vec3(camera.up);
    Vec3 u = cross_product(v, w);

    float left = camera.near_plane.x;
    float right = camera.near_plane.y;
    float bottom = camera.near_plane.z;
    float top = camera.near_plane.w;
    float distance = camera.near_distance;

    int imageWidth = camera.image_width;
    int imageHeight = camera.image_height;

    Vec3 m = eye - (w * distance);
    Vec3 q = m + (u * left) + (v * top);

    float s_u = (right - left) * (x + 0.5f) / imageWidth;
    float s_v = (top - bottom) * (y + 0.5f) / imageHeight;

    Vec3 s = q + (u * s_u) - (v * s_v);

    Ray ray;
    ray.origin = eye;
    ray.direction = normalize(s-eye);
    return ray;
}

// Intersections
// Sphere
float intersect_sphere(const Ray& ray, const parser::Sphere& sphere, const std::vector<parser::Vec3f>& vertex_data) {
    Vec3 center = Vec3(vertex_data[sphere.center_vertex_id - 1]);
    float radius = sphere.radius;

    Vec3 c_to_o = ray.origin - center;

    float a = dot_product(ray.direction, ray.direction);
    float b = 2.0f * dot_product(ray.direction, c_to_o);
    float c = dot_product(c_to_o, c_to_o) - radius * radius;

    float delta = b * b - 4 * a * c;
    if (delta < 0.0f) { return -1.0f; }
    float sqrt_delta = std::sqrt(delta);
    float t1 = (-b - sqrt_delta) / (2.0f * a);
    float t2 = (-b + sqrt_delta) / (2.0f * a);

    const float epsilon = 1e-7f;
    if (t1 > epsilon) { return t1; }
    if (t2 > epsilon) { return t2; }
    return -1.0f;
}

// Plane
float intersect_plane(const Ray& ray, const parser::Plane& plane, const std::vector<parser::Vec3f>& vertex_data) {
    Vec3 normal = normalize(Vec3(plane.normal));
    Vec3 center = Vec3(vertex_data[plane.center_vertex_id - 1]);

    Vec3 origin = ray.origin;
    Vec3 direction = ray.direction;

    float dot_d_n = dot_product(direction, normal);

    const float epsilon = 1e-7f;
    if (std::fabs(dot_d_n) < epsilon)
        return -1.0f;

    float distance = dot_product(center - origin, normal) / dot_d_n;

    if (distance > epsilon)
        return distance;
    return -1.0f;
}

// Triangle
float intersect_triangle(const Ray& ray, const parser::Face& face, const std::vector<parser::Vec3f>& vertex_data) {
    const float epsilon = 1e-7f;

    Vec3 v0 = Vec3(vertex_data[face.v0_id - 1]);
    Vec3 v1 = Vec3(vertex_data[face.v1_id - 1]);
    Vec3 v2 = Vec3(vertex_data[face.v2_id - 1]);

    Vec3 edge1 = v1 - v0;
    Vec3 edge2 = v2 - v0;

    Vec3 h = cross_product(ray.direction, edge2);
    float a = dot_product(edge1, h);

    if (std::fabs(a) < epsilon) { return -1.0f; }

    float f = 1.0f / a;

    Vec3 s = ray.origin - v0;
    float u = dot_product(s, h) * f;
    if (u < 0.0f || u > 1.0f) { return -1.0f; }

    Vec3 q = cross_product(s, edge1);
    float v = dot_product(ray.direction, q) * f;
    if (v < 0.0f || u + v > 1.0f) { return -1.0f; }

    float t = dot_product(edge2, q) * f;

    if (t > epsilon) { return t; }

    return -1.0f;
}

// Cylinder
float intersect_cylinder(const Ray& ray, const parser::Cylinder& cylinder, const std::vector<parser::Vec3f>& vertex_data) {
    Vec3 center = Vec3(vertex_data[cylinder.center_vertex_id - 1]);
    Vec3 axis = normalize(Vec3(cylinder.axis));
    float radius = cylinder.radius;
    float height = cylinder.height;

    Vec3 origin = ray.origin;
    Vec3 direction = ray.direction;

    const float epsilon = 1e-7f;
    float min_t = -1.0f;

    Vec3 c_to_o = origin - center;

    float direct_dot_axis = dot_product(direction, axis);
    float ctoo_dot_axis = dot_product(c_to_o, axis);

    float a = dot_product(direction, direction) - direct_dot_axis * direct_dot_axis;
    float b = 2.0f * (dot_product(direction, c_to_o) - direct_dot_axis * ctoo_dot_axis);
    float c = dot_product(c_to_o, c_to_o) - ctoo_dot_axis * ctoo_dot_axis - radius * radius;

    if (std::fabs(a) > epsilon) {
        float delta = b * b - 4.0f * a * c;
        if (delta >= 0.0f) {
            float sqrt_delta = std::sqrt(delta);
            float t_1 = (-b - sqrt_delta) / (2.0f * a);
            float t_2 = (-b + sqrt_delta) / (2.0f * a);

            if (t_1 > epsilon) {
                Vec3 p1 = origin + t_1 * direction;
                float axial1 = dot_product(p1 - center, axis);
                if (std::fabs(axial1) <= height * 0.5f) {
                    if (min_t < 0 || t_1 < min_t) { min_t = t_1; }
                }
            }
            if (t_2 > epsilon) {
                Vec3 p2 = origin + t_2 * direction;
                float axial2 = dot_product(p2 - center, axis);
                if (std::fabs(axial2) <= height * 0.5f) {
                    if (min_t < 0 || t_2 < min_t) { min_t = t_2; }
                }
            }
        }
    }

    if (std::fabs(direct_dot_axis) > epsilon) {
        Vec3 top_center = center + axis * height * 0.5f;
        Vec3 bottom_center = center - axis * height * 0.5f;

        float t_top = dot_product(top_center - origin, axis) / direct_dot_axis;
        if (t_top > epsilon) {
            Vec3 top_cap = origin + t_top * direction;
            if ((top_cap - top_center).length_squared() <= radius * radius) {
                if (min_t < 0 || t_top < min_t) { min_t = t_top; }
            }
        }

        float t_bottom = dot_product(bottom_center - origin, axis) / direct_dot_axis;
        if (t_bottom > epsilon) {
            Vec3 bottom_cap = origin + t_bottom * direction;
            if ((bottom_cap - bottom_center).length_squared() <= radius * radius) {
                if (min_t < 0 || t_bottom < min_t) { min_t = t_bottom; }
            }
        }
    }
    return min_t;
}

Vec3 compute_cylinder_normal (const Vec3& hit_point, const parser::Cylinder& cylinder, const std::vector<parser::Vec3f>& vertex_data) {
    Vec3 center = Vec3(vertex_data[cylinder.center_vertex_id - 1]);
    Vec3 axis = normalize(Vec3(cylinder.axis));
    float half_height = cylinder.height / 2.0f;

    const float epsilon = 1e-4f;

    float axial_distance = dot_product(hit_point - center, axis);

    if (std::fabs(axial_distance - half_height) < epsilon) { return axis; }

    if (std::fabs(axial_distance + half_height) < epsilon) { return -axis; }

    Vec3 p_minus_c = hit_point - center;
    Vec3 side_normal = normalize(p_minus_c - dot_product(p_minus_c, axis) * axis);
    return side_normal;
}

bool is_in_shadow (const Ray& shadow_ray, const parser::Scene& scene, float distance_to_light) {
    const float MIN_HIT_DISTANCE = 1e-6f;

    // for spheres
    for (const auto& sphere : scene.spheres) {
        float t = intersect_sphere(shadow_ray, sphere, scene.vertex_data);
        if (t > MIN_HIT_DISTANCE  && t < distance_to_light) { return true; }
    }
    // for planes
    for (const auto& plane : scene.planes) {
        float t = intersect_plane(shadow_ray, plane, scene.vertex_data);
        if (t > MIN_HIT_DISTANCE  && t < distance_to_light) { return true; }
    }
    // for triangle
    for (const auto& triangle : scene.triangles) {
        float t = intersect_triangle(shadow_ray, triangle.indices, scene.vertex_data);
        if (t > MIN_HIT_DISTANCE  && t < distance_to_light) { return true; }
    }
    // for meshes
    for (const auto& mesh : scene.meshes) {
        for (const auto& face : mesh.faces) {
            float t = intersect_triangle(shadow_ray, face, scene.vertex_data);
            if (t > MIN_HIT_DISTANCE  && t < distance_to_light) { return true; }
        }
    }
    // for cylinders
    for (const auto& cylinder : scene.cylinders) {
        float t = intersect_cylinder(shadow_ray, cylinder, scene.vertex_data);
        if (t > MIN_HIT_DISTANCE  && t < distance_to_light) { return true; }
    }
    return false;
}

HitInfo find_closest_hit (const Ray& ray, const parser::Scene& scene) {
    HitInfo closest_hit;
    closest_hit.is_hit = false;
    closest_hit.t = -1.0f;

        // for spheres
        for (const auto& sphere : scene.spheres) {
            float t = intersect_sphere(ray, sphere, scene.vertex_data);
            if (t > 0 && (!closest_hit.is_hit || t < closest_hit.t)) {
                closest_hit.is_hit = true;
                closest_hit.t = t;
                closest_hit.material_id = sphere.material_id;
                closest_hit.hit_point = ray.origin + t * ray.direction;
                Vec3 center = Vec3(scene.vertex_data[sphere.center_vertex_id - 1]);
                closest_hit.normal = normalize(closest_hit.hit_point - center);
            }
        }
        // for planes
        for (const auto& plane : scene.planes) {
            float t = intersect_plane(ray, plane, scene.vertex_data);
            if (t > 0 && (!closest_hit.is_hit || t < closest_hit.t)) {
                closest_hit.is_hit = true;
                closest_hit.t = t;
                closest_hit.material_id = plane.material_id;
                closest_hit.hit_point = ray.origin + t * ray.direction;
                closest_hit.normal = normalize(Vec3(plane.normal));
                if (dot_product(ray.direction, closest_hit.normal) > 0) { closest_hit.normal = -closest_hit.normal; }
            }
        }
        // for triangles
        for (const auto& triangle : scene.triangles) {
            float t = intersect_triangle(ray, triangle.indices, scene.vertex_data);
            if (t > 0 && (!closest_hit.is_hit || t < closest_hit.t)) {
                closest_hit.is_hit = true;
                closest_hit.t = t;
                closest_hit.material_id = triangle.material_id;
                closest_hit.hit_point = ray.origin + t * ray.direction;
                Vec3 v0 = Vec3(scene.vertex_data[triangle.indices.v0_id - 1]);
                Vec3 v1 = Vec3(scene.vertex_data[triangle.indices.v1_id - 1]);
                Vec3 v2 = Vec3(scene.vertex_data[triangle.indices.v2_id - 1]);
                closest_hit.normal = normalize(cross_product(v1 - v0, v2 - v0));
            }
        }
        // for meshes
        for (const auto& mesh : scene.meshes) {
            for (const auto& face : mesh.faces) {
                float t = intersect_triangle(ray, face, scene.vertex_data);
                if (t > 0 && (!closest_hit.is_hit || t < closest_hit.t)) {
                    closest_hit.is_hit = true;
                    closest_hit.t = t;
                    closest_hit.material_id = mesh.material_id;
                    closest_hit.hit_point = ray.origin + ray.direction * t;
                    Vec3 v0 = Vec3(scene.vertex_data[face.v0_id - 1]);
                    Vec3 v1 = Vec3(scene.vertex_data[face.v1_id - 1]);
                    Vec3 v2 = Vec3(scene.vertex_data[face.v2_id - 1]);
                    closest_hit.normal = normalize(cross_product(v1 - v0, v2 - v0));
                }
            }
        }
        // for cylinders
        for (const auto& cylinder : scene.cylinders) {
            float t = intersect_cylinder(ray, cylinder, scene.vertex_data);
            if (t > 0 && (!closest_hit.is_hit || t < closest_hit.t)) {
                closest_hit.is_hit = true;
                closest_hit.t = t;
                closest_hit.material_id = cylinder.material_id;
                closest_hit.hit_point = ray.origin + t * ray.direction;
                closest_hit.normal = compute_cylinder_normal(closest_hit.hit_point, cylinder, scene.vertex_data);
            }
        }
    return closest_hit;
}

Vec3 compute_color (const Ray& ray, const parser::Scene& scene, int recursion_depth) {
    HitInfo hit = find_closest_hit(ray, scene);
    if (!hit.is_hit) {
        if (recursion_depth == 0)
        {
            return Vec3(scene.background_color);
        }
        else {
            return Vec3(0.0f, 0.0f, 0.0f);
        }
    }


    const parser::Material& material = scene.materials[hit.material_id - 1];
    Vec3 hit_point = hit.hit_point;
    Vec3 normi = hit.normal;
    float ray_dot_normal = dot_product(ray.direction, normi);

    if (ray_dot_normal > 0.0f) {
        normi = -normi;
    }

    Vec3 total_color = Vec3(material.ambient) * Vec3(scene.ambient_light);

    for (const auto& light : scene.point_lights) {
        Vec3 light_position = Vec3(light.position);
        Vec3 h_to_l = normalize(light_position - hit_point);

        float distance = (light_position - hit_point).length();

        Vec3 shadow_ray_origin = hit_point + normi * scene.shadow_ray_epsilon;
        Ray shadow_ray = {shadow_ray_origin, h_to_l};

        bool is_shadowed = is_in_shadow(shadow_ray,scene, distance);
        if (is_shadowed) { continue; }

        float distance_squared = std::max(1.0f, distance * distance);
        Vec3 irradiance = Vec3(light.intensity) / distance_squared;

        float cos_theta =std::max(0.0f, dot_product(h_to_l, normi));
        Vec3 diffuse_color = Vec3(material.diffuse) * irradiance * cos_theta;

        total_color = total_color + diffuse_color;

        Vec3 view = normalize(ray.origin - hit_point);
        Vec3 halfway = normalize(view + h_to_l);

        float cos_alpha = std::max(0.0f, dot_product(halfway, normi));
        float phong_exp = material.phong_exponent;

        Vec3 specular_color = Vec3(material.specular) * irradiance * std::pow(cos_alpha, phong_exp);

        total_color = total_color + specular_color;
    }

    Vec3 mirror_reflectance = Vec3(material.mirror);
    bool is_mirror = material.is_mirror && (mirror_reflectance.x > 0 || mirror_reflectance.y > 0 || mirror_reflectance.z > 0);
    if (is_mirror && recursion_depth < scene.max_recursion_depth) {
        Vec3 direction = ray.direction;
        Vec3 reflection = normalize(direction - 2.0f * dot_product(direction, normi) * normi);

        Vec3 reflection_ray_origin = hit_point + normi * scene.shadow_ray_epsilon;
        Ray reflection_ray = {reflection_ray_origin, reflection};

        Vec3 reflected_color = compute_color(reflection_ray, scene, recursion_depth + 1);

        total_color = total_color + (reflected_color * mirror_reflectance);
    }
    return total_color;
}

int main(int argc, char* argv[])
{
    // Sample usage for reading an XML scene file
    parser::Scene scene;

    scene.loadFromXml(argv[1]);

    for (const auto& camera : scene.cameras)
    {
        int width = camera.image_width;
        int height = camera.image_height;

        unsigned char* image = new unsigned char[width * height * 3];

#pragma omp parallel for
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int pixelIndex = (y * width + x) * 3;

                Ray ray = generate_ray(camera, x, y);

                Vec3 color = compute_color(ray, scene, 0);

                auto to_uc = [](float v)->unsigned char {
                    v = std::min(255.0f, std::max(0.0f, v));
                    return (unsigned char)std::round(v);
                };

                image[pixelIndex]     = to_uc(color.x);
                image[pixelIndex + 1] = to_uc(color.y);
                image[pixelIndex + 2] = to_uc(color.z);
            }
        }
        write_ppm(camera.image_name.c_str(), image, width, height);
        delete[] image;
    }
    return 0;
}
