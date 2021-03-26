#define _USE_MATH_DEFINES
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "geometry.h"

#include "model.h"

#include <stdio.h>
#include <limits>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <iostream>
#include <algorithm>

int envmap_width, envmap_height;
std::vector<Vec3f> envmap;
//Model duck("../duck.obj");

struct Light {
	Light(const Vec3f& p, const float& i) : position(p), intensity(i) {}
	Vec3f position;
	float intensity;
};

struct Material {
	Material(const float &r, const Vec4f &a, const Vec3f &color, const float &spec) : refractive_index(r), albedo(a), diffuse_color(color), specular_exponent(spec) {}
	Material() : refractive_index(1), albedo(1,0,0,0), diffuse_color(), specular_exponent() {}
	float refractive_index;
	Vec4f albedo;
	Vec3f diffuse_color;
	float specular_exponent;
};

Vec3f reflect(const Vec3f& I, const Vec3f& N) {
	return I - N * 2.f * (I * N);
}

Vec3f refract(const Vec3f &I, const Vec3f &N, const float eta_t, const float eta_i=1.f) { // Snell's law
	float cosi = - std::max(-1.f, std::min(1.f, I * N));
	if (cosi<0) return refract(I, -N, eta_i, eta_t); // if the ray comes from the inside the object, swap the air and the media
	float eta = eta_i / eta_t;
	float k = 1 - eta*eta*(1 - cosi*cosi);
	return k<0 ? Vec3f(1,0,0) : I*eta + N*(eta*cosi - sqrtf(k)); // k<0 = total reflection, no ray to refract. I refract it anyways, this has no physical meaning
}

struct Sphere {
	Vec3f center;
	float radius;
	Material material;

	Sphere(const Vec3f& c, const float& r, const Material m) : center(c), radius(r), material(m) {}
	
	// функция определяет пересекается ли луч, вышедший из orig
	// в направлении dir c данной сферой
	bool ray_intersect(const Vec3f& orig, const Vec3f& dir, float& t0) const {
		// вычисляем проекцию вектора из orig в центр сферы
		Vec3f pr = dir * (dir * (center - orig));
		float h = (center - orig) * (center - orig) - (pr * pr);
		t0 = dir * (center - orig) - sqrtf(radius * radius - h);
		if (t0 < 0) 
			t0 += 2. * sqrtf(radius * radius - h);
		
		if (h > radius * radius)
			return false;

		// точка за сферой
		if (t0 < 0) return false;
		
		return true;
	}
};

bool scene_intersect(const Vec3f& orig, const Vec3f& dir, 
	const std::vector<Sphere>& spheres, 
	Vec3f& hit, Vec3f& N, Material& material) {
	// в этой функции все то же самое, только теперь еще учитываются другие сфферы
	// после цикла наносится материал сферы, у которой было первое пересечение с лучем
	float spheres_dist = std::numeric_limits<float>::max();
	for (size_t i = 0; i < spheres.size(); i++) {
		float dist_i;
		if (spheres[i].ray_intersect(orig, dir, dist_i) && dist_i < spheres_dist) {
			spheres_dist = dist_i;
			hit = orig + dir * dist_i; // точка пересечения со сферой
			N = (hit - spheres[i].center).normalize(); // считаем нормаль
			material = spheres[i].material;
		}
	}

	float checkerboard_dist = std::numeric_limits<float>::max();
  if (std::fabs(dir[1]) > 1e-3)  {
		float d = -(orig[1] + 4) / dir[1]; // the checkerboard plane has equation y = -4
		Vec3f pt = orig + dir * d;
		if (d > 0 && fabs(pt[0]) < 10 && pt[2] < -10 && pt[2] > -30 && d < spheres_dist) {
			checkerboard_dist = d;
			hit = pt;
			N = Vec3f(0,1,0);
			material.diffuse_color = (int(.5 * hit[0] + 1000) + int(.5 * hit[2])) & 1 ? Vec3f(1,1,1) : Vec3f(1, .7, .3);
			material.diffuse_color = material.diffuse_color * .3;
		}
	}

    return std::min(spheres_dist, checkerboard_dist) < 1000;
}


Vec3f cast_ray(const Vec3f& orig, const Vec3f& dir, 
	const std::vector<Sphere>& spheres, const std::vector<Light>& lights, int depth = 0) {
	Vec3f point, N;
	Material material;

	if (depth > 3 || !scene_intersect(orig, dir, spheres, point, N, material)) {
		float x = dir.x, y = dir.y, z = dir.z, r = dir.norm();
		float beta = std::acos(y / r), alpha = std::atan(z / x);

		if (x < 0) {
			alpha += M_PI;
		}

		size_t a = size_t((1 + alpha  / M_PI) * envmap_width / 2);
		size_t b = size_t((beta  / M_PI) * envmap_height);

		return envmap[a + b * envmap_width];
		

		//return Vec3f(0.2, 0.7, 0.8); // задник
	}

	Vec3f reflect_dir = reflect(dir, N).normalize();
	Vec3f refract_dir = refract(dir, N, material.refractive_index).normalize();
Vec3f reflect_orig = reflect_dir * N < 0 ? point - N*1e-3 : point + N*1e-3; // offset the original point to avoid occlusion by the object itself
	Vec3f refract_orig = refract_dir * N < 0 ? point - N*1e-3 : point + N*1e-3;
	Vec3f reflect_color = cast_ray(reflect_orig, reflect_dir, spheres, lights, depth + 1);
	Vec3f refract_color = cast_ray(refract_orig, refract_dir, spheres, lights, depth + 1);
	
		float diffuse_light_intensity = 0, specular_light_intensity = 0;
	for (size_t i = 0; i < lights.size(); i++) {
		Vec3f light_dir = (lights[i].position - point).normalize();
		float light_distance = (lights[i].position - point).norm();
		
		Vec3f shadow_orig = light_dir * N < 0 ? point - N*1e-3 : point + N*1e-3; // checking if the point lies in the shadow of the lights[i]
		Vec3f shadow_pt, shadow_N;
		Material tmpmaterial;

		if (scene_intersect(shadow_orig, light_dir, spheres, shadow_pt, shadow_N, tmpmaterial) && light_distance > (shadow_pt - shadow_orig).norm())
			continue;
		
		diffuse_light_intensity += lights[i].intensity * std::max(0.f, (light_dir * N));
		specular_light_intensity += powf(std::max(0.f, (reflect(light_dir, N) * dir)), material.specular_exponent) * lights[i].intensity;
	}
	return material.diffuse_color * diffuse_light_intensity * material.albedo[0] + 
		Vec3f(1., 1., 1.) * specular_light_intensity * material.albedo[1] + 
		reflect_color * material.albedo[2] + refract_color * material.albedo[3];
}

void to_ppm(const std::string& filename, std::vector<Vec3f>& data, 
		size_t height, size_t width) {
	std::ofstream file;
	file.open(filename);


	file << "P6\n" << width << " " << height << "\n255\n";
	for (size_t i = 0; i < height*width; ++i) {
		Vec3f& c = data[i];
		float max = std::max(c[0], std::max(c[1], c[2]));
		if (max > 1) c = c * (1. / max);
		for (size_t j = 0; j < 3; j++) {
			file << (char)(255 * std::max(0.2f, std::min(1.f, data[i][j])));
		}
	}

	file.close();
}

void to_jpg(const std::string filename, std::vector<Vec3f>& data,
	size_t height, size_t width) {
	std::vector<unsigned char> pixmap(width * height * 3);
	
	for (int i = 0; i < width * height; ++i) {
		Vec3f &c = data[i];
		float max = std::max(c[0], std::max(c[1], c[2]));
		if (max > 1) c = c * (1. / max);
		for (size_t j = 0; j<3; j++) {
			pixmap[i * 3 + j] = (unsigned char)(255 * std::max(0.f, std::min(1.f, data[i][j])));
		}
	}									
	
	stbi_write_jpg(filename.c_str(), width, height, 3, pixmap.data(), 100);
}

void render(const std::vector<Sphere>& spheres, const std::vector<Light>& lights) {
	const int width = 1920;
	const int height = 1080;
	const int fov = M_PI / 2.;

	std::vector<Vec3f> framebuffer(width * height);

	#pragma omp parallel for // параллельное выполнение https://docs.microsoft.com/ru-ru/cpp/parallel/openmp/2-directives?view=msvc-160#23-parallel-construct
	for (size_t j = 0; j < height; j++) {
		for (size_t i = 0; i < width; i++) {
			framebuffer[i + j * width] = Vec3f(j / float(height), i / float(width), 0);
			float x = (2 * (i - 0.5) / (float)width - 1) * tan(fov / 2.) * width / (float)height;
			float y = -(2 * (j - 0.5) / (float)height - 1) * tan(fov / 2.);
			Vec3f dir = Vec3f(x, y, -1).normalize();
			framebuffer[i + j * width] = cast_ray(Vec3f(0., 0., 0.), dir, spheres, lights);
		}
	}

	//to_ppm("new_render.ppm", framebuffer, height, width);
	to_jpg("render.jpg", framebuffer, height, width);
}

int main() {
	int n = -1;
	unsigned char *pixmap = stbi_load("./envmap.jpg", &envmap_width, &envmap_height, &n, 0);
	if (!pixmap || 3 != n) {
		std::cerr << "Error: can not load the environment map" << std::endl;
		return -1;
	}
	envmap = std::vector<Vec3f>(envmap_width * envmap_height);
	for (int j = envmap_height-1; j>=0 ; j--) {
		for (int i = 0; i<envmap_width; i++) {
				envmap[i+j*envmap_width] = Vec3f(pixmap[(i+j*envmap_width)*3+0], pixmap[(i+j*envmap_width)*3+1], pixmap[(i+j*envmap_width)*3+2]) * (1/255.);
		}
	}
	stbi_image_free(pixmap);

	Material ivory(1.0, Vec4f(0.6,  0.3, 0.1, 0.0), Vec3f(0.4, 0.4, 0.3),   50.);
	Material glass(1.5, Vec4f(0.0,  0.5, 0.1, 0.8), Vec3f(0.6, 0.7, 0.8),  125.);
	Material red_rubber(1.0, Vec4f(0.9,  0.1, 0.0, 0.0), Vec3f(0.3, 0.1, 0.1),   10.);
	Material mirror(1.0, Vec4f(0.0, 10.0, 0.8, 0.0), Vec3f(1.0, 1.0, 1.0), 1425.);
	
	std::vector<Light>  lights;
	lights.push_back(Light(Vec3f(-20, 20, 20), 1.5));
	lights.push_back(Light(Vec3f(30, 50, -25), 1.8));
	lights.push_back(Light(Vec3f(30, 20, 30), 1.7));

	std::vector<Sphere> spheres;
	spheres.push_back(Sphere(Vec3f(-3,    0,   -16), 2,      ivory));
	spheres.push_back(Sphere(Vec3f(-1.0, -1.5, -12), 2,      glass));
	spheres.push_back(Sphere(Vec3f( 1.5, -0.5, -18), 3, red_rubber));
	spheres.push_back(Sphere(Vec3f( 7,    5,   -18), 4,     mirror));
	
	render(spheres, lights);
	return 0;
}
