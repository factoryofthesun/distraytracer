//////////////////////////////////////////////////////////////////////////////////
// This is a front end for a set of viewer clases for the Carnegie Mellon
// Motion Capture Database:
//
//    http://mocap.cs.cmu.edu/
//
// The original viewer code was downloaded from:
//
//   http://graphics.cs.cmu.edu/software/mocapPlayer.zip
//
// where it is credited to James McCann (Adobe), Jernej Barbic (USC),
// and Yili Zhao (USC). There are also comments in it that suggest
// and Alla Safonova (UPenn) and Kiran Bhat (ILM) also had a hand in writing it.
//
//////////////////////////////////////////////////////////////////////////////////
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <iostream>
#include <float.h>
#include <random>
#include <unordered_map>
#include <time.h>
#include <memory>
#include <algorithm>
#include <functional>
// #include "SETTINGS.h"
// #include "geometry.h"
// #include "skeleton.h"
// #include "displaySkeleton.h"
#include "motion.h"
#include "dataStructs.h"
#include "objHelper.h"
#include "noise.h"
#include "helpers.h"
#include "scene.h"

// #include "gltf_helper.h"

using namespace std;

// Stick-man classes
DisplaySkeleton displayer;
Skeleton* skeleton;
Motion* motion;

int xRes = 1920;
int yRes = 1080;

// Camera settings
VEC3 eye(-6, 0.5, 1);
VEC3 lookingAt(0.5, 0.5, 1);
VEC3 up(0,1,0);
float aspect = (float) xRes/(float) yRes;
float near = 1; // distance to image plane
float fov = 45.0;
float aperture = 0.2; // diameter of sampling lens
float focal_length = 10; // distance to focal plane
bool use_model = true;
bool nogloss = false;

// Reflection stuff
vector<string> refl_materials = {"glass", "steel", "aluminum", "water", "linoleum"};
float refr_air = 1;
float refr_glass = 1.5;
int max_depth = 10;

// scene geometry
vector<shared_ptr<GeoPrimitive>> shapes;
vector<shared_ptr<LightPrimitive>> lights;
float phong = 10;
VEC3 default_col(0,0,0);

// BVH settings
shared_ptr<BoundingVolume> bvh;
float c_isect = 1; // Random assumption: traversal is 1/3 cost of ray-primitive intersection
float c_trav = 0.33;

// Sampling settings
int antialias_samples = 10;
int brdf_samples = 2; // For reflection
int blur_samples = 2;
int frame_range = 1;
random_device rd;
mt19937 generator(rd());
uniform_real_distribution<double> uniform(0.0, 1.0);

// Textures
vector<string> ad_paths = {"./ads/coke1",
                           "./ads/coke2",
                           "./ads/jap_ads",
                           "./ads/Shiseido",
                         "./ads/us_ads"};
vector<string> frame_paths; // Pathnames for all frame files
vector<vector<VEC3>> texture_frames;
vector<VEC2> texture_dims;

// Colors
VEC3 teal(0.27, 1, 0.88);
VEC3 pink(1, 0.44, 0.81);
VEC3 pastelpink(1, 0.82, 0.863);
VEC3 halogen(1, 0.756, 0.518);
VEC3 sunorange(0.953, 0.51, 0.21);
VEC3 violet(0.541, 0.168, 0.886);
VEC3 indigo(75.0/255, 0, 130.0/255);
VEC3 darkblue(0.0667, 0.1137, 0.37);
VEC3 maroonq(199.0/255, 0, 57.0/255);
VEC3 skyblue(178.0/255, 222.0/255, 236.0/255);

// Scene choreography
int frame_prism = 960; // 120 * 8 frames after is when we start pulling up
int frame_cloud = 1952;
int frame_blur = 1600;
int frame_start = 120;
int frame_move1 = 480;
int frame_move2 = 960;
int frame_sculp = 600;
int total = 2400;
float far = 200; // 100 should be okay?
float move_per_frame = 0.1/8;
float tot_move = 0;
float accel_t = 80/pow(360,3); // factor by which acceleration contributes to distance * t^3
VEC3 cap_center = VEC3(0,0,0);

// Cloud parameters
VEC3 sundir(0, 0.1, -1);
bool perlin_cloud = false; // Indicator for creating perlin cloud default colors
float saturation = 0.2;
float clouddist = 10;
float cloudhoff = 0.2; // height cutoff
VEC3 sun_outer(0.9, 0.3, 0.9);
VEC3 sun_inner(1.0, 0.7, 0.7);
VEC3 sun_core(1, 1, 1);
VEC3 bluesky(0.3, 0.55, 0.8);
VEC3 redsky(0.8, 0.8, 0.6);

bool reflect = true;

// Functions
void rayColor(const VEC3 ray, const VEC3 eye, int depth, VEC3& color, bool& hit, bool& in_motion, const float k = 1);
void rayColorNoBVH(const VEC3 ray, const VEC3 eye, int depth, VEC3& color, bool& hit, bool& in_motion, const float k = 1);
void renderImage(const string& filename, const int frame);

// ============== PERLIN CLOUD FUNCTIONS ================
void skyColor(VEC3 ray, VEC3& color)
{
	color = VEC3(0, 0, 0);
  VEC3 rnorm = ray.normalized();

	// Simulate sun (note: sun vector is direction sun is from us)
  // SUN SETTINGS: in same gaze direction as FINAL PROJECT ENDING
  // VEC3 sun = ((lookingAt - eye) + up * 0.1).normalized();
  VEC3 sun = sundir.normalized();
	float sundot = clamp(rnorm.dot(sun));
	color += 0.05 * sun_outer * pow(sundot, 1.0) + 0.1 * sun_inner * pow(sundot, 2.0) +
           0.9 * sun_core * pow(sundot, 256.0);

	// Simulate sky
	VEC3 sky = bluesky * (1 - 1.5 * pow(sundot, 8)) + redsky * 1.5 * pow(sundot, 8);
	color += sky * (1.0 - 0.8 * rnorm[1]);
}

VEC3 cloudColor(VEC3 ray, VEC3 origin, float frame)
{
  VEC3 color;
  VEC3 skycolor;
  skyColor(ray, skycolor);
  color = skycolor;

  // Average over distance
  for (float z = clouddist; z > 0; z -= 0.05)
  {
    VEC3 p = origin + z * ray;
    float noise = 0.7 * ValueNoise_3D(p[0], p[1], p[2] + frame);
    float clouddistance = p[1] + noise + cloudhoff; // Distance in terms of negative y
    if (clouddistance < 0)
    {
      float density = clamp(abs(clouddistance));
      VEC3 skycol_rev(skycolor[2], skycolor[1], skycolor[0]);
      VEC3 cloudcolor = VEC3(1,1,1) - density * skycol_rev;
      color = (1 - density * 0.4) * color + density * 0.4 * cloudcolor;
    }
  }
  // Apply contrast
  color = VEC3(clamp(color[0]), clamp(color[1]), clamp(color[2]));
  color = 3 * color.array().pow(2) - 2 * color.array().pow(3);

  // Saturation
  color = (1 + saturation) * color - saturation * VEC3(0.33 * color.sum(),0.33 * color.sum(), 0.33 * color.sum());
  return color;
}

// =========== SAMPLING ALGORITHMS ====================
void getDOFSamples(vector<VEC3>& sample_vec, const VEC3 eye, const VEC3 cameraX, const VEC3 cameraY, const int n)
{
  for (int i = 0; i < n; i++)
  {
    if (aperture > 0)
    {
      float r = aperture/2 * uniform(generator);
      float theta = 2 * M_PI * uniform(generator);
      sample_vec.push_back(eye + r * cos(theta) * cameraX + r * sin(theta) * cameraY);
    }
    else
    {
      sample_vec.push_back(eye);
    }
  }
}

// Deprecated version without BVH
void rayColorNoBVH(const VEC3 ray, const VEC3 eye, int depth, VEC3& color, bool& hit, bool& in_motion, const float k)
{
  if (depth == 0) return;
  // CHECK SHAPE INTERSECTIONS
  float t_dist = FLT_MAX;
  float t_min = FLT_MAX;
  VEC3 shape_color;
  int shape_dim;
  string model;
  VEC3 center;
  float radius;
  // For reflection
  float roughness;
  VEC2 refr;
  bool inside;
  // Object-specific
  bool hit_light = false;
  bool any_intersect = false;
  in_motion = false;
  // Ray tracing basics
  VEC3 ray_color(0,0,0);
  VEC3 isectP;
  VEC3 normal;
  // Save shape
  shared_ptr<GeoPrimitive> hit_shape;
  for (shared_ptr<GeoPrimitive> shape : shapes)
  {
    bool inside_tmp;
    bool intersect = shape->intersect(ray, eye, t_dist, inside_tmp);
    if (intersect == true)
    {
      any_intersect = true;
      hit = true;
      if (t_dist < t_min)
      {
        hit_shape = shape;
        inside = inside_tmp;
        t_min = t_dist;
      }
    }
  }

  // KEY SHADING PARAMETERS
  isectP = eye + t_dist * ray;
  normal = hit_shape->getNorm(isectP);
  VEC3 in = isectP - eye;
  in = in/in.norm();
  // Make sure normal points towards origin of ray
  fixNorm(in, normal);
  string material = hit_shape->reflect_params.material;

  // Material reflection
  if (reflect == true)
  {
    if (find(refl_materials.begin(), refl_materials.end(), material) != refl_materials.end())
    {
      // Get shape parameters
      shape_dim = hit_shape->dims;
      shape_color = hit_shape->color;
      hit_light = hit_shape->light;
      in_motion = hit_shape->motion;
      model = hit_shape->model;
      roughness = hit_shape->reflect_params.roughness;
      refr = hit_shape->reflect_params.refr;
      bool glossy = hit_shape->reflect_params.glossy;

      // TODO: FIGURE OUT HOW TO INTERPOLATE CLOUD TRANSARENCY WITHOUT DOING THIS
      center = hit_shape->center;
      radius = hit_shape->radius;

      // Compute reflection first
      VEC3 refl_ray = in - 2 * (normal.dot(in)) * normal;

      // Materials with reflection + refraction (e.g. glass)
      if (material == "glass")
      {
        // Refraction
        // Norm changes direction depending on whether going in or out
        float cos_theta;
        float sin_theta;
        float cos_phi;
        VEC3 adj_org;
        VEC3 out;
        bool refr;
        // Compute sin of angle (sin = sqrt(1 - cos^2))
        cos_theta = normal.dot(-in); // Need to reverse in-ray to get this right
        sin_theta = sqrt(1 - pow(cos_theta, 2));

        // Compute refraction ray -- returns false if total internal reflection
        refr = getRefractionRay(out, in, normal, sin_theta, cos_theta, refr_glass, refr_air);
        if (refr == true)
        {
          // Adjust in different directions depending on whether going in or going out
          float eps = 1e-3;
          adj_org = isectP + ray * eps;
          cos_phi = sqrt(1 - pow(refr_glass/refr_air, 2) * (1 - pow(in.dot(normal), 2)));

          // Fresnel coefficients
          float k_refl;
          float k_refr;
          // Note: refract inds don't matter in this context
          fresnel(cos_theta, cos_phi, refr_air, refr_glass, k_refl, k_refr);

          // Adjust for existing k
          k_refl *= k;
          k_refr *= k;

          rayColor(out, adj_org, depth-1, color, hit, in_motion, k_refr);
          rayColor(refl_ray, isectP + refl_ray*0.1, depth - 1, color, hit, in_motion, k_refl);
        }
      }
      else
      {
        rayColor(refl_ray, isectP + refl_ray*0.1, depth - 1, color, hit, in_motion, k);
      }
    }
  }

  // First check if hit light
  if (hit_light == true)
  {
    // TODO: Radiosity?????
    // Power fade color based on angle b/w ray and hit to center (for sphere lights)
    if (hit_shape->name == "spherelight")
    {
      float hitdot = in.dot((isectP-hit_shape->center).normalized());
      color += k * shape_color * (0.05 * pow(hitdot, 256) + 0.1 * pow(hitdot, 2) + 0.9);
    }
    // Power fade based on avg distance from 4 corners
    if (hit_shape->name == "rectanglelight")
    {
      float dist = ((isectP - hit_shape->A).norm() + (isectP - hit_shape->B).norm() + (isectP - hit_shape->C).norm() +
          (isectP - hit_shape->D).norm())/(4 * (hit_shape->center - hit_shape->A).norm());
      color += k * shape_color * (0.05 * pow(dist, 256) + 0.1 * pow(dist, 2) + 0.9);
    }
  }
  // 3-term lighting
  // TODO: Soft shadows
  else if (any_intersect)
  {
    VEC3 shader_rgb(0,0,0);
    VEC3 e = (eye - isectP).normalized(); // Important: need to REVERSE ray direction back to eye for specular computation
    // For each light: shadow check
    for (shared_ptr<LightPrimitive> light: lights)
    {
      VEC3 sray = light->sampleRay(isectP);
      // cout << "Light center: " << light->center << endl;
      // cout << "Intersection point: " << isectP << endl;
      // cout << "Shadow ray: " << sray << endl;
      // T_max is the distance from the normalized light ray to the light
      float t_max = sray.norm();
      int shadow = 1;
      for (shared_ptr<GeoPrimitive> shape_tmp: shapes)
      {
        // IMPORTANT: skip shape if same object as light!
        shared_ptr<void> light_check = dynamic_pointer_cast<void>(light);
        shared_ptr<void> shape_check = dynamic_pointer_cast<void>(shape_tmp);
        if (light_check == shape_check)
        {
          continue;
        }
        int isect = shape_tmp->intersectShadow(sray.normalized(), isectP, t_max);
        if (isect == 1)
        {
          shadow = 0;
          break;
        }
      }
      if (shadow == 0)
      {
        continue;
      }

      VEC3 r = (-1 * sray + 2 * (normal.dot(sray)) * normal).normalized();
      // Default shading model: lambert diffuse + phong specular
      VEC3 ray_color;
      if (hit_shape->texture == true)
      {
        // TODO: implement getUV function for each primitive! We will deal with rectangles for now
        VEC3 ad = hit_shape->D - hit_shape->A;
        VEC3 dc = hit_shape->C - hit_shape->D;
        float u = (isectP-hit_shape->A).cross(ad).norm()/(ad.norm() * dc.norm()); // "X" distance
        float v = (isectP-hit_shape->D).cross(dc).norm()/(dc.norm() * ad.norm()); // "Y" distance
        int texture_frame = hit_shape->tex_frame;
        VEC2 dims = texture_dims[texture_frame];
        int x_tex = (int(dims[0])-1) * u;
        int y_tex = (int(dims[1])-1) * v;
        int uv_ind = y_tex * dims[0] + x_tex;
        shape_color = texture_frames[texture_frame][uv_ind];
        if (u > 1 || v > 1)
        {
          printf("Out of bounds UV found.\n");
          cout << "Intersection point: " << isectP << endl;
          cout << "Ray: " << ray << endl;
          cout << "Eye: " << eye << endl;
          cout << "A: " << hit_shape->A << endl;
          cout << "B: " << hit_shape->B << endl;
          cout << "C: " << hit_shape->C << endl;
          cout << "D: " << hit_shape->D << endl;
          cout << "Norm: " << normal << endl;
          cout << "Normal check against hit point: " << (isectP - hit_shape->A).dot(normal) << endl;
          cout << "Name: " << hit_shape->name << endl;

          // Why didn't the bounding test catch this?
          VEC3 v_hit = isectP - hit_shape->A;
          VEC3 V1 = hit_shape->B-hit_shape->A;
          VEC3 V2 = hit_shape->D-hit_shape->A;
          float check1 = V1.normalized().dot(v_hit);
          float check2 = V2.normalized().dot(v_hit);
          printf("Check 1: %f, V1 norm: %f\n", check1, V1.norm());
          printf("Check 2: %f, V2 norm: %f\n", check2, V2.norm());
          throw;
        }
      }
      if (model == "oren-nayar")
      {
        float A = 1.0 - (0.5 * pow(roughness, 2)) / (pow(roughness, 2) + 0.33);
        float B = (0.45 * pow(roughness, 2)) / (pow(roughness, 2) + 0.09);
        float vn = e.dot(normal);
        float ln = sray.normalized().dot(normal);

        float irradiance = max(float(0), ln);
        float vn_theta  = acos(vn);
        float ln_theta = acos(ln);

        // max( 0.0 , cos(phi_incident, phi_reflected) )
        float angleDiff = max(0.0, (e - normal * vn).normalized().dot((sray - normal * ln).normalized()));

        float alpha = max(vn_theta, ln_theta);
        float beta  = min(vn_theta, ln_theta);

        ray_color = shape_color.cwiseProduct(light->color) * irradiance * (A +
      	                    B * angleDiff * sin(alpha) * tan(beta));
      }
      else if (model == "cook-torrance")
      {
        VEC3 H = (e + sray).normalized();
        float hn = max(0.0, normal.dot(H));
        float vh = e.dot(H);
        float vn = e.dot(normal);
        float ln = sray.normalized().dot(normal);

        // specular component (Cook-Torrance reflection model)
        // D term (gaussian)
        float alpha = acos(hn);
        float D = 1/(pow(roughness, 2) * pow(cos(alpha), 4)) * exp(- pow(tan(alpha)/roughness, 2));

        // Geometric factor (G)
        float G1 = 2.0 * hn * vn / vh;
        float G2 = 2.0 * hn * ln / vh;
        float G = min({float(1.0), G1, G2});

        // Schlick approximation to fresnel equation
        float F;
        schlick(vn, refr, F);

        shader_rgb = 0.4 * light->color * max(float(0.0), ln) + 0.8 * light->color * (F * D * G) / (ln * vn * M_PI);
        ray_color = shape_color.cwiseProduct(shader_rgb);
      }
      else if (model == "raw") // No lighting effect
      {
        ray_color = shape_color;
      }
      else
      {
        shader_rgb =  light->color * max(0.0, normal.dot(sray.normalized())) + light->color * pow(max(0.0, r.dot(e)), phong);
        ray_color = shape_color.cwiseProduct(shader_rgb);
      }
      // Add ray color to final color, while adjusting for fresnel constants
      color += k * ray_color;
    }
  }
}

void rayColor(const VEC3 ray, const VEC3 eye, int depth, VEC3& color, bool& hit, bool& in_motion, const float k)
{
  if (depth == 0) return;

  // TRAVERSE TREE
  vector<shared_ptr<BoundingVolume>> bvh_stack;
  vector<int> shape_inds;
  bvh_stack.push_back(bvh);
  while (bvh_stack.size() > 0)
  {
    shared_ptr<BoundingVolume> bvh_tmp = bvh_stack.back();
    bvh_stack.pop_back();
    VEC3 inv_ray = ray.cwiseInverse();
    if (bvh_tmp->intersect(ray, inv_ray, eye) == true)
    {
      // Is it possible for interior node to have indices? Not with the current formulation
      if (bvh_tmp->leaf == true && bvh_tmp->indices.size() > 0)
      {
        shape_inds.insert(shape_inds.end(), bvh_tmp->indices.begin(), bvh_tmp->indices.end());
      }
      else if (bvh_tmp->nodes.size() > 0)
      {
        bvh_stack.insert(bvh_stack.end(), bvh_tmp->nodes.begin(), bvh_tmp->nodes.end());
      }
    }
  }

  // CHECK SHAPE INTERSECTIONS
  float t_dist = FLT_MAX;
  float t_min = FLT_MAX;
  bool any_intersect = false;
  bool inside;
  in_motion = false;
  // Save shape
  shared_ptr<GeoPrimitive> hit_shape;
  for (int ind : shape_inds)
  {
    bool inside_tmp;
    shared_ptr<GeoPrimitive> shape = shapes[ind];
    bool intersect = shape->intersect(ray, eye, t_dist, inside);
    if (intersect == true)
    {
      any_intersect = true;
      hit = true;
      if (t_dist < t_min)
      {
        hit_shape = shape;
        inside = inside_tmp;
        t_min = t_dist;
      }
    }
  }

  // If nothing hit, then return, or produce perlin texture
  if (any_intersect == false)
  {
    return;
  }

  //########### KEY SHADING PARAMETERS
  // Vectors
  VEC3 isectP = eye + t_min * ray;
  VEC3 normal = hit_shape->getNorm(isectP);
  VEC3 in = ray.normalized();

  // Shading
  VEC3 shape_color = hit_shape->color;
  string model = hit_shape->model;

  // Reflection
  float roughness = hit_shape->reflect_params.roughness;
  VEC2 refr = hit_shape->reflect_params.refr;
  string material = hit_shape->reflect_params.material;

  // For sampling
  int shape_dim = hit_shape->dims;
  bool hit_light = hit_shape->light;
  in_motion = hit_shape->motion;

  // Make sure normal points towards origin of ray
  fixNorm(in, normal);

  // Misc. REMOVE EVENTUALLY
  VEC3 center = hit_shape->center;
  float radius = hit_shape->radius;

  // Material reflection
  if (reflect == true)
  {
    if (find(refl_materials.begin(), refl_materials.end(), material) != refl_materials.end())
    {
      // For adjustment to surface intersection
      float eps = 1e-3;

      // Get shape parameters
      bool glossy = hit_shape->reflect_params.glossy;

      // TODO: FIGURE OUT HOW TO INTERPOLATE CLOUD TRANSARENCY WITHOUT DOING THIS
      center = hit_shape->center;
      radius = hit_shape->radius;

      // Fresnel coefficients
      float k_refl = 1;
      float k_refr = 1;
      // Materials with refraction (e.g. glass)
      if (material == "glass")
      {
        // Refraction
        // Norm changes direction depending on whether going in or out
        float cos_theta;
        float sin_theta;
        float cos_phi;
        VEC3 adj_org;
        VEC3 out;
        bool refr;
        // Compute sin of angle (sin = sqrt(1 - cos^2))
        cos_theta = normal.dot(-in); // Need to reverse in-ray to get this right
        sin_theta = sqrt(1 - pow(cos_theta, 2));

        // Compute refraction ray -- returns false if total internal reflection
        if (inside == true)
        {
          refr = getRefractionRay(out, in, normal, sin_theta, cos_theta, refr_glass, refr_air);
        }
        else
        {
          refr = getRefractionRay(out, in, normal, sin_theta, cos_theta, refr_air, refr_glass);
        }
        if (refr == true)
        {
          // Adjust in different directions depending on whether going in or going out
          adj_org = isectP + in * eps;
          cos_phi = sqrt(1 - pow(refr_glass/refr_air, 2) * (1 - pow(in.dot(normal), 2)));

          // Fresnel coefficients
          // Note: refract inds don't matter in this context
          fresnel(cos_theta, cos_phi, refr_air, refr_glass, k_refl, k_refr);
          rayColor(out, adj_org, depth-1, color, hit, in_motion, k_refr*k);
        }
      }
      // Reflection
      VEC3 refl_ray = in - 2 * (normal.dot(in)) * normal;

      // Check that actually reflection
      if (refl_ray.dot(normal) <= 0)
      {
        printf("Error with reflection ray\n");
        cout << "Normal: " << normal << endl;
        cout << "Eye ray: " << ray << endl;
        cout << "Reflection: " << refl_ray << endl;
        throw;
      }

      // Edge case: if close to parallel then don't bother computing reflection
      if (refl_ray.dot(normal) > eps)
      {
        // Check for glossy reflection
        if (glossy == true && nogloss == false)
        {
          // Sample over rectangle centered around reflection ray (give it length 1, width 0.5 arbitrarily)
          // TODO: CALIBRATE THIS FOR OUR STEEL DOOR
          // Scale up reflection ray so the rectangle doesn't always intersect with floor
          VEC3 gloss_ray = refl_ray * 2;
          float length = 1;
          float width = 0.5;
          VEC3 length_vector = gloss_ray.cross(VEC3(1,0,0)).normalized();
          if (length_vector.isApprox(VEC3(0,0,0)))
          {
            length_vector = gloss_ray.cross(VEC3(0,0,1));
          }
          VEC3 c = gloss_ray + isectP;
          VEC3 p1 = length/2 * length_vector + c;
          VEC3 width_vector = (-gloss_ray).cross(length_vector).normalized();

          // Fuck it sample from disk
          // for (int i = 0; i < brdf_samples; i++)
          // {
          //   bool hit_tmp;
          //   float r = uniform(generator); // 1 radius disk
          //   float theta = 2 * M_PI * uniform(generator);
          //   VEC3 sample_refl = c + r * cos(theta) * length_vector + r * sin(theta) * width_vector;
          //   int sample_limit = 10;
          //   while (sample_refl.dot(normal) <= 0)
          //   {
          //     cout << "Sampled reflection ray not valid: " << sample_refl << endl;
          //     r = uniform(generator);
          //     theta = 2 * M_PI * uniform(generator);
          //     sample_refl = c + r * cos(theta) * length_vector + r * sin(theta) * width_vector;
          //     sample_limit--;
          //     if (sample_limit < 0)
          //     {
          //       printf("Error: more than 10 invalid brdf samples in a row\n");
          //       throw "Error: more than 10 invalid brdf samples in a row\n";
          //     }
          //   }

          // Rotate CP 90 deg clockwise about P to get width vector
          // VEC3 width_vector = rotate(c-p1, gloss_ray, M_PI/2);
          VEC3 A = width_vector * width/2 + p1;
          VEC3 B = A - length_vector * length;
          VEC3 C = B - width_vector * width;
          VEC3 D = A - width_vector * width;

          // TEST THIS
          // Squeeze rectangle so that none of the vertices force ray to intersect with surface
          VEC3 width_adj = width_vector;
          if (width_vector.dot(normal) <= 0) width_adj = -width_adj;
          VEC3 length_adj = length_vector;
          if (length_vector.dot(normal) <= 0) length_adj = -length_adj;

          while ((A - isectP).dot(normal) <= 0)
          {
            A = A + width_adj * 0.1 + length_adj * 0.1;
          }
          while ((B - isectP).dot(normal) <= 0)
          {
            B = B + width_adj * 0.1 + length_adj * 0.1;
          }
          while ((C - isectP).dot(normal) <= 0)
          {
            C = C + width_adj * 0.1 + length_adj * 0.1;
          }
          while ((D - isectP).dot(normal) <= 0)
          {
            D = D + width_adj * 0.1 + length_adj * 0.1;
          }

          Rectangle sample_rect = Rectangle(A, B, C, D, VEC3(0,0,0));
          for (int i = 0; i < brdf_samples; i++)
          {
            // Hit doesn't matter here
            bool hit_tmp;
            // Resample ray if it goes inside itself
            VEC3 sample_refl = sample_rect.samplePoint() - isectP;
            int sample_limit = 10;
            while (sample_refl.dot(normal) <= 0)
            {
              if (sample_limit < 0)
              {
                printf("Error: more than 10 invalid brdf samples in a row\n");
                cout << "Reflection ray: " << refl_ray << endl;
                cout << "Eye ray: " << ray << endl;
                cout << "Eye: " << eye << endl;
                cout << "Tmin: " << t_min << endl;
                cout << "Center: " << c << endl;
                cout << "A: " << A << endl;
                cout << "B: " << B << endl;
                cout << "C: " << C << endl;
                cout << "D: " << D << endl;
                cout << "Width vector: " << width_vector << endl;
                cout << "Length vector: " << length_vector << endl;
                cout << "Own shape normal: " << normal << endl;
                throw "Error: more than 10 invalid brdf samples in a row\n";
              }
              float multiplier = pow(2, 11 - sample_limit);
              gloss_ray = refl_ray * multiplier;
              length_vector = gloss_ray.cross(VEC3(1,0,0)).normalized();
              if (length_vector.isApprox(VEC3(0,0,0)))
              {
                length_vector = gloss_ray.cross(VEC3(0,0,1));
              }
              c = gloss_ray + isectP;
              p1 = length/2 * length_vector + c;

              width_vector = (-gloss_ray).cross(length_vector).normalized();
              A = width_vector * width/2 + p1;
              B = A - length_vector * length;
              C = B - width_vector * width;
              D = A - width_vector * width;
              sample_rect = Rectangle(A, B, C, D, VEC3(0,0,0));
              sample_refl = sample_rect.samplePoint() - isectP;
              sample_limit--;
            }
            rayColor(sample_refl, isectP + sample_refl*eps, depth - 1, color, hit_tmp, in_motion, k_refl*k/brdf_samples);
          }
        }
        else
        {
          rayColor(refl_ray, isectP + refl_ray*eps, depth - 1, color, hit, in_motion, k_refl*k);
        }
      }
    }
  }


  //########## SHADING ##########
  VEC3 ray_color(0,0,0);
  // First check if hit light
  if (hit_light == true)
  {
    if (hit_shape->name == "spherelight")
    {
      float hitdot = in.dot((hit_shape->center - isectP).normalized());
      color += k * shape_color * (0.1 * pow(hitdot, 1) + 0.05 * pow(hitdot, 5) + 0.9);
    }
    // Power fade based on avg distance from 4 corners
    if (hit_shape->name == "rectanglelight")
    {
      float dist = ((isectP - hit_shape->A).norm() + (isectP - hit_shape->B).norm() + (isectP - hit_shape->C).norm() +
          (isectP - hit_shape->D).norm())/(8 * (hit_shape->center - hit_shape->A).norm());
      color += k * shape_color * (0.1 * pow(dist, 1) + 0.05 * pow(dist, 5) + 0.9);
    }
  }
  // 3-term lighting
  // TODO: Soft shadows
  else if (any_intersect)
  {
    VEC3 shader_rgb(0,0,0);
    VEC3 e = (eye - isectP).normalized(); // Important: need to REVERSE ray direction back to eye for specular computation
    // For each light: shadow check
    // Keep track of how many lights hit to AVERAGE
    int hits = 0;
    VEC3 tmp_color(0,0,0);
    for (shared_ptr<LightPrimitive> light: lights)
    {
      VEC3 sray = light->sampleRay(isectP);
      // t_max is the distance from the normalized light ray to the light
      float t_max = sray.norm();
      int shadow = 1;
      // BVH traversal!
      vector<int> shadow_shape_inds;
      bvh_stack.push_back(bvh);
      while (bvh_stack.size() > 0)
      {
        shared_ptr<BoundingVolume> bvh_tmp = bvh_stack.back();
        bvh_stack.pop_back();
        VEC3 inv_ray = sray.cwiseInverse();
        if (bvh_tmp->intersect(sray, inv_ray, isectP+sray*1e-3) == true)
        {
          // Is it possible for interior node to have indices? Not with the current formulation
          if (bvh_tmp->leaf == true && bvh_tmp->indices.size() > 0)
          {
            shadow_shape_inds.insert(shadow_shape_inds.end(), bvh_tmp->indices.begin(), bvh_tmp->indices.end());
          }
          else if (bvh_tmp->nodes.size() > 0)
          {
            bvh_stack.insert(bvh_stack.end(), bvh_tmp->nodes.begin(), bvh_tmp->nodes.end());
          }
        }
      }

      for (int ind : shadow_shape_inds)
      {
        // IMPORTANT: skip shape if same object as light!
        shared_ptr<GeoPrimitive> shape_tmp = shapes[ind];
        shared_ptr<void> light_check = dynamic_pointer_cast<void>(light);
        shared_ptr<void> shape_check = dynamic_pointer_cast<void>(shape_tmp);
        if (light_check == shape_check)
        {
          continue;
        }
        int isect = shape_tmp->intersectShadow(sray.normalized(), isectP + sray.normalized() * 1e-3, t_max);
        if (isect == 1)
        {
          if (isectP[2] > 21 && light->center.isApprox(VEC3(0, 7.8019, 23)))
          {
            cout << "isectP: " << isectP << endl;
            cout << "Ray: " << ray << endl;
            cout << "Sray: " << sray << endl;
            cout << "isectP: " << isectP << endl;
          }
          shadow = 0;
          break;
        }
      }
      if (shadow == 0)
      {
        continue;
      }
      VEC3 r = (-1 * sray + 2 * (normal.dot(sray)) * normal).normalized();
      // Default shading model: lambert diffuse + phong specular
      VEC3 ray_color;
      if (hit_shape->texture == true)
      {
        float u, v;
        int type;
        VEC2 uv = hit_shape->getUV(isectP, type);
        // cout << "Textured shape hit point: " << isectP << endl;
        // cout << "Textured shape UV: " << uv << endl;
        if (type == 0)
        {
          return;
        }
        if (uv[0] < 0 || uv[1] < 0 || uv[0] > 1 || uv[1] > 1)
        {
          cout << "Shape: " << hit_shape->name << " problem with UV calculation" << endl;
          cout << "Hit point: " << isectP << endl;
          cout << "Shape center: " << hit_shape->center << endl;
          cout << "UV: " << uv << endl;
          throw;
        }
        if (type == 2) // Border color
        {
          shape_color = hit_shape->bordercolor;
        }
        else if (type == 1)
        {
          u = uv[0];
          v = uv[1];
          int texture_frame = hit_shape->tex_frame;
          VEC2 dims = texture_dims[texture_frame];
          int x_tex = (int(dims[0])-1) * u;
          int y_tex = (int(dims[1])-1) * v;
          int uv_ind = y_tex * dims[0] + x_tex;
          shape_color = texture_frames[texture_frame][uv_ind];
        }
      }
      if (model == "oren-nayar")
      {
        float A = 1.0 - (0.5 * pow(roughness, 2)) / (pow(roughness, 2) + 0.33);
        float B = (0.45 * pow(roughness, 2)) / (pow(roughness, 2) + 0.09);
        float vn = e.dot(normal);
        float ln = sray.normalized().dot(normal);

        float irradiance = max(float(0), ln);
        float vn_theta  = acos(vn);
        float ln_theta = acos(ln);

        // max( 0.0 , cos(phi_incident, phi_reflected) )
        float angleDiff = max(0.0, (e - normal * vn).normalized().dot((sray - normal * ln).normalized()));

        float alpha = max(vn_theta, ln_theta);
        float beta  = min(vn_theta, ln_theta);

        ray_color = shape_color.cwiseProduct(light->color) * irradiance * (A +
      	                    B * angleDiff * sin(alpha) * tan(beta));
      }
      else if (model == "cook-torrance")
      {
        VEC3 H = (e + sray).normalized();
        float hn = max(0.0, normal.dot(H));
        float vh = e.dot(H);
        float vn = e.dot(normal);
        float ln = sray.normalized().dot(normal);

        // specular component (Cook-Torrance reflection model)
        // D term (gaussian)
        float alpha = acos(hn);
        float D = 1/(pow(roughness, 2) * pow(cos(alpha), 4)) * exp(- pow(tan(alpha)/roughness, 2));

        // Geometric factor (G)
        float G1 = 2.0 * hn * vn / vh;
        float G2 = 2.0 * hn * ln / vh;
        float G = min({float(1.0), G1, G2});

        // Schlick approximation to fresnel equation
        float F;
        schlick(vn, refr, F);

        shader_rgb = 0.4 * light->color * max(float(0.0), ln) + 0.8 * light->color * (F * D * G) / (ln * vn * M_PI);
        ray_color = shape_color.cwiseProduct(shader_rgb);
      }
      else if (model == "raw") // No lighting effect
      {
        ray_color = shape_color;
      }
      else
      {
        // cout << "Default lambert shading for shape " << hit_shape->name << " with center: " << hit_shape->center << endl;
        shader_rgb =  light->color * max(0.0, normal.dot(sray.normalized())) + light->color * pow(max(0.0, r.dot(e)), phong);
        ray_color = shape_color.cwiseProduct(shader_rgb);
      }
      // Add ray color to final color, while adjusting for fresnel constants
      if (!ray_color.isApprox(VEC3(0,0,0)))
      {
        hits++;
        tmp_color += k * ray_color;
      }
    }
    if (hits > 0)
    {
      color += tmp_color/hits;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void renderImage(const string& filename, const int frame, const function<void (float)> sceneBuilder)
{
  cout << "Eye: " << eye << endl;
  cout << "lookingAt: " << lookingAt << endl;
  cout << "Up: " << up << endl;

  // Generate BVH
  // Edge case: only one shape or no shapes
  if (shapes.size() < 1)
  {
    printf("No shapes to render!\n");
    throw;
  }
  vector<int> range(shapes.size());
  iota(range.begin(), range.end(), 0);
  bvh = generateBVH(range);
  // printBVH(bvh);
  countBVH(bvh);

  // allocate the final image
  const int totalCells = xRes * yRes;
  float* ppmOut = new float[3 * totalCells];

  // Eye coordinate system
  const VEC3 cameraZ = -(lookingAt - eye).normalized();
  // Edge case: cross product is 0 (up vector and Z direction are same/opposite directions)
  VEC3 cameraX = up.cross(cameraZ).normalized();
  if (cameraX.isApprox(VEC3(0,0,0)))
  {
    printf("Gaze direction can't be equal to up vector!\n");
    throw "Gaze direction can't be equal to up vector!!!\n";
  }
  VEC3 cameraY = cameraZ.cross(cameraX).normalized();

  // Pre-compute change of basis (for perlin texturing)
  // NOTE: Looking down POSITIVE z-axis!

  // For final project: TURN the up vector once we hit cloud frames so clouds are sideways
  VEC3 new_up, newX, newY;
  if (frame >= frame_cloud)
  {
    new_up = VEC3(-1, 0, 0);
    newX = new_up.cross(cameraZ).normalized();
    newY = cameraZ.cross(newX).normalized();
  }
  // standard cameraZ -> looks BEHIND
  MATRIX4 cob; cob << cameraX.transpose(), 0, cameraY.transpose(), 0, cameraZ.transpose(), 0,
                      0, 0, 0, 1;
  MATRIX4 origin = MatrixXd::Identity(4,4);
  origin(0,3) = -eye[0];
  origin(1,3) = -eye[1];
  origin(2,3) = -eye[2];
  MATRIX4 mcam = cob * origin;
  MATRIX4 new_cob; new_cob << newX.transpose(), 0, newY.transpose(), 0, cameraZ.transpose(), 0,
                      0, 0, 0, 1;

  MATRIX4 new_mcam = new_cob * origin;

  // Compute near plane parameters
  const float t = tan(fov * M_PI/360.0) * abs(near);
  const float b = -t;
  const float r = aspect * t;
  const float l = -r;

  // TODO: Motion blur for eye
  // TODO: Try Shkurko '17 and compare performance against Monte carlo sampling (just need to keep track of t and t+1 data -> use scene information to interpolate in between)
  for (int y = 0; y < yRes; y++)
  {
    printf("Done with row %d out of %d rows\n", y, yRes);
    for (int x = 0; x < xRes; x++)
    {
      // Set default color based on Perlin Noise texture with time dimension
      // Perlin Noise interpolate between teal and pink
      //  default_col = pink + ValueNoise_3D(x, y, frame) * teal;
      default_col = VEC3(0,0,0);
      VEC3 color(0,0,0);
      // Antialiasing: 25 jittered samples
      // IMPORTANT: this will also be the number of samples for soft shadows and DOF!!
      vector<VEC3> eye_samples;
      getDOFSamples(eye_samples, eye, cameraX, cameraY, antialias_samples);

      int n = int(sqrt(antialias_samples));
      vector<VEC2> jitter_pixels;
      for (int i = 0; i < n; i++)
      {
        for (int j = 0; j < n; j++)
        {
          float adj_x = x + ((float) i + uniform(generator))/(float) 9;
          float adj_y = y + ((float) j + uniform(generator))/(float) 9;
          jitter_pixels.push_back(VEC2(adj_x, adj_y));
        }
      }

      // Shuffle eye samples for extra randomness
      shuffle(eye_samples);

      int sampled_n = pow(n, 2); // Hacky way of dealing with non-square antialias samples
      for (int i = 0; i < sampled_n; i++)
      {
        VEC3 tmp_color(0,0,0);
        VEC3 eye_sample = eye_samples[i];
        VEC2 pixel = jitter_pixels[i];
        const VEC3 rayDir = getPerspEyeRay(l, r, t, b, pixel[0], pixel[1], cameraX, cameraY, cameraZ);

        VEC3 focalPoint = eye + focal_length * rayDir;
        bool hit = false;
        bool motion = false;
        rayColor(focalPoint - eye_sample, eye_sample, max_depth, tmp_color, hit, motion);
        // If never hit, default color is white
        if (hit == false)
        {
          if (perlin_cloud == true)
          {
            // Texture using clouds: in the indoors scene we'll keep just clouds in the negative z-axis
            VEC4 point; point << focalPoint, 1;
            if (frame >= frame_cloud)
            {
              point = new_mcam * point;
            }
            else
            {
              point = mcam * point;
            }
            tmp_color = cloudColor(point.head<3>(), VEC3(0,0,0), frame);
          }
          else
          {
            tmp_color = default_col;
          }
        }
        if (motion == true)
        {
          for (int m = 0; m < blur_samples; m++)
          {
            // TODO: CAN WE SIMULATE SPEED BY SIMPLY INCREASE FRAME RANGE???
            // Sample 1 frame ahead and average
            float frame_sample = float(frame) + uniform(generator) * frame_range;

            // TRICK: We already KNOW what and WHERE things are moving!
            float val;
            float rot_theta;
            if (frame >= frame_prism)
            {
              if (frame >= frame_blur)
              {
                // Bounding volumes: increase y-bound on BOTH ends to be safe
                val = move_per_frame * (frame_sample - frame) + accel_t * pow((frame_sample - frame), 3);
                rot_theta = (frame_sample - frame)/720.0 * M_PI;
                bumpBVH(bvh, val);
                for (shared_ptr<GeoPrimitive> shape : shapes)
                {
                  if (shape->name == "rectangle")
                  {
                    // Shift y AND rotate
                    shape->A[1] += val;
                    shape->B[1] += val;
                    shape->C[1] += val;
                    shape->D[1] += val;

                    // VEC3 rot_pointA =  VEC3(0,-1,0).dot(shape->A - cap_center) * VEC3(0,-1,0) + cap_center;
                    // shape->A = rotate(shape->A - rot_pointA, VEC3(0,1,0), rot_theta) + cap_center;
                    // VEC3 rot_pointB =  VEC3(0,-1,0).dot(shape->B - cap_center) * VEC3(0,-1,0)+ cap_center;
                    // shape->B = rotate(shape->B - rot_pointB, VEC3(0,1,0), rot_theta) + cap_center;
                    // VEC3 rot_pointC =  VEC3(0,-1,0).dot(shape->C - cap_center)* VEC3(0,-1,0) + cap_center;
                    // shape->C = rotate(shape->C - rot_pointC, VEC3(0,1,0), rot_theta) + cap_center;
                    // VEC3 rot_pointD =  VEC3(0,-1,0).dot(shape->D - cap_center)* VEC3(0,-1,0) + cap_center;
                    // shape->D = rotate(shape->D - rot_pointD, VEC3(0,1,0), rot_theta) + cap_center;
                  }
                }
              }
              else
              {
                val = move_per_frame * (frame_sample - frame);
                rot_theta = (frame_sample - frame)/720.0 * M_PI;
                bumpBVH(bvh, val);
                for (shared_ptr<GeoPrimitive> shape : shapes)
                {
                  if (shape->name == "rectangle")
                  {
                    shape->A[1] += val;
                    shape->B[1] += val;
                    shape->C[1] += val;
                    shape->D[1] += val;

                    // VEC3 rot_pointA =  VEC3(0,-1,0).dot(shape->A - cap_center) * VEC3(0,-1,0) + cap_center;
                    // shape->A = rotate(shape->A - rot_pointA, VEC3(0,1,0), rot_theta) + cap_center;
                    // VEC3 rot_pointB =  VEC3(0,-1,0).dot(shape->B - cap_center) * VEC3(0,-1,0)+ cap_center;
                    // shape->B = rotate(shape->B - rot_pointB, VEC3(0,1,0), rot_theta) + cap_center;
                    // VEC3 rot_pointC =  VEC3(0,-1,0).dot(shape->C - cap_center)* VEC3(0,-1,0) + cap_center;
                    // shape->C = rotate(shape->C - rot_pointC, VEC3(0,1,0), rot_theta) + cap_center;
                    // VEC3 rot_pointD =  VEC3(0,-1,0).dot(shape->D - cap_center)* VEC3(0,-1,0) + cap_center;
                    // shape->D = rotate(shape->D - rot_pointD, VEC3(0,1,0), rot_theta) + cap_center;
                  }
                }
              }
            }
            VEC3 motion_color(0,0,0);
            rayColor(focalPoint - eye_sample, eye_sample, max_depth, motion_color, hit, motion);
            // If never hit, default color is white
            if (hit == false)
            {
              if (perlin_cloud == true)
              {
                // Texture using clouds: in the indoors scene we'll keep just clouds in the negative z-axis
                VEC4 point; point << focalPoint, 1;
                if (frame >= frame_cloud)
                {
                  point = new_mcam * point;
                }
                else
                {
                  point = mcam * point;
                }
                motion_color = cloudColor(point.head<3>(), VEC3(0,0,0), frame);
              }
              else
              {
                motion_color = default_col;
              }
            }
            tmp_color += motion_color;

            // Reset values
            bumpBVH(bvh, -val);
            for (shared_ptr<GeoPrimitive> shape : shapes)
            {
              if (shape->name == "rectangle")
              {
                // VEC3 rot_pointA =  VEC3(0,-1,0).dot(shape->A - cap_center) * VEC3(0,-1,0) + cap_center;
                // shape->A = rotate(shape->A - rot_pointA, VEC3(0,1,0), -rot_theta) + cap_center;
                // VEC3 rot_pointB =  VEC3(0,-1,0).dot(shape->B - cap_center) * VEC3(0,-1,0)+ cap_center;
                // shape->B = rotate(shape->B - rot_pointB, VEC3(0,1,0), -rot_theta) + cap_center;
                // VEC3 rot_pointC =  VEC3(0,-1,0).dot(shape->C - cap_center)* VEC3(0,-1,0) + cap_center;
                // shape->C = rotate(shape->C - rot_pointC, VEC3(0,1,0), -rot_theta) + cap_center;
                // VEC3 rot_pointD =  VEC3(0,-1,0).dot(shape->D - cap_center)* VEC3(0,-1,0) + cap_center;
                // shape->D = rotate(shape->D - rot_pointD, VEC3(0,1,0), -rot_theta) + cap_center;

                shape->A[1] -= val;
                shape->B[1] -= val;
                shape->C[1] -= val;
                shape->D[1] -= val;
              }
            }
          }
          tmp_color /= (blur_samples + 1);
        }
        color += tmp_color;
      }
      color /= sampled_n;
      // set, in final image
      ppmOut[3 * ((yRes-1 - y) * xRes + x)] = clamp(color[0]) * 255.0f;
      ppmOut[3 * ((yRes-1 - y) * xRes + x) + 1] = clamp(color[1]) * 255.0f;
      ppmOut[3 * ((yRes-1 - y) * xRes + x) + 2] = clamp(color[2]) * 255.0f;
    }
  }
  writePPM(filename, xRes, yRes, ppmOut);
  delete[] ppmOut;
}

void renderImageCloud(const string& filename, const float frame)
{
  // Set eye and lookingat to final project settings
  eye = VEC3(0.5, 1.5, 1);
  up = VEC3(0,0,1);
  lookingAt = VEC3(0.5, -1, 1);

  // allocate the final image
  const int totalCells = xRes * yRes;
  float* ppmOut = new float[3 * totalCells];

  // Eye coordinate system
  const VEC3 cameraZ = -(lookingAt - eye).normalized();
  // Edge case: cross product is 0 (up vector and Z direction are same/opposite directions)
  const VEC3 cameraX = up.cross(cameraZ).normalized();
  if (cameraX.isApprox(VEC3(0,0,0)))
  {
    printf("Gaze direction can't be equal to up vector!\n");
    throw "Gaze direction can't be equal to up vector!!!\n";
  }
  const VEC3 cameraY = cameraZ.cross(cameraX).normalized();

  // Compute near plane parameters
  const float t = tan(fov * M_PI/360.0) * abs(near);
  const float b = -t;
  const float r = aspect * t;
  const float l = -r;

  // Precompute mcam
  MATRIX4 cob; cob << cameraX.transpose(), 0, cameraY.transpose(), 0, -cameraZ.transpose(), 0,
                      0, 0, 0, 1;
  MATRIX4 origin = MatrixXd::Identity(4,4);
  origin(0,3) = -eye[0];
  origin(1,3) = -eye[1];
  origin(2,3) = -eye[2];
  MATRIX4 mcam = cob * origin;

  for (int y = 0; y < yRes; y++)
  {
    for (int x = 0; x < xRes; x++)
    {
      const VEC3 rayDir = getPerspEyeRay(l, r, t, b, x, y, cameraX, cameraY, cameraZ);
      // Texture using clouds
      VEC4 point; point << rayDir + eye, 1;
      point = mcam * point;
      VEC3 color = cloudColor(point.head<3>(), VEC3(0,0,0), frame);

      // set, in final image
      ppmOut[3 * ((yRes-1 - y) * xRes + x)] = clamp(color[0]) * 255.0f;
      ppmOut[3 * ((yRes-1 - y) * xRes + x) + 1] = clamp(color[1]) * 255.0f;
      ppmOut[3 * ((yRes-1 - y) * xRes + x) + 2] = clamp(color[2]) * 255.0f;
    }
  }
  writePPM(filename, xRes, yRes, ppmOut);
  delete[] ppmOut;
}

void renderImageNoBVH(const string& filename, const int frame, const function<void (float)> sceneBuilder)
{
  printf("Shapes array size: %d\n", shapes.size());

  // allocate the final image
  const int totalCells = xRes * yRes;
  float* ppmOut = new float[3 * totalCells];

  // Eye coordinate system
  const VEC3 cameraZ = -(lookingAt - eye).normalized();
  // Edge case: cross product is 0 (up vector and Z direction are same/opposite directions)
  const VEC3 cameraX = up.cross(cameraZ).normalized();
  if (cameraX.isApprox(VEC3(0,0,0)))
  {
    printf("Gaze direction can't be equal to up vector!!!\n");
    throw "Gaze direction can't be equal to up vector!!!\n";
  }
  const VEC3 cameraY = cameraZ.cross(cameraX).normalized();

  // Compute near plane parameters
  const float t = tan(fov * M_PI/360.0) * abs(near);
  const float b = -t;
  const float r = aspect * t;
  const float l = -r;

  // TODO: Motion blur for eye
  // TODO: Try Shkurko '17 and compare performance against Monte carlo sampling (just need to keep track of t and t+1 data -> use scene information to interpolate in between)
  for (int y = 0; y < yRes; y++)
  {
    for (int x = 0; x < xRes; x++)
    {
      VEC3 color(0,0,0);
      // Antialiasing: 25 jittered samples
      // IMPORTANT: this will also be the number of samples for soft shadows and DOF!!
      vector<VEC3> eye_samples;
      getDOFSamples(eye_samples, eye, cameraX, cameraY, antialias_samples);

      int n = int(sqrt(antialias_samples));
      vector<VEC2> jitter_pixels;
      for (int i = 0; i < n; i++)
      {
        for (int j = 0; j < n; j++)
        {
          float adj_x = x + ((float) i + uniform(generator))/(float) 9;
          float adj_y = y + ((float) j + uniform(generator))/(float) 9;
          jitter_pixels.push_back(VEC2(adj_x, adj_y));
        }
      }

      // Shuffle eye samples for extra randomness
      shuffle(eye_samples);

      int sampled_n = pow(n, 2); // Hacky way of dealing with non-square antialias samples
      for (int i = 0; i < sampled_n; i++)
      {
        VEC3 tmp_color(0,0,0);
        VEC3 eye_sample = eye_samples[i];
        VEC2 pixel = jitter_pixels[i];
        const VEC3 rayDir = getPerspEyeRay(l, r, t, b, pixel[0], pixel[1], cameraX, cameraY, cameraZ);

        VEC3 focalPoint = eye + focal_length * rayDir;
        bool hit = false;
        bool motion = false;
        rayColorNoBVH(focalPoint - eye_sample, eye_sample, max_depth, tmp_color, hit, motion);
        // If never hit, default color is white
        if (hit == false)
        {
          // TODO: Default color using perlin noise interpolation between vaporwave colors
          tmp_color = default_col;
        }
        if (motion == true)
        {
          for (int m = 0; m < blur_samples; m++)
          {
            // Sample in 1-frame interval and average
            float frame_sample = float(frame) + uniform(generator);
            sceneBuilder(frame_sample);
            VEC3 motion_color(0,0,0);
            rayColorNoBVH(focalPoint - eye_sample, eye_sample, max_depth, motion_color, hit, motion);
            // If never hit, default color is white
            if (hit == false)
            {
              motion_color = default_col;
            }
            tmp_color += motion_color;
          }
          tmp_color /= (blur_samples + 1);
          // Reset scene
          sceneBuilder(frame);
        }
        color += tmp_color;
      }
      color /= sampled_n;
      // set, in final image
      ppmOut[3 * ((yRes-1 - y) * xRes + x)] = clamp(color[0]) * 255.0f;
      ppmOut[3 * ((yRes-1 - y) * xRes + x) + 1] = clamp(color[1]) * 255.0f;
      ppmOut[3 * ((yRes-1 - y) * xRes + x) + 2] = clamp(color[2]) * 255.0f;
    }
  }
  writePPM(filename, xRes, yRes, ppmOut);
  delete[] ppmOut;
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  string skeletonFilename("90.asf");
  string motionFilename("90_16_v3.amc");

  // load up skeleton stuff
  skeleton = new Skeleton(skeletonFilename.c_str(), MOCAP_SCALE);
  skeleton->setBasePosture();
  displayer.LoadSkeleton(skeleton);

  // load up the motion
  motion = new Motion(motionFilename.c_str(), MOCAP_SCALE, skeleton);
  displayer.LoadMotion(motion);
  skeleton->setPosture(*(displayer.GetSkeletonMotion(0)->GetPosture(0)));

  // Load in all texture file paths
  for (string file: ad_paths)
  {
    vector<string> f_frames = get_all(file, ".jpg");
    frame_paths.insert(frame_paths.end(), f_frames.begin(), f_frames.end());
  }

  int frame;
  if (argc == 1)
  {
    // Render full video
    xRes = 980;
    yRes = 540;
    antialias_samples = 2;
    brdf_samples = 2;
    frame = 30;
    buildFinal(frame*8);
    char buffer[256];
    sprintf(buffer, "./frame.%04i.ppm", frame);
    time_t time0 = time(NULL);
    renderImage(buffer, frame*8, buildFinal);
    time_t time1 = time(NULL);
    cout << "Rendered low res, low sampling frame " + to_string(frame) << " in " << int(difftime(time1, time0)) << " seconds" << endl;
  }
  // Check input mode
  string mode = argv[1];
  if (mode == "frame")
  {
    // Final project sample render
    xRes = 400;
    yRes = 300;
    aperture = 0;
    antialias_samples = 1;
    reflect = false;

    frame = atoi(argv[2]);
    buildFinal(frame*8);
    //lights.push_back(make_shared<pointLight>(eye, VEC3(1,1,1)));
    char buffer[256];
    sprintf(buffer, "./preview_frames/frame.%04i.ppm", frame);
    time_t time0 = time(NULL);
    renderImage(buffer, frame*8, buildFinal);
    time_t time1 = time(NULL);
    cout << "Rendered frame " + to_string(frame) << " in " << int(difftime(time1, time0)) << " seconds" << endl;
  }
  else if (mode == "final")
  {
    frame = atoi(argv[2]);
    buildFinal(frame*8);
    char buffer[256];
    sprintf(buffer, "./final_frames/frame.%04i.ppm", frame);
    time_t time0 = time(NULL);
    renderImage(buffer, frame*8, buildFinal);
    time_t time1 = time(NULL);
    cout << "Rendered final frame " + to_string(frame) << " in " << int(difftime(time1, time0)) << " seconds" << endl;
  }
  else if (mode == "nodistr")
  {
    // Lower sampling + lower resolution
    antialias_samples = 6;
    brdf_samples = 2;
    frame = atoi(argv[2]);
    buildFinal(frame*8);
    char buffer[256];
    sprintf(buffer, "./nodistr/frame.%04i.ppm", frame);
    time_t time0 = time(NULL);
    renderImage(buffer, frame*8, buildFinal);
    time_t time1 = time(NULL);
    cout << "Rendered low res final frame " + to_string(frame) << " in " << int(difftime(time1, time0)) << " seconds" << endl;
  }
  else if (mode == "superlowres")
  {
    xRes = 480;
    yRes = 270;
    frame = atoi(argv[2]);
    buildFinal(frame*8);
    char buffer[256];
    sprintf(buffer, "./final_lowres_lower/frame.%04i.ppm", frame);
    time_t time0 = time(NULL);
    renderImage(buffer, frame*8, buildFinal);
    time_t time1 = time(NULL);
    cout << "Rendered low res final frame " + to_string(frame) << " in " << int(difftime(time1, time0)) << " seconds" << endl;
  }
  else if (mode == "lowres")
  {
    xRes = 960;
    yRes = 540;
    frame = atoi(argv[2]);
    buildFinal(frame*8);
    char buffer[256];
    sprintf(buffer, "./final_lowres_lower/frame.%04i.ppm", frame);
    time_t time0 = time(NULL);
    renderImage(buffer, frame*8, buildFinal);
    time_t time1 = time(NULL);
    cout << "Rendered low res final frame " + to_string(frame) << " in " << int(difftime(time1, time0)) << " seconds" << endl;
  }
  else if (mode == "nogloss")
  {
    nogloss = true;
    xRes = 1920;
    yRes = 1080;
    frame = atoi(argv[2]);
    buildFinal(frame*8);
    char buffer[256];
    sprintf(buffer, "./final_lowres_nogloss/frame.%04i.ppm", frame);
    time_t time0 = time(NULL);
    renderImage(buffer, frame*8, buildFinal);
    time_t time1 = time(NULL);
    cout << "Rendered no glossy reflection frame " + to_string(frame) << " in " << int(difftime(time1, time0)) << " seconds" << endl;
  }
  else if (mode == "piecon")
  {
    xRes = 1920;
    yRes = 1080;
    frame = atoi(argv[2]);
    buildFinal(frame*8);
    char buffer[256];
    sprintf(buffer, "./final_piecon/frame.%04i.ppm", frame);
    time_t time0 = time(NULL);
    renderImage(buffer, frame*8, buildFinal);
    time_t time1 = time(NULL);
    cout << "Rendered final frame " + to_string(frame) << " in " << int(difftime(time1, time0)) << " seconds" << endl;
  }
  else if (mode == "view")
  {
    // View entire first scene room
    xRes = 400;
    yRes = 300;
    aperture = 0;
    antialias_samples = 1;
    int rot = atoi(argv[2]);
    // Turn off all reflections
    reflect = false;
    buildFinal(0); // Frame with all objects

    // Alternative camera path
    eye = VEC3(-9, 9, -4);
    lookingAt = VEC3(10, 11, 6);

    float final_theta = M_PI;
    float theta = rot * final_theta/30;
    eye = rotate(eye, VEC3(0,1,0), theta);
    // Adjust eye if outside bounds
    while (eye[0] < -10 || eye[0] > 10 || eye[2] < -5 || eye[2] > 8)
    {
      eye *= 0.9;
    }
    lookingAt = rotate(lookingAt, VEC3(0,1,0), theta);
    // // Also move lookingat up/down
    // float vert = cos(rot * M_PI/10) * 6;
    lookingAt -= VEC3(0, rot/30.0 * 10, 0);

    // lights.push_back(make_shared<pointLight>(eye, VEC3(1,1,1)));
    char buffer[256];
    sprintf(buffer, "./test_frames/first_scene.%04i.ppm", rot);
    time_t time0 = time(NULL);
    renderImage(buffer, frame*8, buildFinal);
    time_t time1 = time(NULL);
    cout << "Rendered frame " + to_string(rot) << " in " << int(difftime(time1, time0)) << " seconds" << endl;
  }
  else if (mode == "modelview")
  {
    // View all objects in room while rotating
    xRes = 400;
    yRes = 300;
    aperture = 0;
    antialias_samples = 1;
    int rot = atoi(argv[2]);
    // Turn off all reflections
    reflect = false;
    buildFinal(frame_sculp-1); // Frame with all objects

    eye = VEC3(-9, 9, -4);
    lookingAt = VEC3(10, 11, 6);

    float final_theta = M_PI * 5/4;
    float theta = rot * final_theta/30;
    eye = rotate(eye, VEC3(0,1,0), theta);
    // Adjust eye if outside bounds
    while (eye[0] < -10 || eye[0] > 10 || eye[2] < -5 || eye[2] > 8)
    {
      eye *= 0.9;
    }
    lookingAt = rotate(lookingAt, VEC3(0,1,0), theta);
    // // Also move lookingat up/down
    // float vert = cos(rot * M_PI/10) * 6;
    lookingAt -= VEC3(0, rot/30.0 * 10, 0);

    // lights.push_back(make_shared<pointLight>(eye, VEC3(1,1,1)));
    char buffer[256];
    sprintf(buffer, "./test_frames/first_scene.%04i.ppm", rot);
    time_t time0 = time(NULL);
    renderImage(buffer, frame*8, buildFinal);
    time_t time1 = time(NULL);
    cout << "Rendered frame " + to_string(rot) << " in " << int(difftime(time1, time0)) << " seconds" << endl;
  }
  else if (mode == "camera")
  {
    // View entire first scene room
    xRes = 400;
    yRes = 300;
    aperture = 0;
    antialias_samples = 1;
    int frame = atoi(argv[2]);
    buildCameraPath(frame*8); // Frame with all objects
    char buffer[256];
    sprintf(buffer, "./camera_frames/camera.%04i.ppm", frame);
    time_t time0 = time(NULL);
    renderImage(buffer, frame*8, buildCameraPath);
    time_t time1 = time(NULL);
    cout << "Rendered frame " + to_string(frame) << " in " << int(difftime(time1, time0)) << " seconds" << endl;
  }
  else if (mode == "camerav2")
  {
    // View entire first scene room
    xRes = 400;
    yRes = 300;
    aperture = 0;
    antialias_samples = 1;
    int frame = atoi(argv[2]);
    buildCameraPathV2(frame*8); // Frame with all objects
    char buffer[256];
    sprintf(buffer, "./camera_frames_new/camera.%04i.ppm", frame);
    time_t time0 = time(NULL);
    renderImage(buffer, frame*8, buildCameraPathV2);
    time_t time1 = time(NULL);
    cout << "Rendered frame " + to_string(frame) << " in " << int(difftime(time1, time0)) << " seconds" << endl;
  }
  else if (mode == "model")
  {
    // NOTE: Frame 34 is model facing forward
    aperture = 0;
    frame = atoi(argv[2]);
    time_t time0 = time(NULL);
    buildSceneModels(frame);
    char buffer[256];
    sprintf(buffer, "./test_frames/model/frame.%04i.ppm", frame);
    renderImage(buffer, frame, buildSceneModels);
    time_t time1 = time(NULL);
    cout << "Finished all models frame " << frame << " in " << int(difftime(time1, time0)) << " seconds." << endl;
  }
  else if (mode == "prism")
  {
    aperture = 0;
    antialias_samples = 1;
    frame = atoi(argv[2]);
    time_t time0 = time(NULL);
    buildScenePrism(frame);
    char buffer[256];
    sprintf(buffer, "./test_frames/prism/frame.%04i.ppm", frame);
    renderImage(buffer, frame, buildScenePrism);
    time_t time1 = time(NULL);
    cout << "Finished prism frame " << frame << " in " << int(difftime(time1, time0)) << " seconds." << endl;
  }
  else if (mode == "pedestal")
  {
    xRes = 640;
    yRes = 480;
    aperture = 0;
    antialias_samples = 1;
    frame = atoi(argv[2]);
    time_t time0 = time(NULL);
    buildScenePedestal(frame);
    char buffer[256];
    sprintf(buffer, "./test_frames/pedestal/frame.%04i.ppm", frame);
    renderImage(buffer, frame, buildScenePedestal);
    time_t time1 = time(NULL);
    cout << "Finished pedestal frame " << frame << " in " << int(difftime(time1, time0)) << " seconds." << endl;
  }
  else if (mode == "startscene")
  {
    // Low-res test
    xRes = 640;
    yRes = 480;
    aperture = 0;
    int frame = atoi(argv[2]);
    time_t time0 = time(NULL);
    buildSceneBoundary(frame);
    char buffer[256];
    sprintf(buffer, "./test_frames/startscene/frame.%04i.ppm", frame);
    renderImage(buffer, frame, buildSceneBoundary);
    time_t time1 = time(NULL);
    cout << "Finished floor + walls + ceiling in " << int(difftime(time1, time0)) << " seconds." << endl;
  }
  else if (mode == "perlin")
  {
    xRes = 640;
    yRes = 480;
    aperture = 0;
    antialias_samples = 1;
    time_t time0 = time(NULL);
    char buffer[256];
    int i = atoi(argv[2]);
    sprintf(buffer, "./test_frames/perlin/frame.%04i.ppm", i);
    renderImageCloud(buffer, i);
    time_t time1 = time(NULL);
    cout << "Finished perlin cloud frame " << i << " in " << int(difftime(time1, time0)) << " seconds." << endl;
  }
  else if (mode == "backframe")
  {
    // Testing back half of animation (full settings)
    int frame = atoi(argv[2]);
    time_t time0 = time(NULL);
    buildFinal(frame*8);
    char buffer[256];
    sprintf(buffer, "./test_frames/back_frames/frame.%04i.ppm", frame);
    renderImage(buffer, frame*8, buildFinal);
    time_t time1 = time(NULL);
    cout << "Finished prism frame " << frame << " in " << int(difftime(time1, time0)) << " seconds." << endl;
  }
  else if (mode == "prismcyl")
  {
    xRes = 640;
    yRes = 480;
    int frame = atoi(argv[2]);
    time_t time0 = time(NULL);
    BuildScenePrismCylinder(frame);
    char buffer[256];
    sprintf(buffer, "./test_frames/prismcyl/frame.%04i.ppm", frame);
    renderImage(buffer, frame, BuildScenePrismCylinder);
    time_t time1 = time(NULL);
    cout << "Finished prism cylinder " << frame << " in " << int(difftime(time1, time0)) << " seconds." << endl;
  }
  else if (mode == "test")
  {
    // Loop through possible test requests
    for (int i = 2; i < argc; i++)
    {
      string test = argv[i];
      if (test == "dof")
      {
        // Depth of field testing: aperture and focal length grid
        buildSceneDOF(0);
        int n_frames = 150;
        float min_focal = 0.1;
        float max_focal = 10;
        float step = (max_focal - min_focal)/n_frames;
        time_t time0 = time(NULL);
        for (int i = 0; i < n_frames; i++)
        {
          focal_length = min_focal + step * i;
          char buffer[256];
          sprintf(buffer, "./test_frames/dof/focal/frame.%04i.ppm", i);
          renderImage(buffer, 0, buildSceneDOF);
          cout << "Rendered frame " + to_string(i) + " with focal length " + to_string(int(focal_length*100)/float(100)) << endl;
        }
        time_t time1 = time(NULL);
        cout << "Finished " << n_frames << " focal test frames in " << int(difftime(time1, time0)) << " seconds." << endl;
        focal_length = 6;
        float max_aperture = 0.5;
        float min_aperture = 0.01;
        step = (max_aperture - min_aperture)/n_frames;
        time0 = time(NULL);
        for (int i = 0; i < n_frames; i++)
        {
          aperture = min_aperture + step * i;
          char buffer[256];
          sprintf(buffer, "./test_frames/dof/aperture/frame.%04i.ppm", i);
          renderImage(buffer, 0, buildSceneDOF);
          cout << "Rendered frame " + to_string(i) + " with aperture " + to_string(int(aperture*100)/float(100)) << endl;
        }
        time1 = time(NULL);
        cout << "Finished " << n_frames << " aperture test frames in " << int(difftime(time1, time0)) << " seconds." << endl;
      }
      else if (test == "reflectance")
      {
        time_t time0 = time(NULL);
        for (int i = 0; i < 150; i++)
        {
          buildSceneReflectance(i);
          char buffer[256];
          sprintf(buffer, "./test_frames/reflectance/frame.%04i.ppm", i);
          renderImage(buffer, i, buildSceneReflectance);
          cout << "Rendered frame " + to_string(i) << endl;
        }
        time_t time1 = time(NULL);
        cout << "Finished " << 150 << " reflectance test frames in " << int(difftime(time1, time0)) << " seconds." << endl;
      }
      else if (test == "blur")
      {
        // Set aperture to 0 so we don't confuse the DOF effect
        aperture = 0;
        antialias_samples = 1;
        // Motion blur: increase the frame sampling range
        time_t time0;
        time_t time1;
        for (int n = 1; n < 4; n++)
        {
          frame_range = n;
          time0 = time(NULL);
          for (int i = (n-1) * 60; i < (n) * 60; i++)
          {
            // 2 seconds of frames
            buildSceneSpheres(i % 60);
            char buffer[256];
            sprintf(buffer, "./test_frames/motion_blur/frame.%04i.ppm", i);
            renderImage(buffer, i, buildSceneSpheres);
            cout << "Rendered frame " + to_string(i) << endl;
          }
          time1 = time(NULL);
          cout << "Finished " << 60 << " motion blur frames with " << n << " blur samples PER antialias sample in " <<
                int(difftime(time1, time0)) << " seconds." << endl;
        }
      }
      else if (test == "perlin")
      {
        // Simple perlin noise: map sphere cloud
        // time_t time0 = time(NULL);
        // buildSceneCloud(0);
        // char buffer[256];
        // sprintf(buffer, "./test_frames/perlin/frame.%04i.ppm", 0);
        // renderImage(buffer, 0, buildSceneCloud);
        // time_t time1 = time(NULL);
        // cout << "Finished " << 1 << " perlin cloud frames in " << int(difftime(time1, time0)) << " seconds." << endl;

        // Even simpler: 2D image texture and zoom in each frame
        time_t time0 = time(NULL);
        for (int i = 1; i < 50; i++)
        {
          char buffer[256];
          sprintf(buffer, "./test_frames/perlin/frame.%04i.ppm", i);
          renderImageCloud(buffer, i);
        }
        time_t time1 = time(NULL);
        cout << "Finished " << 50 << " perlin cloud frames in " << int(difftime(time1, time0)) << " seconds." << endl;
      }
      else if (test == "checkercylinder")
      {
        xRes = 640;
        yRes = 480;
        aperture = 0;
        time_t time0 = time(NULL);
        buildSceneCylinder(0);
        char buffer[256];
        sprintf(buffer, "./test_frames/checkercylinder/frame.%04i.ppm", 0);
        renderImage(buffer, 0, buildSceneCylinder);
        time_t time1 = time(NULL);
        cout << "Finished " << 1 << " checkercylinder frame in " << int(difftime(time1, time0)) << " seconds." << endl;
      }
      else if (test == "checkertexture")
      {
        xRes = 640;
        yRes = 480;
        aperture = 0;
        antialias_samples = 1;
        time_t time0 = time(NULL);
        buildSceneCheckerTexture(0);
        char buffer[256];
        sprintf(buffer, "./test_frames/checkertexture/frame.%04i.ppm", 0);
        renderImage(buffer, 0, buildSceneCheckerTexture);
        time_t time1 = time(NULL);
        cout << "Finished " << 1 << " checkertexture frame in " << int(difftime(time1, time0)) << " seconds." << endl;
      }
      else if (test == "model")
      {
        // Test low-poly helios model
        aperture = 0;
        focal_length = 4;
        time_t time0 = time(NULL);
        buildSceneModel(0);
        char buffer[256];
        sprintf(buffer, "./test_frames/model/frame.%04i.ppm", 0);
        renderImage(buffer, 0, buildSceneModel);
        time_t time1 = time(NULL);
        cout << "Finished " << 1 << " low-poly model frame in " << int(difftime(time1, time0)) << " seconds." << endl;
      }
      else if (test == "window")
      {
        // Test low-poly helios model
        aperture = 0;
        antialias_samples = 1;
        xRes = 640;
        yRes = 480;
        time_t time0 = time(NULL);
        buildAggWall(0);
        char buffer[256];
        sprintf(buffer, "./test_frames/window.ppm");
        renderImage(buffer, 0, buildAggWall);
        time_t time1 = time(NULL);
        cout << "Finished " << 1 << " aggwall frame in " << int(difftime(time1, time0)) << " seconds." << endl;
      }
      else if (test == "spherelight")
      {
        // Basic rectangle texture with glossy reflection
        xRes = 640;
        yRes = 480;
        aperture = 0;
        buildSphereLightTest(0);
        char buffer[256];
        sprintf(buffer, "./test_frames/spherelight.ppm");
        renderImage(buffer, 0, buildSphereLightTest);
      }
      else if (test == "staircase")
      {
        // Basic rectangle texture with glossy reflection
        xRes = 640;
        yRes = 480;
        aperture = 0;
        antialias_samples = 1;
        for (int i = 0; i < 50; i++)
        {
          buildStaircaseTest(i);
          char buffer[256];
          sprintf(buffer, "./test_frames/staircase/staircase.%04i.ppm", i);
          renderImage(buffer, 0, buildStaircaseTest);
        }
      }
      else if (test == "rectprism")
      {
        // Basic rectangle texture with glossy reflection
        xRes = 640;
        yRes = 480;
        aperture = 0;
        antialias_samples = 1;
        // for (int i = 0; i < 20; i++)
        // {
        //   buildRectPrismV2Test(i);
        //   char buffer[256];
        //   sprintf(buffer, "./test_frames/RectPrismV2.%04i.ppm", i);
        //   renderImage(buffer, i, buildRectPrismV2Test);
        // }
        buildRectPrismV2Test(13);
        char buffer[256];
        sprintf(buffer, "./test_frames/rectprism.%04i.ppm", 13);
        renderImage(buffer, 13, buildRectPrismV2Test);
      }
      else if (test == "texture")
      {
        // For testing: turn off antialiasing and dof
        xRes = 640;
        yRes = 480;
        aperture = 0;
        antialias_samples = 1;
        BuildSceneRectangleTexture(0);
        char buffer[256];
        sprintf(buffer, "./test_frames/texture/frame.%04i.ppm", 0);
        renderImage(buffer, frame*8, BuildSceneRectangleTexture);
      }
      else if (test == "textureog")
      {
        // Basic rectangle texture with glossy reflection
        xRes = 640;
        yRes = 480;
        aperture = 0;
        antialias_samples = 1;
        BuildSceneRectangleTextureOG(0);
        char buffer[256];
        sprintf(buffer, "./test_frames/texture/og.ppm");
        renderImage(buffer, 0, BuildSceneRectangleTextureOG);
      }
    }
  }
  shapes.clear();
  lights.clear();
  return 0;
}
