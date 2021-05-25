//=================================================================================
// HELPER FUNCTIONS
//=================================================================================
#ifndef HELPERS
#define HELPERS

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
#include <filesystem>
// #define BOOST_FILESYSTEM_VERSION 3
// #define BOOST_FILESYSTEM_NO_DEPRECATED
// #include <boost/filesystem.hpp>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include "skeleton.h"
#include "displaySkeleton.h"
#include "SETTINGS.h"
#include "geometry.h"

using namespace std;

// EXTERNAL GLOBALS
// Stick-man classes
extern DisplaySkeleton displayer;
extern Skeleton* skeleton;
extern Motion* motion;

extern int xRes;
extern int yRes;
extern int max_depth;

// Camera settings
extern VEC3 eye;
extern VEC3 lookingAt;
extern VEC3 up;
extern float aspect;
extern float near; // distance to image plane
extern float fov;
extern float aperture; // diameter of sampling lens
extern float focal_length; // distance to focal plane

// Refraction
extern float refr_air;
extern float refr_glass;

// scene geometry
extern vector<shared_ptr<GeoPrimitive>> shapes;
extern vector<shared_ptr<LightPrimitive>> lights;
extern float phong;
extern VEC3 default_col;

// BVH settings
extern shared_ptr<BoundingVolume> bvh;
extern float c_isect; // Random assumption: traversal is 1/3 cost of ray-primitive intersection
extern float c_trav;

// Sampling settings
extern int antialias_samples;
extern int blur_samples;
extern mt19937 generator;
extern uniform_real_distribution<double> uniform;

// Texture
extern vector<vector<VEC3>> texture_frames;
extern vector<VEC2> texture_dims;
extern vector<string> frame_paths;

// Choreography
extern int frame_prism;
extern float far;

// Colors
extern VEC3 teal;
extern VEC3 pink;

namespace fs = std::filesystem;

// =========================================
// IO FUNCTIONS
// =========================================
// Push back texture image
void loadTexture(string f)
{
  int width, height, n;
  unsigned char* frame_data = stbi_load(f.c_str(), &width, &height, &n, 0);
  vector<VEC3> frame_tmp;
  if (frame_data != NULL)
  {
    for (int i = 0; i < width*height; i++)
    {
      // Note: STBI data 0-255
      frame_tmp.push_back(VEC3(frame_data[i*n]/255.0, frame_data[i*n + 1]/255.0, frame_data[i*n + 2]/255.0));
    }
    texture_frames.push_back(frame_tmp);
    texture_dims.push_back(VEC2(width, height));
    stbi_image_free(frame_data);
  }
  else
  {
    cout << "Image loading failed for: " << f << endl;
    throw;
  }
}

/**
 * \brief   Return the filenames of all files that have the specified extension
 *          in the specified directory and all subdirectories.
 */
std::vector<std::string> get_all(std::string const& path, std::string const& ext)
{
    std::vector<std::string> paths;
    int base = 1;
    if (fs::exists(path) && fs::is_directory(path))
    {
        for (auto const& entry : fs::recursive_directory_iterator(path))
        {
            if (fs::is_regular_file(entry) && entry.path().extension() == ext)
            {
              paths.push_back(path + "/" + to_string(base) + ".jpg");
              base++;
            }
        }
    }
    return paths;
}

void readPPM(const string& filename, int& xRes, int& yRes, float*& values)
{
  // try to open the file
  FILE *fp;
  fp = fopen(filename.c_str(), "rb");
  if (fp == NULL)
  {
    cout << " Could not open file \"" << filename.c_str() << "\" for reading." << endl;
    cout << " Make sure you're not trying to read from a weird location or with a " << endl;
    cout << " strange filename. Bailing ... " << endl;
    exit(0);
  }

  // get the dimensions
  unsigned char newline;
  fscanf(fp, "P6\n%d %d\n255%c", &xRes, &yRes, &newline);
  if (newline != '\n') {
    cout << " The header of " << filename.c_str() << " may be improperly formatted." << endl;
    cout << " The program will continue, but you may want to check your input. " << endl;
  }
  int totalCells = xRes * yRes;

  // grab the pixel values
  unsigned char* pixels = new unsigned char[3 * totalCells];
  fread(pixels, 1, totalCells * 3, fp);

  // copy to a nicer data type
  values = new float[3 * totalCells];
  for (int i = 0; i < 3 * totalCells; i++)
    values[i] = pixels[i];

  // clean up
  delete[] pixels;
  fclose(fp);
  cout << " Read in file " << filename.c_str() << endl;
}

void writePPM(const string& filename, int& xRes, int& yRes, const float* values)
{
  int totalCells = xRes * yRes;
  unsigned char* pixels = new unsigned char[3 * totalCells];
  for (int i = 0; i < 3 * totalCells; i++)
    pixels[i] = values[i];

  FILE *fp;
  fp = fopen(filename.c_str(), "wb");
  if (fp == NULL)
  {
    cout << " Could not open file \"" << filename.c_str() << "\" for writing." << endl;
    cout << " Make sure you're not trying to write from a weird location or with a " << endl;
    cout << " strange filename. Bailing ... " << endl;
    exit(0);
  }

  fprintf(fp, "P6\n%d %d\n255\n", xRes, yRes);
  fwrite(pixels, 1, totalCells * 3, fp);
  fclose(fp);
  delete[] pixels;
}

// =========================================
// ALGORITHMS
// =========================================
// Frustum culling
// void frustumCull()
// {
//   for (int i = 0; i < shapes.size(); i++)
//   {
//     VEC3 lbound, ubound;
//     shapes[i]->getBounds(lbound, ubound);
//     // Define image plane rectange: if line between bounds misses frustum then outside
//     float t = tan(fov * M_PI/360.0) * abs(near);
//     float r = aspect * t;
//     VEC3 center = (lookingAt - eye).normalized() * abs(near) + eye;
//     VEC3 left = (lookingAt - eye).cross(up);
//     Rectangle nearplane = Rectangle(center + up * t + r * left, center - up * t + r * left,
//                                       center - up*t - r*left, center + up*t - r*left, VEC3(0,0,0));
//     Rectangle farplane = Rectangle(center + up * t * far + r * left * far, center - up * t*far + r * left*far,
//                                       center - up*t*far - r*left*far, center + up*t*far - r*left*far, VEC3(0,0,0));
//
//   }
// }

// Rotate point about any axis
// NOTE: MUST BE CENTERED AROUND ORIGIN FIRST
VEC3 rotate(VEC3 point, VEC3 axis, float theta)
{
  VEC3 anorm = axis.normalized();
  MATRIX3 R; R << cos(theta) + pow(anorm[0],2)*(1-cos(theta)), anorm[0]*anorm[1]*(1-cos(theta)) - anorm[2]*sin(theta), anorm[0]*anorm[2]*(1-cos(theta))+anorm[1]*sin(theta),
    anorm[1]*anorm[0]*(1-cos(theta))+anorm[2]*sin(theta), cos(theta)+pow(anorm[1],2)*(1-cos(theta)), anorm[1]*anorm[2]*(1-cos(theta))-anorm[0]*sin(theta),
    anorm[2]*anorm[1]*(1-cos(theta))-anorm[1]*sin(theta), anorm[2]*anorm[1]*(1-cos(theta))+anorm[0]*sin(theta), cos(theta)+pow(anorm[2],2)*(1-cos(theta));
  return(R*point);
}

float clamp(float value)
{
  if (value < 0.0)      return 0.0;
  else if (value > 1.0) return 1.0;
  return value;
}

void remap(float& value, float start, float end, float new_start, float new_end)
{
  value = (value - start)/(end - start) * (new_end - new_start) + new_start;
}

// Order shape indices increasing along axis: quicksort
void orderIndex(vector<int>& indices, int axis, int low, int high)
{
  if (low < high)
  {
    // Create partition
    float pivot = shapes[indices[high]]->center[axis];
    int i = low;
    for (int j = low; j < high; j++)
    {
      if (shapes[indices[j]]->center[axis] < pivot)
      {
        int tmp = indices[j];
        indices[j] = indices[i];
        indices[i] = tmp;
        i++;
      }
    }
    int tmp = indices[i];
    indices[i] = indices[high];
    indices[high] = tmp;

    orderIndex(indices, axis, low, i-1);
    orderIndex(indices, axis, i+1, high);
  }
}

void shuffle(vector<VEC3>& vec)
{
  for (int i = vec.size()-1; i > 0; i--)
  {
    int j = round(uniform(generator) * i);
    VEC3 tmp = vec[i];
    vec[i] = vec[j];
    vec[j] = tmp;
  }
}

// =========================================
// PHYSICS
// =========================================
bool getRefractionRay(VEC3& out, const VEC3 in, const VEC3 normal, const float sin_theta, const float cos_theta, const float refr_1, const float refr_2)
{
  float int_refl_check = 1 - pow(refr_1/refr_2, 2) * (1 - pow(in.dot(normal), 2));
  if (int_refl_check < 0)
  {
    return false;
  }
  out = (refr_1/refr_2 * sin_theta) * (1/sin_theta * (in + normal * cos_theta)) - sqrt(int_refl_check) * normal;
  return true;
}


// Fresnel equation
void fresnel(const float cos_theta, const float cos_phi, const float refr_1, const float refr_2, float& k_refl, float& k_refr)
{
  float rho_par = (refr_2 * cos_theta - refr_1 * cos_phi)/(refr_2 * cos_theta + refr_1 * cos_phi);
  float rho_perp = (refr_1 * cos_theta - refr_2 * cos_phi)/(refr_1 * cos_theta + refr_2 * cos_phi);
  k_refl = 0.5 * (pow(rho_par, 2) + pow(rho_perp, 2));
  k_refr = 1 - k_refl;
}

// Schlick approximation (reflectance) with air as n1
void schlick(const float cos_theta, const float refr, float& k_refl)
{
  float R0 = pow((1 - refr)/(1 + refr), 2);
  k_refl = R0 + (1 - R0) + pow(1 - cos_theta, 5);
}

// Schlick with complex refraction index
void schlick(const float cos_theta, const VEC2 refr, float& k_refl)
{
  float R0 = (pow(refr[0] - 1, 2) + pow(refr[1], 2))/(pow(refr[0] + 1, 2) + pow(refr[1], 2));
  k_refl = R0 + (1 - R0) + pow(1 - cos_theta, 5);
}

// Pixel coordinate with perspective
VEC3 getPerspEyeRay(float l, float r, float t, float b, int i, int j, VEC3 u, VEC3 v, VEC3 w)
{
  VEC3 S = (l + (r-l)*i/xRes) * u + (b + (t-b) * j/yRes) * v - near * w;
  return S;
}

// =========================================
// BVH FUNCTIONS
// =========================================
// Approximate bounds of volume that bounds shapes in indices: using centroids
void getBounds(const vector<int> indices, VEC2& x, VEC2& y, VEC2& z)
{
  x = VEC2(FLT_MAX, FLT_MIN);
  y = VEC2(FLT_MAX, FLT_MIN);
  z = VEC2(FLT_MAX, FLT_MIN);
  for (int ind : indices)
  {
    if (shapes[ind]->center[0] < x[0])
    {
      x[0] = shapes[ind]->center[0];
    }
    if (shapes[ind]->center[0] > x[1])
    {
      x[1] = shapes[ind]->center[0];
    }
    if (shapes[ind]->center[1] < y[0])
    {
      y[0] = shapes[ind]->center[1];
    }
    if (shapes[ind]->center[1] > y[1])
    {
      y[1] = shapes[ind]->center[1];
    }
    if (shapes[ind]->center[2] < z[0])
    {
      z[0] = shapes[ind]->center[2];
    }
    if (shapes[ind]->center[2] > z[1])
    {
      z[1] = shapes[ind]->center[2];
    }
  }
}

// Compute cost using surface area heuristic of the child node split
float getSAH(const vector<int> v1, const vector<int> v2, const float base_area)
{
  VEC2 xbounds;
  VEC2 ybounds;
  VEC2 zbounds;
  getBounds(v1, xbounds, ybounds, zbounds);
  float v1_cost = ((xbounds[1]-xbounds[0]) * (ybounds[1]-ybounds[0]) * 2 + (xbounds[1]-xbounds[0]) * (zbounds[1]-zbounds[0]) * 2
                    + (ybounds[1]-ybounds[0]) * (zbounds[1]-zbounds[0]) * 2)/base_area * v1.size();
  getBounds(v2, xbounds, ybounds, zbounds);
  float v2_cost = ((xbounds[1]-xbounds[0]) * (ybounds[1]-ybounds[0]) * 2 + (xbounds[1]-xbounds[0]) * (zbounds[1]-zbounds[0]) * 2
                    + (ybounds[1]-ybounds[0]) * (zbounds[1]-zbounds[0]) * 2)/base_area * v2.size();
  float cost = c_trav + c_isect * (v1_cost + v2_cost);
  return cost;
}

// Generate BVH: implement surface-area heuristic, largest extent heuristic for splitting decision
shared_ptr<BoundingVolume> generateBVH(vector<int> indices)
{
  if (indices.size() == 1)
  {
    return make_shared<BoundingVolume>(BoundingVolume(indices, shapes, true));
  }
  else
  {
    // Get axis bounds defined by indices (shapes approximated by centers)
    VEC2 xbounds;
    VEC2 ybounds;
    VEC2 zbounds;
    getBounds(indices, xbounds, ybounds, zbounds);

    // Largest extent heuristic: choice of axis
    VEC3 extent(xbounds[1]-xbounds[0], ybounds[1]-ybounds[0], zbounds[1]-zbounds[0]);
    int axis = 0;
    if (extent[1] > extent[0])
    {
      if (extent[2] > extent[1])
      {
        axis = 2;
      }
      else
      {
        axis = 1;
      }
    }
    else if (extent[2] > extent[0]) { axis = 2;}

    // If axis bounds under threshold, then create leaf
    if (extent[axis] < 1e-3)
    {
      return make_shared<BoundingVolume>(BoundingVolume(indices, shapes, true));
    }

    // Reorder vector index by axis choice
    orderIndex(indices, axis, 0, indices.size()-1);

    // Create parent node
    shared_ptr<BoundingVolume> tmp = make_shared<BoundingVolume>(BoundingVolume(indices, shapes, false));

    // If number of indices small, then just divide manually
    if (indices.size() == 2)
    {
      tmp->nodes.push_back(generateBVH(vector<int>(1, indices[0])));
      tmp->nodes.push_back(generateBVH(vector<int>(1, indices[1])));
      return tmp;
    }
    if (indices.size() == 3)
    {
      tmp->nodes.push_back(generateBVH(vector<int>(1, indices[0])));
      tmp->nodes.push_back(generateBVH(vector<int>(indices.begin()+1, indices.end())));
      return tmp;
    }
    if (indices.size() == 4)
    {
      tmp->nodes.push_back(generateBVH(vector<int>(indices.begin(), indices.begin()+2)));
      tmp->nodes.push_back(generateBVH(vector<int>(indices.begin()+2, indices.end())));
      return tmp;
    }

    // Sweep through all planes and apply surface area heuristic
    // Surface area heuristic: approximate child boxes as leaves
    // We already know surface area of parent node
    float base_area = extent[0] * extent[1] * 2 + extent[1] * extent[2] * 2 + extent[0] * extent[2] * 2;
    float sah_cost = FLT_MAX;
    int slice = 1;
    for (int i = 1; i < indices.size()-1; i++)
    {
      float tmp_cost;
      tmp_cost = getSAH(vector<int>(indices.begin(), indices.begin()+i),
                          vector<int>(indices.begin()+i, indices.end()), base_area);
      if (tmp_cost < sah_cost)
      {
        sah_cost = tmp_cost;
        slice = i;
      }
    }

    // Termination criterion: if estimated minimized cost is under cost of being a leaf
    if (c_isect * indices.size() <= sah_cost)
    {
      return make_shared<BoundingVolume>(BoundingVolume(indices, shapes, true));
    }

    // Create child nodes
    tmp->nodes.push_back(generateBVH(vector<int>(indices.begin(), indices.begin() + slice)));
    tmp->nodes.push_back(generateBVH(vector<int>(indices.begin() + slice, indices.end())));
    return tmp;
  }
}

void printBVH(shared_ptr<BoundingVolume> bvh, bool bounds = false)
{
  vector<shared_ptr<BoundingVolume>> bvh_stack;
  vector<int> shape_inds;
  bvh_stack.push_back(bvh);
  printf("Root\n");
  while (bvh_stack.size() > 0)
  {
    shared_ptr<BoundingVolume> bvh_tmp = bvh_stack.back();
    bvh_stack.pop_back();
    if (bvh_tmp->leaf == true && bvh_tmp->indices.size() > 0)
    {
      printf("Leaf: ");
      for (int i : bvh_tmp->indices)
      {
        printf("%d ", i);
        cout << shapes[i]->name << " ";
      }
      printf("\n");
      if (bounds == true)
      {
        cout << "Box lower bounds: " << bvh_tmp->lbound << endl;
        cout << "Box upper bounds: " << bvh_tmp->ubound << endl;
      }
      shape_inds.insert(shape_inds.end(), bvh_tmp->indices.begin(), bvh_tmp->indices.end());
    }
    else if (bvh_tmp->nodes.size() > 0)
    {
      printf("Interior node: %f\n", (float) bvh_tmp->nodes.size());
      bvh_stack.insert(bvh_stack.end(), bvh_tmp->nodes.begin(), bvh_tmp->nodes.end());
    }
  }
}

// Count shapes in bvh
void countBVH(shared_ptr<BoundingVolume> bvh)
{
  vector<shared_ptr<BoundingVolume>> bvh_stack;
  vector<int> shape_inds;
  bvh_stack.push_back(bvh);
  while (bvh_stack.size() > 0)
  {
    shared_ptr<BoundingVolume> bvh_tmp = bvh_stack.back();
    bvh_stack.pop_back();
    if (bvh_tmp->leaf == true && bvh_tmp->indices.size() > 0)
    {
      shape_inds.insert(shape_inds.end(), bvh_tmp->indices.begin(), bvh_tmp->indices.end());
    }
    else if (bvh_tmp->nodes.size() > 0)
    {
      bvh_stack.insert(bvh_stack.end(), bvh_tmp->nodes.begin(), bvh_tmp->nodes.end());
    }
  }
  printf("Total number of shapes in BVH: %d\n", shape_inds.size());
}

void bumpBVH(shared_ptr<BoundingVolume> bvh, float val)
{
  vector<shared_ptr<BoundingVolume>> bvh_stack;
  bvh_stack.push_back(bvh);
  while (bvh_stack.size() > 0)
  {
    shared_ptr<BoundingVolume> bvh_tmp = bvh_stack.back();
    bvh_stack.pop_back();
    if (bvh_tmp->leaf == true && bvh_tmp->indices.size() > 0)
    {
      bvh_tmp->lbound[1] -= val;
      bvh_tmp->ubound[1] += val;
      // bvh_tmp->lbound[0] -= val;
      // bvh_tmp->ubound[0] += val;
      // bvh_tmp->lbound[2] -= val;
      // bvh_tmp->ubound[2] += val;
    }
    else if (bvh_tmp->nodes.size() > 0)
    {
      bvh_stack.insert(bvh_stack.end(), bvh_tmp->nodes.begin(), bvh_tmp->nodes.end());
    }
  }
}

// Clear BVH
void clearBVH(shared_ptr<BoundingVolume> bvh)
{
  vector<shared_ptr<BoundingVolume>> bvh_stack;
  bvh_stack.push_back(bvh);
  while (bvh_stack.size() > 0)
  {
    shared_ptr<BoundingVolume> bvh_tmp = bvh_stack.back();
    bvh_stack.pop_back();
    if (bvh_tmp->leaf == true && bvh_tmp->indices.size() > 0)
    {
      bvh_tmp.reset();
    }
    else if (bvh_tmp->nodes.size() > 0)
    {
      bvh_stack.insert(bvh_stack.end(), bvh_tmp->nodes.begin(), bvh_tmp->nodes.end());
    }
  }
  bvh_stack.clear();
}

#endif
