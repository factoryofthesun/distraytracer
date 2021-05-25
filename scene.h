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
#include "helpers.h"

using namespace std;

// EXTERNAL GLOBALS
// Stick-man classes
extern DisplaySkeleton displayer;
extern Skeleton* skeleton;
extern Motion* motion;

extern int xRes;
extern int yRes;
extern int max_depth;
extern bool use_model;

// Camera settings
extern VEC3 eye;
extern VEC3 lookingAt;
extern VEC3 up;
extern float aspect;
extern float near; // distance to image plane
extern float fov;
extern float aperture; // diameter of sampling lens
extern float focal_length; // distance to focal plane

// Reflection
extern vector<string> refl_materials;
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
mt19937 gen(0); // deterministic engine for triangle textures

// Texture
extern vector<vector<VEC3>> texture_frames;
extern vector<VEC2> texture_dims;
extern vector<string> frame_paths;

// Choreography
extern int frame_prism;
extern int frame_cloud;
extern int frame_blur;
extern int frame_range;
extern int frame_start;
extern int frame_move1;
extern int frame_move2;
extern int frame_sculp;
extern int total;
extern float far;
extern float tot_move;
extern float move_per_frame;
extern float accel_t;
extern VEC3 cap_center;

// Colors
extern VEC3 teal;
extern VEC3 pink;
extern VEC3 pastelpink;
extern VEC3 halogen;
extern VEC3 sunorange;
extern VEC3 violet;
extern VEC3 indigo;
extern VEC3 darkblue;
extern VEC3 maroonq;
extern VEC3 skyblue;

// Cloud parameters
extern bool perlin_cloud; // Indicator for creating perlin cloud default colors
extern float saturation;
extern float clouddist;
extern float cloudhoff; // height cutoff
extern VEC3 sun_outer;
extern VEC3 sun_inner;
extern VEC3 sun_core;
extern VEC3 bluesky;
extern VEC3 redsky;
//=================================================================================
// SCENE FUNCTIONS
//=================================================================================

//////////////////////////////////////////////////////////////////////////////////
// Load up a new motion captured frame
//////////////////////////////////////////////////////////////////////////////////
void setSkeletonsToSpecifiedFrame(int frameIndex)
{
  if (frameIndex < 0)
  {
    printf("Error in SetSkeletonsToSpecifiedFrame: frameIndex %d is illegal.\n", frameIndex);
    exit(0);
  }
  if (displayer.GetSkeletonMotion(0) != NULL)
  {
    int postureID;
    if (frameIndex >= displayer.GetSkeletonMotion(0)->GetNumFrames())
    {
      cout << " We hit the last frame! You might want to pick a different sequence. " << endl;
      postureID = displayer.GetSkeletonMotion(0)->GetNumFrames() - 1;
    }
    else
      postureID = frameIndex;
    displayer.GetSkeleton(0)->setPosture(* (displayer.GetSkeletonMotion(0)->GetPosture(postureID)));
  }
}

// Create mesh of rectangles in shape of triangle prism
// 6 rectangles per side: scaled to triangle prism size
// Rectangles separated by 7 bounding strips
// Each frame pulls up the mesh by 1/210 * prism_length distance
// TODO: slowly rotate mesh by axis (y-axis?)
void generateTrianglePrismMesh(VEC3 a_cap0, VEC3 b_cap0, VEC3 c_cap0, VEC3 a_cap1, VEC3 b_cap1, VEC3 c_cap1, int time_frame,
                                bool texture = false, bool motion = false)
{
  uniform_int_distribution<> unif(0, int(frame_paths.size())-1000);

  // TODO: Make bottom triangle cap a light???
  shapes.push_back(make_shared<Triangle>(a_cap1, b_cap1, c_cap1, VEC3(1, 1, 1)));
  // Ball light in the center of mesh
  // VEC3 flux_color(1, 0.756, 0.518);
  // VEC3 mesh_center = ((a_cap1 + b_cap1 + c_cap1)/3 - (a_cap0 + b_cap0 + c_cap0)/3)/2 + (a_cap0 + b_cap0 + c_cap0)/3;
  // shared_ptr<sphereLight> halogen_ball = make_shared<sphereLight>(mesh_center, 0.3, flux_color);
  // lights.push_back(halogen_ball);
  // shapes.push_back(halogen_ball);

  // Point light in center of mesh
  // VEC3 mesh_center = ((a_cap1 + b_cap1 + c_cap1)/3 - (a_cap0 + b_cap0 + c_cap0)/3)/2 + (a_cap0 + b_cap0 + c_cap0)/3;
  // lights.push_back(make_shared<pointLight>(mesh_center, VEC3(1,1,1)));

  int n_rect = 4; // # of rectangles for each side of prism
  float bounding_width = 0.1;
  float rect_height = ((a_cap0 - b_cap0).norm() - bounding_width * n_rect)/n_rect;
  float rect_width = float(5)/3 * rect_height;

  VEC3 length_v = (c_cap1 - c_cap0).normalized();
  VEC3 ac_v = (a_cap0 - c_cap0).normalized();
  VEC3 ab_v = (a_cap0 - b_cap0).normalized();
  VEC3 bc_v = (c_cap0 - b_cap0).normalized();

  VEC3 left_a = b_cap0 + bounding_width * length_v;
  VEC3 left_b = left_a + rect_width * length_v;
  VEC3 left_c = left_b + rect_height * ab_v;
  VEC3 left_d = left_a + rect_height * ab_v;

  // Note: We will be viewing rectangles from INSIDE so right side will need to have clockwise vetices
  VEC3 right_b = c_cap0 + bounding_width * length_v;
  VEC3 right_a = right_b + rect_width * length_v;
  VEC3 right_d = right_a + rect_height * ac_v;
  VEC3 right_c = right_b + rect_height * ac_v;

  VEC3 bottom_a = b_cap0 + bounding_width * length_v + bounding_width * bc_v;
  VEC3 bottom_b = bottom_a + rect_width * length_v;
  VEC3 bottom_c = bottom_b + rect_height * bc_v;
  VEC3 bottom_d = bottom_a + rect_height * bc_v;

  VEC3 adj_b0 = b_cap0 + bc_v * bounding_width/2;
  int tot = 0;
  int seed_counter = 0;
  while ((bottom_b - adj_b0).norm() <= (c_cap1 - c_cap0).norm())
  {
    // TODO: Skip below loop if furthest points are BEHIND eye (y-threshold)
    for (int i = 0; i < n_rect; i++)
    {
      // SKIP CONDITIONS: Rectangle behind eye OR rectangle too far away (<1 pixel size)
      if (!(left_b[1] > eye[1] && left_c[1] > eye[1]) & !((left_b - eye).norm() > far && (left_c - eye).norm() > far))
      {
        gen.seed(seed_counter);
        int frame_ind = unif(gen) + (time_frame - frame_prism);
        // Read in image for frame
        loadTexture(frame_paths[frame_ind]);
        int tex_index = texture_frames.size()-1;
        shared_ptr<Rectangle> tmp = make_shared<Rectangle>(left_a + (bounding_width + rect_height) * ab_v * i,
        left_b + (bounding_width + rect_height) * ab_v * i, left_c + (bounding_width + rect_height) * ab_v * i,
        left_d + (bounding_width + rect_height) * ab_v * i, VEC3(0,1,0), "", motion, tex_index, "raw");
        tmp->motion = motion;
        tmp->texture = texture;
        shapes.push_back(make_shared<Rectangle>(*tmp));
        tot++;
      }
      seed_counter++;

      if (!(right_a[1] > eye[1] && right_d[1] > eye[1]) & !((right_a - eye).norm() > far && (right_d - eye).norm() > far))
      {
        gen.seed(seed_counter+1);
        int frame_ind = unif(gen) + (time_frame - frame_prism);
        // Read in image for frame
        loadTexture(frame_paths[frame_ind]);
        int tex_index = texture_frames.size()-1;
        shared_ptr<Rectangle> tmp = make_shared<Rectangle>(right_a + (bounding_width + rect_height) * ac_v * i,
        right_b + (bounding_width + rect_height) * ac_v * i, right_c + (bounding_width + rect_height) * ac_v * i,
        right_d + (bounding_width + rect_height) * ac_v * i, VEC3(0,1,0),"", motion, tex_index, "raw");
        tmp->motion = motion;
        tmp->texture = texture;
        shapes.push_back(make_shared<Rectangle>(*tmp));
        tot++;
      }
      seed_counter++;

      if (!(bottom_b[1] > eye[1] && bottom_c[1] > eye[1]) & !((bottom_b - eye).norm() > far && (bottom_c - eye).norm() > far))
      {
        gen.seed(seed_counter+2);
        int frame_ind = unif(gen) + (time_frame - frame_prism);
        // Read in image for frame
        loadTexture(frame_paths[frame_ind]);
        int tex_index = texture_frames.size()-1;
        shared_ptr<Rectangle> tmp = make_shared<Rectangle>(bottom_a + (bounding_width + rect_height) * bc_v * i,
        bottom_b + (bounding_width + rect_height) * bc_v * i, bottom_c + (bounding_width + rect_height) * bc_v * i,
        bottom_d + (bounding_width + rect_height) * bc_v * i, VEC3(0,1,0),"", motion, tex_index, "raw");
        tmp->motion = motion;
        tmp->texture = texture;
        shapes.push_back(make_shared<Rectangle>(*tmp));
        tot++;
      }
      seed_counter++;
    }
    left_a += length_v * (rect_width + bounding_width);
    left_b += length_v * (rect_width + bounding_width);
    left_c += length_v * (rect_width + bounding_width);
    left_d += length_v * (rect_width + bounding_width);

    right_a += length_v * (rect_width + bounding_width);
    right_b += length_v * (rect_width + bounding_width);
    right_c += length_v * (rect_width + bounding_width);
    right_d += length_v * (rect_width + bounding_width);

    bottom_a += length_v * (rect_width + bounding_width);
    bottom_b += length_v * (rect_width + bounding_width);
    bottom_c += length_v * (rect_width + bounding_width);
    bottom_d += length_v * (rect_width + bounding_width);
  }
  printf("Pushed back %d rectangles to make mesh\n", tot);
}

// FINAL MODEL BUILDS + TRANSFORMS
void finalBuildModels(float frame)
{
  float min_y = 0.301897 + tot_move;
  float xmin = -0.1;
  float xmax = 1.7;
  float zmin = -0.4;
  float zmax = 1.9;

  VEC3 A(xmin, min_y, zmax);
  VEC3 B((xmin+xmax)/2, min_y, zmax);
  VEC3 C((xmin+xmax)/2, min_y, zmin);
  VEC3 D(xmin, min_y, zmin);
  VEC3 E(xmax, min_y, zmax);
  VEC3 F(xmax, min_y, zmin);

  VEC3 globalA(-10, min_y, -10);
  VEC3 globalB(-10, min_y, 20);
  VEC3 globalC(30, min_y, 20);
  VEC3 globalD(30, min_y, -10);
  VEC3 globalE(-10, 15, -10);
  VEC3 globalF(-10, 15, 20);
  VEC3 globalG(30, 15, 20);
  VEC3 globalH(30, 15, -10);

  // LOAD MODELS ====================================
  // Position: STRADDLING WINDOW
  // Load model vertices
  vector<VEC3> vertices;
  vector<VEC2> texcoords;
  vector<VEC3I> v_inds;
  vector<VEC3I> t_inds;
  vector<VEC3> normals;
  vector<VEC3I> n_inds;

  loadObj("./models/Column_LP_obj/Column_LP.obj", "./models/Column_LP_obj", vertices, v_inds, texcoords, t_inds, normals, n_inds);
  printf("Pedestal ===========\n# of vertices: %d, # of texcoords: %d, # of triangles: %d\n",
            int(vertices.size()), int(texcoords.size()), int(v_inds.size()));
  // Pedestal transform: scale up by 5x, move to right wall
  // Note: we can apply scaling matrix directly because bottom is at WORLD ORIGIN
  MATRIX4 column_transf; column_transf << 3, 0, 0, -3,
                                          0, 3, 0, min_y,
                                          0, 0, 3, 5,
                                          0, 0, 0, 1;
  vector<VEC3> pverts1;
  for (int i = 0; i < vertices.size(); i++)
  {
    VEC4 tmp; tmp << vertices[i], 1;
    VEC4 tmp2 = column_transf * tmp;
    pverts1.push_back(tmp2.head<3>());
  }

  // Keep track of shape bounds
  float x_min = FLT_MAX;
  float y_min = FLT_MAX;
  float z_min = FLT_MAX;
  float x_max = FLT_MIN;
  float y_max = FLT_MIN;
  float z_max = FLT_MIN;

  // Texture data
  loadTexture("./models/Column_LP_obj/Textures/Marble_Base_Color.jpg");
  int width, height, n;
  string rough_file = "./models/Column_LP_obj/Textures/Marble_Roughness.jpg";
  unsigned char* frame_data = stbi_load(rough_file.c_str(), &width, &height, &n, 0);
  for (int i = 0; i < v_inds.size(); i++)
  {
    VEC3I v_ind = v_inds[i];
    // Increase shape size by 3
    VEC3 tmp_center = (pverts1[v_ind[0]] + pverts1[v_ind[1]] + pverts1[v_ind[2]])/3;

    shared_ptr<Triangle> tmp = make_shared<Triangle>(pverts1[v_ind[0]],
                                          pverts1[v_ind[1]],
                                          pverts1[v_ind[2]],
                                          VEC3(0.75, 0.75, 0.75), "marble", false, "oren-nayar");

    // Texture coordinates can sometimes be over 1: repeat texture in that case
    VEC2 uvA = texcoords[t_inds[i][0]];
    VEC2 uvB = texcoords[t_inds[i][1]];
    VEC2 uvC = texcoords[t_inds[i][2]];
    if (uvA[0] > 1) uvA[0] = uvA[0] - int(uvA[0]);
    if (uvA[1] > 1) uvA[1] = uvA[1] - int(uvA[1]);
    if (uvB[0] > 1) uvB[0] = uvB[0] - int(uvB[0]);
    if (uvB[1] > 1) uvB[1] = uvB[1] - int(uvB[1]);
    if (uvC[0] > 1) uvC[0] = uvC[0] - int(uvC[0]);
    if (uvC[1] > 1) uvC[1] = uvC[1] - int(uvC[1]);

    // Check that texcoords are within bounds
    if (!(uvA[0] >= 0 && uvA[1] <= 1 &&
          uvB[0] >= 0 && uvB[1] <= 1 &&
          uvC[0] >= 0 && uvC[1] <= 1))
    {
      cout << "A: " << uvA << endl;
      cout << "B: " << uvB << endl;
      cout << "C: " << uvC << endl;

      printf("Texcoords out of bounds\n");
      throw;
    }

    // Test: maybe flip Y value of UV
    uvA[1] = 1 - uvA[1];
    uvB[1] = 1 - uvB[1];
    uvC[1] = 1 - uvC[1];

    // Set vertex UVs
    tmp->uv_verts = true;
    tmp->uvA = uvA;
    tmp->uvB = uvB;
    tmp->uvC = uvC;
    tmp->tex_frame = texture_frames.size()-1;
    tmp->texture = true;

    // Set roughness based on roughness map
    // TODO: FIGURE OUT HOW TO ASSIGN THIS -- IS IT INTERPOLATED ACROSS VERTICES?
    // Just assign average roughness across texcoords for now
    float r1 = frame_data[int(texcoords[t_inds[i][0]][0] * int(width-1) + texcoords[t_inds[i][0]][1] * int(height-1) * (width-1))];
    float r2 = frame_data[int(texcoords[t_inds[i][1]][0] * int(width-1) + texcoords[t_inds[i][1]][1] * int(height-1) * (width-1))];
    float r3 = frame_data[int(texcoords[t_inds[i][2]][0] * int(width-1) + texcoords[t_inds[i][2]][1] * int(height-1) * (width-1))];
    tmp->reflect_params.roughness = (r1+r2+r3)/(3 * 255);

    shapes.push_back(make_shared<Triangle>(*tmp));
    x_min = min({x_min, float(vertices[v_ind[0]][0]), float(vertices[v_ind[1]][0]), float(vertices[v_ind[2]][0])});
    y_min = min({y_min, float(vertices[v_ind[0]][1]), float(vertices[v_ind[1]][1]), float(vertices[v_ind[2]][1])});
    z_min = min({z_min, float(vertices[v_ind[0]][2]), float(vertices[v_ind[1]][2]), float(vertices[v_ind[2]][2])});

    x_max = max({x_max, float(vertices[v_ind[0]][0]), float(vertices[v_ind[1]][0]), float(vertices[v_ind[2]][0])});
    y_max = max({y_max, float(vertices[v_ind[0]][1]), float(vertices[v_ind[1]][1]), float(vertices[v_ind[2]][1])});
    z_max = max({z_max, float(vertices[v_ind[0]][2]), float(vertices[v_ind[1]][2]), float(vertices[v_ind[2]][2])});
  }
  // Right straddle column
  column_transf << 3, 0, 0, 3,
                    0, 3, 0, min_y,
                    0, 0, 3, 5,
                    0, 0, 0, 1;
  vector<VEC3> pverts2;
  for (int i = 0; i < vertices.size(); i++)
  {
    VEC4 tmp; tmp << vertices[i], 1;
    VEC4 tmp2 = column_transf * tmp;
    pverts2.push_back(tmp2.head<3>());
  }
  for (int i = 0; i < v_inds.size(); i++)
  {
    VEC3I v_ind = v_inds[i];
    // Increase shape size by 3
    VEC3 tmp_center = (pverts2[v_ind[0]] + pverts2[v_ind[1]] + pverts2[v_ind[2]])/3;

    shared_ptr<Triangle> tmp = make_shared<Triangle>(pverts2[v_ind[0]],
                                          pverts2[v_ind[1]],
                                          pverts2[v_ind[2]],
                                          VEC3(0.75, 0.75, 0.75), "marble", false, "oren-nayar");

    // Texture coordinates can sometimes be over 1: repeat texture in that case
    VEC2 uvA = texcoords[t_inds[i][0]];
    VEC2 uvB = texcoords[t_inds[i][1]];
    VEC2 uvC = texcoords[t_inds[i][2]];
    if (uvA[0] > 1) uvA[0] = uvA[0] - int(uvA[0]);
    if (uvA[1] > 1) uvA[1] = uvA[1] - int(uvA[1]);
    if (uvB[0] > 1) uvB[0] = uvB[0] - int(uvB[0]);
    if (uvB[1] > 1) uvB[1] = uvB[1] - int(uvB[1]);
    if (uvC[0] > 1) uvC[0] = uvC[0] - int(uvC[0]);
    if (uvC[1] > 1) uvC[1] = uvC[1] - int(uvC[1]);

    // Check that texcoords are within bounds
    if (!(uvA[0] >= 0 && uvA[1] <= 1 &&
          uvB[0] >= 0 && uvB[1] <= 1 &&
          uvC[0] >= 0 && uvC[1] <= 1))
    {
      cout << "A: " << uvA << endl;
      cout << "B: " << uvB << endl;
      cout << "C: " << uvC << endl;

      printf("Texcoords out of bounds\n");
      throw;
    }

    // Test: maybe flip Y value of UV
    uvA[1] = 1 - uvA[1];
    uvB[1] = 1 - uvB[1];
    uvC[1] = 1 - uvC[1];

    // Set vertex UVs
    tmp->uv_verts = true;
    tmp->uvA = uvA;
    tmp->uvB = uvB;
    tmp->uvC = uvC;
    tmp->tex_frame = texture_frames.size()-1;
    tmp->texture = true;

    // Set roughness based on roughness map
    // TODO: FIGURE OUT HOW TO ASSIGN THIS -- IS IT INTERPOLATED ACROSS VERTICES?
    // Just assign average roughness across texcoords for now
    float r1 = frame_data[int(texcoords[t_inds[i][0]][0] * int(width-1) + texcoords[t_inds[i][0]][1] * int(height-1) * (width-1))];
    float r2 = frame_data[int(texcoords[t_inds[i][1]][0] * int(width-1) + texcoords[t_inds[i][1]][1] * int(height-1) * (width-1))];
    float r3 = frame_data[int(texcoords[t_inds[i][2]][0] * int(width-1) + texcoords[t_inds[i][2]][1] * int(height-1) * (width-1))];
    tmp->reflect_params.roughness = (r1+r2+r3)/(3 * 255);
    shapes.push_back(make_shared<Triangle>(*tmp));
  }

  // Left side of room (FOR BALL LIGHT)
  // column_transf << 5, 0, 0, 0,
  //                   0, 5, 0, min_y,
  //                   0, 0, 5, -3,
  //                   0, 0, 0, 1;
  // vector<VEC3> pverts3;
  // for (int i = 0; i < vertices.size(); i++)
  // {
  //   VEC4 tmp; tmp << vertices[i], 1;
  //   VEC4 tmp2 = column_transf * tmp;
  //   pverts3.push_back(tmp2.head<3>());
  // }
  // for (int i = 0; i < v_inds.size(); i++)
  // {
  //   VEC3I v_ind = v_inds[i];
  //   // Increase shape size by 3
  //   VEC3 tmp_center = (pverts3[v_ind[0]] + pverts3[v_ind[1]] + pverts3[v_ind[2]])/3;
  //
  //   shared_ptr<Triangle> tmp = make_shared<Triangle>(pverts3[v_ind[0]],
  //                                         pverts3[v_ind[1]],
  //                                         pverts3[v_ind[2]],
  //                                         VEC3(0.75, 0.75, 0.75), "marble", false, "oren-nayar");
  //
  //   // Texture coordinates can sometimes be over 1: repeat texture in that case
  //   VEC2 uvA = texcoords[t_inds[i][0]];
  //   VEC2 uvB = texcoords[t_inds[i][1]];
  //   VEC2 uvC = texcoords[t_inds[i][2]];
  //   if (uvA[0] > 1) uvA[0] = uvA[0] - int(uvA[0]);
  //   if (uvA[1] > 1) uvA[1] = uvA[1] - int(uvA[1]);
  //   if (uvB[0] > 1) uvB[0] = uvB[0] - int(uvB[0]);
  //   if (uvB[1] > 1) uvB[1] = uvB[1] - int(uvB[1]);
  //   if (uvC[0] > 1) uvC[0] = uvC[0] - int(uvC[0]);
  //   if (uvC[1] > 1) uvC[1] = uvC[1] - int(uvC[1]);
  //
  //   // Check that texcoords are within bounds
  //   if (!(uvA[0] >= 0 && uvA[1] <= 1 &&
  //         uvB[0] >= 0 && uvB[1] <= 1 &&
  //         uvC[0] >= 0 && uvC[1] <= 1))
  //   {
  //     cout << "A: " << uvA << endl;
  //     cout << "B: " << uvB << endl;
  //     cout << "C: " << uvC << endl;
  //
  //     printf("Texcoords out of bounds\n");
  //     throw;
  //   }
  //
  //   // Test: maybe flip Y value of UV
  //   uvA[1] = 1 - uvA[1];
  //   uvB[1] = 1 - uvB[1];
  //   uvC[1] = 1 - uvC[1];
  //
  //   // Set vertex UVs
  //   tmp->uv_verts = true;
  //   tmp->uvA = uvA;
  //   tmp->uvB = uvB;
  //   tmp->uvC = uvC;
  //   tmp->tex_frame = texture_frames.size()-1;
  //   tmp->texture = true;
  //
  //   // Set roughness based on roughness map
  //   // TODO: FIGURE OUT HOW TO ASSIGN THIS -- IS IT INTERPOLATED ACROSS VERTICES?
  //   // Just assign average roughness across texcoords for now
  //   float r1 = frame_data[int(texcoords[t_inds[i][0]][0] * int(width-1) + texcoords[t_inds[i][0]][1] * int(height-1) * (width-1))];
  //   float r2 = frame_data[int(texcoords[t_inds[i][1]][0] * int(width-1) + texcoords[t_inds[i][1]][1] * int(height-1) * (width-1))];
  //   float r3 = frame_data[int(texcoords[t_inds[i][2]][0] * int(width-1) + texcoords[t_inds[i][2]][1] * int(height-1) * (width-1))];
  //   tmp->reflect_params.roughness = (r1+r2+r3)/(3 * 255);
  //   shapes.push_back(make_shared<Triangle>(*tmp));
  // }
  stbi_image_free(frame_data);

  // TODO: SPHERE LIGHT ON PEDESTAL IN CENTER
  // shared_ptr<sphereLight> halogen_ball = make_shared<sphereLight>(VEC3(0, 6.5 + min_y + 1.5, -3), 1.5, halogen);
  // shapes.push_back(halogen_ball);
  // lights.push_back(halogen_ball);

  vertices.clear();
  v_inds.clear();
  texcoords.clear();
  t_inds.clear();
  normals.clear();
  n_inds.clear();

  loadObj("./models/helios_statue/helios_20.obj", "./models/helios_statue", vertices, v_inds, texcoords, t_inds, normals, n_inds);
  printf("Helios bust ============\n# of vertices: %d, # of texcoords: %d, # of triangles: %d, # of normals: %d\n",
            int(vertices.size()), int(texcoords.size()), int(v_inds.size()), int(normals.size()));
  // Rotate helios head 180 to face down negative z-axis + standard transform (3x scale)
  MATRIX4 bust_transf; bust_transf << 1.8 * cos(M_PI), 0, 1.8 * sin(M_PI), -3,
                                      0, 1.8, 0, 3.9 + min_y,
                                      -1.8 * sin(M_PI), 0, 1.8 * cos(M_PI), 4,
                                      0, 0, 0, 1;
  vector<VEC3> hvert1;
  for (int i = 0; i < vertices.size(); i++)
  {
    VEC4 tmp; tmp << vertices[i], 1;
    VEC4 tmp2 = bust_transf * tmp;
    hvert1.push_back(tmp2.head<3>());
  }

  // Load triangles: FIRST TWO ARE ROOT
  // Keep track of shape bounds
  for (int i = 2; i < v_inds.size(); i++)
  {
    VEC3I v_ind = v_inds[i];
    // Increase shape size by 3
    VEC3 tmp_center = (hvert1[v_ind[0]] + hvert1[v_ind[1]] + hvert1[v_ind[2]])/3;

    shared_ptr<Triangle> tmp = make_shared<Triangle>(hvert1[v_ind[0]],
                                          hvert1[v_ind[1]],
                                          hvert1[v_ind[2]],
                                          VEC3(0.75, 0.75, 0.75), "marble", false, "oren-nayar");
    tmp->reflect_params.roughness = 0.5;
    shapes.push_back(make_shared<Triangle>(*tmp));
  }

  // GLASS HELIOS: BOLD
  // SAVE NORMALS
  bust_transf << 1.8 * cos(M_PI), 0, 1.8 * sin(M_PI), 3,
                  0, 1.8, 0, 3.9 + min_y,
                  -1.8 * sin(M_PI), 0, 1.8 * cos(M_PI), 4,
                  0, 0, 0, 1;
  vector<VEC3> hvert2;
  for (int i = 0; i < vertices.size(); i++)
  {
    VEC4 tmp; tmp << vertices[i], 1;
    VEC4 tmp2 = bust_transf * tmp;
    hvert2.push_back(tmp2.head<3>());
  }

  // Load triangles: FIRST TWO ARE ROOT
  for (int i = 2; i < v_inds.size(); i++)
  {
    VEC3I v_ind = v_inds[i];
    // Increase shape size by 3
    VEC3 tmp_center = (hvert2[v_ind[0]] + hvert2[v_ind[1]] + hvert2[v_ind[2]])/3;

    shared_ptr<Triangle> tmp = make_shared<Triangle>(hvert2[v_ind[0]],
                                          hvert2[v_ind[1]],
                                          hvert2[v_ind[2]],
                                          VEC3(0.75, 0.75, 0.75), "marble", false, "oren-nayar");
    // tmp->mesh = true;
    // tmp->mesh_normal = normals[n_inds[i][0]]; // We only need the normal of one vertex
    tmp->reflect_params.roughness = 0.5;
    shapes.push_back(make_shared<Triangle>(*tmp));
  }
}

// FINAL BUILD
void buildFinal(float frame)
{
  // FINAL TIMINGS: GENERATE 70*8 EXTRA STATIC FRAMES IN BEGINNING
  // Beginning 960 (0-119) frames: first room
  //   - Eye starts at wall: facing
  // Middle 960 (120 - 239) frames: falling through triangle prism
  //    - Frames 180-239: exponential motion blur
  // Last 240 (240-299) frames: perlin cloud texture + stickman falling in distance

  // Perlin cloud texturing always on
  perlin_cloud = true;

  setSkeletonsToSpecifiedFrame(int(frame));

  // Explicitly free memory
  shapes.clear();
  lights.clear();

  displayer.ComputeBonePositions(DisplaySkeleton::BONES_AND_LOCAL_FRAMES);

  // Get bone names in skeleton
  Skeleton* skel = displayer.GetSkeleton(0);

  // retrieve all the bones of the skeleton
  vector<MATRIX4>& rotations = displayer.rotations();
  vector<MATRIX4>& scalings  = displayer.scalings();
  vector<VEC4>& translations = displayer.translations();
  vector<float>& lengths     = displayer.lengths();

  // Skip the first bone,
  // it's just the origin
  int totalBones = rotations.size();
  for (int x = 1; x < totalBones; x++)
  {
    MATRIX4& rotation = rotations[x];
    MATRIX4& scaling = scalings[x];
    VEC4& translation = translations[x];

    // get the endpoints of the cylinder
    VEC4 leftVertex(0,0,0,1);
    VEC4 rightVertex(0,0,lengths[x],1);

    leftVertex = rotation * scaling * leftVertex + translation;
    rightVertex = rotation * scaling * rightVertex + translation;

    // CLOUD FRAMES: drop the cylinder man (just apply negative y)
    if (frame >= frame_cloud)
    {
      leftVertex -= VEC4(0, (frame-frame_cloud), 0, 0);
      rightVertex -= VEC4(0, (frame-frame_cloud), 0, 0);
    }

    // Cylinders
    shapes.push_back(make_shared<Cylinder>(leftVertex.head<3>(), rightVertex.head<3>(), 0.05, VEC3(1,0,0)));
  }

  // CAMERA SETTINGS -----------------------
  // Choreography
  // Eye: starts with looking out window at ???
  // 480 frames (2 sec): Slow move backwards for 10 pixels (should see helios statues at this point)
  // 480 frames (2 sec): try the existing rotation code below
  // TODO: REDO EYE SETTINGS FOR PRE-FALL SCENE
  VEC3 init_eye(-7, 9, -4);
  VEC3 init_lookingAt(8, 11, 6);
  VEC3 int_eye(-3, 8, 2); // Set after end of first move
  VEC3 int_lookingAt(-5, 8, 0);
  VEC3 init_up(0,1,0);
  VEC3 final_eye(0.5, 8, 1.1);
  VEC3 final_lookingAt(0.5, 0.5, 1); //Set after frame_prism (4 seconds)
  VEC3 final_up(0,0,-1);

  if (frame <= frame_prism)
  {
    eye = init_eye;
    lookingAt = init_lookingAt;
    float final_theta = M_PI * 9/8;
    float theta = min(final_theta, frame * final_theta/frame_move1);
    eye = rotate(eye, VEC3(0,1,0), theta);
    // Adjust eye if outside bounds
    while (eye[0] < -10 || eye[0] > 10 || eye[2] < -5 || eye[2] > 8)
    {
      eye *= 0.999;
    }
    lookingAt = rotate(lookingAt, VEC3(0,1,0), theta);
    lookingAt -= VEC3(0, frame/frame_move1 * 10, 0);
  }
  if (frame <= frame_prism && frame >= frame_move1)
  {
    // Linear interpolation of eye and lookingat
    eye = eye + (final_eye - eye) * min(1.0, (double) (frame-frame_move1)/(frame_move2 - frame_move1));

    // Linear interpolate lookingat vector
    lookingAt = lookingAt + (final_lookingAt - lookingAt) * min(1.0, (double) (frame-frame_move1)/(frame_move2 - frame_move1));

    // Rotate "up" vector simultaneously (rotate around x axis to get z-axis)
    float theta = -M_PI/2 * min(1.0, (double) (frame-frame_move1)/(frame_move2 - frame_move1));
    up = rotate(up, VEC3(1,0,0), theta);
  }
  if (frame > frame_prism) // Set eye, up, and looking at to (almost) final positions (not including below acceleration)
  {
    eye = final_eye;
    lookingAt = final_lookingAt;
    up = VEC3(0,0,-1);
    focal_length = 20;
  }
  // GLOBALS ======================
  // MOVEMENT PARAMETERS
  float tunnel_transition = 20 * 8;
  float movement_multiplier = max(float(0), frame-frame_prism);
  // Slight acceleration to terminal velocity (0.5/8)
  // ALWAYS RESET: because of motion blur sampling
  move_per_frame = 0.1/8;
  move_per_frame *= (1 + min(float(2), 2 * (movement_multiplier)/tunnel_transition));
  tot_move = movement_multiplier * move_per_frame;

  // Accelerate for 2 seconds (motion blur ON) until clouds
  // We're going to need to adjust the prism height accordingly
  float accel_d = accel_t * pow(frame-frame_blur, 3);
  float dist = 263;
  if (frame > frame_blur && frame <= frame_cloud)
  {
    // Let's be more accurate about distances
    tot_move += accel_d;

    // Update velocity as well (for motion blur)
    move_per_frame += 0.1/(2 * 64) * pow(frame-frame_blur, 2);
  }

  // Trapdoor: NEVER TOUCH THESE
  float min_y = 0.301897 + tot_move;
  float xmin = -0.1;
  float xmax = 1.7;
  float zmin = -0.4;
  float zmax = 1.9;

  VEC3 A(xmin, min_y, zmax);
  VEC3 B((xmin+xmax)/2, min_y, zmax);
  VEC3 C((xmin+xmax)/2, min_y, zmin);
  VEC3 D(xmin, min_y, zmin);
  VEC3 E(xmax, min_y, zmax);
  VEC3 F(xmax, min_y, zmin);

  VEC3 globalA(-10, min_y, -5);
  VEC3 globalB(-10, min_y, 8);
  VEC3 globalC(10, min_y, 8);
  VEC3 globalD(10, min_y, -5);
  VEC3 globalE(-10, 10 + min_y, -5);
  VEC3 globalF(-10, 10 + min_y, 8);
  VEC3 globalG(10, 10 + min_y, 8);
  VEC3 globalH(10, 10 + min_y, -5);

  VEC3 globalEH = (globalE-globalH).normalized();
  VEC3 globalEF = (globalE-globalF).normalized();
  VEC3 globalAD = (globalA-globalD).normalized();
  VEC3 globalCD = (globalC - globalD).normalized();
  VEC3 globalcenter = (globalA + globalB + globalC + globalD + globalE + globalF + globalG + globalH)/8;

  // TUNNEL TRANSITION ===========================
  // Min calls ensure that these won't kick in prematurely
  // Movement: bound below by frame_prism (0) and above by tunnel_transition (160)
  // 2/3 second to transition into falling through tunnel
  // Accelerate eye towards body (lookingAt point also moves)
  VEC3 tunnel_point = VEC3((xmin+xmax)/2, 5, (zmin+zmax)/2);
  VEC3 eye_path = (tunnel_point - eye).normalized();
  float accel = (tunnel_point-eye).norm()/pow(tunnel_transition, 2);
  eye = accel * pow(min(tunnel_transition, movement_multiplier), 2) * eye_path + eye;
  lookingAt = accel * pow(min(tunnel_transition, movement_multiplier), 2) * eye_path + lookingAt;

  // Create rectangle mesh that forms triangle prism
  // Starting position: long axis is y-axis and top is right on the surface of the floor
  // Length: 133 (computed by taking velocy + acceleration into account)

  // Expand mesh by a lot: MUST MAKE EQUILATERAL TRIANGLE
  VEC3 b_cap0(xmin, min_y, zmax);
  VEC3 c_cap0(xmax, min_y, zmax);
  VEC3 a_cap0((xmin+xmax)/2, min_y, (xmin - xmax) * sqrt(3)/2 + zmax); // Set A such that triangle is equilateral

  cap_center = (a_cap0 + b_cap0 + c_cap0)/3;
  a_cap0 = (a_cap0 - cap_center) * 5 + cap_center;
  b_cap0 = (b_cap0 - cap_center) * 5 + cap_center;
  c_cap0 = (c_cap0 - cap_center) * 5 + cap_center;

  // Light source during tunnel (from eye)
  if (frame >= frame_prism + tunnel_transition)
  {
    lights.push_back(make_shared<pointLight>(eye, VEC3(1,1,1)));
  }
  if (frame >= frame_cloud)
  {
    // DONT NEED THIS ANYMORE
    aperture = 0;
    antialias_samples = 1;
    // Only have cylinder man left to render
    // EXPLODE after 80 frames

    redsky = redsky + (sunorange - redsky) * (frame-frame_cloud)/(total-frame_cloud);
    bluesky = bluesky + (pastelpink - bluesky) * (frame-frame_cloud)/(total-frame_cloud);
    sun_outer = sun_outer + (violet - sun_outer) * (frame-frame_cloud)/(total-frame_cloud);
    sun_inner = sun_inner + (indigo - sun_inner) * (frame-frame_cloud)/(total-frame_cloud);
    sun_core = sun_core + (darkblue - sun_core) * (frame-frame_cloud)/(total-frame_cloud);

    // if (frame - frame_cloud > 80)
    // {
    //   int frame_diff = int((frame-frame_cloud)/8) % 4;
    //   if (frame_diff == 0)
    //   {
    //     // Vanilla
    //     redsky = sunorange;
    //     bluesky = pastelpink;
    //     sun_outer = violet;
    //     sun_inner = indigo;
    //     sun_core = darkblue;
    //   }
    //   if (frame_diff == 1)
    //   {
    //     redsky = teal;
    //     bluesky = sunorange;
    //     sun_core = darkblue;
    //     sun_inner = indigo;
    //     sun_outer = violet;
    //   }
    //   if (frame_diff == 2)
    //   {
    //     redsky = violet;
    //     bluesky = darkblue;
    //     sun_core = halogen;
    //     sun_inner = sunorange;
    //     sun_outer = pastelpink;
    //   }
    //   if (frame_diff == 3)
    //   {
    //     redsky = pastelpink;
    //     bluesky = maroonq;
    //     sun_core = teal;
    //     sun_inner = skyblue;
    //     sun_outer = darkblue;
    //   }
    // }
    return;
  }

  // Pull up mesh by given velocity
  a_cap0 += VEC3(0, tot_move, 0);
  b_cap0 += VEC3(0, tot_move, 0);
  c_cap0 += VEC3(0, tot_move, 0);

  // Rotation: slow (180 every 3 seconds = 90 * 8 frames)
  float rot_theta = (movement_multiplier)/720.0 * M_PI;
  a_cap0 = rotate(a_cap0 - cap_center, VEC3(0,1,0), rot_theta) + cap_center;
  b_cap0 = rotate(b_cap0 - cap_center, VEC3(0,1,0), rot_theta) + cap_center;
  c_cap0 = rotate(c_cap0 - cap_center, VEC3(0,1,0), rot_theta) + cap_center;

  // Length of tunnel: 4 seconds == 120 frames
  VEC3 a_cap1 = a_cap0 - VEC3(0, dist, 0);
  VEC3 b_cap1 = b_cap0 - VEC3(0, dist, 0);
  VEC3 c_cap1 = c_cap0 - VEC3(0, dist, 0);

  // Print out scene settings
  // printf("Frame: %f ==========\n", frame);
  // printf("Movement multiplier: %f\n", movement_multiplier);
  // cout << "Eye: " << eye << endl;
  // cout << "lookingAt: " << lookingAt << endl;
  // cout << "Min y: " << min_y << endl;

  if (frame >= frame_prism)
  {
    generateTrianglePrismMesh(a_cap0, b_cap0, c_cap0, a_cap1, b_cap1, c_cap1, int(frame), true, true);
  }

  // Only render objects while they are still relevant
  if ((min_y + tot_move  <= eye[1] + 2) || frame < frame_prism + tunnel_transition)
  {
    // Rotate inwards about hinges AD, EF
    // TODO: MAKE THIS RECTANGULAR PRISM
    float angle = min(float(1.1), movement_multiplier/(tunnel_transition)) * M_PI/2;
    VEC3 B_left = rotate(B-A, D-A, angle) + A;
    VEC3 C_left = rotate(C-A, D-A, angle) + A;
    VEC3 B_right = rotate(B-E, F-E, -angle) + E;
    VEC3 C_right = rotate(C-E, F-E, -angle) + E;

    loadTexture("./textures/sad_finder1_adj.jpg");
    int tex_index = texture_frames.size()-1;
    shared_ptr<Rectangle> tmp = make_shared<Rectangle>(A, B_left, C_left, D, VEC3(0,0,0), "steel", false, tex_index, "cook-torrance");
    tmp->reflect_params.roughness = 0.7;
    tmp->reflect_params.refr = VEC2(2.75, 3.79);
    tmp->reflect_params.glossy = true;
    tmp->texture = true;
    shapes.push_back(make_shared<Rectangle>(*tmp));

    loadTexture("./textures/sad_finder2_adj.jpg");
    tex_index = texture_frames.size()-1;
    shared_ptr<Rectangle> tmp2 = make_shared<Rectangle>(B_right,E,F,C_right, VEC3(0,0,0), "steel", false, tex_index, "cook-torrance");
    tmp2->reflect_params.roughness = 0.7;
    tmp2->reflect_params.refr = VEC2(2.75, 3.79);
    tmp2->reflect_params.glossy = true;
    tmp2->texture = true;
    shapes.push_back(make_shared<Rectangle>(*tmp2));

    // OTHER INITIAL SCENE OBJECTS: indoors box
    // Floor: linoleum (texture applies to each SQUARE) + bounding lines
    loadTexture("./textures/floor.jpeg");
    tex_index = texture_frames.size()-1;
    float s = 1;
    VEC3 checker_rgb0(0.58, 0.82, 1);
    VEC3 checker_rgb1(1, 0.416, 0.835);
    shared_ptr<CheckerboardWithHole> floor = make_shared<CheckerboardWithHole>(globalA,
                                                globalB,
                                                globalC,
                                                globalD,
                                              checker_rgb0, checker_rgb1, s,
                                            make_shared<Rectangle>(A, E, F, D, VEC3(0,0,0)));
    floor->tex_frame = tex_index;
    floor->borderwidth = 0.05;
    floor->bordercolor = VEC3(0.55, 0.55, 0.55);
    floor->texture = true;
    floor->reflect_params.material = "linoleum";
    floor->reflect_params.roughness = 0.6;
    floor->reflect_params.refr = VEC2(1.543,0);
    floor->reflect_params.glossy = true;
    shapes.push_back(make_shared<CheckerboardWithHole>(*floor));

    // Walls
    shapes.push_back(make_shared<Rectangle>(globalA, globalD, globalH, globalE, VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<Rectangle>(globalA, globalB, globalF, globalE, VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<Rectangle>(globalD, globalC, globalG, globalH, VEC3(0,0.81,0.99)));

    // Right wall: aggregated prisms with window
    VEC3 height_vector = (globalF-globalB).normalized();
    float height = (globalF-globalB).norm();
    VEC3 length_vector = (globalB-globalC).normalized();
    float length = (globalC-globalB).norm();
    VEC3 width_vector = (globalB-globalA).normalized();
    float width = 2;

    VEC3 globalBp = globalB + width_vector * width;
    VEC3 globalCp = globalC + width_vector * width;
    VEC3 globalFp = globalF + width_vector * width;
    VEC3 globalGp = globalG + width_vector * width;

    // Window centered eye position: 8, 8 (x,y) -> spans (7-9)
    float window_size = 2;
    float mid_height = (height-window_size)/2;
    float mid_length = (length-window_size)/2;

    VEC3 a1 = globalC + height_vector * mid_height;
    VEC3 b1 = globalB + height_vector * mid_height;
    VEC3 c1 = globalBp + height_vector * mid_height;
    VEC3 d1 = globalCp + height_vector * mid_height;

    VEC3 am1 = a1 + length_vector * mid_length;
    VEC3 bm1 = am1 + length_vector * window_size;
    VEC3 cm1 = bm1 + width_vector * width;
    VEC3 dm1 = am1 + width_vector * width;

    VEC3 a2 = a1 + height_vector * window_size;
    VEC3 b2 = b1 + height_vector * window_size;
    VEC3 c2 = c1 + height_vector * window_size;
    VEC3 d2 = d1 + height_vector * window_size;

    VEC3 am2 = am1 + height_vector * window_size;
    VEC3 bm2 = bm1 + height_vector * window_size;
    VEC3 cm2 = cm1 + height_vector * window_size;
    VEC3 dm2 = dm1 + height_vector * window_size;

    // OVERLAP PRISMS SLIGHTLY TO PREVENT WEIRD CRACKS
    shapes.push_back(make_shared<RectPrismV2>(globalC, globalB, globalBp, globalCp,
              a1 + VEC3(0, 1e3, 0), b1 + VEC3(0, 1e3, 0), c1 + VEC3(0, 1e3, 0), d1+ VEC3(0, 1e3, 0),
              VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<RectPrismV2>(a1-VEC3(0, 1e3, 0), am1-VEC3(0, 1e3, 0), dm1-VEC3(0, 1e3, 0),
      d1-VEC3(0, 1e3, 0), a2, am2, dm2, d2,
      VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<RectPrismV2>(bm1-VEC3(0, 1e3, 0), b1-VEC3(0, 1e3, 0), c1-VEC3(0, 1e3, 0),
        cm1-VEC3(0, 1e3, 0), bm2, b2, c2, cm2,
        VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<RectPrismV2>(a2, b2, c2, d2, globalG, globalF, globalFp, globalGp, VEC3(0,0.81,0.99)));

    // point light right outside window simulates real light?
    VEC3 windowlight_c = (cm1 + dm1 + cm2 + dm2)/4 + VEC3(0,0,1);
    lights.push_back(make_shared<pointLight>(windowlight_c, VEC3(1,1,1)));

    // Ceiling + lights
    // 4 square lights on ceiling
    shapes.push_back(make_shared<Rectangle>(globalE, globalF, globalG, globalH, VEC3(0,0.81,0.99)));
    int nlights = 4;
    VEC3 lightcol(1, 1, 1);
    float lighth = (globalD[0]-globalA[0])/(nlights + (nlights+2)/2); // 10 l + (10+2) * hbound (1/2 l)
    float hbound = lighth/2;
    float lightw = (globalB[2] - globalA[2])/(2 + 4.0/5); // 2 w + (2 + 2) * wbound (1/5 w)
    float wbound = lightw/5;

    // Each light: lighth x lighth with wbound distance from center
    VEC3 ceiling_center = (globalE + globalF + globalG + globalH)/4 - VEC3(0,0.05, 0);
    VEC3 a_tmp = ceiling_center + globalEF * (lighth + wbound) - globalAD * wbound;
    shared_ptr<rectangleLight> left_f = make_shared<rectangleLight>(a_tmp, a_tmp - lighth * globalEF,
          a_tmp - lighth * globalEH - lighth *globalEF, a_tmp - lighth*globalEH, lightcol);
    lights.push_back(left_f);
    shapes.push_back(left_f);
    //shapes.push_back(make_shared<rectangleLight>(*left_f));

    VEC3 a_tmp1 = ceiling_center - globalEF * wbound - globalAD * wbound;
    shared_ptr<rectangleLight> right_f = make_shared<rectangleLight>(a_tmp1, a_tmp1 - lighth * globalEF,
          a_tmp1 - lighth * globalEH - lighth *globalEF, a_tmp1 - lighth*globalEH, lightcol);
    lights.push_back(right_f);
    shapes.push_back(right_f);
    //shapes.push_back(make_shared<rectangleLight>(*right_f));

    VEC3 a_tmp2 = ceiling_center + globalEF * (wbound + lighth) + globalAD * (wbound + lighth);
    shared_ptr<rectangleLight> left_b = make_shared<rectangleLight>(a_tmp2, a_tmp2 - lighth * globalEF,
          a_tmp2 - lighth * globalEH - lighth *globalEF, a_tmp2 - lighth*globalEH, lightcol);
    lights.push_back(left_b);
    shapes.push_back(left_b);
    //shapes.push_back(make_shared<rectangleLight>(*left_b));

    VEC3 a_tmp3 = ceiling_center - globalEF * wbound + globalAD * (wbound + lighth);
    shared_ptr<rectangleLight> right_b = make_shared<rectangleLight>(a_tmp3, a_tmp3 - lighth * globalEF,
          a_tmp3 - lighth * globalEH - lighth *globalEF, a_tmp3 - lighth*globalEH, lightcol);
    lights.push_back(right_b);
    shapes.push_back(right_b);

    //shapes.push_back(make_shared<rectangleLight>(*right_b));

    // Not enough: add dim center light
    // lights.push_back(make_shared<pointLight>(VEC3(0,9,1), VEC3(0.8,0.8,0.8)));

    // Corner cylinder with texture (SAME MATERIAL + TEXTURE AS FLOOR)
    float l_prism = 2;
    float w_prism = 3.0/4;
    float h_prism = 1;

    float r = 4 * w_prism;
    VEC3 c_top = globalH + globalEH * (l_prism + r) + 1 * globalCD * w_prism;
    VEC3 c_bot = globalD + globalAD * (l_prism + r) + 1 * globalCD * w_prism;
    shared_ptr<CheckerCylinder> cyl = make_shared<CheckerCylinder>(c_top, c_bot, r, VEC3(1,1,1), s, "linoleum");
    cyl->tex_frame = tex_index;
    cyl->borderwidth = 0.05;
    cyl->bordercolor = VEC3(0.55, 0.55, 0.55);
    cyl->texture = true;
    cyl->reflect_params.material = "linoleum";
    cyl->reflect_params.roughness = 0.6;
    cyl->reflect_params.refr = VEC2(1.543,0);
    cyl->reflect_params.glossy = true;
    shapes.push_back(make_shared<CheckerCylinder>(*cyl));

    // Staircase: rotate vertices about center of cylinder and make way up counterclockwise
    // Stop conditions: height reaches 80% of cylinder height OR hit left wall
    // Rotate DA vector 10 degrees counterclockwise centered at cylinder bottom center
    width_vector = VEC3(-1, 0, -0.8).normalized();
    length_vector = VEC3(0,1,0).cross(width_vector).normalized();
    height_vector = VEC3(0,1,0);

    // Start the stairs parallel to the left wall
    VEC3 stair_d = globalAD * r + c_bot;
    VEC3 stair_a = stair_d + width_vector * w_prism;
    VEC3 stair_b = stair_a + length_vector * l_prism;
    VEC3 stair_c = stair_d + length_vector * l_prism;
    VEC3 stair_e = stair_a + height_vector * h_prism;
    VEC3 stair_f = stair_b + height_vector * h_prism;
    VEC3 stair_g = stair_c + height_vector * h_prism;
    VEC3 stair_h = stair_d + height_vector * h_prism;

    VEC3 tmp_center = c_bot;
    float theta = acos(1 - pow(w_prism, 2)/pow((stair_a - tmp_center).norm(), 2));
    while (stair_e[1] <= 13 && length_vector.dot(globalCD) > 0)
    {
      shapes.push_back(make_shared<RectPrismV2>(stair_a, stair_b, stair_c, stair_d, stair_e, stair_f, stair_g,
                                              stair_h, pink));
      // Rotation formula (this is guaranteed by the fact that D touches the surface of the cylinder!)
      // theta = acos(1 - w^2/(A-center).normsq)
      stair_a = rotate(stair_a - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;
      stair_b = rotate(stair_b - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;
      stair_c = rotate(stair_c - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;
      stair_d = rotate(stair_d - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;

      stair_e = stair_a + height_vector * h_prism;
      stair_f = stair_b + height_vector * h_prism;
      stair_g = stair_c + height_vector * h_prism;
      stair_h = stair_d + height_vector * h_prism;

      tmp_center += height_vector;
      // Recompute new length and width vectors
      width_vector = (stair_a - stair_d).normalized();
      length_vector = (stair_b - stair_a).normalized();
      // Compute new theta
      theta = acos(1 - pow(w_prism, 2)/pow((stair_a - tmp_center).norm(), 2));
    }
    // Sphere light on the left wall
    // shared_ptr<sphereLight> halogen_ball = make_shared<sphereLight>(VEC3(0, 8, -2), 2, halogen);
    // shapes.push_back(halogen_ball);
    // lights.push_back(halogen_ball);

    if (use_model == true && frame < frame_prism)
    {
      finalBuildModels(frame);
    }
  }
}

void crappybuildFinal(float frame)
{
  // NO TIME VERSION: SAME CAMERA MOVEMENT AS OG

  // Perlin cloud texturing always on
  perlin_cloud = true;

  setSkeletonsToSpecifiedFrame(int(frame));

  // Explicitly free memory
  shapes.clear();
  lights.clear();

  displayer.ComputeBonePositions(DisplaySkeleton::BONES_AND_LOCAL_FRAMES);

  // Get bone names in skeleton
  Skeleton* skel = displayer.GetSkeleton(0);

  // retrieve all the bones of the skeleton
  vector<MATRIX4>& rotations = displayer.rotations();
  vector<MATRIX4>& scalings  = displayer.scalings();
  vector<VEC4>& translations = displayer.translations();
  vector<float>& lengths     = displayer.lengths();

  // Skip the first bone,
  // it's just the origin
  int totalBones = rotations.size();
  for (int x = 1; x < totalBones; x++)
  {
    MATRIX4& rotation = rotations[x];
    MATRIX4& scaling = scalings[x];
    VEC4& translation = translations[x];

    // get the endpoints of the cylinder
    VEC4 leftVertex(0,0,0,1);
    VEC4 rightVertex(0,0,lengths[x],1);

    leftVertex = rotation * scaling * leftVertex + translation;
    rightVertex = rotation * scaling * rightVertex + translation;

    // CLOUD FRAMES: drop the cylinder man (just apply negative y)
    if (frame >= frame_cloud)
    {
      leftVertex -= VEC4(0, (frame-frame_cloud), 0, 0);
      rightVertex -= VEC4(0, (frame-frame_cloud), 0, 0);
    }

    // Cylinders
    shapes.push_back(make_shared<Cylinder>(leftVertex.head<3>(), rightVertex.head<3>(), 0.05, VEC3(1,0,0)));
  }

  // CAMERA SETTINGS -----------------------
  // Choreography
  // Eye: starts with looking out window at ???
  // 480 frames (2 sec): Slow move backwards for 10 pixels (should see helios statues at this point)
  // 480 frames (2 sec): try the existing rotation code below
  // TODO: REDO EYE SETTINGS FOR PRE-FALL SCENE
  VEC3 init_eye(0, 8, 6);
  VEC3 init_lookingAt(0, 7.8, 6);
  VEC3 int_eye(-3, 8, 2); // Set after end of first move
  VEC3 int_lookingAt(-5, 8, 0);
  VEC3 init_up(0,1,0);
  VEC3 final_eye(0.5, 8, 1.1);
  VEC3 final_lookingAt(0.5, 0.5, 1); //Set after frame_prism (4 seconds)
  VEC3 final_up(0,0,-1);

  if (frame < frame_prism) // Do nothing... just observe the clouds
  {
    eye = VEC3(-6, 0.5, 1);
    lookingAt = final_lookingAt;

    // 2/3 second to transition into falling through tunnel
    float tunnel_transition = 20 * 8;
    float eye_transition = 40 * 8;

    // TODO: move eye along parametric curve
    // TODO: Also change looking at point
    // Rotate eye to look down y-axis at center of frame after 40 frames
    VEC3 end = VEC3(0.49, 2, 1) - final_lookingAt;
    VEC3 start = eye - final_lookingAt;
    VEC3 axis = start.cross(end).normalized();
    float theta = acos(start.normalized().dot(end.normalized())) * min(1.0, (double) frame/eye_transition);
    eye = rotate(eye-final_lookingAt, axis, theta) + final_lookingAt;

    // Rotate "up" vector simultaneously
    up = VEC3(0,1,0);
    theta = -M_PI/2 * min(1.0, (double) frame/320);
    up = rotate(up, VEC3(1,0,0), theta);
  }
  if (frame >= frame_prism) // Set eye, up, and looking at to (almost) final positions (not including below acceleration)
  {
    eye = final_eye;
    lookingAt = final_lookingAt;
    up = VEC3(0,0,-1);
  }
  // GLOBALS ======================
  // MOVEMENT PARAMETERS
  float tunnel_transition = 20 * 8;
  float movement_multiplier = max(float(0), frame-frame_prism);
  // Slight acceleration to terminal velocity (0.5/8)
  // ALWAYS RESET: because of motion blur sampling
  move_per_frame = 0.1/8;
  move_per_frame *= (1 + min(float(4), 4 * (movement_multiplier)/tunnel_transition));
  tot_move = movement_multiplier * move_per_frame;

  // Accelerate for 2 seconds (motion blur ON) until clouds
  // We're going to need to adjust the prism height accordingly
  float accel_d = accel_t * pow(frame-frame_blur, 3);
  float dist = 263;
  if (frame > frame_blur && frame <= frame_cloud)
  {
    // Let's be more accurate about distances
    tot_move += accel_d;

    // Update velocity as well (for motion blur)
    move_per_frame += 0.1/(2 * 64) * pow(frame-frame_blur, 2);
  }

  // Trapdoor: NEVER TOUCH THESE
  float min_y = 0.301897 + tot_move;
  float xmin = -0.1;
  float xmax = 1.7;
  float zmin = -0.4;
  float zmax = 1.9;

  VEC3 A(xmin, min_y, zmax);
  VEC3 B((xmin+xmax)/2, min_y, zmax);
  VEC3 C((xmin+xmax)/2, min_y, zmin);
  VEC3 D(xmin, min_y, zmin);
  VEC3 E(xmax, min_y, zmax);
  VEC3 F(xmax, min_y, zmin);

  VEC3 globalA(-10, min_y, -5);
  VEC3 globalB(-10, min_y, 8);
  VEC3 globalC(10, min_y, 8);
  VEC3 globalD(10, min_y, -5);
  VEC3 globalE(-10, 15 + tot_move + min_y, -5);
  VEC3 globalF(-10, 15 + tot_move + min_y, 8);
  VEC3 globalG(10, 15 + tot_move + min_y, 8);
  VEC3 globalH(10, 15 + tot_move + min_y, -5);

  VEC3 globalEH = (globalE-globalH).normalized();
  VEC3 globalEF = (globalE-globalF).normalized();
  VEC3 globalAD = (globalA-globalD).normalized();
  VEC3 globalCD = (globalC - globalD).normalized();
  VEC3 globalcenter = (globalA + globalB + globalC + globalD + globalE + globalF + globalG + globalH)/8;

  // TUNNEL TRANSITION ===========================
  // Min calls ensure that these won't kick in prematurely
  // Movement: bound below by frame_prism (0) and above by tunnel_transition (160)
  // 2/3 second to transition into falling through tunnel
  // Accelerate eye towards body (lookingAt point also moves)
  VEC3 tunnel_point = VEC3((xmin+xmax)/2, 5, (zmin+zmax)/2);
  VEC3 eye_path = (tunnel_point - eye).normalized();
  float accel = (tunnel_point-eye).norm()/pow(tunnel_transition, 2);
  eye = accel * pow(min(tunnel_transition, movement_multiplier), 2) * eye_path + eye;
  lookingAt = accel * pow(min(tunnel_transition, movement_multiplier), 2) * eye_path + lookingAt;

  // Create rectangle mesh that forms triangle prism
  // Starting position: long axis is y-axis and top is right on the surface of the floor
  // Length: 133 (computed by taking velocy + acceleration into account)

  // Expand mesh by a lot: MUST MAKE EQUILATERAL TRIANGLE
  VEC3 b_cap0(xmin, min_y, zmax);
  VEC3 c_cap0(xmax, min_y, zmax);
  VEC3 a_cap0((xmin+xmax)/2, min_y, (xmin - xmax) * sqrt(3)/2 + zmax); // Set A such that triangle is equilateral

  VEC3 cap_center = (a_cap0 + b_cap0 + c_cap0)/3;
  a_cap0 = (a_cap0 - cap_center) * 5 + cap_center;
  b_cap0 = (b_cap0 - cap_center) * 5 + cap_center;
  c_cap0 = (c_cap0 - cap_center) * 5 + cap_center;

  // Light source during tunnel (from eye)
  if (frame >= frame_prism + tunnel_transition)
  {
    lights.push_back(make_shared<pointLight>(eye, VEC3(1,1,1)));
  }
  if (frame >= frame_cloud)
  {
    // Only have cylinder man left to render
    // EXPLODE after 80 frames

    // redsky = redsky + (sunorange - redsky) * (frame-frame_cloud)/(total-frame_cloud);
    // bluesky = bluesky + (pastelpink - bluesky) * (frame-frame_cloud)/(total-frame_cloud);
    // sun_outer = sun_outer + (violet - sun_outer) * (frame-frame_cloud)/(total-frame_cloud);
    // sun_inner = sun_inner + (indigo - sun_inner) * (frame-frame_cloud)/(total-frame_cloud);
    // sun_core = sun_core + (darkblue - sun_core) * (frame-frame_cloud)/(total-frame_cloud);

    if (frame - frame_cloud > 80)
    {
      int frame_diff = (int)(frame-frame_cloud) % 4;
      if (frame_diff == 0)
      {
        // Vanilla
        redsky = sunorange;
        bluesky = pastelpink;
        sun_outer = violet;
        sun_inner = indigo;
        sun_core = darkblue;
      }
      if (frame_diff == 1)
      {
        redsky = teal;
        bluesky = sunorange;
        sun_core = darkblue;
        sun_inner = indigo;
        sun_outer = violet;
      }
      if (frame_diff == 2)
      {
        redsky = violet;
        bluesky = darkblue;
        sun_core = halogen;
        sun_inner = sunorange;
        sun_outer = pastelpink;
      }
      if (frame_diff == 3)
      {
        redsky = pastelpink;
        bluesky = maroonq;
        sun_core = teal;
        sun_inner = skyblue;
        sun_outer = darkblue;
      }
    }
    return;
  }

  // Pull up mesh by given velocity
  a_cap0 += VEC3(0, tot_move, 0);
  b_cap0 += VEC3(0, tot_move, 0);
  c_cap0 += VEC3(0, tot_move, 0);

  // Rotation: slow (180 every 3 seconds = 90 * 8 frames)
  float rot_theta = (movement_multiplier)/720.0 * M_PI;
  a_cap0 = rotate(a_cap0 - cap_center, VEC3(0,1,0), rot_theta) + cap_center;
  b_cap0 = rotate(b_cap0 - cap_center, VEC3(0,1,0), rot_theta) + cap_center;
  c_cap0 = rotate(c_cap0 - cap_center, VEC3(0,1,0), rot_theta) + cap_center;

  // Length of tunnel: 4 seconds == 120 frames
  VEC3 a_cap1 = a_cap0 - VEC3(0, dist, 0);
  VEC3 b_cap1 = b_cap0 - VEC3(0, dist, 0);
  VEC3 c_cap1 = c_cap0 - VEC3(0, dist, 0);

  // Print out scene settings
  // printf("Frame: %f ==========\n", frame);
  // printf("Movement multiplier: %f\n", movement_multiplier);
  // cout << "Eye: " << eye << endl;
  // cout << "lookingAt: " << lookingAt << endl;
  // cout << "Min y: " << min_y << endl;

  if (frame >= frame_prism)
  {
    generateTrianglePrismMesh(a_cap0, b_cap0, c_cap0, a_cap1, b_cap1, c_cap1, int(frame), true, true);
  }

  // Only render objects while they are still relevant
  if ((min_y + tot_move  <= eye[1] + 2) || frame < frame_prism + tunnel_transition)
  {
    // Rotate inwards about hinges AD, EF
    // TODO: MAKE THIS RECTANGULAR PRISM
    float angle = min(float(1.1), movement_multiplier/(tunnel_transition)) * M_PI/2;
    VEC3 B_left = rotate(B-A, D-A, angle) + A;
    VEC3 C_left = rotate(C-A, D-A, angle) + A;
    VEC3 B_right = rotate(B-E, F-E, -angle) + E;
    VEC3 C_right = rotate(C-E, F-E, -angle) + E;

    loadTexture("./textures/sad_finder1_adj.jpg");
    int tex_index = texture_frames.size()-1;
    shared_ptr<Rectangle> tmp = make_shared<Rectangle>(A, B_left, C_left, D, VEC3(0,0,0), "steel", false, tex_index, "cook-torrance");
    tmp->reflect_params.roughness = 0.7;
    tmp->reflect_params.refr = VEC2(2.75, 3.79);
    tmp->reflect_params.glossy = true;
    tmp->texture = true;
    shapes.push_back(make_shared<Rectangle>(*tmp));

    loadTexture("./textures/sad_finder2_adj.jpg");
    tex_index = texture_frames.size()-1;
    shared_ptr<Rectangle> tmp2 = make_shared<Rectangle>(B_right,E,F,C_right, VEC3(0,0,0), "steel", false, tex_index, "cook-torrance");
    tmp2->reflect_params.roughness = 0.7;
    tmp2->reflect_params.refr = VEC2(2.75, 3.79);
    tmp2->reflect_params.glossy = true;
    tmp2->texture = true;
    shapes.push_back(make_shared<Rectangle>(*tmp2));

    // OTHER INITIAL SCENE OBJECTS: indoors box
    // Floor: linoleum (texture applies to each SQUARE) + bounding lines
    loadTexture("./textures/floor.jpeg");
    tex_index = texture_frames.size()-1;
    float s = 1;
    VEC3 checker_rgb0(0.58, 0.82, 1);
    VEC3 checker_rgb1(1, 0.416, 0.835);
    shared_ptr<CheckerboardWithHole> floor = make_shared<CheckerboardWithHole>(globalA,
                                                globalB,
                                                globalC,
                                                globalD,
                                              checker_rgb0, checker_rgb1, s,
                                            make_shared<Rectangle>(A, E, F, D, VEC3(0,0,0)));
    floor->tex_frame = tex_index;
    floor->borderwidth = 0.05;
    floor->bordercolor = VEC3(0.55, 0.55, 0.55);
    floor->texture = true;
    floor->reflect_params.material = "linoleum";
    floor->reflect_params.roughness = 0.6;
    floor->reflect_params.refr = VEC2(1.543,0);
    floor->reflect_params.glossy = true;
    shapes.push_back(make_shared<CheckerboardWithHole>(*floor));

    // Walls
    shapes.push_back(make_shared<Rectangle>(globalA, globalD, globalH, globalE, VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<Rectangle>(globalA, globalB, globalF, globalE, VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<Rectangle>(globalD, globalC, globalG, globalH, VEC3(0,0.81,0.99)));

    // Right wall: aggregated prisms with window
    VEC3 height_vector = (globalF-globalB).normalized();
    float height = (globalF-globalB).norm();
    VEC3 length_vector = (globalB-globalC).normalized();
    float length = (globalC-globalB).norm();
    VEC3 width_vector = (globalB-globalA).normalized();
    float width = 2;

    VEC3 globalBp = globalB + width_vector * width;
    VEC3 globalCp = globalC + width_vector * width;
    VEC3 globalFp = globalF + width_vector * width;
    VEC3 globalGp = globalG + width_vector * width;

    // Window centered eye position: 8, 8 (x,y) -> spans (7-9)
    float window_size = 2;
    float mid_height = (height-window_size)/2;
    float mid_length = (length-window_size)/2;

    VEC3 a1 = globalC + height_vector * mid_height;
    VEC3 b1 = globalB + height_vector * mid_height;
    VEC3 c1 = globalBp + height_vector * mid_height;
    VEC3 d1 = globalCp + height_vector * mid_height;

    VEC3 am1 = a1 + length_vector * mid_length;
    VEC3 bm1 = am1 + length_vector * window_size;
    VEC3 cm1 = bm1 + width_vector * width;
    VEC3 dm1 = am1 + width_vector * width;

    VEC3 a2 = a1 + height_vector * window_size;
    VEC3 b2 = b1 + height_vector * window_size;
    VEC3 c2 = c1 + height_vector * window_size;
    VEC3 d2 = d1 + height_vector * window_size;

    VEC3 am2 = am1 + height_vector * window_size;
    VEC3 bm2 = bm1 + height_vector * window_size;
    VEC3 cm2 = cm1 + height_vector * window_size;
    VEC3 dm2 = dm1 + height_vector * window_size;

    // OVERLAP PRISMS SLIGHTLY TO PREVENT WEIRD CRACKS
    shapes.push_back(make_shared<RectPrismV2>(globalC, globalB, globalBp, globalCp,
              a1 + VEC3(0, 1e3, 0), b1 + VEC3(0, 1e3, 0), c1 + VEC3(0, 1e3, 0), d1+ VEC3(0, 1e3, 0),
              VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<RectPrismV2>(a1-VEC3(0, 1e3, 0), am1-VEC3(0, 1e3, 0), dm1-VEC3(0, 1e3, 0),
      d1-VEC3(0, 1e3, 0), a2, am2, dm2, d2,
      VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<RectPrismV2>(bm1-VEC3(0, 1e3, 0), b1-VEC3(0, 1e3, 0), c1-VEC3(0, 1e3, 0),
        cm1-VEC3(0, 1e3, 0), bm2, b2, c2, cm2,
        VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<RectPrismV2>(a2, b2, c2, d2, globalG, globalF, globalFp, globalGp, VEC3(0,0.81,0.99)));

    // point light right outside window simulates real light?
    VEC3 windowlight_c = (cm1 + dm1 + cm2 + dm2)/4 + VEC3(0,0,1);
    lights.push_back(make_shared<pointLight>(windowlight_c, VEC3(1,1,1)));

    // Ceiling + lights
    // 4 square lights on ceiling
    shapes.push_back(make_shared<Rectangle>(globalE, globalF, globalG, globalH, VEC3(0,0.81,0.99)));
    int nlights = 4;
    VEC3 lightcol(0.9, 0.9, 0.9);
    float lighth = (globalD[0]-globalA[0])/(nlights + (nlights+2)/2); // 10 l + (10+2) * hbound (1/2 l)
    float hbound = lighth/2;
    float lightw = (globalB[2] - globalA[2])/(2 + 4.0/5); // 2 w + (2 + 2) * wbound (1/5 w)
    float wbound = lightw/5;

    // Each light: lighth x lighth with wbound distance from center
    VEC3 ceiling_center = (globalE + globalF + globalG + globalH)/4;
    VEC3 a_tmp = ceiling_center + globalEF * (lighth + wbound) - globalAD * wbound;
    shared_ptr<rectangleLight> left_f = make_shared<rectangleLight>(a_tmp, a_tmp - lighth * globalEF,
          a_tmp - lighth * globalEH - lighth *globalEF, a_tmp - lighth*globalEH, lightcol);
    lights.push_back(left_f);
    shapes.push_back(left_f);

    a_tmp = ceiling_center - globalEF * wbound - globalAD * wbound;
    shared_ptr<rectangleLight> right_f = make_shared<rectangleLight>(a_tmp, a_tmp - lighth * globalEF,
          a_tmp - lighth * globalEH - lighth *globalEF, a_tmp - lighth*globalEH, lightcol);
    lights.push_back(right_f);
    shapes.push_back(right_f);

    a_tmp = ceiling_center + globalEF * (wbound + lighth) + globalAD * (wbound + lighth);
    shared_ptr<rectangleLight> left_b = make_shared<rectangleLight>(a_tmp, a_tmp - lighth * globalEF,
          a_tmp - lighth * globalEH - lighth *globalEF, a_tmp - lighth*globalEH, lightcol);
    lights.push_back(left_b);
    shapes.push_back(left_b);

    a_tmp = ceiling_center - globalEF * wbound + globalAD * (wbound + lighth);
    shared_ptr<rectangleLight> right_b = make_shared<rectangleLight>(a_tmp, a_tmp - lighth * globalEF,
          a_tmp - lighth * globalEH - lighth *globalEF, a_tmp - lighth*globalEH, lightcol);
    lights.push_back(right_b);
    shapes.push_back(right_b);

    // Not enough: add center light
    lights.push_back(make_shared<pointLight>(VEC3(0,9,1), VEC3(1,1,1)));

    // Corner cylinder with texture (SAME MATERIAL + TEXTURE AS FLOOR)
    float l_prism = 2;
    float w_prism = 3.0/4;
    float h_prism = 1;

    float r = 4 * w_prism;
    VEC3 c_top = globalH + globalEH * (l_prism + r) + 1 * globalCD * w_prism;
    VEC3 c_bot = globalD + globalAD * (l_prism + r) + 1 * globalCD * w_prism;
    shared_ptr<CheckerCylinder> cyl = make_shared<CheckerCylinder>(c_top, c_bot, r, VEC3(1,1,1), s, "linoleum");
    cyl->tex_frame = tex_index;
    cyl->borderwidth = 0.05;
    cyl->bordercolor = VEC3(0.55, 0.55, 0.55);
    cyl->texture = true;
    cyl->reflect_params.material = "linoleum";
    cyl->reflect_params.roughness = 0.6;
    cyl->reflect_params.refr = VEC2(1.543,0);
    cyl->reflect_params.glossy = true;
    shapes.push_back(make_shared<CheckerCylinder>(*cyl));

    // Staircase: rotate vertices about center of cylinder and make way up counterclockwise
    // Stop conditions: height reaches 80% of cylinder height OR hit left wall
    // Rotate DA vector 10 degrees counterclockwise centered at cylinder bottom center
    width_vector = VEC3(-1, 0, -0.8).normalized();
    length_vector = VEC3(0,1,0).cross(width_vector).normalized();
    height_vector = VEC3(0,1,0);

    // Start the stairs parallel to the left wall
    VEC3 stair_d = globalAD * r + c_bot;
    VEC3 stair_a = stair_d + width_vector * w_prism;
    VEC3 stair_b = stair_a + length_vector * l_prism;
    VEC3 stair_c = stair_d + length_vector * l_prism;
    VEC3 stair_e = stair_a + height_vector * h_prism;
    VEC3 stair_f = stair_b + height_vector * h_prism;
    VEC3 stair_g = stair_c + height_vector * h_prism;
    VEC3 stair_h = stair_d + height_vector * h_prism;

    VEC3 tmp_center = c_bot;
    float theta = acos(1 - pow(w_prism, 2)/pow((stair_a - tmp_center).norm(), 2));
    while (stair_e[1] <= 13 && length_vector.dot(globalCD) > 0)
    {
      shapes.push_back(make_shared<RectPrismV2>(stair_a, stair_b, stair_c, stair_d, stair_e, stair_f, stair_g,
                                              stair_h, pink));
      // Rotation formula (this is guaranteed by the fact that D touches the surface of the cylinder!)
      // theta = acos(1 - w^2/(A-center).normsq)
      stair_a = rotate(stair_a - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;
      stair_b = rotate(stair_b - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;
      stair_c = rotate(stair_c - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;
      stair_d = rotate(stair_d - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;

      stair_e = stair_a + height_vector * h_prism;
      stair_f = stair_b + height_vector * h_prism;
      stair_g = stair_c + height_vector * h_prism;
      stair_h = stair_d + height_vector * h_prism;

      tmp_center += height_vector;
      // Recompute new length and width vectors
      width_vector = (stair_a - stair_d).normalized();
      length_vector = (stair_b - stair_a).normalized();
      // Compute new theta
      theta = acos(1 - pow(w_prism, 2)/pow((stair_a - tmp_center).norm(), 2));
    }
    // Sphere light on the left wall
    // shared_ptr<sphereLight> halogen_ball = make_shared<sphereLight>(VEC3(0, 8, -2), 2, halogen);
    // shapes.push_back(halogen_ball);
    // lights.push_back(halogen_ball);

    // Only render the helios models in the first 600 frames
    // TODO: ADJUST THIS BASED ON HOW LONG IT TAKES FOR THE CAMERA TO TURN
    // if (frame <= frame_sculp && frame >= frame_start)
    // {
    //   finalBuildModels(frame);
    // }
  }
}

void buildCameraPath(float frame)
{
  shapes.clear();
  lights.clear();
  // Just do basic four walls and see how camera works
  // FINAL TIMINGS: GENERATE 70*8 EXTRA STATIC FRAMES IN BEGINNING
  // Beginning 960 (0-119) frames: first room
  //   - Eye starts at wall: facing
  // Middle 960 (120 - 239) frames: falling through triangle prism
  //    - Frames 180-239: exponential motion blur
  // Last 240 (240-299) frames: perlin cloud texture + stickman falling in distance
  // GLOBALS ======================
  // Trapdoor: NEVER TOUCH THESE
  float min_y = 0.301897 + tot_move;
  float xmin = -0.1;
  float xmax = 1.7;
  float zmin = -0.4;
  float zmax = 1.9;

  VEC3 A(xmin, min_y, zmax);
  VEC3 B((xmin+xmax)/2, min_y, zmax);
  VEC3 C((xmin+xmax)/2, min_y, zmin);
  VEC3 D(xmin, min_y, zmin);
  VEC3 E(xmax, min_y, zmax);
  VEC3 F(xmax, min_y, zmin);

  VEC3 globalA(-10, min_y, -5);
  VEC3 globalB(-10, min_y, 8);
  VEC3 globalC(10, min_y, 8);
  VEC3 globalD(10, min_y, -5);
  VEC3 globalE(-10, 15 + min_y, -5);
  VEC3 globalF(-10, 15 + min_y, 8);
  VEC3 globalG(10, 15 + min_y, 8);
  VEC3 globalH(10, 15 + min_y, -5);

  VEC3 globalEH = (globalE-globalH).normalized();
  VEC3 globalEF = (globalE-globalF).normalized();
  VEC3 globalAD = (globalA-globalD).normalized();
  VEC3 globalCD = (globalC - globalD).normalized();
  VEC3 globalcenter = (globalA + globalB + globalC + globalD + globalE + globalF + globalG + globalH)/8;

  // CAMERA SETTINGS -----------------------
  // Choreography
  // Eye: starts with looking out window at ???
  // 480 frames (2 sec): Slow move backwards for 10 pixels (should see helios statues at this point)
  // 480 frames (2 sec): try the existing rotation code below
  // TODO: REDO EYE SETTINGS FOR PRE-FALL SCENE
  VEC3 init_eye(0, 8, 6);
  VEC3 init_lookingAt(0, 7.8, 20);
  VEC3 int_eye(-3, 8, 2); // Set after end of first move
  VEC3 int_lookingAt(-5, 8, 0);
  VEC3 init_up(0,1,0);
  VEC3 final_eye(0.5, 8, 1.1);
  VEC3 final_lookingAt(0.5, 0.5, 1); //Set after frame_prism (4 seconds)
  VEC3 final_up(0,0,-1);

  if (frame <= frame_start) // Do nothing... just observe the clouds
  {
    eye = init_eye;
    lookingAt = init_lookingAt;
  }
  else if (frame <= frame_move1)
  {
    eye = init_eye + (frame - frame_move1)/(frame_move1 - frame_start)* (int_eye - init_eye); // Linear move

    // Rotate looingAt
    lookingAt = rotate(init_lookingAt - int_lookingAt, VEC3(0,1,0),
                (frame - frame_move1)/(frame_move1 - frame_start) * M_PI) + int_lookingAt; // Rotate looking at vector
  }
  // Rotate eye to look down y-axis at center of frame after frame_prism frames
  else if (frame > frame_move1 && frame <= frame_prism)
  {
    lookingAt = int_lookingAt;
    eye = int_eye;
    VEC3 end = final_eye - final_lookingAt;
    VEC3 start = eye - final_lookingAt;
    VEC3 axis = start.cross(end).normalized();
    float theta = acos(start.normalized().dot(end.normalized())) * min(1.0, (double) (frame-frame_move1)/(frame_move2 - frame_move1));
    eye = rotate(eye-init_lookingAt, axis, theta) + init_lookingAt;

    // Rotate "up" vector simultaneously (rotate around x axis to get z-axis)
    up = VEC3(0,1,0);
    theta = -M_PI/2 * min(1.0, (double) (frame-frame_move1)/(frame_move2 - frame_move1));
    up = rotate(up, VEC3(1,0,0), theta);

    // Linear interpolate lookingat vector
    lookingAt = init_lookingAt + (final_lookingAt - init_lookingAt) * min(1.0, (double) (frame-frame_move1)/(frame_move2 - frame_move1));
  }
  if (frame >= frame_prism) // Set eye, up, and looking at to (almost) final positions (not including below acceleration)
  {
    eye = final_eye;
    lookingAt = final_lookingAt;
    up = VEC3(0,0,-1);
  }

  // TUNNEL TRANSITION ===========================
  // Min calls ensure that these won't kick in prematurely
  // Movement: bound below by frame_prism (0) and above by tunnel_transition (160)
  // 2/3 second to transition into falling through tunnel
  // Accelerate eye towards body (lookingAt point also moves)
  float movement_multiplier = 0.05;
  float tunnel_transition = 160;
  VEC3 tunnel_point = VEC3((xmin+xmax)/2, 5, (zmin+zmax)/2);
  VEC3 eye_path = (tunnel_point - eye).normalized();
  float accel = (tunnel_point-eye).norm()/pow(tunnel_transition, 2);
  eye = accel * pow(min(tunnel_transition, movement_multiplier), 2) * eye_path + eye;
  lookingAt = accel * pow(min(tunnel_transition, movement_multiplier), 2) * eye_path + lookingAt;

  float s = 1;
  VEC3 checker_rgb0(0.58, 0.82, 1);
  VEC3 checker_rgb1(1, 0.416, 0.835);
  shapes.push_back(make_shared<Checkerboard>(globalA,
                                              globalB,
                                              globalC,
                                              globalD,
                                            checker_rgb0, checker_rgb1, s));

  // Walls
  shapes.push_back(make_shared<Rectangle>(globalA, globalD, globalH, globalE, VEC3(0,0.81,0.99)));
  shapes.push_back(make_shared<Rectangle>(globalA, globalB, globalF, globalE, VEC3(0,0.81,0.99)));
  shapes.push_back(make_shared<Rectangle>(globalD, globalC, globalG, globalH, VEC3(0,0.81,0.99)));

  // Right wall: aggregated prisms with window
  VEC3 height_vector = (globalF-globalB).normalized();
  float height = (globalF-globalB).norm();
  VEC3 length_vector = (globalB-globalC).normalized();
  float length = (globalC-globalB).norm();
  VEC3 width_vector = (globalB-globalA).normalized();
  float width = 2;

  VEC3 globalBp = globalB + width_vector * width;
  VEC3 globalCp = globalC + width_vector * width;
  VEC3 globalFp = globalF + width_vector * width;
  VEC3 globalGp = globalG + width_vector * width;

  // Window centered eye position: 8, 8 (x,y) -> spans (7-9)
  float window_size = 2;
  float mid_height = (height-window_size)/2;
  float mid_length = (length-window_size)/2;

  VEC3 a1 = globalC + height_vector * mid_height;
  VEC3 b1 = globalB + height_vector * mid_height;
  VEC3 c1 = globalBp + height_vector * mid_height;
  VEC3 d1 = globalCp + height_vector * mid_height;

  VEC3 am1 = a1 + length_vector * mid_length;
  VEC3 bm1 = am1 + length_vector * window_size;
  VEC3 cm1 = bm1 + width_vector * width;
  VEC3 dm1 = am1 + width_vector * width;

  VEC3 a2 = a1 + height_vector * window_size;
  VEC3 b2 = b1 + height_vector * window_size;
  VEC3 c2 = c1 + height_vector * window_size;
  VEC3 d2 = d1 + height_vector * window_size;

  VEC3 am2 = am1 + height_vector * window_size;
  VEC3 bm2 = bm1 + height_vector * window_size;
  VEC3 cm2 = cm1 + height_vector * window_size;
  VEC3 dm2 = dm1 + height_vector * window_size;

  // OVERLAP PRISMS SLIGHTLY TO PREVENT WEIRD CRACKS
  shapes.push_back(make_shared<RectPrismV2>(globalC, globalB, globalBp, globalCp,
            a1 + VEC3(0, 1e3, 0), b1 + VEC3(0, 1e3, 0), c1 + VEC3(0, 1e3, 0), d1+ VEC3(0, 1e3, 0),
            VEC3(0,0.81,0.99)));
  shapes.push_back(make_shared<RectPrismV2>(a1-VEC3(0, 1e3, 0), am1-VEC3(0, 1e3, 0), dm1-VEC3(0, 1e3, 0),
    d1-VEC3(0, 1e3, 0), a2, am2, dm2, d2,
    VEC3(0,0.81,0.99)));
  shapes.push_back(make_shared<RectPrismV2>(bm1-VEC3(0, 1e3, 0), b1-VEC3(0, 1e3, 0), c1-VEC3(0, 1e3, 0),
      cm1-VEC3(0, 1e3, 0), bm2, b2, c2, cm2,
      VEC3(0,0.81,0.99)));
  shapes.push_back(make_shared<RectPrismV2>(a2, b2, c2, d2, globalG, globalF, globalFp, globalGp, VEC3(0,0.81,0.99)));

  cout << "Window A1: " << am1 << endl;
  cout << "Window B1: " << bm1 << endl;
  cout << "Window A2: " << am2 << endl;
  cout << "Window B2: " << bm2 << endl;

  // point light right outside window simulates real light?
  VEC3 windowlight_c = (cm1 + dm1 + cm2 + dm2)/4 + VEC3(0,0,1);
  lights.push_back(make_shared<pointLight>(windowlight_c, VEC3(1,1,1)));
  // Ceiling
  shapes.push_back(make_shared<Rectangle>(globalE, globalF, globalG, globalH, VEC3(0,0.81,0.99)));
  lights.push_back(make_shared<pointLight>(VEC3(0, 9, 0), VEC3(1,1,1)));
}

void buildCameraPathV2(float frame)
{
  shapes.clear();
  lights.clear();

  // Just do basic four walls and see how camera works
  // FINAL TIMINGS: GENERATE 70*8 EXTRA STATIC FRAMES IN BEGINNING
  // Beginning 960 (0-119) frames: first room
  //   - Eye starts at wall: facing
  // Middle 960 (120 - 239) frames: falling through triangle prism
  //    - Frames 180-239: exponential motion blur
  // Last 240 (240-299) frames: perlin cloud texture + stickman falling in distance
  // GLOBALS ======================
  // Trapdoor: NEVER TOUCH THESE
  float min_y = 0.301897 + tot_move;
  float xmin = -0.1;
  float xmax = 1.7;
  float zmin = -0.4;
  float zmax = 1.9;

  VEC3 A(xmin, min_y, zmax);
  VEC3 B((xmin+xmax)/2, min_y, zmax);
  VEC3 C((xmin+xmax)/2, min_y, zmin);
  VEC3 D(xmin, min_y, zmin);
  VEC3 E(xmax, min_y, zmax);
  VEC3 F(xmax, min_y, zmin);

  VEC3 globalA(-10, min_y, -5);
  VEC3 globalB(-10, min_y, 8);
  VEC3 globalC(10, min_y, 8);
  VEC3 globalD(10, min_y, -5);
  VEC3 globalE(-10, 15 + min_y, -5);
  VEC3 globalF(-10, 15 + min_y, 8);
  VEC3 globalG(10, 15 + min_y, 8);
  VEC3 globalH(10, 15 + min_y, -5);

  VEC3 globalEH = (globalE-globalH).normalized();
  VEC3 globalEF = (globalE-globalF).normalized();
  VEC3 globalAD = (globalA-globalD).normalized();
  VEC3 globalCD = (globalC - globalD).normalized();
  VEC3 globalcenter = (globalA + globalB + globalC + globalD + globalE + globalF + globalG + globalH)/8;

  // CAMERA SETTINGS -----------------------
  // Choreography
  // Eye: starts with looking out window at ???
  // 480 frames (2 sec): Slow move backwards for 10 pixels (should see helios statues at this point)
  // 480 frames (2 sec): try the existing rotation code below
  // TODO: REDO EYE SETTINGS FOR PRE-FALL SCENE
  VEC3 init_eye(6, 2, -4);
  VEC3 init_lookingAt(0, 2, 20);
  VEC3 int_eye(-3, 2, 2); // Set after end of first move
  VEC3 int_lookingAt(-5, 8, 0);
  VEC3 init_up(0,1,0);
  VEC3 final_eye(0.5, 8, 1.1);
  VEC3 final_lookingAt(0.5, 0.5, 1); //Set after frame_prism (4 seconds)
  VEC3 final_up(0,0,-1);

  if (frame <= frame_start) // Do nothing... just observe the clouds
  {
    eye = init_eye;
    lookingAt = init_lookingAt;
  }
  else if (frame <= frame_move1)
  {
    // Rotate eye around looking at
    lookingAt = init_lookingAt;
    eye = init_eye;
    VEC3 end = int_eye - init_lookingAt;
    VEC3 start = eye - lookingAt;
    VEC3 axis = start.cross(end).normalized();
    float theta = acos(start.normalized().dot(end.normalized())) * min(1.0, (double) (frame-frame_start)/(frame_move1-frame_start));
    eye = rotate(eye - lookingAt, axis, theta);
  }
  // Rotate eye to look down y-axis at center of frame after frame_prism frames
  else if (frame > frame_move1 && frame <= frame_prism)
  {
    lookingAt = init_lookingAt;
    eye = int_eye;
    VEC3 end = final_eye - final_lookingAt;
    VEC3 start = eye - final_lookingAt;
    VEC3 axis = start.cross(end).normalized();
    float theta = acos(start.normalized().dot(end.normalized())) * min(1.0, (double) (frame-frame_move1)/(frame_move2 - frame_move1));
    eye = rotate(eye-init_lookingAt, axis, theta) + init_lookingAt;

    // Rotate "up" vector simultaneously (rotate around x axis to get z-axis)
    up = VEC3(0,1,0);
    theta = -M_PI/2 * min(1.0, (double) (frame-frame_move1)/(frame_move2 - frame_move1));
    up = rotate(up, VEC3(1,0,0), theta);

  }
  if (frame >= frame_prism) // Set eye, up, and looking at to (almost) final positions (not including below acceleration)
  {
    eye = final_eye;
    lookingAt = final_lookingAt;
    up = VEC3(0,0,-1);
  }

  // TUNNEL TRANSITION ===========================
  // Min calls ensure that these won't kick in prematurely
  // Movement: bound below by frame_prism (0) and above by tunnel_transition (160)
  // 2/3 second to transition into falling through tunnel
  // Accelerate eye towards body (lookingAt point also moves)
  float movement_multiplier = 0.05;
  float tunnel_transition = 160;
  VEC3 tunnel_point = VEC3((xmin+xmax)/2, 5, (zmin+zmax)/2);
  VEC3 eye_path = (tunnel_point - eye).normalized();
  float accel = (tunnel_point-eye).norm()/pow(tunnel_transition, 2);
  eye = accel * pow(min(tunnel_transition, movement_multiplier), 2) * eye_path + eye;
  lookingAt = accel * pow(min(tunnel_transition, movement_multiplier), 2) * eye_path + lookingAt;

  float s = 1;
  VEC3 checker_rgb0(0.58, 0.82, 1);
  VEC3 checker_rgb1(1, 0.416, 0.835);
  shapes.push_back(make_shared<Checkerboard>(globalA,
                                              globalB,
                                              globalC,
                                              globalD,
                                            checker_rgb0, checker_rgb1, s));

  // Walls
  shapes.push_back(make_shared<Rectangle>(globalA, globalD, globalH, globalE, VEC3(1,0,0)));
  shapes.push_back(make_shared<Rectangle>(globalA, globalB, globalF, globalE, VEC3(0,1,0)));
  shapes.push_back(make_shared<Rectangle>(globalD, globalC, globalG, globalH, VEC3(1,0,1)));

  // Right wall: aggregated prisms with window
  VEC3 height_vector = (globalF-globalB).normalized();
  float height = (globalF-globalB).norm();
  VEC3 length_vector = (globalB-globalC).normalized();
  float length = (globalC-globalB).norm();
  VEC3 width_vector = (globalB-globalA).normalized();
  float width = 2;

  VEC3 globalBp = globalB + width_vector * width;
  VEC3 globalCp = globalC + width_vector * width;
  VEC3 globalFp = globalF + width_vector * width;
  VEC3 globalGp = globalG + width_vector * width;

  // Window centered eye position: 8, 8 (x,y) -> spans (7-9)
  float window_size = 2;
  float mid_height = (height-window_size)/2;
  float mid_length = (length-window_size)/2;

  VEC3 a1 = globalC + height_vector * mid_height;
  VEC3 b1 = globalB + height_vector * mid_height;
  VEC3 c1 = globalBp + height_vector * mid_height;
  VEC3 d1 = globalCp + height_vector * mid_height;

  VEC3 am1 = a1 + length_vector * mid_length;
  VEC3 bm1 = am1 + length_vector * window_size;
  VEC3 cm1 = bm1 + width_vector * width;
  VEC3 dm1 = am1 + width_vector * width;

  VEC3 a2 = a1 + height_vector * window_size;
  VEC3 b2 = b1 + height_vector * window_size;
  VEC3 c2 = c1 + height_vector * window_size;
  VEC3 d2 = d1 + height_vector * window_size;

  VEC3 am2 = am1 + height_vector * window_size;
  VEC3 bm2 = bm1 + height_vector * window_size;
  VEC3 cm2 = cm1 + height_vector * window_size;
  VEC3 dm2 = dm1 + height_vector * window_size;

  // OVERLAP PRISMS SLIGHTLY TO PREVENT WEIRD CRACKS
  shapes.push_back(make_shared<RectPrismV2>(globalC, globalB, globalBp, globalCp,
            a1 + VEC3(0, 1e3, 0), b1 + VEC3(0, 1e3, 0), c1 + VEC3(0, 1e3, 0), d1+ VEC3(0, 1e3, 0),
            VEC3(0,0.81,0.99)));
  shapes.push_back(make_shared<RectPrismV2>(a1-VEC3(0, 1e3, 0), am1-VEC3(0, 1e3, 0), dm1-VEC3(0, 1e3, 0),
    d1-VEC3(0, 1e3, 0), a2, am2, dm2, d2,
    VEC3(0,0.81,0.99)));
  shapes.push_back(make_shared<RectPrismV2>(bm1-VEC3(0, 1e3, 0), b1-VEC3(0, 1e3, 0), c1-VEC3(0, 1e3, 0),
      cm1-VEC3(0, 1e3, 0), bm2, b2, c2, cm2,
      VEC3(0,0.81,0.99)));
  shapes.push_back(make_shared<RectPrismV2>(a2, b2, c2, d2, globalG, globalF, globalFp, globalGp, VEC3(0,0.81,0.99)));

  // point light right outside window simulates real light?
  // VEC3 windowlight_c = (cm1 + dm1 + cm2 + dm2)/4 + VEC3(0,0,1);
  // lights.push_back(make_shared<pointLight>(windowlight_c, VEC3(1,1,1)));
  // Ceiling
  shapes.push_back(make_shared<Rectangle>(globalE, globalF, globalG, globalH, VEC3(0,0.81,0.99)));
  lights.push_back(make_shared<pointLight>(VEC3(0, 9, 0), VEC3(1,1,1)));
}

//////////////////////////////////////////////////////////////////////////////////
// Build objects in scene given frame
//////////////////////////////////////////////////////////////////////////////////
void buildScene(float frame)
{
  // TODO: Motion blur skeleton????
  setSkeletonsToSpecifiedFrame(int(frame));

  shapes.clear();
  lights.clear();

  displayer.ComputeBonePositions(DisplaySkeleton::BONES_AND_LOCAL_FRAMES);

  // Get bone names in skeleton
  Skeleton* skel = displayer.GetSkeleton(0);

  // retrieve all the bones of the skeleton
  vector<MATRIX4>& rotations = displayer.rotations();
  vector<MATRIX4>& scalings  = displayer.scalings();
  vector<VEC4>& translations = displayer.translations();
  vector<float>& lengths     = displayer.lengths();

  // Save bounding dimensions
  // float x_min = FLT_MAX;
  // float y_min = FLT_MAX;
  // float z_min = FLT_MAX;
  // float x_max = FLT_MIN;
  // float y_max = FLT_MIN;
  // float z_max = FLT_MIN;

  // Skip the first bone,
  // it's just the origin
  float min_y = 0.301897;
  int totalBones = rotations.size();
  for (int x = 1; x < totalBones; x++)
  {
    MATRIX4& rotation = rotations[x];
    MATRIX4& scaling = scalings[x];
    VEC4& translation = translations[x];

    // get the endpoints of the cylinder
    VEC4 leftVertex(0,0,0,1);
    VEC4 rightVertex(0,0,lengths[x],1);

    leftVertex = rotation * scaling * leftVertex + translation;
    rightVertex = rotation * scaling * rightVertex + translation;

    // Cylinders
    shapes.push_back(make_shared<Cylinder>(leftVertex.head<3>(), rightVertex.head<3>(), 0.05, VEC3(1,0,0)));

    // x_min = min({x_min, float(leftVertex[0]), float(rightVertex[0])});
    // y_min = min({y_min, float(leftVertex[1]), float(rightVertex[1])});
    // z_min = min({z_min, float(leftVertex[2]), float(rightVertex[2])});
    //
    // x_max = max({x_max, float(leftVertex[0]), float(rightVertex[0])});
    // y_max = max({y_max, float(leftVertex[1]), float(rightVertex[1])});
    // z_max = max({z_max, float(leftVertex[2]), float(rightVertex[2])});
  }
  // CAMERA SETTINGS -----------------------
  // TODO: REDO EYE SETTINGS FOR PRE-FALL SCENE
  eye = VEC3(-6, 0.5, 1);
  lookingAt = VEC3(30, 7, 30); // Starting looking: upwards right from lower left of room

  // 2/3 second to transition into falling through tunnel
  float tunnel_transition = 20 * 8;
  float eye_transition = 40 * 8;

  // TODO: move eye along parametric curve
  // TODO: Also change looking at point
  // Rotate eye to look down y-axis at center of frame after 40 frames
  VEC3 lookingAt1(0.5, 0.5, 1);
  VEC3 end = VEC3(0.49, 2, 1) - lookingAt1;
  VEC3 start = eye - lookingAt1;
  VEC3 axis = start.cross(end).normalized();
  float theta = acos(start.normalized().dot(end.normalized())) * min(1.0, (double) frame/eye_transition);
  eye = rotate(eye-lookingAt1, axis, theta) + lookingAt1;

  // Rotate "up" vector simultaneously
  up = VEC3(0,1,0);
  theta = -M_PI/2 * min(1.0, (double) frame/320);
  up = rotate(up, VEC3(1,0,0), theta);

  // TODO: Create z-buffer for model

  // Create rectangle mesh that forms triangle prism
  // Starting position: long axis is y-axis and top is right on the surface of the floor
  // Length: 150*8 frames
  float xmin = -0.1;
  float xmax = 1.7;
  float zmin = -0.4;
  float zmax = 1.9;

  // Expand mesh by a lot: MUST MAKE EQUILATERAL TRIANGLE
  VEC3 b_cap0(xmin, min_y, zmax);
  VEC3 c_cap0(xmax, min_y, zmax);
  VEC3 a_cap0((xmin+xmax)/2, min_y, (xmin - xmax) * sqrt(3)/2 + zmax); // Set A such that triangle is equilateral

  VEC3 cap_center = (a_cap0 + b_cap0 + c_cap0)/3;
  a_cap0 = (a_cap0 - cap_center) * 4 + cap_center;
  b_cap0 = (b_cap0 - cap_center) * 4 + cap_center;
  c_cap0 = (c_cap0 - cap_center) * 4 + cap_center;

  // Movement: bound below by frame_prism (0) and above by tunnel_transition (160)
  float movement_multiplier = max(float(0), frame-frame_prism);
  // Accelerate eye towards body (lookingAt point also moves)
  VEC3 tunnel_point = VEC3((xmin+xmax)/2, min_y + 0.5, (zmin+zmax)/2);
  VEC3 eye_path = (tunnel_point - eye).normalized();
  float accel = (tunnel_point-eye).norm()/pow(tunnel_transition, 2);
  eye = accel * pow(min(tunnel_transition, movement_multiplier), 2) * eye_path + eye;
  lookingAt = accel * pow(min(tunnel_transition, movement_multiplier), 2) * eye_path + lookingAt;

  // Add another light following the eye
  lights.push_back(make_shared<pointLight>(eye, VEC3(1,1,1)));

  // Slight acceleration to terminal velocity (0.5/8)
  move_per_frame *= (1 + min(float(4), 4 * (movement_multiplier)/tunnel_transition));

  // Pull up mesh by given velocity
  a_cap0 += VEC3(0, movement_multiplier * move_per_frame, 0);
  b_cap0 += VEC3(0, movement_multiplier * move_per_frame, 0);
  c_cap0 += VEC3(0, movement_multiplier * move_per_frame, 0);

  // Rotation: slow (180 every 2 seconds = 60 * 8 frames)
  float rot_theta = (movement_multiplier)/480.0 * M_PI;
  a_cap0 = rotate(a_cap0 - cap_center, VEC3(0,1,0), rot_theta) + cap_center;
  b_cap0 = rotate(b_cap0 - cap_center, VEC3(0,1,0), rot_theta) + cap_center;
  c_cap0 = rotate(c_cap0 - cap_center, VEC3(0,1,0), rot_theta) + cap_center;

  // Length of tunnel: 4 seconds == 120 frames * 0.5/8 move/frame * 8
  VEC3 a_cap1 = a_cap0 - VEC3(0, 60, 0);
  VEC3 b_cap1 = b_cap0 - VEC3(0, 60, 0);
  VEC3 c_cap1 = c_cap0 - VEC3(0, 60, 0);

  // Print out scene settings
  printf("Frame: %f ==========\n", frame);
  printf("Movement multiplier: %f\n", movement_multiplier);
  cout << "Eye: " << eye << endl;
  cout << "lookingAt: " << lookingAt << endl;
  cout << "Min y: " << min_y << endl;

  // TODO: Choreograph OMEGA MOTION BLUR (basically just sample across higher range of t)
  if (frame >= frame_prism)
  {
    generateTrianglePrismMesh(a_cap0, b_cap0, c_cap0, a_cap1, b_cap1, c_cap1, int(frame), true);
  }

  // INITIAL SCENE GLOBAL VERTICES (start eye in lower left corner)
  VEC3 globalA(-10, min_y + movement_multiplier * move_per_frame, -10);
  VEC3 globalB(-10, min_y + movement_multiplier * move_per_frame, 30);
  VEC3 globalC(50, min_y + movement_multiplier * move_per_frame, 30);
  VEC3 globalD(50, min_y + movement_multiplier * move_per_frame, -10);
  VEC3 globalE(-10, 10 + movement_multiplier * move_per_frame, -10);
  VEC3 globalF(-10, 10 + movement_multiplier * move_per_frame, 30);
  VEC3 globalG(50, 10 + movement_multiplier * move_per_frame, 30);
  VEC3 globalH(50, 10 + movement_multiplier * move_per_frame, -10);

  // Cull if behind eye (only AFTER finished eye transition)
  if (min_y + movement_multiplier * move_per_frame <= eye[1]+0.5 || frame < eye_transition)
  {
    // Trapdoor
    VEC3 A(xmin, min_y+movement_multiplier * move_per_frame, zmax);
    VEC3 B((xmin+xmax)/2, min_y+ movement_multiplier * move_per_frame, zmax);
    VEC3 C((xmin+xmax)/2, min_y+ movement_multiplier * move_per_frame, zmin);
    VEC3 D(xmin, min_y+movement_multiplier * move_per_frame, zmin);
    VEC3 E(xmax, min_y+movement_multiplier * move_per_frame, zmax);
    VEC3 F(xmax, min_y+movement_multiplier * move_per_frame, zmin);

    // Rotate inwards about hinges AD, EF
    // TODO: MAKE DOOR FALL LIKE REAL PHYSICS
    // TODO: MAKE THIS RECTANGULAR PRISM
    float angle = min(float(1.1), movement_multiplier/(tunnel_transition)) * M_PI/2;
    VEC3 B_left = rotate(B-A, D-A, angle) + A;
    VEC3 C_left = rotate(C-A, D-A, angle) + A;
    VEC3 B_right = rotate(B-E, F-E, -angle) + E;
    VEC3 C_right = rotate(C-E, F-E, -angle) + E;

    loadTexture("./textures/sad_finder1_adj.jpg");
    int tex_index = texture_frames.size()-1;
    shared_ptr<Rectangle> tmp = make_shared<Rectangle>(A, B_left, C_left, D, VEC3(0,0,0), "steel", false, tex_index, "cook-torrance");
    tmp->reflect_params.roughness = 0.7;
    tmp->reflect_params.refr = VEC2(2.75, 3.79);
    tmp->reflect_params.glossy = true;
    tmp->texture = true;
    shapes.push_back(make_shared<Rectangle>(*tmp));

    loadTexture("./textures/sad_finder2_adj.jpg");
    tex_index = texture_frames.size()-1;
    shared_ptr<Rectangle> tmp2 = make_shared<Rectangle>(B_right,E,F,C_right, VEC3(0,0,0), "steel", false, tex_index, "cook-torrance");
    tmp2->reflect_params.roughness = 0.7;
    tmp2->reflect_params.refr = VEC2(2.75, 3.79);
    tmp2->reflect_params.glossy = true;
    tmp2->texture = true;
    shapes.push_back(make_shared<Rectangle>(*tmp2));

    // OTHER INITIAL SCENE OBJECTS: indoors box
    // Floor: linoleum (texture applies to each SQUARE) + bounding lines
    loadTexture("./textures/floor.jpeg");
    tex_index = texture_frames.size()-1;
    float s = 1;
    VEC3 checker_rgb0(0.58, 0.82, 1);
    VEC3 checker_rgb1(1, 0.416, 0.835);
    shared_ptr<CheckerboardWithHole> floor = make_shared<CheckerboardWithHole>(globalA,
                                                globalB,
                                                globalC,
                                                globalD,
                                              checker_rgb0, checker_rgb1, s,
                                            make_shared<Rectangle>(A, E, F, D, VEC3(0,0,0)));
    floor->tex_frame = tex_index;
    floor->borderwidth = 0.05;
    floor->bordercolor = VEC3(0.33, 0.33, 0.33);
    floor->texture = true;
    floor->reflect_params.material = "linoleum";
    floor->reflect_params.roughness = 0.6;
    floor->reflect_params.refr = VEC2(1.543,0);
    floor->reflect_params.glossy = true;
    shapes.push_back(make_shared<CheckerboardWithHole>(*floor));

    // Walls
    shapes.push_back(make_shared<Rectangle>(globalA, globalD, globalH, globalE, VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<Rectangle>(globalA, globalB, globalF, globalE, VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<Rectangle>(globalB, globalC, globalF, globalG, VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<Rectangle>(globalD, globalC, globalG, globalH, VEC3(0,0.81,0.99)));

    // Ceiling
    shapes.push_back(make_shared<Rectangle>(globalE, globalF, globalG, globalH, VEC3(0,0.81,0.99)));

    // TODO: Floor -- linoleum checkerboard WITH bounding lines AND rectangle hole where stickman is falling
    // 1 x 1 squares
    // float s = 1;
    // VEC3 checker_rgb0(0.58, 0.82, 1);
    // VEC3 checker_rgb1(1, 0.416, 0.835);
    // shapes.push_back(make_shared<CheckerboardWithHole>(VEC3(-20, min_y+movement_multiplier * move_per_frame, -20),
    //                                             VEC3(-20, min_y+movement_multiplier * move_per_frame, 20),
    //                                             VEC3(20, min_y+movement_multiplier * move_per_frame, 20),
    //                                             VEC3(20, min_y+movement_multiplier * move_per_frame, -20),
    //                                           checker_rgb0, checker_rgb1, s,
    //                                         make_shared<Rectangle>(A, E, F, D, VEC3(0,0,0))));
  }
  cout << "Global A: " << globalA << endl;
  cout << "Global B: " << globalB << endl;
  cout << "Global C: " << globalC << endl;
  cout << "Global D: " << globalD << endl;
  cout << "Global E: " << globalE << endl;
  cout << "Global F: " << globalF << endl;
  cout << "Global G: " << globalG << endl;
  cout << "Global H: " << globalH << endl;

  // Ceiling lights + flickering (?) + semisphere light (cut in half by ceiling) w/ time interpolating hues
  // TODO: FIGURE OUT WHEN TO STOP RENDERING THE CEILING LIGHTS
  std::random_device rd;
  std::mt19937 generator(rd());
  std::uniform_real_distribution<double> uniform(0.0, 1.0);

  VEC3 lightcol(0.8, 0.8, 0.8);
  float lighth = 30.0/7;
  float hbound = 15.0/7;
  float lightw = 14;
  float wbound = 3;
  VEC3 globalEH = (globalH-globalE).normalized();
  VEC3 globalEF = (globalF-globalE).normalized();
  for (int col = 0; col < 2; col++)
  {
    for (int i = 0; i < 8; i++)
    {
      VEC3 a_tmp = globalE + hbound * globalEH + wbound * globalEF - VEC3(0,0.001,0);
      if (i > 3)
      {
        a_tmp += hbound * globalEH;
      }
      if (col > 0)
      {
        a_tmp += wbound * globalEF;
      }
      a_tmp += i * (lighth + hbound) * globalEH + col * (lightw + wbound) * globalEF;
      // TODO: WILL THIS STAY IN MEMORY???
      // Every light has a 20% chance of turning off
      shared_ptr<rectangleLight> tmp_light = make_shared<rectangleLight>(a_tmp, a_tmp + lightw * globalEF,
            a_tmp + lighth * globalEH + lightw*globalEF, a_tmp + lighth*globalEH, lightcol);
      // if (uniform(generator) <= 0.2)
      // {
      //   tmp_light->light = false;
      //   tmp_light->Rectangle::color = VEC3(0.1, 0.1, 0.1);
      // }
      // else
      // {
      //   lights.push_back(tmp_light);
      // }
      lights.push_back(tmp_light);
      shapes.push_back(tmp_light);

      cout << "Rectangle light coordinates ==========" << endl;
      cout << "A: " << a_tmp << endl;
      cout << "B: " << a_tmp + lightw * globalEF << endl;
      cout << "C: " << a_tmp + lighth * globalEH + lightw*globalEF << endl;
      cout << "D: " << a_tmp + lighth*globalEH << endl;
    }
  }

  // Sphere light
  VEC3 ceilingCenter = (globalE + globalF + globalG + globalH)/4;

  // Modulate color based on time (loop in 1.5 seconds = 45*8 frames)
  VEC3 c1(0.73, 0.4, 1);
  VEC3 c2(1, 0.44, 0.81);
  VEC3 cmod = c1 + (c2 - c1) * abs(cos(M_PI * frame/(45*8)));
  shared_ptr<sphereLight> centerlight = make_shared<sphereLight>(ceilingCenter, 2, cmod);
  centerlight->baxis = VEC3(0,-1,0);
  lights.push_back(centerlight);
  shapes.push_back(centerlight);

  cout << "Ceiling sphere light color: " << cmod << endl;

  // TODO: Corner cylinder + stairs + glass surface

  // TODO: stick figure: cook-torrance reflectance with REFLECTION (reflect ads!)

  // TODO: flying coke can (motion blurred)

  // TODO: Giant helios head -- CULL triangles that are outside of viewing frustum
}

// Test one thing at a time: first the textured floor and walls
// Test the inclusion of different objects with each run
void buildSceneBoundary(float frame)
{
  shapes.clear();
  lights.clear();

  float min_y = 0.301897;
  float xmin = -0.1;
  float xmax = 1.7;
  float zmin = -0.4;
  float zmax = 1.9;

  eye = VEC3(-6, 5, 1);
  lookingAt = VEC3(50, 4, 1); // Starting looking: straight at back of room

  VEC3 globalA(-10, min_y, -10);
  VEC3 globalB(-10, min_y, 30);
  VEC3 globalC(70, min_y, 30);
  VEC3 globalD(70, min_y, -10);
  VEC3 globalE(-10, 15, -10);
  VEC3 globalF(-10, 15, 30);
  VEC3 globalG(70, 15, 30);
  VEC3 globalH(70, 15, -10);

  VEC3 globalEH = (globalH-globalE).normalized();
  VEC3 globalEF = (globalF-globalE).normalized();
  VEC3 globalAD = (globalA-globalD).normalized();
  VEC3 globalCD = (globalC - globalD).normalized();

  // Trapdoor
  VEC3 A(xmin, min_y, zmax);
  VEC3 B((xmin+xmax)/2, min_y, zmax);
  VEC3 C((xmin+xmax)/2, min_y, zmin);
  VEC3 D(xmin, min_y, zmin);
  VEC3 E(xmax, min_y, zmax);
  VEC3 F(xmax, min_y, zmin);

  if (frame == -1) // Literally just rectangles man...
  {
    shapes.push_back(make_shared<Rectangle>(VEC3(0, 4, 9), VEC3(0, 4, 11), VEC3(0, 6, 11), VEC3(0, 6, 9), VEC3(1,0,0)));

    // Floor
    shapes.push_back(make_shared<Rectangle>(globalA, globalB, globalC, globalD, VEC3(1,0,0)));

    // Walls
    shapes.push_back(make_shared<Rectangle>(globalA, globalD, globalH, globalE, VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<Rectangle>(globalA, globalB, globalF, globalE, VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<Rectangle>(globalB, globalC, globalG, globalF, VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<Rectangle>(globalD, globalC, globalG, globalH, VEC3(0,0.81,0.99)));
    //
    // // Ceiling
    shapes.push_back(make_shared<Rectangle>(globalE, globalF, globalG, globalH, VEC3(0,0.81,0.99)));

    // Light: at eye
    lights.push_back(make_shared<pointLight>(eye, VEC3(1,1,1)));
  }
  else if (frame == 0) // Basic 4 walls + ceiling + floor
  {
    loadTexture("./textures/floor.jpeg");
    int tex_index = texture_frames.size()-1;
    float s = 1;
    VEC3 checker_rgb0(0.58, 0.82, 1);
    VEC3 checker_rgb1(1, 0.416, 0.835);
    shared_ptr<CheckerboardWithHole> floor = make_shared<CheckerboardWithHole>(globalA,
                                                globalB,
                                                globalC,
                                                globalD,
                                              checker_rgb0, checker_rgb1, s,
                                            make_shared<Rectangle>(A, E, F, D, VEC3(0,0,0)));
    floor->tex_frame = tex_index;
    floor->borderwidth = 0.05;
    floor->bordercolor = VEC3(0.55, 0.55, 0.55);
    floor->texture = true;
    floor->reflect_params.material = "linoleum";
    floor->reflect_params.roughness = 0.6;
    floor->reflect_params.refr = VEC2(1.543,0);
    floor->reflect_params.glossy = true;
    shapes.push_back(make_shared<CheckerboardWithHole>(*floor));

    // Walls
    shapes.push_back(make_shared<Rectangle>(globalA, globalD, globalH, globalE, VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<Rectangle>(globalA, globalB, globalF, globalE, VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<Rectangle>(globalB, globalC, globalF, globalG, VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<Rectangle>(globalD, globalC, globalG, globalH, VEC3(0,0.81,0.99)));

    // Ceiling
    shapes.push_back(make_shared<Rectangle>(globalE, globalF, globalG, globalH, VEC3(0,0.81,0.99)));

    // Light: spherelight in center of room
    shared_ptr<sphereLight> pinklight = make_shared<sphereLight>(VEC3(40, 7, 20), 2, halogen);
    lights.push_back(pinklight);
    shapes.push_back(pinklight);
  }
  else if (frame == 1) // Ceiling rectangle lights + sphere light + basic checkerboard floor + walls + trapdoor + cylinder man
  {
    // Get bone names in skeleton
    Skeleton* skel = displayer.GetSkeleton(0);

    // retrieve all the bones of the skeleton
    vector<MATRIX4>& rotations = displayer.rotations();
    vector<MATRIX4>& scalings  = displayer.scalings();
    vector<VEC4>& translations = displayer.translations();
    vector<float>& lengths     = displayer.lengths();

    // Skip the first bone,
    // it's just the origin
    int totalBones = rotations.size();
    for (int x = 1; x < totalBones; x++)
    {
      MATRIX4& rotation = rotations[x];
      MATRIX4& scaling = scalings[x];
      VEC4& translation = translations[x];

      // get the endpoints of the cylinder
      VEC4 leftVertex(0,0,0,1);
      VEC4 rightVertex(0,0,lengths[x],1);

      leftVertex = rotation * scaling * leftVertex + translation;
      rightVertex = rotation * scaling * rightVertex + translation;

      // Cylinders
      shapes.push_back(make_shared<Cylinder>(leftVertex.head<3>(), rightVertex.head<3>(), 0.05, VEC3(1,0,0)));
    }

    // Checkerboard floor
    float s = 1;
    VEC3 checker_rgb0(0.58, 0.82, 1);
    VEC3 checker_rgb1(1, 0.416, 0.835);
    shared_ptr<CheckerboardWithHole> floor = make_shared<CheckerboardWithHole>(globalA,
                                                globalB,
                                                globalC,
                                                globalD,
                                              checker_rgb0, checker_rgb1, s,
                                            make_shared<Rectangle>(A, E, F, D, VEC3(0,0,0)));
    shapes.push_back(make_shared<CheckerboardWithHole>(*floor));

    // Trapdoor
    loadTexture("./textures/sad_finder1_adj.jpg");
    int tex_index = texture_frames.size()-1;
    shared_ptr<Rectangle> tmp = make_shared<Rectangle>(A, B, C, D, VEC3(0,0,0), "steel", false, tex_index, "cook-torrance");
    tmp->reflect_params.roughness = 0.7;
    tmp->reflect_params.refr = VEC2(2.75, 3.79);
    tmp->reflect_params.glossy = true;
    tmp->texture = true;
    shapes.push_back(make_shared<Rectangle>(*tmp));

    loadTexture("./textures/sad_finder2_adj.jpg");
    tex_index = texture_frames.size()-1;
    shared_ptr<Rectangle> tmp2 = make_shared<Rectangle>(B,E,F,C, VEC3(0,0,0), "steel", false, tex_index, "cook-torrance");
    tmp2->reflect_params.roughness = 0.7;
    tmp2->reflect_params.refr = VEC2(2.75, 3.79);
    tmp2->reflect_params.glossy = true;
    tmp2->texture = true;
    shapes.push_back(make_shared<Rectangle>(*tmp2));

    // Walls
    shapes.push_back(make_shared<Rectangle>(globalA, globalD, globalH, globalE, VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<Rectangle>(globalA, globalB, globalF, globalE, VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<Rectangle>(globalB, globalC, globalF, globalG, VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<Rectangle>(globalD, globalC, globalG, globalH, VEC3(0,0.81,0.99)));

    // Ceiling + lights
    shapes.push_back(make_shared<Rectangle>(globalE, globalF, globalG, globalH, VEC3(0,0.81,0.99)));
    int nlights = 4; // 4 lights each column
    VEC3 lightcol(0.8, 0.8, 0.8);
    float lighth = (globalD[0]-globalA[0])/(nlights + (nlights+2)/2); // 10 l + 3 * hbound (1/2 l)
    float hbound = lighth/2;
    float lightw = (globalB[2] - globalA[2])/(2 + 3.0/7); // 2 w + 3 * wbound (1/7 w)
    float wbound = lightw/7;
    VEC3 globalEH = (globalH-globalE).normalized();
    VEC3 globalEF = (globalF-globalE).normalized();
    // for (int col = 0; col < 2; col++)
    // {
    //   for (int i = 0; i < nlights; i++)
    //   {
    //     VEC3 a_tmp = globalE + hbound * globalEH + wbound * globalEF;
    //     if (i > 3)
    //     {
    //       a_tmp += hbound * globalEH;
    //     }
    //     if (col > 0)
    //     {
    //       a_tmp += wbound * globalEF;
    //     }
    //     a_tmp += i * (lighth + hbound) * globalEH + col * (lightw + wbound) * globalEF;
    //     shared_ptr<rectangleLight> tmp_light = make_shared<rectangleLight>(a_tmp, a_tmp + lightw * globalEF,
    //           a_tmp + lighth * globalEH + lightw*globalEF, a_tmp + lighth*globalEH, lightcol);
    //     lights.push_back(tmp_light);
    //     shapes.push_back(tmp_light);
    //
    //     cout << "Rectangle light coordinates ==========" << endl;
    //     cout << "A: " << a_tmp << endl;
    //     cout << "B: " << a_tmp + lightw * globalEF << endl;
    //     cout << "C: " << a_tmp + lighth * globalEH + lightw*globalEF << endl;
    //     cout << "D: " << a_tmp + lighth*globalEH << endl;
    //   }
    // }

    // Try: Two panel lights in the back of the room
    VEC3 a_tmp = globalH + (lighth + hbound) * (-globalEH) + wbound * globalEF;
    shared_ptr<rectangleLight> left_light = make_shared<rectangleLight>(a_tmp, a_tmp + lightw * globalEF,
          a_tmp + lighth * globalEH + lightw*globalEF, a_tmp + lighth*globalEH, lightcol);
    lights.push_back(left_light);
    shapes.push_back(left_light);

    a_tmp = globalH + (lighth + hbound) * (-globalEH) + (3 * wbound + lightw) * globalEF;
    shared_ptr<rectangleLight> right_light = make_shared<rectangleLight>(a_tmp, a_tmp + lightw * globalEF,
          a_tmp + lighth * globalEH + lightw*globalEF, a_tmp + lighth*globalEH, lightcol);
    lights.push_back(right_light);
    shapes.push_back(right_light);

    // Light: spherelight in center of room
    shared_ptr<sphereLight> pinklight = make_shared<sphereLight>(VEC3(40, 7, 20), 2, halogen);
    lights.push_back(pinklight);
    shapes.push_back(pinklight);
    shapes.push_back(make_shared<Rectangle>(globalE, globalF, globalG, globalH, VEC3(0,0.81,0.99)));

    // Sphere light + modulated color
    // VEC3 ceilingCenter = (globalE + globalF + globalG + globalH)/4;
    // VEC3 c1(0.73, 0.4, 1);
    // VEC3 c2(1, 0.44, 0.81);
    // VEC3 cmod = c1 + (c2 - c1) * abs(cos(M_PI * frame/(45*8)));
    // shared_ptr<sphereLight> centerlight = make_shared<sphereLight>(ceilingCenter, 2, cmod);
    // centerlight->baxis = VEC3(0,-1,0);
    // lights.push_back(centerlight);
    // shapes.push_back(centerlight);
  }
  else if (frame == 2) // Corner cylinder + stairs + glass banister (maybe)
  {
    loadTexture("./textures/floor.jpeg");
    int tex_index = texture_frames.size()-1;
    float s = 1;
    VEC3 checker_rgb0(0.58, 0.82, 1);
    VEC3 checker_rgb1(1, 0.416, 0.835);
    shared_ptr<CheckerboardWithHole> floor = make_shared<CheckerboardWithHole>(globalA,
                                                globalB,
                                                globalC,
                                                globalD,
                                              checker_rgb0, checker_rgb1, s,
                                            make_shared<Rectangle>(A, E, F, D, VEC3(0,0,0)));
    floor->tex_frame = tex_index;
    floor->borderwidth = 0.05;
    floor->bordercolor = VEC3(0.55, 0.55, 0.55);
    floor->texture = true;
    floor->reflect_params.material = "linoleum";
    floor->reflect_params.roughness = 0.6;
    floor->reflect_params.refr = VEC2(1.543,0);
    floor->reflect_params.glossy = true;
    shapes.push_back(make_shared<CheckerboardWithHole>(*floor));

    // Walls
    shapes.push_back(make_shared<Rectangle>(globalA, globalD, globalH, globalE, VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<Rectangle>(globalA, globalB, globalF, globalE, VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<Rectangle>(globalB, globalC, globalF, globalG, VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<Rectangle>(globalD, globalC, globalG, globalH, VEC3(0,0.81,0.99)));

    // Ceiling
    shapes.push_back(make_shared<Rectangle>(globalE, globalF, globalG, globalH, VEC3(0,0.81,0.99)));

    // #### Staircase
    // Set useful global params
    VEC3 globalEH = (globalE - globalH).normalized();
    VEC3 globalAD = (globalA-globalD).normalized();
    VEC3 globalCD = (globalC - globalD).normalized();

    // TODO: Figure out the right staircase prism dimensions
    float l_prism = 2;
    float w_prism = 5.0/20;
    float h_prism = 10.0/20;

    // Corner cylinder with texture (SAME MATERIAL + TEXTURE AS FLOOR)
    float r = 6 * w_prism;
    VEC3 c_top = globalH + globalEH * (l_prism + r);
    VEC3 c_bot = globalD + globalAD * (l_prism + r);
    shared_ptr<CheckerCylinder> cyl = make_shared<CheckerCylinder>(c_top, c_bot, r, VEC3(1,1,1), s, "linoleum");
    cyl->tex_frame = tex_index;
    cyl->borderwidth = 0.05;
    cyl->bordercolor = VEC3(0.55, 0.55, 0.55);
    cyl->texture = true;
    cyl->reflect_params.material = "linoleum";
    cyl->reflect_params.roughness = 0.6;
    cyl->reflect_params.refr = VEC2(1.543,0);
    cyl->reflect_params.glossy = true;
    shapes.push_back(make_shared<CheckerCylinder>(*cyl));

    // Staircase: rotate vertices about center of cylinder and make way up counterclockwise
    // Stop conditions: height reaches 80% of cylinder height OR hit left wall
    // Rotate DA vector 10 degrees counterclockwise centered at cylinder bottom center
    VEC3 width_vector = VEC3(-1, 0, -0.8).normalized();
    VEC3 length_vector = VEC3(0,1,0).cross(width_vector).normalized();
    VEC3 height_vector = VEC3(0,1,0);

    VEC3 stair_d = rotate(globalAD * r, (c_top - c_bot).normalized(), -10 * M_PI/180) + c_bot;
    VEC3 stair_a = stair_d + width_vector * w_prism;
    VEC3 stair_b = stair_a + length_vector * l_prism;
    VEC3 stair_c = stair_d + length_vector * l_prism;
    VEC3 stair_e = stair_a + height_vector * h_prism;
    VEC3 stair_f = stair_b + height_vector * h_prism;
    VEC3 stair_g = stair_c + height_vector * h_prism;
    VEC3 stair_h = stair_d + height_vector * h_prism;

    cout << "Length vector: " << length_vector << endl;
    cout << "Width vector: " << width_vector << endl;
    cout << "Stair a: " << stair_a << endl;
    cout << "Stair b: " << stair_b << endl;
    cout << "Stair c: " << stair_c << endl;
    cout << "Stair d: " << stair_d << endl;
    cout << "Stair e: " << stair_e << endl;
    cout << "Stair f: " << stair_f << endl;
    cout << "Stair g: " << stair_g << endl;
    cout << "Stair h: " << stair_h << endl;

    VEC3 tmp_center = c_bot;
    float theta = acos(1 - pow(w_prism, 2)/pow((stair_a - tmp_center).norm(), 2));
    while (stair_e[1] <= 8 && length_vector.dot(globalCD) > 0)
    {
      shapes.push_back(make_shared<RectPrismV2>(stair_a, stair_b, stair_c, stair_d, stair_e, stair_f, stair_g,
                                              stair_h, pink));
      // Rotation formula (this is guaranteed by the fact that D touches the surface of the cylinder!)
      // theta = acos(1 - w^2/(A-center).normsq)
      stair_a = rotate(stair_a - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;
      stair_b = rotate(stair_b - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;
      stair_c = rotate(stair_c - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;
      stair_d = rotate(stair_d - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;

      tmp_center += height_vector;
      stair_e = rotate(stair_e - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;
      stair_f = rotate(stair_f - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;
      stair_g = rotate(stair_g - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;
      stair_h = rotate(stair_h - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;

      // Recompute new length and width vectors
      width_vector = (stair_a - stair_d).normalized();
      length_vector = (stair_b - stair_a).normalized();

      // Compute new theta
      theta = acos(1 - pow(w_prism, 2)/pow((stair_a - tmp_center).norm(), 2));
    }

    // Light: at eye
    lights.push_back(make_shared<pointLight>(eye, VEC3(1,1,1)));
  }
  else if (frame == 3) // Putting all the above together + models
  {
    // Get bone names in skeleton
    Skeleton* skel = displayer.GetSkeleton(0);

    // retrieve all the bones of the skeleton
    vector<MATRIX4>& rotations = displayer.rotations();
    vector<MATRIX4>& scalings  = displayer.scalings();
    vector<VEC4>& translations = displayer.translations();
    vector<float>& lengths     = displayer.lengths();

    // Skip the first bone,
    // it's just the origin
    int totalBones = rotations.size();
    for (int x = 1; x < totalBones; x++)
    {
      MATRIX4& rotation = rotations[x];
      MATRIX4& scaling = scalings[x];
      VEC4& translation = translations[x];

      // get the endpoints of the cylinder
      VEC4 leftVertex(0,0,0,1);
      VEC4 rightVertex(0,0,lengths[x],1);

      leftVertex = rotation * scaling * leftVertex + translation;
      rightVertex = rotation * scaling * rightVertex + translation;

      // Cylinders
      shapes.push_back(make_shared<Cylinder>(leftVertex.head<3>(), rightVertex.head<3>(), 0.05, VEC3(1,0,0)));
    }

    // Trapdoor
    loadTexture("./textures/sad_finder1_adj.jpg");
    int tex_index = texture_frames.size()-1;
    shared_ptr<Rectangle> tmp = make_shared<Rectangle>(A, B, C, D, VEC3(0,0,0), "steel", false, tex_index, "cook-torrance");
    tmp->reflect_params.roughness = 0.7;
    tmp->reflect_params.refr = VEC2(2.75, 3.79);
    tmp->reflect_params.glossy = true;
    tmp->texture = true;
    shapes.push_back(make_shared<Rectangle>(*tmp));

    loadTexture("./textures/sad_finder2_adj.jpg");
    tex_index = texture_frames.size()-1;
    shared_ptr<Rectangle> tmp2 = make_shared<Rectangle>(B,E,F,C, VEC3(0,0,0), "steel", false, tex_index, "cook-torrance");
    tmp2->reflect_params.roughness = 0.7;
    tmp2->reflect_params.refr = VEC2(2.75, 3.79);
    tmp2->reflect_params.glossy = true;
    tmp2->texture = true;
    shapes.push_back(make_shared<Rectangle>(*tmp2));

    loadTexture("./textures/floor.jpeg");
    tex_index = texture_frames.size()-1;
    float s = 1;
    VEC3 checker_rgb0(0.58, 0.82, 1);
    VEC3 checker_rgb1(1, 0.416, 0.835);
    shared_ptr<CheckerboardWithHole> floor = make_shared<CheckerboardWithHole>(globalA,
                                                globalB,
                                                globalC,
                                                globalD,
                                              checker_rgb0, checker_rgb1, s,
                                            make_shared<Rectangle>(A, E, F, D, VEC3(0,0,0)));
    floor->tex_frame = tex_index;
    floor->borderwidth = 0.05;
    floor->bordercolor = VEC3(0.55, 0.55, 0.55);
    floor->texture = true;
    floor->reflect_params.material = "linoleum";
    floor->reflect_params.roughness = 0.6;
    floor->reflect_params.refr = VEC2(1.543,0);
    floor->reflect_params.glossy = true;
    shapes.push_back(make_shared<CheckerboardWithHole>(*floor));

    // Walls
    shapes.push_back(make_shared<Rectangle>(globalA, globalD, globalH, globalE, VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<Rectangle>(globalA, globalB, globalF, globalE, VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<Rectangle>(globalB, globalC, globalF, globalG, VEC3(0,0.81,0.99)));
    shapes.push_back(make_shared<Rectangle>(globalD, globalC, globalG, globalH, VEC3(0,0.81,0.99)));

    // Ceiling + lights
    shapes.push_back(make_shared<Rectangle>(globalE, globalF, globalG, globalH, VEC3(0,0.81,0.99)));
    int nlights = 4;
    VEC3 lightcol(1, 1, 1);
    float lighth = (globalD[0]-globalA[0])/(nlights + (nlights+2)/2); // 10 l + (10+2) * hbound (1/2 l)
    float hbound = lighth/2;
    float lightw = (globalB[2] - globalA[2])/(2 + 4.0/5); // 2 w + (2 + 2) * wbound (1/5 w)
    float wbound = lightw/5;
    for (int col = 0; col < 2; col++)
    {
      for (int i = 0; i < 8; i++)
      {
        VEC3 a_tmp = globalE + hbound * globalEH + wbound * globalEF - VEC3(0,0.001,0);
        if (i > 3)
        {
          a_tmp += hbound * globalEH;
        }
        if (col > 0)
        {
          a_tmp += wbound * globalEF;
        }
        a_tmp += i * (lighth + hbound) * globalEH + col * (lightw + wbound) * globalEF;
        shared_ptr<rectangleLight> tmp_light = make_shared<rectangleLight>(a_tmp, a_tmp + lightw * globalEF,
              a_tmp + lighth * globalEH + lightw*globalEF, a_tmp + lighth*globalEH, lightcol);
        lights.push_back(tmp_light);
        shapes.push_back(tmp_light);

        cout << "Rectangle light coordinates ==========" << endl;
        cout << "A: " << a_tmp << endl;
        cout << "B: " << a_tmp + lightw * globalEF << endl;
        cout << "C: " << a_tmp + lighth * globalEH + lightw*globalEF << endl;
        cout << "D: " << a_tmp + lighth*globalEH << endl;
      }
    }

    // #### Staircase
    // TODO: Figure out the right staircase prism dimensions
    float l_prism = 2;
    float w_prism = 5.0/20;
    float h_prism = 10.0/20;

    // Corner cylinder with texture (SAME MATERIAL + TEXTURE AS FLOOR)
    float r = 6 * w_prism;
    VEC3 c_top = globalH + globalEH * (l_prism + r);
    VEC3 c_bot = globalD + globalAD * (l_prism + r);
    shared_ptr<CheckerCylinder> cyl = make_shared<CheckerCylinder>(c_top, c_bot, r, VEC3(1,1,1), s, "linoleum");
    cyl->tex_frame = tex_index;
    cyl->borderwidth = 0.05;
    cyl->bordercolor = VEC3(0.55, 0.55, 0.55);
    cyl->texture = true;
    cyl->reflect_params.material = "linoleum";
    cyl->reflect_params.roughness = 0.6;
    cyl->reflect_params.refr = VEC2(1.543,0);
    cyl->reflect_params.glossy = true;
    shapes.push_back(make_shared<CheckerCylinder>(*cyl));

    // Staircase: rotate vertices about center of cylinder and make way up counterclockwise
    // Stop conditions: height reaches 80% of cylinder height OR hit left wall
    // Rotate DA vector 10 degrees counterclockwise centered at cylinder bottom center
    VEC3 width_vector = VEC3(-1, 0, -0.8).normalized();
    VEC3 length_vector = VEC3(0,1,0).cross(width_vector).normalized();
    VEC3 height_vector = VEC3(0,1,0);

    VEC3 stair_d = rotate(globalAD * r, (c_top - c_bot).normalized(), -10 * M_PI/180) + c_bot;
    VEC3 stair_a = stair_d + width_vector * w_prism;
    VEC3 stair_b = stair_a + length_vector * l_prism;
    VEC3 stair_c = stair_d + length_vector * l_prism;
    VEC3 stair_e = stair_a + height_vector * h_prism;
    VEC3 stair_f = stair_b + height_vector * h_prism;
    VEC3 stair_g = stair_c + height_vector * h_prism;
    VEC3 stair_h = stair_d + height_vector * h_prism;

    VEC3 tmp_center = c_bot;
    float theta = acos(1 - pow(w_prism, 2)/pow((stair_a - tmp_center).norm(), 2));
    cout << "Staircase E: " << stair_e << endl;
    cout << "Length vector: " << length_vector << endl;
    cout << "Length vector dot cylinder norm: " << length_vector.dot(globalCD) << endl;
    while (stair_e[1] <= 8 && length_vector.dot(globalCD) > 0)
    {
      shapes.push_back(make_shared<RectPrismV2>(stair_a, stair_b, stair_c, stair_d, stair_e, stair_f, stair_g,
                                              stair_h, pink));
      // Rotation formula (this is guaranteed by the fact that D touches the surface of the cylinder!)
      // theta = acos(1 - w^2/(A-center).normsq)
      stair_a = rotate(stair_a - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;
      stair_b = rotate(stair_b - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;
      stair_c = rotate(stair_c - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;
      stair_d = rotate(stair_d - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;

      tmp_center += height_vector;
      stair_e = rotate(stair_e - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;
      stair_f = rotate(stair_f - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;
      stair_g = rotate(stair_g - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;
      stair_h = rotate(stair_h - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;

      // Recompute new length and width vectors
      width_vector = (stair_a - stair_d).normalized();
      length_vector = (stair_b - stair_a).normalized();
      cout << "Staircase E: " << stair_e << endl;
      cout << "Length vector dot cylinder norm: " << length_vector.dot(globalCD) << endl;
      // Compute new theta
      theta = acos(1 - pow(w_prism, 2)/pow((stair_a - tmp_center).norm(), 2));
    }

    // LOAD MODELS ====================================
    // Load model vertices
    vector<VEC3> vertices;
    vector<VEC2> texcoords;
    vector<VEC3I> v_inds;
    vector<VEC3I> t_inds;
    vector<VEC3> normals;
    vector<VEC3I> n_inds;

    loadObj("./models/Column_LP_obj/Column_LP.obj", "./models/Column_LP_obj", vertices, v_inds, texcoords, t_inds, normals, n_inds);
    printf("Pedestal ===========\n# of vertices: %d, # of texcoords: %d, # of triangles: %d\n",
              int(vertices.size()), int(texcoords.size()), int(v_inds.size()));
    // Pedestal transform: scale up by 5x, add VEC3(20, 0, 20)
    // Note: we can apply scaling matrix directly because bottom is at WORLD ORIGIN
    MATRIX4 column_transf; column_transf << 5, 0, 0, 20,
                                            0, 5, 0, 0,
                                            0, 0, 5, 20,
                                            0, 0, 0, 1;
    vector<VEC3> pverts1;
    for (int i = 0; i < vertices.size(); i++)
    {
      VEC4 tmp; tmp << vertices[i], 1;
      VEC4 tmp2 = column_transf * tmp;
      pverts1.push_back(tmp2.head<3>());
    }

    // Keep track of shape bounds
    float x_min = FLT_MAX;
    float y_min = FLT_MAX;
    float z_min = FLT_MAX;
    float x_max = FLT_MIN;
    float y_max = FLT_MIN;
    float z_max = FLT_MIN;

    // Texture data
    loadTexture("./models/Column_LP_obj/Textures/Marble_Base_Color.jpg");
    int width, height, n;
    string rough_file = "./models/Column_LP_obj/Textures/Marble_Roughness.jpg";
    unsigned char* frame_data = stbi_load(rough_file.c_str(), &width, &height, &n, 0);
    for (int i = 0; i < v_inds.size(); i++)
    {
      VEC3I v_ind = v_inds[i];
      // Increase shape size by 3
      VEC3 tmp_center = (pverts1[v_ind[0]] + pverts1[v_ind[1]] + pverts1[v_ind[2]])/3;

      shared_ptr<Triangle> tmp = make_shared<Triangle>(pverts1[v_ind[0]],
                                            pverts1[v_ind[1]],
                                            pverts1[v_ind[2]],
                                            VEC3(0.75, 0.75, 0.75), "marble", false, "oren-nayar");

      // Texture coordinates can sometimes be over 1: repeat texture in that case
      VEC2 uvA = texcoords[t_inds[i][0]];
      VEC2 uvB = texcoords[t_inds[i][1]];
      VEC2 uvC = texcoords[t_inds[i][2]];
      if (uvA[0] > 1) uvA[0] = uvA[0] - int(uvA[0]);
      if (uvA[1] > 1) uvA[1] = uvA[1] - int(uvA[1]);
      if (uvB[0] > 1) uvB[0] = uvB[0] - int(uvB[0]);
      if (uvB[1] > 1) uvB[1] = uvB[1] - int(uvB[1]);
      if (uvC[0] > 1) uvC[0] = uvC[0] - int(uvC[0]);
      if (uvC[1] > 1) uvC[1] = uvC[1] - int(uvC[1]);

      // Check that texcoords are within bounds
      if (!(uvA[0] >= 0 && uvA[1] <= 1 &&
            uvB[0] >= 0 && uvB[1] <= 1 &&
            uvC[0] >= 0 && uvC[1] <= 1))
      {
        cout << "A: " << uvA << endl;
        cout << "B: " << uvB << endl;
        cout << "C: " << uvC << endl;

        printf("Texcoords out of bounds\n");
        throw;
      }

      // Test: maybe flip Y value of UV
      uvA[1] = 1 - uvA[1];
      uvB[1] = 1 - uvB[1];
      uvC[1] = 1 - uvC[1];

      // Set vertex UVs
      tmp->uv_verts = true;
      tmp->uvA = uvA;
      tmp->uvB = uvB;
      tmp->uvC = uvC;
      tmp->tex_frame = texture_frames.size()-1;
      tmp->texture = true;

      // Set roughness based on roughness map
      // TODO: FIGURE OUT HOW TO ASSIGN THIS -- IS IT INTERPOLATED ACROSS VERTICES?
      // Just assign average roughness across texcoords for now
      float r1 = frame_data[int(texcoords[t_inds[i][0]][0] * int(width-1) + texcoords[t_inds[i][0]][1] * int(height-1) * (width-1))];
      float r2 = frame_data[int(texcoords[t_inds[i][1]][0] * int(width-1) + texcoords[t_inds[i][1]][1] * int(height-1) * (width-1))];
      float r3 = frame_data[int(texcoords[t_inds[i][2]][0] * int(width-1) + texcoords[t_inds[i][2]][1] * int(height-1) * (width-1))];
      tmp->reflect_params.roughness = (r1+r2+r3)/(3 * 255);

      shapes.push_back(make_shared<Triangle>(*tmp));
      x_min = min({x_min, float(vertices[v_ind[0]][0]), float(vertices[v_ind[1]][0]), float(vertices[v_ind[2]][0])});
      y_min = min({y_min, float(vertices[v_ind[0]][1]), float(vertices[v_ind[1]][1]), float(vertices[v_ind[2]][1])});
      z_min = min({z_min, float(vertices[v_ind[0]][2]), float(vertices[v_ind[1]][2]), float(vertices[v_ind[2]][2])});

      x_max = max({x_max, float(vertices[v_ind[0]][0]), float(vertices[v_ind[1]][0]), float(vertices[v_ind[2]][0])});
      y_max = max({y_max, float(vertices[v_ind[0]][1]), float(vertices[v_ind[1]][1]), float(vertices[v_ind[2]][1])});
      z_max = max({z_max, float(vertices[v_ind[0]][2]), float(vertices[v_ind[1]][2]), float(vertices[v_ind[2]][2])});
    }
    // Run again for other pedestals
    // Make another column to the left of main column that will hold glass head
    column_transf << 3, 0, 0, 28,
                      0, 3, 0, 0,
                      0, 0, 3, 20,
                      0, 0, 0, 1;
    vector<VEC3> pverts2;
    for (int i = 0; i < vertices.size(); i++)
    {
      VEC4 tmp; tmp << vertices[i], 1;
      VEC4 tmp2 = column_transf * tmp;
      pverts2.push_back(tmp2.head<3>());
    }
    for (int i = 0; i < v_inds.size(); i++)
    {
      VEC3I v_ind = v_inds[i];
      // Increase shape size by 3
      VEC3 tmp_center = (pverts2[v_ind[0]] + pverts2[v_ind[1]] + pverts2[v_ind[2]])/3;

      shared_ptr<Triangle> tmp = make_shared<Triangle>(pverts2[v_ind[0]],
                                            pverts2[v_ind[1]],
                                            pverts2[v_ind[2]],
                                            VEC3(0.75, 0.75, 0.75), "marble", false, "oren-nayar");

      // Texture coordinates can sometimes be over 1: repeat texture in that case
      VEC2 uvA = texcoords[t_inds[i][0]];
      VEC2 uvB = texcoords[t_inds[i][1]];
      VEC2 uvC = texcoords[t_inds[i][2]];
      if (uvA[0] > 1) uvA[0] = uvA[0] - int(uvA[0]);
      if (uvA[1] > 1) uvA[1] = uvA[1] - int(uvA[1]);
      if (uvB[0] > 1) uvB[0] = uvB[0] - int(uvB[0]);
      if (uvB[1] > 1) uvB[1] = uvB[1] - int(uvB[1]);
      if (uvC[0] > 1) uvC[0] = uvC[0] - int(uvC[0]);
      if (uvC[1] > 1) uvC[1] = uvC[1] - int(uvC[1]);

      // Check that texcoords are within bounds
      if (!(uvA[0] >= 0 && uvA[1] <= 1 &&
            uvB[0] >= 0 && uvB[1] <= 1 &&
            uvC[0] >= 0 && uvC[1] <= 1))
      {
        cout << "A: " << uvA << endl;
        cout << "B: " << uvB << endl;
        cout << "C: " << uvC << endl;

        printf("Texcoords out of bounds\n");
        throw;
      }

      // Test: maybe flip Y value of UV
      uvA[1] = 1 - uvA[1];
      uvB[1] = 1 - uvB[1];
      uvC[1] = 1 - uvC[1];

      // Set vertex UVs
      tmp->uv_verts = true;
      tmp->uvA = uvA;
      tmp->uvB = uvB;
      tmp->uvC = uvC;
      tmp->tex_frame = texture_frames.size()-1;
      tmp->texture = true;

      // Set roughness based on roughness map
      // TODO: FIGURE OUT HOW TO ASSIGN THIS -- IS IT INTERPOLATED ACROSS VERTICES?
      // Just assign average roughness across texcoords for now
      float r1 = frame_data[int(texcoords[t_inds[i][0]][0] * int(width-1) + texcoords[t_inds[i][0]][1] * int(height-1) * (width-1))];
      float r2 = frame_data[int(texcoords[t_inds[i][1]][0] * int(width-1) + texcoords[t_inds[i][1]][1] * int(height-1) * (width-1))];
      float r3 = frame_data[int(texcoords[t_inds[i][2]][0] * int(width-1) + texcoords[t_inds[i][2]][1] * int(height-1) * (width-1))];
      tmp->reflect_params.roughness = (r1+r2+r3)/(3 * 255);
      shapes.push_back(make_shared<Triangle>(*tmp));
    }
    stbi_image_free(frame_data);

    vertices.clear();
    v_inds.clear();
    texcoords.clear();
    t_inds.clear();
    normals.clear();
    n_inds.clear();

    loadObj("./models/helios_statue/helios_20.obj", "./models/helios_statue", vertices, v_inds, texcoords, t_inds, normals, n_inds);
    printf("Helios bust ============\n# of vertices: %d, # of texcoords: %d, # of triangles: %d, # of normals: %d\n",
              int(vertices.size()), int(texcoords.size()), int(v_inds.size()), int(normals.size()));
    // Rotate helios head 180 to face down negative z-axis + standard transform (3x scale)
    MATRIX4 bust_transf; bust_transf << 3 * cos(M_PI), 0, 3 * sin(M_PI), 20,
                                        0, 3, 0, 6.5,
                                        -3 * sin(M_PI), 0, 3 * cos(M_PI), 20,
                                        0, 0, 0, 1;
    vector<VEC3> hvert1;
    for (int i = 0; i < vertices.size(); i++)
    {
      VEC4 tmp; tmp << vertices[i], 1;
      VEC4 tmp2 = bust_transf * tmp;
      hvert1.push_back(tmp2.head<3>());
    }

    // Load triangles: FIRST TWO ARE ROOT
    // Keep track of shape bounds
    for (int i = 2; i < v_inds.size(); i++)
    {
      VEC3I v_ind = v_inds[i];
      // Increase shape size by 3
      VEC3 tmp_center = (hvert1[v_ind[0]] + hvert1[v_ind[1]] + hvert1[v_ind[2]])/3;

      shared_ptr<Triangle> tmp = make_shared<Triangle>(hvert1[v_ind[0]],
                                            hvert1[v_ind[1]],
                                            hvert1[v_ind[2]],
                                            VEC3(0.75, 0.75, 0.75), "marble", false, "oren-nayar");
      tmp->reflect_params.roughness = 0.5;
      shapes.push_back(make_shared<Triangle>(*tmp));
    }

    // GLASS HELIOS: BOLD
    // SAVE NORMALS
    bust_transf << 3 * cos(M_PI), 0, 3 * sin(M_PI), 28,
                    0, 3, 0, 4,
                    -3 * sin(M_PI), 0, 3 * cos(M_PI), 20,
                    0, 0, 0, 1;
    vector<VEC3> hvert2;
    for (int i = 0; i < vertices.size(); i++)
    {
      VEC4 tmp; tmp << vertices[i], 1;
      VEC4 tmp2 = bust_transf * tmp;
      hvert2.push_back(tmp2.head<3>());
    }

    // Load triangles: FIRST TWO ARE ROOT
    for (int i = 2; i < v_inds.size(); i++)
    {
      VEC3I v_ind = v_inds[i];
      // Increase shape size by 3
      VEC3 tmp_center = (hvert2[v_ind[0]] + hvert2[v_ind[1]] + hvert2[v_ind[2]])/3;

      shared_ptr<Triangle> tmp = make_shared<Triangle>(hvert2[v_ind[0]],
                                            hvert2[v_ind[1]],
                                            hvert2[v_ind[2]],
                                            VEC3(0, 0, 0), "glass", false);
      tmp->mesh = true;
      tmp->mesh_normal = normals[n_inds[i][0]]; // We only need the normal of one vertex
      tmp->reflect_params.roughness = 0.5;
      shapes.push_back(make_shared<Triangle>(*tmp));
    }

    // Light: spherelight in center of room
    shared_ptr<sphereLight> pinklight = make_shared<sphereLight>(VEC3(40, 7, 20), 2, pastelpink);
    lights.push_back(pinklight);
    shapes.push_back(pinklight);
  }
  else if (frame == 5) // Floor + helios model + pedestal (just 1 in center for now)
  {
    // TODO: make sure lookingAt set to model!
    // TODO: first load pedestal and check boundaries
  }
}

void buildSceneCheckerTexture(float frame)
{
  shapes.clear();
  lights.clear();

  float min_y = 0.301897;
  float xmin = -0.1;
  float xmax = 1.7;
  float zmin = -0.4;
  float zmax = 1.9;

  VEC3 globalA(-10, min_y, -10);
  VEC3 globalB(-10, min_y, 30);
  VEC3 globalC(70, min_y, 30);
  VEC3 globalD(70, min_y, -10);
  VEC3 globalE(-10, 15, -10);
  VEC3 globalF(-10, 15, 30);
  VEC3 globalG(70, 15, 30);
  VEC3 globalH(70, 15, -10);

  VEC3 globalEH = (globalH-globalE).normalized();
  VEC3 globalEF = (globalF-globalE).normalized();
  VEC3 globalAD = (globalA-globalD).normalized();
  VEC3 globalCD = (globalC - globalD).normalized();

  // Trapdoor
  VEC3 A(xmin, min_y, zmax);
  VEC3 B((xmin+xmax)/2, min_y, zmax);
  VEC3 C((xmin+xmax)/2, min_y, zmin);
  VEC3 D(xmin, min_y, zmin);
  VEC3 E(xmax, min_y, zmax);
  VEC3 F(xmax, min_y, zmin);

  VEC3 y(0, 0.5, 0);

  eye = VEC3(0.9, 8, 2);
  lookingAt = VEC3(0.9,0,0.5);

  // Trapdoor
  loadTexture("./textures/sad_finder1_adj.jpg");
  int tex_index = texture_frames.size()-1;
  shared_ptr<RectPrismV2> tmp = make_shared<RectPrismV2>(A, B, C, D, A-y, B-y, C-y, D-y, VEC3(0,0,0),
                                                        "steel", false, tex_index, "cook-torrance");
  tmp->reflect_params.roughness = 0.7;
  tmp->reflect_params.refr = VEC2(2.75, 3.79);
  tmp->reflect_params.glossy = true;
  tmp->texture = true;
  shapes.push_back(make_shared<RectPrismV2>(*tmp));

  // Rotate just the right door
  float angle = M_PI/4;
  VEC3 B_right = rotate(B-E, F-E, -angle) + E;
  VEC3 C_right = rotate(C-E, F-E, -angle) + E;
  VEC3 E_right = rotate(B-y-E, F-E, -angle) + E;
  VEC3 F_right = rotate(E-E-y, F-E, -angle) + E;
  VEC3 G_right = rotate(F-E-y, F-E, -angle) + E;
  VEC3 H_right = rotate(C-E-y, F-E, -angle) + E;

  loadTexture("./textures/sad_finder2_adj.jpg");
  tex_index = texture_frames.size()-1;
  shared_ptr<RectPrismV2> tmp2 = make_shared<RectPrismV2>(B_right,E,F,C_right,E_right, F_right, G_right, H_right, VEC3(0,0,0),
                                            "steel", false, tex_index, "cook-torrance");
  tmp2->reflect_params.roughness = 0.7;
  tmp2->reflect_params.refr = VEC2(2.75, 3.79);
  tmp2->reflect_params.glossy = true;
  tmp2->texture = true;
  shapes.push_back(make_shared<RectPrismV2>(*tmp2));

  // loadTexture("./textures/sad_finder1_adj.jpg");
  // int tex_index = texture_frames.size()-1;
  // shared_ptr<Rectangle> tmp = make_shared<Rectangle>(A, B, C, D, VEC3(0,0,0),
  //                                                       "steel", false, tex_index, "cook-torrance");
  // tmp->reflect_params.roughness = 0.7;
  // tmp->reflect_params.refr = VEC2(2.75, 3.79);
  // tmp->reflect_params.glossy = true;
  // tmp->texture = true;
  // shapes.push_back(make_shared<Rectangle>(*tmp));
  //
  // loadTexture("./textures/sad_finder2_adj.jpg");
  // tex_index = texture_frames.size()-1;
  // shared_ptr<Rectangle> tmp2 = make_shared<Rectangle>(B,E,F,C, VEC3(0,0,0),
  //                                           "steel", false, tex_index, "cook-torrance");
  // tmp2->reflect_params.roughness = 0.7;
  // tmp2->reflect_params.refr = VEC2(2.75, 3.79);
  // tmp2->reflect_params.glossy = true;
  // tmp2->texture = true;
  // shapes.push_back(make_shared<Rectangle>(*tmp2));

  loadTexture("./textures/floor.jpeg");
  tex_index = texture_frames.size()-1;
  float s = 1;
  VEC3 checker_rgb0(0.58, 0.82, 1);
  VEC3 checker_rgb1(1, 0.416, 0.835);
  shared_ptr<CheckerboardWithHole> floor = make_shared<CheckerboardWithHole>(globalA,
                                              globalB,
                                              globalC,
                                              globalD,
                                            checker_rgb0, checker_rgb1, s,
                                          make_shared<Rectangle>(A, E, F, D, VEC3(0,0,0)));
  floor->tex_frame = tex_index;
  floor->borderwidth = 0.05;
  floor->bordercolor = VEC3(0.33, 0.33, 0.33);
  floor->texture = true;
  floor->reflect_params.material = "linoleum";
  floor->reflect_params.roughness = 0.6;
  floor->reflect_params.refr = VEC2(1.543,0);
  floor->reflect_params.glossy = false;
  shapes.push_back(make_shared<CheckerboardWithHole>(*floor));

  shapes.push_back(make_shared<Sphere>(A + VEC3(0,2,0), 0.5, VEC3(1,0,0))); // Sphere to test reflection
  lights.push_back(make_shared<pointLight>(eye, VEC3(1,1,1)));
}

// Prism wall with cylinder hole
// void buildScenePrismWall(float frame)
// {
//   shapes.clear();
//   lights.clear();
//
//   eye = VEC3(-6, 1.5, 0);
//   lookingAt = VEC3(0, 1.3, 0);
//
//   VEC3 col(1,0,0);
//   VEC3 a(0, 0, -3);
//   VEC3 b(0, 0, 3);
//   VEC3 c(0, 3, 3);
//   VEC3 d(0, 3, -3);
//   VEC3 e(1, 0, -3);
//   VEC3 f(1, 0, 3);
//   VEC3 g(1, 3, 3);
//   VEC3 h(1, 3, -3);
//   shared_ptr<RectPrismV2WithHoles> tmp = make_shared<RectPrismV2WithHoles>(a,b,c,d,e,f,g,h,col);
//   tmp->holes.push_back(make_shared<Cylinder>(VEC3(0, 1.5, 0), VEC3(1,1.5,0), 1, VEC3(0,1,0)));
//   shapes.push_back(make_shared<RectPrismV2WithHoles>(*tmp));
//   lights.push_back(make_shared<pointLight>(eye, VEC3(1,1,1)));
// }


void buildScenePrism(float frame)
{
  shapes.clear();
  lights.clear();
  float min_y = 0.301897;
  eye = VEC3(-10, -2, 0.9);
  frame_prism = 0;

  // Rotate eye
  float theta = frame/120 * M_PI;
  eye = rotate(eye, VEC3(0,1,0), theta);

  float xmin = -0.1;
  float xmax = 1.7;
  float zmin = -0.2;
  float zmax = 1.9;

  VEC3 a_cap0((xmin+xmax)/2, min_y, zmin);
  VEC3 b_cap0(xmin, min_y, zmax);
  VEC3 c_cap0(xmax, min_y, zmax);

  a_cap0 += VEC3(0, (frame) * move_per_frame, 0);
  b_cap0 += VEC3(0, (frame) * move_per_frame, 0);
  c_cap0 += VEC3(0, (frame) * move_per_frame, 0);
  shapes.push_back(make_shared<Triangle>(a_cap0, b_cap0, c_cap0, VEC3(0, 0, 1)));

  // TEST: Smaller triangle prism
  VEC3 a_cap1 = a_cap0 - VEC3(0, 5, 0);
  VEC3 b_cap1 = b_cap0 - VEC3(0, 5, 0);
  VEC3 c_cap1 = c_cap0 - VEC3(0, 5, 0);

  generateTrianglePrismMesh(a_cap0, b_cap0, c_cap0, a_cap1, b_cap1, c_cap1, int(frame), true);
  lights.push_back(make_shared<pointLight>(eye, VEC3(1,1,1)));
  lights.push_back(make_shared<pointLight>(VEC3(0, 10, 1), VEC3(1,1,1)));
}

// Test prism wall with cylinder hole inside
void BuildScenePrismCylinder(float frame)
{
  shapes.clear();
  lights.clear();

  VEC3 A(0, -2, -2);
  VEC3 B(0, -2, 2);
  VEC3 C(0, 2, 2);
  VEC3 D(0, 2, -2);
  VEC3 back(1, 0, 0);
  shared_ptr<RectPrismWithCylinder> tmp = make_shared<RectPrismWithCylinder>(A, B, C, D, A+back, B+back, C+back, D+back,
                                                                              VEC3(1,0,0));
  VEC3 c1 = (A+B+C+D)/4;
  VEC3 c2 = c1 + back;
  tmp->holes.push_back(make_shared<Cylinder>(c1, c2, 1, VEC3(0,0,1)));
  shapes.push_back(make_shared<RectPrismWithCylinder>(*tmp));
  lights.push_back(make_shared<pointLight>(VEC3(-5, 1, 0), VEC3(1,1,1)));

  // Rotate eye
  VEC4 og_eye(-6, 0.5,1,1);
  eye = og_eye.head<3>();
  float theta = M_PI * 2 * frame/50;
  MATRIX4 to_origin; to_origin << 1, 0, 0, -og_eye[0],
                                  0, 1, 0, -og_eye[1],
                                  0, 0, 1, -og_eye[2],
                                  0, 0, 0, 1;
  MATRIX4 rot; rot << cos(theta), 0, sin(theta), 0,
                      0, 1, 0, 0,
                      -sin(theta), 0, cos(theta), 0,
                      0, 0, 0, 1;
  MATRIX4 from_origin; from_origin << 1, 0, 0, og_eye[0],
                                      0, 1, 0, og_eye[1],
                                      0, 0, 1, og_eye[2],
                                      0, 0, 0, 1;
  VEC4 trans_eye = from_origin * rot * to_origin * og_eye;
  eye = trans_eye.head<3>();
}

// Adapted: for testing rectangle prism
void BuildSceneRectangleTexture(float frame)
{
  shapes.clear();
  lights.clear();

  VEC3 A(0, -1, -1);
  VEC3 B(0, -1, 0);
  VEC3 C(0, 1, 0);
  VEC3 D(0, 1, -1);
  VEC3 back(1, 0, 0);
  loadTexture("./textures/sad_finder1_adj.jpg");
  int tex_index = texture_frames.size()-1;
  shared_ptr<RectPrismV2> tmp = make_shared<RectPrismV2>(A, B, C, D, A+back, B+back, C+back, D+back,
                                          VEC3(0,0,0), "steel", false, tex_index, "cook-torrance");
  tmp->reflect_params.roughness = 0.7;
  tmp->reflect_params.refr = VEC2(2.75, 3.79);
  tmp->texture = true;
  tmp->reflect_params.glossy = true;

  VEC3 E(0, -1, 1);
  VEC3 F(0, 1, 1);
  loadTexture("./textures/sad_finder2_adj.jpg");
  tex_index = texture_frames.size()-1;
  shared_ptr<RectPrismV2> tmp2 = make_shared<RectPrismV2>(B,E,F,C, B+back, E+back, F+back, C+back,
                                                  VEC3(0,0,0), "steel", false, tex_index, "cook-torrance");
  tmp2->reflect_params.roughness = 0.7;
  tmp2->reflect_params.refr = VEC2(2.75, 3.79);
  tmp2->texture = true;
  tmp2->reflect_params.glossy = true;

  shapes.push_back(make_shared<RectPrismV2>(*tmp2));
  shapes.push_back(make_shared<RectPrismV2>(*tmp));
  shapes.push_back(make_shared<Sphere>(VEC3(-7, 0, 2), 1, VEC3(1,0,0)));
  lights.push_back(make_shared<pointLight>(eye, VEC3(1,1,1)));

  // For testing: write back out PPM using read in texture values
  // vector<VEC3> texture = texture_frames.back();
  // VEC2 dim = texture_dims.back();
  // const int tot = int(dim[0]) * int(dim[1]);
  // float* ppmOut = new float[3 * tot];
  // printf("Texture size: %d, ppm size: %d\n", texture.size(), tot);
  // for (int i = 0; i < texture.size(); i++)
  // {
  //   ppmOut[3*i] = texture[i][0] * 255.0f;
  //   ppmOut[3*i+1] = texture[i][1] * 255.0f;
  //   ppmOut[3*i+2] = texture[i][2] * 255.0f;
  // }
  // char buffer[256];
  // sprintf(buffer, "./test_frames/texture/texture.ppm");
  // int width = dim[0];
  // int height = dim[1];
  // writePPM(buffer, width, height, ppmOut);
  // delete[] ppmOut;

}

void BuildSceneRectangleTextureOG(float frame)
{
  shapes.clear();
  lights.clear();

  VEC3 A(0, -1, -1);
  VEC3 B(0, -1, 0);
  VEC3 C(0, 1, 0);
  VEC3 D(0, 1, -1);
  loadTexture("./textures/sad_finder1_adj.jpg");
  int tex_index = texture_frames.size()-1;
  shared_ptr<Rectangle> tmp = make_shared<Rectangle>(A, B, C, D,
                                          VEC3(0,0,0), "steel", false, tex_index, "cook-torrance");
  tmp->reflect_params.roughness = 0.7;
  tmp->reflect_params.refr = VEC2(2.75, 3.79);
  tmp->texture = true;
  tmp->reflect_params.glossy = true;

  VEC3 E(0, -1, 1);
  VEC3 F(0, 1, 1);
  loadTexture("./textures/sad_finder2_adj.jpg");
  tex_index = texture_frames.size()-1;
  shared_ptr<Rectangle> tmp2 = make_shared<Rectangle>(B,E,F,C,
                                                  VEC3(0,0,0), "steel", false, tex_index, "cook-torrance");
  tmp2->reflect_params.roughness = 0.7;
  tmp2->reflect_params.refr = VEC2(2.75, 3.79);
  tmp2->texture = true;
  tmp2->reflect_params.glossy = true;

  shapes.push_back(make_shared<Rectangle>(*tmp2));
  shapes.push_back(make_shared<Rectangle>(*tmp));
  shapes.push_back(make_shared<Sphere>(VEC3(-7, 0, 2), 1, VEC3(1,0,0)));
  lights.push_back(make_shared<pointLight>(eye, VEC3(1,1,1)));
}

// Alternative wall: just stack four prisms lol
void buildAggWall(float frame)
{
  shapes.clear();
  lights.clear();
  perlin_cloud = false;

  // Testing: move eye forward and back
  eye = VEC3(-5,0.5,0);
  lookingAt = VEC3(5,0.5,0);

  VEC3 a0(3, -3, -5);
  VEC3 b0(3, -3, 5);
  VEC3 c0(5, -3, 5);
  VEC3 d0(5, -3, -5);
  VEC3 mwidth(0,0,4);
  VEC3 mheight(0,3,0);
  VEC3 holedim(0,1,0);
  VEC3 mx(2,0,0);

  VEC3 a1 = a0 + mheight;
  VEC3 b1 = b0 + mheight;
  VEC3 c1 = c0 + mheight;
  VEC3 d1 = d0 + mheight;
  VEC3 am1 = a1 + mwidth;
  VEC3 bm1 = b1 - mwidth;
  VEC3 cm1 = c1 - mwidth;
  VEC3 dm1 = d1 + mwidth;
  VEC3 a2 = a1 + holedim;
  VEC3 b2 = b1 + holedim;
  VEC3 c2 = c1 + holedim;
  VEC3 d2 = d1 + holedim;
  VEC3 am2 = am1 + holedim;
  VEC3 bm2 = bm1 + holedim;
  VEC3 cm2 = cm1 + holedim;
  VEC3 dm2 = dm1 + holedim;
  VEC3 a3 = a2 + mheight;
  VEC3 b3 = b2 + mheight;
  VEC3 c3 = c2 + mheight;
  VEC3 d3 = d2 + mheight;

  shapes.push_back(make_shared<RectPrismV2>(a0, b0, c0, d0,
                  a1, b1, c1, d1, VEC3(1,0,0)));
  shapes.push_back(make_shared<RectPrismV2>(a1-VEC3(0,0.01,0), am1-VEC3(0,0.01,0), dm1-VEC3(0,0.01,0), d1-VEC3(0,0.01,0),
          a2 +VEC3(0,0.01,0), am2+VEC3(0,0.01,0), dm2+VEC3(0,0.01,0), d2+VEC3(0,0.01,0), VEC3(1,0,0)));
  shapes.push_back(make_shared<RectPrismV2>(bm1, b1, c1, cm1, bm2, b2, c2, cm2, VEC3(1,0,0)));
  shapes.push_back(make_shared<RectPrismV2>(a2, b2, c2, d2, a3, b3, c3, d3, VEC3(1,0,0)));

  // Lights todo: one outside and one IN the hole
    lights.push_back(make_shared<pointLight>(VEC3(-6, 1, 1), VEC3(1,1,1)));
  // lights.push_back(make_shared<pointLight>(VEC3(4, 0.5, 0), VEC3(1,1,1)));
}

void buildSphereLightTest(float frame)
{
  // Sphere light testing: point light at sphere light with wall behind it
  shapes.clear();
  lights.clear();

  eye = VEC3(-6, 5, 0);
  // lights.push_back(make_shared<pointLight>(VEC3(-6, 0, 0), VEC3(1,1,1)));

  shared_ptr<sphereLight> ball = make_shared<sphereLight>(VEC3(0, 0, 0), 2, VEC3(1, 1, 1));
  lights.push_back(ball);
  shapes.push_back(ball);

  VEC3 A(5, -5, -10);
  VEC3 length(0,0,20);
  VEC3 width(2,0,0);
  VEC3 height(0,10,0);
  // shapes.push_back(make_shared<RectPrismV2>(A, A + length, A + length + height, A + height,
  //               A + width, A + length + width, A + length + width + height, A + width + height, VEC3(0.1,0.1,1)));
  shapes.push_back(make_shared<Rectangle>(A, A + length, A + length + height, A + height, VEC3(0.1,0.1,1)));
}

void buildRectPrismV2Test(float frame)
{
  shapes.clear();
  lights.clear();

  eye = VEC3(-6, 3, 0);

  // Linear interpolate over time
  VEC3 dest(-6, 0, 2);
  eye = eye + (dest - eye) * frame/20;
  float l_prism = 1;
  float w_prism = 0.5;
  float h_prism = 0.5;

  VEC3 A(0, 0, -1);
  VEC3 B(0, 0, 1);
  VEC3 C(0.5, 0, 1);
  VEC3 D(0.5, 0, -1);
  VEC3 E(0, 1, -1);
  VEC3 F(0, 1, 1);
  VEC3 G(0.5, 1, 1);
  VEC3 H(0.5, 1, -1);

  // shapes.push_back(make_shared<RectPrismV2>(A, B, C, D, E, F, G,
  //                                         H, VEC3(1,0,0)));

  // Can't handle rotations...
  MATRIX3 rot; rot << cos(M_PI/4), 0, sin(M_PI/4), 0,1,0, -sin(M_PI/4), 0, cos(M_PI/4);

  shapes.push_back(make_shared<RectPrismV2>(rot*A, rot*B, rot*C, rot*D, rot*E, rot*F, rot*G,
                                          rot*H, VEC3(0,1,0)));

  lights.push_back(make_shared<pointLight>(eye, VEC3(1,1,1)));
  // lights.push_back(make_shared<pointLight>(VEC3(1, 10, 0), VEC3(1,1,1)));
}

void buildStaircaseTest(float frame)
{
  shapes.clear();
  lights.clear();

  // Rotate eye
  VEC4 og_eye(-6, 0.5,1, 1);
  eye = og_eye.head<3>();
  float alpha = M_PI * 2 * frame/50.0;
  MATRIX4 to_origin; to_origin << 1, 0, 0, -og_eye[0],
                                  0, 1, 0, -og_eye[1],
                                  0, 0, 1, -og_eye[2],
                                  0, 0, 0, 1;
  MATRIX4 rot; rot << cos(alpha), 0, sin(alpha), 0,
                      0, 1, 0, 0,
                      -sin(alpha), 0, cos(alpha), 0,
                      0, 0, 0, 1;
  MATRIX4 from_origin; from_origin << 1, 0, 0, og_eye[0],
                                      0, 1, 0, og_eye[1],
                                      0, 0, 1, og_eye[2],
                                      0, 0, 0, 1;
  VEC4 trans_eye = from_origin * rot * to_origin * og_eye;
  eye = trans_eye.head<3>();

  float s = 1;
  VEC3 c2(2, 10, -1);
  VEC3 c1(2, 0, -1);
  float r = 2;

  // #### Staircase
  // TODO: Figure out the right staircase prism dimensions
  float l_prism = 1;
  float w_prism = 0.5;
  float h_prism = 0.5;

  // Corner cylinder: normal
  shapes.push_back(make_shared<Cylinder>(c1, c2, r, VEC3(0.5, 0.5, 0.5)));

  // Staircase: align length vector to x-axis and work from there
  VEC3 length_vector(-1, 0, 0);
  VEC3 width_vector(0, 0, 1);
  VEC3 height_vector(0,1,0);

  // D is ALWAYS touching the cylinder
  VEC3 stair_d = c1 + length_vector * r;
  VEC3 stair_a = stair_d - width_vector * w_prism;
  VEC3 stair_b = stair_a + length_vector * l_prism;
  VEC3 stair_c = stair_b + width_vector * w_prism;
  VEC3 stair_e = stair_a + height_vector * h_prism;
  VEC3 stair_f = stair_b + height_vector * h_prism;
  VEC3 stair_g = stair_c + height_vector * h_prism;
  VEC3 stair_h = stair_d + height_vector * h_prism;

  // Just render 3 rotations for now
  VEC3 tmp_center = c1;
  float theta = acos(1 - pow(w_prism, 2)/pow((stair_a - tmp_center).norm(), 2));
  for (int i = 0; i < 6; i++)
  {
    shapes.push_back(make_shared<RectPrismV2>(stair_a, stair_b, stair_c, stair_d, stair_e, stair_f, stair_g,
                                            stair_h, pink));
    // Rotation formula (this is guaranteed by the fact that D touches the surface of the cylinder!)
    // theta = acos(1 - w^2/(A-center).normsq)
    stair_a = rotate(stair_a - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;
    stair_b = rotate(stair_b - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;
    stair_c = rotate(stair_c - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;
    stair_d = rotate(stair_d - tmp_center, height_vector, theta) + tmp_center + height_vector * h_prism;

    stair_e = stair_a + height_vector * h_prism;
    stair_f = stair_b + height_vector * h_prism;
    stair_g = stair_c + height_vector * h_prism;
    stair_h = stair_d + height_vector * h_prism;

    tmp_center += height_vector;

    // Recompute new length and width vectors
    width_vector = (stair_d - stair_a).normalized();
    length_vector = (stair_b - stair_a).normalized();

    // Compute new theta
    theta = acos(1 - pow(w_prism, 2)/pow((stair_a - tmp_center).norm(), 2));
  }

  // Light: at eye
  lights.push_back(make_shared<pointLight>(VEC3(-5, 10, -2), VEC3(1,1,1)));
  lights.push_back(make_shared<pointLight>(VEC3(0, 8, 5), VEC3(1,1,1)));
  lights.push_back(make_shared<pointLight>(eye, VEC3(1,1,1)));
}

// Checkpoint 2 scene
void buildSceneChkpt2(float frame)
{
  // TODO: Motion blur skeleton????
  setSkeletonsToSpecifiedFrame(int(frame));

  shapes.clear();
  lights.clear();

  displayer.ComputeBonePositions(DisplaySkeleton::BONES_AND_LOCAL_FRAMES);

  // Get bone names in skeleton
  Skeleton* skel = displayer.GetSkeleton(0);

  // retrieve all the bones of the skeleton
  vector<MATRIX4>& rotations = displayer.rotations();
  vector<MATRIX4>& scalings  = displayer.scalings();
  vector<VEC4>& translations = displayer.translations();
  vector<float>& lengths     = displayer.lengths();

  // build a sphere list, but skip the first bone,
  // it's just the origin
  float min_y = 0.251897; // MIN Y FROM STANDING POSITION
  VEC3 rfoot_lv;
  VEC3 rfoot_rv;
  VEC3 lfoot_lv;
  VEC3 lfoot_rv;
  int totalBones = rotations.size();
  for (int x = 1; x < totalBones; x++)
  {
    MATRIX4& rotation = rotations[x];
    MATRIX4& scaling = scalings[x];
    VEC4& translation = translations[x];

    // get the endpoints of the cylinder
    VEC4 leftVertex(0,0,0,1);
    VEC4 rightVertex(0,0,lengths[x],1);

    leftVertex = rotation * scaling * leftVertex + translation;
    rightVertex = rotation * scaling * rightVertex + translation;

    Bone tmp = skel->getBone(x);
    string name(tmp.name);
    if (name == "rfoot" || name == "rtoes")
    {
      rfoot_lv = leftVertex.head<3>();
      rfoot_rv = rightVertex.head<3>();
    }
    else if (name == "lfoot" || name == "ltoes")
    {
      lfoot_lv = leftVertex.head<3>();
      lfoot_rv = rightVertex.head<3>();
    }

    // Cylinders
    shapes.push_back(make_shared<Cylinder>(leftVertex.head<3>(), rightVertex.head<3>(), 0.05, VEC3(1,0,0)));
  }
  // Adjust min_y manually
  min_y = min_y + 0.05;

  // Easier: linear movement of eye
  VEC3 eye_start(-6, 0.5, 1);
  VEC3 eye_end(0.49, 10, 1);
  float tmp = frame/320;
  if (tmp > 1)
  {
    eye = eye_end;
  }
  else
  {
    eye = eye_start + (eye_end - eye_start) * tmp;
  }

  // Floor: checkerboard (vaporwave colors)
  // 1 x 1 squares
  float s = 1;
  VEC3 checker_rgb0(0.58, 0.82, 1);
  VEC3 checker_rgb1(1, 0.416, 0.835);
  VEC3 a(-6, min_y, -6);
  shapes.push_back(make_shared<Checkerboard>(VEC3(-6, min_y, -6), VEC3(-6, min_y, 6), VEC3(6, min_y, 6), VEC3(6, min_y, -6),
                                            checker_rgb0, checker_rgb1, s));


  // Lights: rotate around Z = 1, X = 0 axis
  float theta = M_PI * 2 * frame/480;
  MATRIX4 to_origin; to_origin << 1, 0, 0, 0,
                                  0, 1, 0, 0,
                                  0, 0, 1, -1,
                                  0, 0, 0, 1;
  MATRIX4 rot; rot << cos(theta), 0, sin(theta), 0,
                      0, 1, 0, 0,
                      -sin(theta), 0, cos(theta), 0,
                      0, 0, 0, 1;
  MATRIX4 from_origin; from_origin << 1, 0, 0, 0,
                                      0, 1, 0, 0,
                                      0, 0, 1, 1,
                                      0, 0, 0, 1;

  VEC3 flux_color(1, 0.756, 0.518);
  VEC4 halogen_center(-3, 1.5, 2, 1);
  halogen_center = from_origin * rot * to_origin * halogen_center;
  shared_ptr<sphereLight> halogen_ball = make_shared<sphereLight>(halogen_center.head<3>(), 0.3, flux_color);
  lights.push_back(halogen_ball);
  shapes.push_back(halogen_ball);

  VEC4 orange_center(2, 1, -1, 1);
  orange_center = from_origin * rot * to_origin * orange_center;
  shared_ptr<sphereLight> orange_ball = make_shared<sphereLight>(orange_center.head<3>(), 0.3, VEC3(1, 0.65, 0));
  lights.push_back(orange_ball);
  shapes.push_back(orange_ball);
}

void buildSceneReflectance(int frame)
{
  shapes.clear();
  lights.clear();

  // Three spheres: lambertian, oren-nayar, cook-torrance
  shapes.push_back(make_shared<Sphere>(VEC3(3, 0.5, -4), 1, VEC3(0.5, 0.5, 0.5)));
  shared_ptr<Sphere> marble_sphere = make_shared<Sphere>(VEC3(3, 0.5, -1.5), 1, VEC3(0.5, 0.5, 0.5), "marble", false, "oren-nayar");
  marble_sphere->reflect_params.roughness = sqrt(0.2);
  shapes.push_back(make_shared<Sphere>(*marble_sphere));
  shared_ptr<Sphere> metal_sphere = make_shared<Sphere>(VEC3(3, 0.5, 1), 1, VEC3(0.5, 0.5, 0.5), "aluminum", false, "cook-torrance");
  metal_sphere->reflect_params.roughness = sqrt(0.2);
  metal_sphere->reflect_params.refr = VEC2(0.958, 6.69); // Aluminum = 0.958 + 6.69i
  shapes.push_back(make_shared<Sphere>(*metal_sphere));
  shared_ptr<Sphere> glossy_sphere = make_shared<Sphere>(VEC3(3, 0.5, 3.5), 1, VEC3(0.5, 0.5, 0.5), "aluminum", false, "cook-torrance");
  glossy_sphere->reflect_params.roughness = sqrt(0.2);
  glossy_sphere->reflect_params.refr = VEC2(0.958, 6.69); // Aluminum = 0.958 + 6.69i
  glossy_sphere->reflect_params.glossy = true;
  shapes.push_back(make_shared<Sphere>(*glossy_sphere));

  // Smaller sphere in front of metal to test reflectance
  shapes.push_back(make_shared<Sphere>(VEC3(-7, 0.5, 4), 3, VEC3(1, 0, 0)));

  // Pass light over objects (5 second video)
  VEC3 light_center = VEC3(-6, 5, -10) + VEC3(0, 0, 20) * (float) frame/150;
  lights.push_back(make_shared<pointLight>(light_center, VEC3(1,1,1)));
}

// Test cylinder texturing
void buildSceneCylinder(float frame)
{
  shapes.clear();
  lights.clear();

  eye = VEC3(-6, 0, 0);
  lookingAt = VEC3(0, 0, 0);

  loadTexture("./textures/floor.jpeg");
  int tex_index = texture_frames.size()-1;

  // #### Staircase
  // Corner cylinder with texture (SAME MATERIAL + TEXTURE AS FLOOR)
  float r = 5;
  VEC3 c_top(5, 7, 0);
  VEC3 c_bot(5, -7, 0);
  shared_ptr<CheckerCylinder> cyl = make_shared<CheckerCylinder>(c_top, c_bot, r, VEC3(1,1,1), 1, "linoleum");
  cyl->tex_frame = tex_index;
  cyl->borderwidth = 0.05;
  cyl->bordercolor = VEC3(0.33, 0.33, 0.33);
  cyl->texture = true;
  cyl->reflect_params.material = "linoleum";
  cyl->reflect_params.roughness = 0.6;
  cyl->reflect_params.refr = VEC2(1.543,0);
  cyl->reflect_params.glossy = true;
  shapes.push_back(make_shared<CheckerCylinder>(*cyl));
  shapes.push_back(make_shared<Sphere>(VEC3(0, 3, 3), 2, VEC3(1, 0, 0)));
  lights.push_back(make_shared<pointLight>(eye, VEC3(1,1,1)));
}

// All models combined
void buildSceneModels(float frame)
{
  // Building scene for pedestal
  shapes.clear();
  lights.clear();

  // Orientation: Eye at 20x origin and looking down POSITIVE z-axis

  // Rotate eye about lookingAt
  lookingAt = VEC3(20, 5, 20);
  VEC4 og_eye(20,5,0,1);
  eye = og_eye.head<3>();
  float theta = M_PI * 2 * frame/150;
  MATRIX4 to_origin; to_origin << 1, 0, 0, -20,
                                  0, 1, 0, -3,
                                  0, 0, 1, 0,
                                  0, 0, 0, 1;
  MATRIX4 rot; rot << cos(theta), 0, sin(theta), 0,
                      0, 1, 0, 0,
                      -sin(theta), 0, cos(theta), 0,
                      0, 0, 0, 1;
  MATRIX4 from_origin; from_origin << 1, 0, 0, 20,
                                      0, 1, 0, 3,
                                      0, 0, 1, 0,
                                      0, 0, 0, 1;
  VEC4 trans_eye = from_origin * rot * to_origin * og_eye;
  eye = trans_eye.head<3>();

  // Load model vertices
  vector<VEC3> vertices;
  vector<VEC2> texcoords;
  vector<VEC3I> v_inds;
  vector<VEC3I> t_inds;
  vector<VEC3> normals;
  vector<VEC3I> n_inds;

  loadObj("./models/Column_LP_obj/Column_LP.obj", "./models/Column_LP_obj", vertices, v_inds, texcoords, t_inds, normals, n_inds);
  printf("Pedestal ===========\n# of vertices: %d, # of texcoords: %d, # of triangles: %d\n",
            int(vertices.size()), int(texcoords.size()), int(v_inds.size()));
  // Pedestal transform: scale up by 5x, add VEC3(20, 0, 20)
  // Note: we can apply scaling matrix directly because bottom is at WORLD ORIGIN
  MATRIX4 column_transf; column_transf << 5, 0, 0, 20,
                                          0, 5, 0, 0,
                                          0, 0, 5, 20,
                                          0, 0, 0, 1;
  vector<VEC3> pverts1;
  for (int i = 0; i < vertices.size(); i++)
  {
    VEC4 tmp; tmp << vertices[i], 1;
    VEC4 tmp2 = column_transf * tmp;
    pverts1.push_back(tmp2.head<3>());
  }

  // Keep track of shape bounds
  float x_min = FLT_MAX;
  float y_min = FLT_MAX;
  float z_min = FLT_MAX;
  float x_max = FLT_MIN;
  float y_max = FLT_MIN;
  float z_max = FLT_MIN;

  // Texture data
  loadTexture("./models/Column_LP_obj/Textures/Marble_Base_Color.jpg");
  int width, height, n;
  string rough_file = "./models/Column_LP_obj/Textures/Marble_Roughness.jpg";
  unsigned char* frame_data = stbi_load(rough_file.c_str(), &width, &height, &n, 0);
  for (int i = 0; i < v_inds.size(); i++)
  {
    VEC3I v_ind = v_inds[i];
    // Increase shape size by 3
    VEC3 tmp_center = (pverts1[v_ind[0]] + pverts1[v_ind[1]] + pverts1[v_ind[2]])/3;

    shared_ptr<Triangle> tmp = make_shared<Triangle>(pverts1[v_ind[0]],
                                          pverts1[v_ind[1]],
                                          pverts1[v_ind[2]],
                                          VEC3(0.75, 0.75, 0.75), "marble", false, "oren-nayar");

    // Texture coordinates can sometimes be over 1: repeat texture in that case
    VEC2 uvA = texcoords[t_inds[i][0]];
    VEC2 uvB = texcoords[t_inds[i][1]];
    VEC2 uvC = texcoords[t_inds[i][2]];
    if (uvA[0] > 1) uvA[0] = uvA[0] - int(uvA[0]);
    if (uvA[1] > 1) uvA[1] = uvA[1] - int(uvA[1]);
    if (uvB[0] > 1) uvB[0] = uvB[0] - int(uvB[0]);
    if (uvB[1] > 1) uvB[1] = uvB[1] - int(uvB[1]);
    if (uvC[0] > 1) uvC[0] = uvC[0] - int(uvC[0]);
    if (uvC[1] > 1) uvC[1] = uvC[1] - int(uvC[1]);

    // Check that texcoords are within bounds
    if (!(uvA[0] >= 0 && uvA[1] <= 1 &&
          uvB[0] >= 0 && uvB[1] <= 1 &&
          uvC[0] >= 0 && uvC[1] <= 1))
    {
      cout << "A: " << uvA << endl;
      cout << "B: " << uvB << endl;
      cout << "C: " << uvC << endl;

      printf("Texcoords out of bounds\n");
      throw;
    }

    // Test: maybe flip Y value of UV
    uvA[1] = 1 - uvA[1];
    uvB[1] = 1 - uvB[1];
    uvC[1] = 1 - uvC[1];

    // Set vertex UVs
    tmp->uv_verts = true;
    tmp->uvA = uvA;
    tmp->uvB = uvB;
    tmp->uvC = uvC;
    tmp->tex_frame = texture_frames.size()-1;
    tmp->texture = true;

    // Set roughness based on roughness map
    // TODO: FIGURE OUT HOW TO ASSIGN THIS -- IS IT INTERPOLATED ACROSS VERTICES?
    // Just assign average roughness across texcoords for now
    float r1 = frame_data[int(texcoords[t_inds[i][0]][0] * int(width-1) + texcoords[t_inds[i][0]][1] * int(height-1) * (width-1))];
    float r2 = frame_data[int(texcoords[t_inds[i][1]][0] * int(width-1) + texcoords[t_inds[i][1]][1] * int(height-1) * (width-1))];
    float r3 = frame_data[int(texcoords[t_inds[i][2]][0] * int(width-1) + texcoords[t_inds[i][2]][1] * int(height-1) * (width-1))];
    tmp->reflect_params.roughness = (r1+r2+r3)/(3 * 255);

    shapes.push_back(make_shared<Triangle>(*tmp));
    x_min = min({x_min, float(vertices[v_ind[0]][0]), float(vertices[v_ind[1]][0]), float(vertices[v_ind[2]][0])});
    y_min = min({y_min, float(vertices[v_ind[0]][1]), float(vertices[v_ind[1]][1]), float(vertices[v_ind[2]][1])});
    z_min = min({z_min, float(vertices[v_ind[0]][2]), float(vertices[v_ind[1]][2]), float(vertices[v_ind[2]][2])});

    x_max = max({x_max, float(vertices[v_ind[0]][0]), float(vertices[v_ind[1]][0]), float(vertices[v_ind[2]][0])});
    y_max = max({y_max, float(vertices[v_ind[0]][1]), float(vertices[v_ind[1]][1]), float(vertices[v_ind[2]][1])});
    z_max = max({z_max, float(vertices[v_ind[0]][2]), float(vertices[v_ind[1]][2]), float(vertices[v_ind[2]][2])});
  }
  // Run again for other pedestals
  // Make another column to the left of main column that will hold glass head
  column_transf << 3, 0, 0, 28,
                    0, 3, 0, 0,
                    0, 0, 3, 20,
                    0, 0, 0, 1;
  vector<VEC3> pverts2;
  for (int i = 0; i < vertices.size(); i++)
  {
    VEC4 tmp; tmp << vertices[i], 1;
    VEC4 tmp2 = column_transf * tmp;
    pverts2.push_back(tmp2.head<3>());
  }
  for (int i = 0; i < v_inds.size(); i++)
  {
    VEC3I v_ind = v_inds[i];
    // Increase shape size by 3
    VEC3 tmp_center = (pverts2[v_ind[0]] + pverts2[v_ind[1]] + pverts2[v_ind[2]])/3;

    shared_ptr<Triangle> tmp = make_shared<Triangle>(pverts2[v_ind[0]],
                                          pverts2[v_ind[1]],
                                          pverts2[v_ind[2]],
                                          VEC3(0.75, 0.75, 0.75), "marble", false, "oren-nayar");

    // Texture coordinates can sometimes be over 1: repeat texture in that case
    VEC2 uvA = texcoords[t_inds[i][0]];
    VEC2 uvB = texcoords[t_inds[i][1]];
    VEC2 uvC = texcoords[t_inds[i][2]];
    if (uvA[0] > 1) uvA[0] = uvA[0] - int(uvA[0]);
    if (uvA[1] > 1) uvA[1] = uvA[1] - int(uvA[1]);
    if (uvB[0] > 1) uvB[0] = uvB[0] - int(uvB[0]);
    if (uvB[1] > 1) uvB[1] = uvB[1] - int(uvB[1]);
    if (uvC[0] > 1) uvC[0] = uvC[0] - int(uvC[0]);
    if (uvC[1] > 1) uvC[1] = uvC[1] - int(uvC[1]);

    // Check that texcoords are within bounds
    if (!(uvA[0] >= 0 && uvA[1] <= 1 &&
          uvB[0] >= 0 && uvB[1] <= 1 &&
          uvC[0] >= 0 && uvC[1] <= 1))
    {
      cout << "A: " << uvA << endl;
      cout << "B: " << uvB << endl;
      cout << "C: " << uvC << endl;

      printf("Texcoords out of bounds\n");
      throw;
    }

    // Test: maybe flip Y value of UV
    uvA[1] = 1 - uvA[1];
    uvB[1] = 1 - uvB[1];
    uvC[1] = 1 - uvC[1];

    // Set vertex UVs
    tmp->uv_verts = true;
    tmp->uvA = uvA;
    tmp->uvB = uvB;
    tmp->uvC = uvC;
    tmp->tex_frame = texture_frames.size()-1;
    tmp->texture = true;

    // Set roughness based on roughness map
    // TODO: FIGURE OUT HOW TO ASSIGN THIS -- IS IT INTERPOLATED ACROSS VERTICES?
    // Just assign average roughness across texcoords for now
    float r1 = frame_data[int(texcoords[t_inds[i][0]][0] * int(width-1) + texcoords[t_inds[i][0]][1] * int(height-1) * (width-1))];
    float r2 = frame_data[int(texcoords[t_inds[i][1]][0] * int(width-1) + texcoords[t_inds[i][1]][1] * int(height-1) * (width-1))];
    float r3 = frame_data[int(texcoords[t_inds[i][2]][0] * int(width-1) + texcoords[t_inds[i][2]][1] * int(height-1) * (width-1))];
    tmp->reflect_params.roughness = (r1+r2+r3)/(3 * 255);
    shapes.push_back(make_shared<Triangle>(*tmp));
  }
  stbi_image_free(frame_data);

  vertices.clear();
  v_inds.clear();
  texcoords.clear();
  t_inds.clear();
  normals.clear();
  n_inds.clear();

  loadObj("./models/helios_statue/helios_20.obj", "./models/helios_statue", vertices, v_inds, texcoords, t_inds, normals, n_inds);
  printf("Helios bust ============\n# of vertices: %d, # of texcoords: %d, # of triangles: %d, # of normals: %d\n",
            int(vertices.size()), int(texcoords.size()), int(v_inds.size()), int(normals.size()));
  // Rotate helios head 180 to face down negative z-axis + standard transform (3x scale)
  MATRIX4 bust_transf; bust_transf << 3 * cos(M_PI), 0, 3 * sin(M_PI), 20,
                                      0, 3, 0, 6.5,
                                      -3 * sin(M_PI), 0, 3 * cos(M_PI), 20,
                                      0, 0, 0, 1;
  vector<VEC3> hvert1;
  for (int i = 0; i < vertices.size(); i++)
  {
    VEC4 tmp; tmp << vertices[i], 1;
    VEC4 tmp2 = bust_transf * tmp;
    hvert1.push_back(tmp2.head<3>());
  }

  // Load triangles: FIRST TWO ARE ROOT
  // Keep track of shape bounds
  for (int i = 2; i < v_inds.size(); i++)
  {
    VEC3I v_ind = v_inds[i];
    // Increase shape size by 3
    VEC3 tmp_center = (hvert1[v_ind[0]] + hvert1[v_ind[1]] + hvert1[v_ind[2]])/3;

    shared_ptr<Triangle> tmp = make_shared<Triangle>(hvert1[v_ind[0]],
                                          hvert1[v_ind[1]],
                                          hvert1[v_ind[2]],
                                          VEC3(0.75, 0.75, 0.75), "marble", false, "oren-nayar");
    tmp->reflect_params.roughness = 0.5;
    shapes.push_back(make_shared<Triangle>(*tmp));
  }

  // GLASS HELIOS: BOLD
  // SAVE NORMALS
  bust_transf << 3 * cos(M_PI), 0, 3 * sin(M_PI), 28,
                  0, 3, 0, 4,
                  -3 * sin(M_PI), 0, 3 * cos(M_PI), 20,
                  0, 0, 0, 1;
  vector<VEC3> hvert2;
  for (int i = 0; i < vertices.size(); i++)
  {
    VEC4 tmp; tmp << vertices[i], 1;
    VEC4 tmp2 = bust_transf * tmp;
    hvert2.push_back(tmp2.head<3>());
  }

  // Load triangles: FIRST TWO ARE ROOT
  for (int i = 2; i < v_inds.size(); i++)
  {
    VEC3I v_ind = v_inds[i];
    // Increase shape size by 3
    VEC3 tmp_center = (hvert2[v_ind[0]] + hvert2[v_ind[1]] + hvert2[v_ind[2]])/3;

    shared_ptr<Triangle> tmp = make_shared<Triangle>(hvert2[v_ind[0]],
                                          hvert2[v_ind[1]],
                                          hvert2[v_ind[2]],
                                          VEC3(0, 0, 0), "glass", false);
    tmp->mesh = true;
    tmp->mesh_normal = normals[n_inds[i][0]]; // We only need the normal of one vertex
    tmp->reflect_params.roughness = 0.5;
    shapes.push_back(make_shared<Triangle>(*tmp));
  }

  // Put wall behind objects
  shapes.push_back(make_shared<Rectangle>(VEC3(40, 0, 30), VEC3(0, 0, 30), VEC3(0, 20, 30), VEC3(40, 20, 30),
                                          VEC3(1, 0, 0)));

  // Light: at eye
  lights.push_back(make_shared<pointLight>(eye, VEC3(1,1,1)));
}

void buildScenePedestal(float frame)
{
  // Building scene for pedestal
  shapes.clear();
  lights.clear();

  // Rotate eye about origin
  eye = VEC3(-3, 0.5, 0);
  lookingAt = VEC3(0, 0.5, 0);
  VEC4 og_eye(-3, 0.5, 0, 1);
  float theta = M_PI * 2 * frame/150;
  MATRIX4 to_origin; to_origin << 1, 0, 0, -2,
                                  0, 1, 0, 0,
                                  0, 0, 1, -1,
                                  0, 0, 0, 1;
  MATRIX4 rot; rot << cos(theta), 0, sin(theta), 0,
                      0, 1, 0, 0,
                      -sin(theta), 0, cos(theta), 0,
                      0, 0, 0, 1;
  MATRIX4 from_origin; from_origin << 1, 0, 0, 2,
                                      0, 1, 0, 0,
                                      0, 0, 1, 1,
                                      0, 0, 0, 1;
  VEC4 trans_eye = rot * og_eye;
  eye = trans_eye.head<3>();

  // Load model vertices
  vector<VEC3> vertices;
  vector<VEC2> texcoords;
  vector<VEC3I> v_inds;
  vector<VEC3I> t_inds;
  vector<VEC3> normals;
  vector<VEC3I> n_inds;

  loadObj("./models/Column_LP_obj/Column_LP.obj", "./models/Column_LP_obj", vertices, v_inds, texcoords, t_inds, normals, n_inds);
  printf("# of vertices: %d, # of texcoords: %d, # of triangles: %d\n",
            int(vertices.size()), int(texcoords.size()), int(v_inds.size()));

  // TODO: FIGURE OUT ORIENTATION
  // DONE: CENTERED AT ORIGIN + BOTTOM AT 0 NICE

  // Keep track of shape bounds
  float x_min = FLT_MAX;
  float y_min = FLT_MAX;
  float z_min = FLT_MAX;
  float x_max = FLT_MIN;
  float y_max = FLT_MIN;
  float z_max = FLT_MIN;

  // Texture data
  loadTexture("./models/Column_LP_obj/Textures/Marble_Base_Color.jpg");
  int width, height, n;
  string rough_file = "./models/Column_LP_obj/Textures/Marble_Roughness.jpg";
  unsigned char* frame_data = stbi_load(rough_file.c_str(), &width, &height, &n, 0);
  for (int i = 0; i < v_inds.size(); i++)
  {
    VEC3I v_ind = v_inds[i];
    // Increase shape size by 3
    VEC3 tmp_center = (vertices[v_ind[0]] + vertices[v_ind[1]] + vertices[v_ind[2]])/3;

    shared_ptr<Triangle> tmp = make_shared<Triangle>(vertices[v_ind[0]],
                                          vertices[v_ind[1]],
                                          vertices[v_ind[2]],
                                          VEC3(0.75, 0.75, 0.75), "marble", false, "oren-nayar");

    // Texture coordinates can sometimes be over 1: repeat texture in that case
    VEC2 uvA = texcoords[t_inds[i][0]];
    VEC2 uvB = texcoords[t_inds[i][1]];
    VEC2 uvC = texcoords[t_inds[i][2]];
    if (uvA[0] > 1) uvA[0] = uvA[0] - int(uvA[0]);
    if (uvA[1] > 1) uvA[1] = uvA[1] - int(uvA[1]);
    if (uvB[0] > 1) uvB[0] = uvB[0] - int(uvB[0]);
    if (uvB[1] > 1) uvB[1] = uvB[1] - int(uvB[1]);
    if (uvC[0] > 1) uvC[0] = uvC[0] - int(uvC[0]);
    if (uvC[1] > 1) uvC[1] = uvC[1] - int(uvC[1]);

    // Check that texcoords are within bounds
    if (!(uvA[0] >= 0 && uvA[1] <= 1 &&
          uvB[0] >= 0 && uvB[1] <= 1 &&
          uvC[0] >= 0 && uvC[1] <= 1))
    {
      cout << "A: " << uvA << endl;
      cout << "B: " << uvB << endl;
      cout << "C: " << uvC << endl;

      printf("Texcoords out of bounds\n");
      throw;
    }

    // Test: maybe flip Y value of UV
    uvA[1] = 1 - uvA[1];
    uvB[1] = 1 - uvB[1];
    uvC[1] = 1 - uvC[1];

    // Set vertex UVs
    tmp->uv_verts = true;
    tmp->uvA = uvA;
    tmp->uvB = uvB;
    tmp->uvC = uvC;
    tmp->tex_frame = texture_frames.size()-1;
    tmp->texture = true;

    // Set roughness based on roughness map
    // TODO: FIGURE OUT HOW TO ASSIGN THIS -- IS IT INTERPOLATED ACROSS VERTICES?
    // Just assign average roughness across texcoords for now
    float r1 = frame_data[int(texcoords[t_inds[i][0]][0] * int(width-1) + texcoords[t_inds[i][0]][1] * int(height-1) * (width-1))];
    float r2 = frame_data[int(texcoords[t_inds[i][1]][0] * int(width-1) + texcoords[t_inds[i][1]][1] * int(height-1) * (width-1))];
    float r3 = frame_data[int(texcoords[t_inds[i][2]][0] * int(width-1) + texcoords[t_inds[i][2]][1] * int(height-1) * (width-1))];
    tmp->reflect_params.roughness = (r1+r2+r3)/(3 * 255);

    // Check other roughness
    // VEC3 roughA(frame_data[int(texcoords[t_inds[i][0]][0] * int(width-1) + texcoords[t_inds[i][0]][1] * int(height-1) * (width-1))],
    //         frame_data[int(texcoords[t_inds[i][0]][0] * int(width-1) + texcoords[t_inds[i][0]][1] * int(height-1) * (width-1)) + 1],
    //       frame_data[int(texcoords[t_inds[i][0]][0] * int(width-1) + texcoords[t_inds[i][0]][1] * int(height-1) * (width-1))+2]);
    // VEC3 roughB(frame_data[int(texcoords[t_inds[i][1]][0] * int(width-1) + texcoords[t_inds[i][1]][1] * int(height-1) * (width-1))],
    //         frame_data[int(texcoords[t_inds[i][1]][0] * int(width-1) + texcoords[t_inds[i][1]][1] * int(height-1) * (width-1)) + 1],
    //         frame_data[int(texcoords[t_inds[i][1]][0] * int(width-1) + texcoords[t_inds[i][1]][1] * int(height-1) * (width-1))+2]);
    // VEC3 roughC(frame_data[int(texcoords[t_inds[i][2]][0] * int(width-1) + texcoords[t_inds[i][2]][1] * int(height-1) * (width-1))],
    //         frame_data[int(texcoords[t_inds[i][2]][0] * int(width-1) + texcoords[t_inds[i][2]][1] * int(height-1) * (width-1)) + 1],
    //         frame_data[int(texcoords[t_inds[i][2]][0] * int(width-1) + texcoords[t_inds[i][2]][1] * int(height-1) * (width-1))+2]);
    // cout << "Roughness A:" << roughA << endl;
    // cout << "Roughness B:" << roughB << endl;
    // cout << "Roughness C:" << roughC << endl;

    shapes.push_back(make_shared<Triangle>(*tmp));
    x_min = min({x_min, float(vertices[v_ind[0]][0]), float(vertices[v_ind[1]][0]), float(vertices[v_ind[2]][0])});
    y_min = min({y_min, float(vertices[v_ind[0]][1]), float(vertices[v_ind[1]][1]), float(vertices[v_ind[2]][1])});
    z_min = min({z_min, float(vertices[v_ind[0]][2]), float(vertices[v_ind[1]][2]), float(vertices[v_ind[2]][2])});

    x_max = max({x_max, float(vertices[v_ind[0]][0]), float(vertices[v_ind[1]][0]), float(vertices[v_ind[2]][0])});
    y_max = max({y_max, float(vertices[v_ind[0]][1]), float(vertices[v_ind[1]][1]), float(vertices[v_ind[2]][1])});
    z_max = max({z_max, float(vertices[v_ind[0]][2]), float(vertices[v_ind[1]][2]), float(vertices[v_ind[2]][2])});
  }
  stbi_image_free(frame_data);
  printf("Xmin: %f, Xmax: %f\n", x_min, x_max);
  printf("Ymin: %f, Ymax: %f\n", y_min, y_max);
  printf("Zmin: %f, Zmax: %f\n", z_min, z_max);

  // Light: at eye
  lights.push_back(make_shared<pointLight>(eye, VEC3(1,1,1)));
}

void buildSceneBig(int frame, int distance)
{
  // Building scene for huge helios head
  // TODO: implement out of frustum culling for this
  shapes.clear();
  lights.clear();
  focal_length = distance;

  // Rotate eye about x = 2, z = 1
  eye = VEC3(0.3, 0.5, 0);
  lookingAt = VEC3(0.8, 0, -30);

  // Load model vertices
  vector<VEC3> vertices;
  vector<VEC2> texcoords;
  vector<VEC3I> v_inds;
  vector<VEC3I> t_inds;
  vector<VEC3> normals;
  vector<VEC3I> n_inds;

  loadObj("./models/helios_statue/helios.obj", "./models/helios_statue", vertices, v_inds, texcoords, t_inds, normals, n_inds);
  printf("# of vertices: %d, # of texcoords: %d, # of triangles: %d\n",
            int(vertices.size()), int(texcoords.size()), int(v_inds.size()));

  // Rotate vertices so that model facing down positive z-axis (this is the last eye position)
  // NOTE: Model probably already faces this direction!
  VEC3 modelcenter(0, 0.8, 0); // Center? of shape
  for (int i = 0; i < vertices.size(); i++)
  {
    VEC3 v = vertices[i];
    // Rescale by 100, push back by 30, and slowly lift up over time
    vertices[i] = (v - modelcenter) * 100 + modelcenter - VEC3(0,0,30) + VEC3(0, -30 + frame, 0);
  }

  // Load triangles: FIRST TWO ARE ROOT
  // Keep track of shape bounds
  float x_min = FLT_MAX;
  float y_min = FLT_MAX;
  float z_min = FLT_MAX;
  float x_max = FLT_MIN;
  float y_max = FLT_MIN;
  float z_max = FLT_MIN;
  for (int i = 2; i < v_inds.size(); i++)
  {
    VEC3I v_ind = v_inds[i];
    // Increase shape size by 3
    VEC3 tmp_center = (vertices[v_ind[0]] + vertices[v_ind[1]] + vertices[v_ind[2]])/3;

    shared_ptr<Triangle> tmp = make_shared<Triangle>(vertices[v_ind[0]],
                                          vertices[v_ind[1]],
                                          vertices[v_ind[2]],
                                          VEC3(0.75, 0.75, 0.75), "marble", false, "oren-nayar");
    tmp->reflect_params.roughness = 0.5;
    shapes.push_back(make_shared<Triangle>(*tmp));
    x_min = min({x_min, float(vertices[v_ind[0]][0]), float(vertices[v_ind[1]][0]), float(vertices[v_ind[2]][0])});
    y_min = min({y_min, float(vertices[v_ind[0]][1]), float(vertices[v_ind[1]][1]), float(vertices[v_ind[2]][1])});
    z_min = min({z_min, float(vertices[v_ind[0]][2]), float(vertices[v_ind[1]][2]), float(vertices[v_ind[2]][2])});

    x_max = max({x_max, float(vertices[v_ind[0]][0]), float(vertices[v_ind[1]][0]), float(vertices[v_ind[2]][0])});
    y_max = max({y_max, float(vertices[v_ind[0]][1]), float(vertices[v_ind[1]][1]), float(vertices[v_ind[2]][1])});
    z_max = max({z_max, float(vertices[v_ind[0]][2]), float(vertices[v_ind[1]][2]), float(vertices[v_ind[2]][2])});
  }
  printf("Xmin: %f, Xmax: %f\n", x_min, x_max);
  printf("Ymin: %f, Ymax: %f\n", y_min, y_max);
  printf("Zmin: %f, Zmax: %f\n", z_min, z_max);

  // Put a bunch of reference spheres
  VEC3 spherecenter(0.8, 0.5, -2);
  float radius = 2;
  while ((lookingAt - eye).norm() + 10 > (spherecenter-eye).norm())
  {
    shapes.push_back(make_shared<Sphere>(spherecenter, radius, VEC3(1,0,0)));
    spherecenter -= VEC3(0,0,5);
  }

  // Light: at eye
  lights.push_back(make_shared<pointLight>(eye, VEC3(1,1,1)));
}

void buildSceneModel(int frame)
{
  // Building scene for huge helios head
  // TODO: implement out of frustum culling for this
  shapes.clear();
  lights.clear();

  // Rotate eye about x = 0, z = 0
  VEC4 og_eye(0, 1, 4, 1);
  eye = og_eye.head<3>();
  lookingAt = VEC3(0,0,0);
  float theta = M_PI * 2 * frame/150;
  MATRIX4 to_origin; to_origin << 1, 0, 0, -2,
                                  0, 1, 0, 0,
                                  0, 0, 1, -1,
                                  0, 0, 0, 1;
  MATRIX4 rot; rot << cos(theta), 0, sin(theta), 0,
                      0, 1, 0, 0,
                      -sin(theta), 0, cos(theta), 0,
                      0, 0, 0, 1;
  MATRIX4 from_origin; from_origin << 1, 0, 0, 2,
                                      0, 1, 0, 0,
                                      0, 0, 1, 1,
                                      0, 0, 0, 1;
  VEC4 trans_eye = rot * og_eye;
  eye = trans_eye.head<3>();

  // Load model vertices
  vector<VEC3> vertices;
  vector<VEC2> texcoords;
  vector<VEC3I> v_inds;
  vector<VEC3I> t_inds;
  vector<VEC3> normals;
  vector<VEC3I> n_inds;

  loadObj("./models/helios_statue/helios_20.obj", "./models/helios_statue", vertices, v_inds, texcoords, t_inds, normals, n_inds);
  printf("# of vertices: %d, # of texcoords: %d, # of triangles: %d\n",
            int(vertices.size()), int(texcoords.size()), int(v_inds.size()));

  // Rotate vertices so that model facing down positive z-axis (this is the last eye position)

  // Load triangles: FIRST TWO ARE ROOT
  // Keep track of shape bounds
  float x_min = FLT_MAX;
  float y_min = FLT_MAX;
  float z_min = FLT_MAX;
  float x_max = FLT_MIN;
  float y_max = FLT_MIN;
  float z_max = FLT_MIN;
  for (int i = 2; i < v_inds.size(); i++)
  {
    VEC3I v_ind = v_inds[i];
    // Increase shape size by 3
    VEC3 tmp_center = (vertices[v_ind[0]] + vertices[v_ind[1]] + vertices[v_ind[2]])/3;

    shared_ptr<Triangle> tmp = make_shared<Triangle>(vertices[v_ind[0]],
                                          vertices[v_ind[1]],
                                          vertices[v_ind[2]],
                                          VEC3(0.75, 0.75, 0.75), "marble", false, "oren-nayar");
    tmp->reflect_params.roughness = 0.5;
    shapes.push_back(make_shared<Triangle>(*tmp));
    x_min = min({x_min, float(vertices[v_ind[0]][0]), float(vertices[v_ind[1]][0]), float(vertices[v_ind[2]][0])});
    y_min = min({y_min, float(vertices[v_ind[0]][1]), float(vertices[v_ind[1]][1]), float(vertices[v_ind[2]][1])});
    z_min = min({z_min, float(vertices[v_ind[0]][2]), float(vertices[v_ind[1]][2]), float(vertices[v_ind[2]][2])});

    x_max = max({x_max, float(vertices[v_ind[0]][0]), float(vertices[v_ind[1]][0]), float(vertices[v_ind[2]][0])});
    y_max = max({y_max, float(vertices[v_ind[0]][1]), float(vertices[v_ind[1]][1]), float(vertices[v_ind[2]][1])});
    z_max = max({z_max, float(vertices[v_ind[0]][2]), float(vertices[v_ind[1]][2]), float(vertices[v_ind[2]][2])});
  }
  printf("Xmin: %f, Xmax: %f\n", x_min, x_max);
  printf("Ymin: %f, Ymax: %f\n", y_min, y_max);
  printf("Zmin: %f, Zmax: %f\n", z_min, z_max);

  // Floor: checkerboard (vaporwave colors)
  // 1 x 1 squares
  float s = 1;
  VEC3 checker_rgb0(0.58, 0.82, 1);
  VEC3 checker_rgb1(1, 0.416, 0.835);
  shapes.push_back(make_shared<Checkerboard>(VEC3(-6, -1, -6), VEC3(-6, -1, 6), VEC3(6, -1, 6), VEC3(6, -1, -6),
                                            checker_rgb0, checker_rgb1, s));

  // Light: at eye
  lights.push_back(make_shared<pointLight>(eye, VEC3(1,1,1)));
}

void buildSceneCloud(int frame)
{
  shapes.clear();
  lights.clear();
  default_col = VEC3(0.53, 0.81, 0.92);

  shared_ptr<Sphere> sphere = make_shared<Sphere>(VEC3(0, 0.5, 1), 2, VEC3(0.5, 0.5, 0.5), "", false, "cloud");
  shapes.push_back(sphere);
}

void buildSceneRectangle(int frame)
{
  // TEST: Rectangle vertex ordering and intersection
  shapes.clear();
  lights.clear();
  VEC3 col = VEC3(0.53, 0.81, 0.92);
  VEC3 A(1.0, -0.5, -0.5);
  VEC3 B(0.5, -0.5, 0.5);
  VEC3 C(0.5, 1.5, 0.5);
  VEC3 D(1.0, 1.5, -0.5);

  shapes.push_back(make_shared<Rectangle>(A, B, C, D, col));
  shapes.push_back(make_shared<Rectangle>(B + VEC3(0,0,1.5), A + VEC3(0,0,1.5), D + VEC3(0,0,1.5), C + VEC3(0,0,1.5), col));

  lights.push_back(make_shared<pointLight>(eye, VEC3(1,1,1)));
}

void setSceneSkeletonBounding(float frame)
{
  // TODO: Motion blur skeleton????
  setSkeletonsToSpecifiedFrame(int(frame));

  shapes.clear();
  lights.clear();

  displayer.ComputeBonePositions(DisplaySkeleton::BONES_AND_LOCAL_FRAMES);

  // Get bone names in skeleton
  Skeleton* skel = displayer.GetSkeleton(0);

  // retrieve all the bones of the skeleton
  vector<MATRIX4>& rotations = displayer.rotations();
  vector<MATRIX4>& scalings  = displayer.scalings();
  vector<VEC4>& translations = displayer.translations();
  vector<float>& lengths     = displayer.lengths();

  // build a sphere list, but skip the first bone,
  // it's just the origin
  float min_y = FLT_MAX;
  float min_x = FLT_MAX;
  float max_x = FLT_MIN;
  float min_z = FLT_MAX;
  float max_z = FLT_MIN;
  int totalBones = rotations.size();
  for (int x = 1; x < totalBones; x++)
  {
    MATRIX4& rotation = rotations[x];
    MATRIX4& scaling = scalings[x];
    VEC4& translation = translations[x];

    // get the endpoints of the cylinder
    VEC4 leftVertex(0,0,0,1);
    VEC4 rightVertex(0,0,lengths[x],1);

    leftVertex = rotation * scaling * leftVertex + translation;
    rightVertex = rotation * scaling * rightVertex + translation;

    // Cylinders
    shapes.push_back(make_shared<Cylinder>(leftVertex.head<3>(), rightVertex.head<3>(), 0.05, VEC3(1,0,0)));

    float tmp_min_y = min(leftVertex[1], rightVertex[1]);
    min_y = min(min_y, tmp_min_y);
    float tmp_min_x = min(leftVertex[0], rightVertex[0]);
    min_x = min(min_x, tmp_min_x);
    float tmp_max_x = max(leftVertex[0], rightVertex[0]);
    max_x = max(max_x, tmp_max_x);
    float tmp_min_z = min(leftVertex[2], rightVertex[2]);
    min_z = min(min_z, tmp_min_z);
    float tmp_max_z = max(leftVertex[2], rightVertex[2]);
    max_z = max(max_z, tmp_max_z);
  }
  printf("Min/max x = %f, %f, Min/max z: %f, %f\n", min_x, max_x, min_z, max_z);
}

void buildSceneSpheres(float frame)
{
  shapes.clear();
  lights.clear();

  // Testing objects: column of spheres centered but oscillating at different speeds
  VEC3 center(0, 0.5, 1);
  float x = 0;
  for (int i = 0; i < 4; i++)
  {
    float r = 0.3 * pow(1.5, i);
    float width = aspect * tan(fov * M_PI/360.0) * (x - eye[0]);
    VEC3 center_adj = center + VEC3(x, 0, sin((i+1) * float(frame)/180 * 2 * M_PI) * width);
    shapes.push_back(make_shared<Sphere>(center_adj, r, VEC3(1, 0, 0), "", true));
    x += r * 2 * (i+1);
  }

  // Floor: big sphere
  // Set floor radius to min y starting position of bones
  shapes.push_back(make_shared<Sphere>(VEC3(0.5, -1000, 1), 999, VEC3(0.5, 0.5, 0.5)));
  lights.push_back(make_shared<pointLight>(eye, VEC3(0.9, 0.9, 0.9)));
}

void buildSceneDOF(float frame)
{
  shapes.clear();
  lights.clear();

  // Testing objects: row of diagonal spheres
  VEC3 start(0, 0.5, 1);
  float r = 0.3;
  VEC3 dir = VEC3(1, 0, 1).normalized();
  shapes.push_back(make_shared<Sphere>(start, 0.3, VEC3(1,0,0)));
  for (int i = 1; i < 8; i++)
  {
    if (i%2 == 0)
    {
      shapes.push_back(make_shared<Sphere>(start + 2 * i * r * dir, 0.3, VEC3(1,0,0)));
      shapes.push_back(make_shared<Sphere>(start - 2 * i * r * dir, 0.3, VEC3(1,0,0)));
    }
    else
    {
      shapes.push_back(make_shared<Sphere>(start + 2 * i * r * dir, 0.3, VEC3(0,1,0)));
      shapes.push_back(make_shared<Sphere>(start - 2 * i * r * dir, 0.3, VEC3(0,1,0)));
    }
  }
  // Floor
  shapes.push_back(make_shared<Sphere>(VEC3(0.5, -1000, 1), 999, VEC3(0.5, 0.5, 0.5)));
  // Light
  lights.push_back(make_shared<pointLight>(eye, VEC3(0.9, 0.9, 0.9)));
}

void buildSceneHW4(float frame)
{
  shapes.clear();
  lights.clear();

  // Scene geometry
  VEC3 c0(-3.5, 0, -10);
  float r0 = 3;
  VEC3 rgb0(1, 0.25, 0.25);
  VEC3 c1(3.5, 0, -10);
  float r1 = 3;
  VEC3 rgb1(0.25, 0.25, 1);
  VEC3 c2(0, -1000, -10);
  float r2 = 997;
  VEC3 rgb2(0.5, 0.5, 0.5);
  VEC3 light_pos_0(10, 3, -5);
  VEC3 light_rgb_0(1,1,1); // Albedo of both specular and diffuse components
  VEC3 light_pos_1(-10, 3, -7.5);
  VEC3 light_rgb_1(0.5, 0, 0);

  shapes.push_back(make_shared<Sphere>(c0, r0, rgb0));
  shapes.push_back(make_shared<Sphere>(c1, r1, rgb1));
  shapes.push_back(make_shared<Sphere>(c2, r2, rgb2));

  lights.push_back(make_shared<pointLight>(light_pos_0, light_rgb_0));
  lights.push_back(make_shared<pointLight>(light_pos_1, light_rgb_1));
}
