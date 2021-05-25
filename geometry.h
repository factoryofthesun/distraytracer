#ifndef GEOMETRY
#define GEOMETRY

#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "Eigen/Geometry"
#include <cmath>
#include <memory>
#include "SETTINGS.h"

// Geometry helper functions
VEC3 getBarycentricCoords3D(VEC3 p, VEC3 A, VEC3 B, VEC3 C);
void fixNorm(const VEC3 ray, VEC3& norm);
MATRIX4 buildCOB(VEC3 z);
void shortestLine(VEC3 P1, VEC3 P2, VEC3 P3, VEC3 P4, float& u1, float& u2);
bool segmentIntersect(VEC3 A, VEC3 B, VEC3 ray, VEC3 origin, float& t);
bool lineIntersect(VEC3 l1, VEC3 o1, VEC3 l2, VEC3 o2, float& t);

// Helper structs
struct Reflectance {
  std::string material;
  float roughness;
  bool glossy = false;
  VEC2 refr; // This covers BOTH dielectric (1 real value) AND conductor
};

// Define geometry GeoPrimitives
class GeoPrimitive {
  public:
    virtual ~GeoPrimitive(){}
    virtual bool intersect(const VEC3 ray, const VEC3 start, float& t, bool& inside) = 0;
    virtual bool intersectShadow(const VEC3 ray, const VEC3 start, const float t_max) = 0; // faster because just checks for intersection
    virtual VEC3 getNorm(const VEC3 point) = 0;
    virtual void getBounds(VEC3& lbound, VEC3& ubound) = 0; // For BVH construction
    virtual VEC3 getObjCoords(const VEC3 p) {}; // Convert any point to object coordinates (change of basis)
    virtual VEC2 getUV(const VEC3 p, int& valid) {}; // Texture mapping
    virtual bool intersectMax(const VEC3 ray, const VEC3 start, float& t){}; // Get t_max from double intersection

    VEC3 color;
    bool light = false; // light indicator
    bool motion; // Motion indicator
    bool uv_verts = false; // Indicator for whether vertices also have UV data
    int dims;
    MATRIX4 objM; // Object coordinate conversion matrix
    // Shading model
    std::string name;
    std::string model;
    bool texture = false;
    int tex_frame;
    VEC3 bordercolor = VEC3(0,0,0); // For checkered textures

    // Reflectance attributes
    Reflectance reflect_params;

    // For polygons that are part of larger meshes: determine inside using norm direction relative to ray
    bool mesh = false;
    VEC3 mesh_normal;

    // Sphere attributes
    VEC3 center; // NOTE: this will be defined for every shape for BVH construction
    float radius;
    // Triangle attributes
    VEC3 A;
    VEC3 B;
    VEC3 C;
    VEC3 D; // Rectangle
    // Prism attributes
    VEC3 E;
    VEC3 F;
    VEC3 G;
    VEC3 H;
    float height;
    // Rectangle attributes
    float length;
    float width;
    // Finite cylinder attributes
    VEC3 c1;
    VEC3 c2;
    VEC3 axis;
    // Vertex textures
    VEC2 uvA;
    VEC2 uvB;
    VEC2 uvC;
    VEC2 uvD;
};

class Sphere : public GeoPrimitive {
  public:
    ~Sphere(){}
    Sphere();
    Sphere(VEC3 c, float r, VEC3 col, std::string material = "", bool in_motion = false, std::string shader = "lambert");
    bool intersect(const VEC3 ray, const VEC3 start, float& t, bool& inside);
    bool intersectShadow(const VEC3 ray, const VEC3 start, const float t_max);
    VEC3 getNorm(const VEC3 point);
    void getBounds(VEC3& lbound, VEC3& ubound);
    bool intersectMax(const VEC3 ray, const VEC3 start, float& t);
};

// Cylinder: support just circular for now
class Cylinder : public GeoPrimitive {
  public:
    ~Cylinder() {}
    Cylinder();
    Cylinder(VEC3 v1, VEC3 v2, float r, VEC3 col, std::string material = "", bool in_motion = false, std::string shader = "lambert"); // Circular
    // TODO: Support for initialization by axis?
    // Cylinder(VEC3 v1, VEC3 v2, float r1, float r2); // Elliptical
    bool intersect(const VEC3 ray, const VEC3 start, float& t, bool& inside);
    bool intersectCap(const VEC3 ray, const VEC3 start, float& t, bool& inside);
    bool intersectShadow(const VEC3 ray, const VEC3 start, const float t_max);
    VEC3 getNorm(const VEC3 point);
    void getBounds(VEC3& lbound, VEC3& ubound);
    bool intersectMax(const VEC3 ray, const VEC3 start, float& t);
};

class Triangle : public GeoPrimitive {
  public:
    ~Triangle() {};
    Triangle(VEC3 a_vert, VEC3 b_vert, VEC3 c_vert, VEC3 col, std::string material = "", bool in_motion = false,
              std::string shader = "lambert");
    bool intersect(const VEC3 ray, const VEC3 start, float& t, bool& inside);
    bool intersectShadow(const VEC3 ray, const VEC3 start, const float t_max);
    VEC3 getNorm(const VEC3 point = VEC3(0,0,0));
    void getBounds(VEC3& lbound, VEC3& ubound);
    VEC2 getUV(const VEC3 p, int& valid);
};

class Rectangle: public GeoPrimitive {
  public:
    ~Rectangle() {};
    Rectangle();
    // IMPORTANT: a,b,c,d MUST BE CLOCKWISE/COUNTERCLOCKWISE
    Rectangle(VEC3 a, VEC3 b, VEC3 c, VEC3 d, VEC3 col, std::string material = "", bool in_motion = false,
              int texframe = -1, std::string shader = "lambert");
    bool intersect(const VEC3 ray, const VEC3 start, float& t, bool& inside);
    bool intersectShadow(const VEC3 ray, const VEC3 start, const float t_max);
    VEC3 getNorm(const VEC3 point = VEC3(0,0,0));
    VEC2 getUV(const VEC3 p, int& valid);
    void getBounds(VEC3& lbound, VEC3& ubound);
    VEC3 samplePoint();
};

// Screw it use triangles
class RectPrismV2: public GeoPrimitive {
  public:
    ~RectPrismV2() {};
    RectPrismV2(){};
    RectPrismV2(VEC3 a, VEC3 b, VEC3 c, VEC3 d, VEC3 e, VEC3 f, VEC3 g, VEC3 h,
              VEC3 col, std::string material = "", bool in_motion = false,
              int texframe = -1, std::string shader = "lambert");
    bool intersect(const VEC3 ray, const VEC3 start, float& t, bool& inside);
    bool intersectShadow(const VEC3 ray, const VEC3 start, const float t_max);
    VEC3 getNorm(const VEC3 point);
    void getBounds(VEC3& lbound, VEC3& ubound);
    VEC2 getUV(const VEC3 p, int& valid);
    std::vector<std::shared_ptr<Rectangle>> faces;
};

// Actual box
class RectPrism: public GeoPrimitive {
  public:
    ~RectPrism() {};
    RectPrism(){};
    RectPrism(VEC3 a, VEC3 b, VEC3 c, VEC3 d, VEC3 e, VEC3 f, VEC3 g, VEC3 h,
              VEC3 col, std::string material = "", bool in_motion = false,
              int texframe = -1, std::string shader = "lambert");
    bool intersect(const VEC3 ray, const VEC3 start, float& t, bool& inside);
    bool intersectShadow(const VEC3 ray, const VEC3 start, const float t_max);
    VEC3 getNorm(const VEC3 point);
    void getBounds(VEC3& lbound, VEC3& ubound);
    void getObjBounds(VEC3& lbound, VEC3& ubound);
    VEC2 getUV(const VEC3 p, int& valid);
    bool intersectShadowBad(const VEC3 ray, const VEC3 start, const float t_max);
    VEC3 ubound; // OBJECT COORD BOUNDS
    VEC3 lbound;
};

// ****************************
// ***** Derived Shapes ***************
// ****************************
// Rectangular prism with cylinder holes
class RectPrismWithCylinder: public RectPrism {
  public:
    ~RectPrismWithCylinder() {
      holes.clear();
      std::vector<std::shared_ptr<Cylinder>>().swap(holes);
    };
    RectPrismWithCylinder(VEC3 a, VEC3 b, VEC3 c, VEC3 d, VEC3 e, VEC3 f, VEC3 g, VEC3 h,
              VEC3 col, std::string material = "", bool in_motion = false,
              int texframe = -1, std::string shader = "lambert");
    bool intersect(const VEC3 ray, const VEC3 start, float& t, bool& inside);
    bool intersectShadow(const VEC3 ray, const VEC3 start, const float t_max);
    VEC3 getNorm(const VEC3 point);
    VEC3 ubound;
    VEC3 lbound;
    int lastHit = -1; // Keep track of last hit cylinder (for norm calculation)
    std::vector<std::shared_ptr<Cylinder>> holes;
};

// Rectangular prism with internal vector of shape holes (each with own coloring)
class RectPrismWithHoles: public RectPrism {
  public:
    ~RectPrismWithHoles() {
      holes.clear();
      std::vector<std::shared_ptr<GeoPrimitive>>().swap(holes);
    };
    RectPrismWithHoles(VEC3 a, VEC3 b, VEC3 c, VEC3 d, VEC3 e, VEC3 f, VEC3 g, VEC3 h,
                      VEC3 col, std::string material = "", bool in_motion = false,
                      int texframe = -1, std::string shader = "lambert");
    bool intersect(const VEC3 ray, const VEC3 start, float& t, bool& inside);
    bool intersectShadow(const VEC3 ray, const VEC3 start, const float t_max);
    void getBounds(VEC3& lbound, VEC3& ubound);
    VEC3 getNorm(const VEC3 point);
    VEC3 ubound;
    VEC3 lbound;
    int lastHit = -1;
    std::vector<std::shared_ptr<GeoPrimitive>> holes;
};

// Checkerboard with UV-mapped color (changes every-time hit)
class Checkerboard: public Rectangle {
  public:
    ~Checkerboard() {};
    Checkerboard(VEC3 a, VEC3 b, VEC3 c, VEC3 d, VEC3 col1, VEC3 col2, float S_square, std::string material = "",
                  bool in_motion = false, std::string shader = "lambert");
    bool intersect(const VEC3 ray, const VEC3 start, float& t, bool& inside);
    float S; // side length of square checker
    VEC3 color1;
    VEC3 color2;
};

// Checkerboard with rectangle hole
class CheckerboardWithHole: public Rectangle {
  public:
    ~CheckerboardWithHole() {};
    CheckerboardWithHole(VEC3 a, VEC3 b, VEC3 c, VEC3 d, VEC3 col1, VEC3 col2, float S_square, std::shared_ptr<Rectangle> shape,
                           std::string material = "", bool in_motion = false, std::string shader = "lambert");
    bool intersect(const VEC3 ray, const VEC3 start, float& t, bool& inside);
    bool intersectShadow(const VEC3 ray, const VEC3 start, const float t_max);
    VEC2 getUV(const VEC3 p, int& valid); // UV coordinate of intersection point; valid = 2 => in border
    float S; // side length of checker square
    VEC3 color1;
    VEC3 color2;
    std::shared_ptr<Rectangle> hole;
    float borderwidth = 0;
};

// Cylinder with checker texture (single square again)
class CheckerCylinder : public Cylinder {
  public:
    ~CheckerCylinder() {}
    CheckerCylinder(VEC3 v1, VEC3 v2, float r, VEC3 col, float s,
                    std::string material = "", bool in_motion = false, std::string shader = "lambert"); // Circular
    VEC2 getUV(const VEC3 p, int& valid); // UV coordinate of intersection point; valid = 2 => in border
    float S; // side length of checker square
    float borderwidth = 0;
};

// AABB: Axis Aligned Bounding Box
class BoundingVolume {
  public:
    ~BoundingVolume() {
            indices.clear();
            nodes.clear();
          };
    BoundingVolume(std::vector<int> inds, std::vector<std::shared_ptr<GeoPrimitive>> shapes, bool is_leaf = false);
    void setBounds(std::vector<std::shared_ptr<GeoPrimitive>> shapes);
    bool intersect(const VEC3 ray, const VEC3 inv_ray, const VEC3 start);
    std::vector<std::shared_ptr<BoundingVolume>> nodes;
    std::vector<int> indices;
    bool leaf;
    VEC3 lbound;
    VEC3 ubound;
};

// ****************************
// ***** LIGHTS ***************
// ****************************
// LightPrimitive primitive
class LightPrimitive {
  public:
    virtual ~LightPrimitive(){}
    virtual VEC3 sampleRay(const VEC3 point) = 0;
    VEC3 color;
    VEC3 center;
};

class pointLight: public LightPrimitive {
  public:
    ~pointLight() {};
    pointLight(const VEC3 c, const VEC3 col);
    VEC3 sampleRay(const VEC3 point);
};

class sphereLight: public LightPrimitive, public Sphere {
  public:
    ~sphereLight() {};
    sphereLight(const VEC3 c, const float r, const VEC3 col, std::string material = "", bool in_motion = false);
    VEC3 sampleRay(const VEC3 point);
    VEC3 baxis = VEC3(0,0,0); // Hacky: set bounding axis for semisphere sampling
};

class rectangleLight: public LightPrimitive, public Rectangle {
public:
  ~rectangleLight() {};
  rectangleLight(VEC3 a, VEC3 b, VEC3 c, VEC3 d, VEC3 col, std::string material = "", bool in_motion = false);
  VEC3 sampleRay(const VEC3 point);
};

#endif
