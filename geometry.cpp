#include "geometry.h"
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

// =========== GEOMETRY HELPERS =================
// Ensure norm always points in opposite direction of ray
void fixNorm(const VEC3 ray, VEC3& norm)
{
  // Multiply by 1e4 to prevent dumb floating point stuff
  if ((ray*1e4).dot(norm) >= 0)
  {
    norm *= -1;
  }
}

// Change of basis (from standard cartesian)
MATRIX4 buildCOB(VEC3 z)
{
  VEC3 w = z.normalized();
  VEC3 u = VEC3(1,0,0).cross(w).normalized();
  if (u.isApprox(VEC3(0,0,0)))
  {
    u = VEC3(0,1,0).cross(w).normalized();
  }
  VEC3 v = w.cross(u).normalized();

  MATRIX4 cob; cob << u.transpose(), 0, v.transpose(), 0, w.transpose(), 0,
                      0, 0, 0, 1;
  return cob;
}

// Coefficients for shortest line between two lines (P4-P3) and (P2-P1)
void shortestLine(VEC3 P1, VEC3 P2, VEC3 P3, VEC3 P4, float& u1, float& u2)
{
 u1 = ((P1-P3).dot(P4-P3) * (P4-P3).dot(P2-P1) - (P1-P3).dot(P2-P1)*(P4-P3).dot(P4-P3))/
    (pow((P2-P1).norm(), 2) * pow((P4-P3).norm(), 2) - pow((P4-P3).dot(P2-P1), 2));
 u2 = ((P1-P3).dot(P4-P3) + u1 * (P4-P3).dot(P2-P1))/pow((P4-P3).norm(), 2);
}

bool segmentIntersect(VEC3 A, VEC3 B, VEC3 ray, VEC3 origin, float& t)
{
 // Use shortest line method to compute
 float u1, u2;
 shortestLine(A, B, ray+origin, origin, u1, u2);

 // BA is a segment: check that first coefficient between 0 and 1
 if (u1 < 0 || u1 > 1)
 {
  return false;
 }
 // Check that second segment is non-negative
 if (u2 < 0)
 {
  return false;
 }
 // Intersection points should be within epsilon
 VEC3 p1 = A + u1 * (B-A);
 VEC3 p2 = origin + u2 * ray;
 if ((p2-p1).norm() < 1e-4)
 {
  t = u2;
  return true;
 }
 return false;
}

bool lineIntersect(VEC3 l1, VEC3 o1, VEC3 l2, VEC3 o2, float& t)
{
 return false;
}

// =========== SHAPES =================
Sphere::Sphere()
{
 center = VEC3(0,0,0);
 radius = 1;
 color = VEC3(1,1,1);
 dims = 3;
 reflect_params.material = "";
 model = "lambert";
 name ="sphere";
}

Sphere::Sphere(VEC3 c, float r, VEC3 col, std::string material, bool in_motion, std::string shader)
{
 center = c;
 radius = r;
 color = col;
 reflect_params.material = material;
 dims = 3;
 motion = in_motion;
 model = shader;
 name="sphere";
}

bool Sphere::intersect(const VEC3 ray, const VEC3 start, float& t, bool& inside)
{
 float A = ray.dot(ray);
 float B = 2 * ray.dot(start - center);
 float C = (start - center).dot(start - center) - pow(radius, 2);
 float discriminant = pow(B, 2) - 4 * A * C;
 if (discriminant < 0) // No real roots = no intersection
 {
  return false;
 }
 else
 {
  float t0 = (-B + sqrt(discriminant))/(2*A);
  float t1 = (-B - sqrt(discriminant))/(2*A);
  if (t0 <= 0.001 && t1 <= 0.001) // If sphere is behind camera then no intersection
  {
   inside = false;
   return false;
  }
  else if (t0 <= 0.001 || t1 <= 0.001) // If inside sphere then return the intersection in front
  {
   float t_ret = std::max(t0, t1);
   t = t_ret;
   inside = true;
   return true;
  }
  else // Standard double intersection
  {
   float t_ret = std::min(t0, t1);
   t = t_ret;
   inside = false;
   return true;
  }
 }
}

bool Sphere::intersectMax(const VEC3 ray, const VEC3 start, float& t)
{
 float A = ray.dot(ray);
 float B = 2 * ray.dot(start - center);
 float C = (start - center).dot(start - center) - pow(radius, 2);
 float discriminant = pow(B, 2) - 4 * A * C;
 if (discriminant < 0) // No real roots = no intersection
 {
  return false;
 }
 else
 {
  float t0 = (-B + sqrt(discriminant))/(2*A);
  float t1 = (-B - sqrt(discriminant))/(2*A);
  if (t0 <= 0.001 && t1 <= 0.001) // If sphere is behind camera then no intersection
  {
   return false;
  }
  else if (t0 <= 0.001 || t1 <= 0.001) // Can't be inside sphere
  {
   return false;
  }
  else // Standard double intersection
  {
   float t_ret = std::max(t0, t1);
   t = t_ret;
   return true;
  }
 }
}

bool Sphere::intersectShadow(const VEC3 ray, const VEC3 start, const float t_max)
{
 float eps = 1e-3;
 float A = ray.dot(ray);
 float B = 2 * ray.dot(start - center);
 float C = (start - center).dot(start - center) - pow(radius, 2);
 float discriminant = pow(B, 2) - 4 * A * C;
 if (discriminant < 0) // No real roots = no intersection
 {
  return false;
 }
 else
 {
  float t0 = (-B + sqrt(discriminant))/(2*A);
  float t1 = (-B - sqrt(discriminant))/(2*A);
  if ((t0 <= eps || t0 >= t_max) && (t1 <= eps || t1 >= t_max)) // Need small constant to avoid intersection with surface
  {
   return false;
  }
  else
  {
   return true;
  }
 }
}

VEC3 Sphere::getNorm(const VEC3 point)
{
 VEC3 norm = point - center;
 norm = norm/norm.norm();
 return norm;
}

void Sphere::getBounds(VEC3& lbound, VEC3& ubound)
{
 lbound = VEC3(center[0] - radius, center[1] - radius, center[2] - radius);
 ubound = VEC3(center[0] + radius, center[1] + radius, center[2] + radius);
}

Cylinder::Cylinder()
{
 c1 = VEC3(0,-1,0);
 c2 = VEC3(0,1,0);
 axis = (c2 - c1).normalized();
 radius = 1;
 color = VEC3(1,1,1);
 reflect_params.material = "";
 dims = 3;
 motion = false;
 model = "lambert";
 center = VEC3(0,0,0);
 name = "cylinder";
}

Cylinder::Cylinder(VEC3 v1, VEC3 v2, float r, VEC3 col, std::string material, bool in_motion, std::string shader)
{
 c1 = v1;
 c2 = v2;
 axis = (v2 - v1).normalized();
 radius = r;
 color = col;
 reflect_params.material = material;
 dims = 3;
 motion = in_motion;
 model = shader;
 center = (v1 + v2)/2;
 name = "cylinder";
}

bool Cylinder::intersect(const VEC3 ray, const VEC3 start, float& t, bool& inside)
{
 float eps = 1e-3;
 // Check body intersection
 VEC3 ray_a_proj = ray - ray.dot(axis) * axis;
 VEC3 constant = start - c1 - ((start-c1).dot(axis)) * axis;
 float A = ray_a_proj.dot(ray_a_proj);
 float B = 2 * ray_a_proj.dot(constant);
 float C = constant.dot(constant) - pow(radius,2);
 float discriminant = pow(B, 2) - 4 * A * C;
 float t1_body = FLT_MIN, t2_body = FLT_MIN;

 if (discriminant >= 0)
 {
  t1_body = (-B + sqrt(discriminant))/(2*A);
  t2_body = (-B - sqrt(discriminant))/(2*A);
  if (t1_body <= eps && t2_body <= eps) // If behind camera then no intersection
  {
   inside = false;
   return false;
  }
  else if (t1_body <= eps || t2_body <= eps) // If inside then return the intersection in front (max)
  {
   // Check within caps
   VEC3 p = start + t1_body * ray;
   if (axis.dot(p - c1) > 0 && axis.dot(p - c2) < 0)
   {
    t = t1_body;
    inside = true;
    return true;
   }
   else
   {
    return false;
   }
  }
  else // Standard double intersection (min)
  {
   // Check within caps
   VEC3 p = start + t2_body * ray;
   if (axis.dot(p - c1) > 0 && axis.dot(p - c2) < 0)
   {
    t = t2_body;
    inside = false;
    return true;
   }
   else
   {
    return false;
   }
  }
 }
 return false;
}

bool Cylinder::intersectCap(const VEC3 ray, const VEC3 start, float& t, bool& inside)
{
 float eps = 1e-3;
 inside = false;
 // Basic circle intersection test
 float rdota = ray.dot(axis);
 if (rdota == 0)
 {
   return false;
 }
 float t1 = (c1.dot(axis) - start.dot(axis))/rdota;
 float t2 = (c2.dot(axis) - start.dot(axis))/rdota;
 if (t1 < eps && t2 < eps)
 {
   return false;
 }
 else if (t1 < eps || t2 < eps)
 {
   inside = true;
   t = (float) std::max(t1,t2);
   return true;
 }
 else
 {
   t = (float) std::min(t1, t2);
   return true;
 }
}

bool Cylinder::intersectMax(const VEC3 ray, const VEC3 start, float& t)
{
 float eps = 1e-3;
 // Check body intersection
 VEC3 ray_a_proj = ray - ray.dot(axis) * axis;
 VEC3 constant = start - c1 - ((start-c1).dot(axis)) * axis;
 float A = ray_a_proj.dot(ray_a_proj);
 float B = 2 * ray_a_proj.dot(constant);
 float C = constant.dot(constant) - pow(radius,2);
 float discriminant = pow(B, 2) - 4 * A * C;
 float t1_body = FLT_MIN, t2_body = FLT_MIN;

 if (discriminant >= 0)
 {
  t1_body = (-B + sqrt(discriminant))/(2*A);
  t2_body = (-B - sqrt(discriminant))/(2*A);
  if (t1_body <= eps && t2_body <= eps) // If behind camera then no intersection
  {
   return false;
  }
  else if (t1_body <= eps || t2_body <= eps) // Can't be inside
  {
   return false;
  }
  else // Standard double intersection (min)
  {
   // Check within caps
   VEC3 p = start + t2_body * ray;
   if (axis.dot(p - c1) > 0 && axis.dot(p - c2) < 0)
   {
    t = t1_body;
    return true;
   }
   else
   {
    return false;
   }
  }
 }
 return false;
}

bool Cylinder::intersectShadow(const VEC3 ray, const VEC3 start, const float t_max)
{
 float eps = 1e-3;
 // Check body intersection
 VEC3 ray_a_proj = ray - ray.dot(axis) * axis;
 VEC3 constant = start - c1 - ((start-c1).dot(axis)) * axis;
 float A = ray_a_proj.dot(ray_a_proj);
 float B = 2 * ray_a_proj.dot(constant);
 float C = constant.dot(constant) - pow(radius,2);
 float discriminant = pow(B, 2) - 4 * A * C;
 float t1_body, t2_body;

 if (discriminant >= 0)
 {
  t1_body = (-B + sqrt(discriminant))/(2*A);
  t2_body = (-B - sqrt(discriminant))/(2*A);
   // If behind camera OR behind light then no intersection
  if ((t1_body <= eps || t1_body >= t_max) && (t2_body <= eps || t2_body >= t_max))
  {
   return false;
  }
  else if (t1_body <= eps || t2_body <= eps) // If inside then return the intersection in front
  {
   // Check within caps
   VEC3 p = start + t1_body * ray;
   if (axis.dot(p - c1) > 0 && axis.dot(p - c2) < 0 && t1_body < t_max)
   {
    return true;
   }
   else
   {
    return false;
   }
  }
  else // Standard double intersection
  {
   // Check within caps
   VEC3 p = start + t2_body * ray;
   if (axis.dot(p - c1) > 0 && axis.dot(p - c2) < 0 && t2_body < t_max)
   {
    return true;
   }
   else
   {
    return false;
   }
  }
 }
 return false;
}

VEC3 Cylinder::getNorm(const VEC3 point)
{
 // Project p-c to axis, then subtract
 VEC3 pc = point - c1;
 VEC3 norm = pc - pc.dot(axis) * axis;
 return(norm.normalized());
}

void Cylinder::getBounds(VEC3& lbound, VEC3& ubound)
{
 lbound = (c1.array() - radius).min(c2.array() - radius);
 ubound = (c1.array() + radius).max(c2.array() + radius);
}

Triangle::Triangle(VEC3 a_vert, VEC3 b_vert, VEC3 c_vert, VEC3 col, std::string material, bool in_motion, std::string shader)
{
 A = a_vert;
 B = b_vert;
 C = c_vert;
 color = col;
 reflect_params.material = material;
 dims = 2;
 motion = in_motion;
 model = shader;
 center = (A + B + C)/3;
 name = "triangle";
}

VEC2 Triangle::getUV(const VEC3 p, int& valid)
{
  VEC3 bcent = getBarycentricCoords3D(p, A, B, C);
  if (bcent[0] < 0 || bcent[0] > 1 || bcent[1] < 0 || bcent[1] > 1 || bcent[2] < 0 || bcent[2] > 1)
  {
    valid = 0;
    return VEC2(-1,-1);
  }
  // This only makes sense if have UV-mapped vertices
  if (uv_verts == false)
  {
    printf("Triangle getUV: need UV-mapped vertices\n");
    throw "Triangle getUV: need UV-mapped vertices";
  }
  valid = 1;
  return(bcent[0] * uvA + bcent[1] * uvB + bcent[2] * uvC);
}

VEC3 getBarycentricCoords2D(VEC2 p, VEC2 A, VEC2 B, VEC2 C)
{
 MATRIX2 left; left << B[0] - A[0], C[0] - A[0], B[1]-A[1], C[1]-A[1];
 VEC2 right; right << p[0]-A[0], p[1]-A[1];
 VEC2 bg = left.inverse()*right;
 VEC3 ret(1-bg[0]-bg[1], bg[0], bg[1]);
 return ret;
}

VEC3 getBarycentricCoords3D(VEC3 p, VEC3 A, VEC3 B, VEC3 C)
{
 VEC3 n = (B - A).cross(C - A);
 VEC3 n_a = (C - B).cross(p - B);
 VEC3 n_b = (A - C).cross(p - C);
 VEC3 n_c = (B - A).cross(p - A);
 float n_sqnorm = n.squaredNorm();
 float alpha = n.dot(n_a)/n_sqnorm;
 float beta = n.dot(n_b)/n_sqnorm;
 float gamma = 1 - alpha - beta;
 VEC3 ret(alpha, beta, gamma);
 return ret;
}

bool Triangle::intersect(const VEC3 ray, const VEC3 start, float& t, bool& inside)
{
 inside = false;
 float eps = 1e-5;
 // Check for EDGE INTERSECTIONS: whenever viewing ray is perpendicular to normal
 // if (getNorm(start).dot(ray) == 0)
 // {
 //  // Compute t for every line: should take care of parallel, and behind eye edge cases
 //  float tmin = FLT_MAX;
 //  float t_tmp;
 //  bool hit = false;
 //  hit = segmentIntersect(A, B, ray, start, t_tmp);
 //  if (hit == true) {tmin = std::min(t_tmp, tmin);}
 //
 //  hit = segmentIntersect(B, C, ray, start, t_tmp);
 //  if (hit == true) {tmin = std::min(t_tmp, tmin);}
 //
 //  hit = segmentIntersect(C, A, ray, start, t_tmp);
 //  if (hit == true) {tmin = std::min(t_tmp, tmin);}
 //
 //  if (tmin < FLT_MAX)
 //  {
 //   t = tmin;
 //   return true;
 //  }
 //  else return false;
 // }
 // Implement Moller and Trumbore 97
 VEC3 r1 = B-A;
 VEC3 r2 = C-A;
 VEC3 norm = r1.cross(r2);
 VEC3 h = ray.cross(r2);
 float det = r1.dot(h);
 float invdet = 1.0/det;
 if (det >= -0.0001 && det <= 0.0001)
 {
  return false;
 }
 VEC3 A0 = start - A;
 float u = invdet * A0.dot(h);
 if (u < 0 || u > 1)
 {
  return false;
 }
 VEC3 DA0 = A0.cross(r1);
 float v = ray.dot(DA0) * invdet;
 if (v < 0 || u+v > 1)
 {
  return false;
 }
 float t_final = r2.dot(DA0) * invdet;
 if (t_final > 0.0001)
 {
  // Check ray against mesh normal if necessary
  if (mesh == true)
  {
    if (ray.dot(mesh_normal) > 0)
    {
      inside = true;
    }
  }
  t = t_final;
  return true;
 }
 return false;
}

bool Triangle::intersectShadow(const VEC3 ray, const VEC3 start, const float t_max)
{
 // Trumbore algorithm
 VEC3 r1 = B-A;
 VEC3 r2 = C-A;
 VEC3 norm = r1.cross(r2);
 VEC3 h = ray.cross(r2);
 float det = r1.dot(h);
 float invdet = 1.0/det;
 if (det >= -0.0001 && det <= 0.0001)
 {
  return false;
 }
 VEC3 A0 = start - A;
 float u = invdet * A0.dot(h);
 if (u < 0 || u > 1)
 {
  return false;
 }
 VEC3 DA0 = A0.cross(r1);
 float v = ray.dot(DA0) * invdet;
 if (v < 0 || u+v > 1)
 {
  return false;
 }
 float t_final = r2.dot(DA0) * invdet;
 if (t_final > 0.001 && t_final < t_max)
 {
  return true;
 }
 return false;
}

VEC3 Triangle::getNorm(const VEC3 point)
{
 VEC3 r1 = B - A;
 VEC3 r2 = C - A;
 VEC3 norm = r1.cross(r2).normalized();
 return norm;
}

void Triangle::getBounds(VEC3& lbound, VEC3& ubound)
{
 VEC3 min_tmp = A.array().min(B.array());
 lbound = min_tmp.array().min(C.array());
 VEC3 max_tmp = A.array().max(B.array());
 ubound = max_tmp.array().max(C.array());
}

Rectangle::Rectangle()
{
 motion = false;
 color = VEC3(1,0,0);
 length = 1;
 width = 1;
 A = VEC3(0,0,0);
 B = VEC3(1,0,0);
 C = VEC3(1,1,0);
 D = VEC3(0,1,0);
 dims = 2;
 model = "lambert";
 reflect_params.material = "";
 center = (A + B + C + D)/4;
 name = "rectangle";
}

Rectangle::Rectangle(VEC3 a, VEC3 b, VEC3 c, VEC3 d, VEC3 col, std::string material, bool in_motion,
                      int texframe, std::string shader)
{
 A = a;
 B = b;
 C = c;
 D = d;
 length = (b-a).norm();
 width = (d-a).norm();
 color = col;
 reflect_params.material = material;
 dims = 2;
 motion = in_motion;
 model = shader;
 center = (A + B + C + D)/4;
 name = "rectangle";
 if (texframe >= 0) tex_frame=texframe;
}

bool Rectangle::intersect(const VEC3 ray, const VEC3 start, float& t, bool& inside)
{
  float eps = 1e-4;
  inside = false;
  // Check for EDGE INTERSECTIONS: whenever viewing ray is perpendicular to normal
  // if (getNorm(start).dot(ray) == 0)
  // {
  //  // Compute t for every line: should take care of parallel, and behind eye edge cases
  //  float tmin = FLT_MAX;
  //  float t_tmp;
  //  bool hit = false;
  //  hit = segmentIntersect(A, B, ray, start, t_tmp);
  //  if (hit == true) {tmin = std::min(t_tmp, tmin);}
  //
  //  hit = segmentIntersect(A, D, ray, start, t_tmp);
  //  if (hit == true) {tmin = std::min(t_tmp, tmin);}
  //
  //  hit = segmentIntersect(B, C, ray, start, t_tmp);
  //  if (hit == true) {tmin = std::min(t_tmp, tmin);}
  //
  //  hit = segmentIntersect(C, D, ray, start, t_tmp);
  //  if (hit == true) {tmin = std::min(t_tmp, tmin);}
  //
  //  if (tmin < FLT_MAX)
  //  {
  //   t = tmin;
  //   return true;
  //  }
  //  else return false;
  // }
  // Hardcode solution
  VEC3 norm = getNorm(start).normalized();
  float dn = ray.dot(norm); // Parallel edge case: should be taken care of with above but just in case
  if (dn == 0)
  {
    return false;
  }
  float t_final = (A - start).dot(norm)/dn;
  if (t_final <= eps)
  {
    return false;
  }
  VEC3 point = start + t_final * ray;
  VEC3 V_hit = point - A;
  VEC3 V1 = B-A;
  VEC3 V2 = D-A;
  float check1 = V1.normalized().dot(V_hit);
  float check2 = V2.normalized().dot(V_hit);
  if (0 <= check1 && check1 <= V1.norm() && 0 <= check2 && check2 <= V2.norm())
  {
    t = t_final;
    return true;
  }
  return false;
}

bool Rectangle::intersectShadow(const VEC3 ray, const VEC3 start, const float t_max)
{
 // Check for EDGE INTERSECTIONS: whenever viewing ray is perpendicular to normal
 float eps = 1e-4;
 // if (getNorm(start).dot(ray) == 0)
 // {
 //  float t_tmp;
 //  bool hit = false;
 //  hit = segmentIntersect(A, B, ray, start, t_tmp);
 //  if (hit == true) {return true;}
 //
 //  hit = segmentIntersect(A, D, ray, start, t_tmp);
 //  if (hit == true) {return true;}
 //
 //  hit = segmentIntersect(B, C, ray, start, t_tmp);
 //  if (hit == true) {return true;}
 //
 //  hit = segmentIntersect(C, D, ray, start, t_tmp);
 //  if (hit == true) {return true;}
 //
 //  return false;
 // }
 // Hardcode solution
 VEC3 norm = getNorm(start).normalized();
 float dn = ray.dot(norm); // Parallel edge case: should be taken care of with above but just in case
 if (dn == 0)
 {
  return false;
 }
 float t_final = (A - start).dot(norm)/dn;
 if (t_final <= eps)
 {
  return false;
 }
 VEC3 point = start + t_final * ray;
 VEC3 V_hit = point - A;
 VEC3 V1 = B-A;
 VEC3 V2 = D-A;
 float check1 = V1.normalized().dot(V_hit);
 float check2 = V2.normalized().dot(V_hit);
 if (0 <= check1 && check1 <= V1.norm() && 0 <= check2 && check2 <= V2.norm() && t_final < t_max)
 {
   return true;
 }
 return false;
}

VEC3 Rectangle::getNorm(const VEC3 point)
{
 VEC3 r1 = B - A;
 VEC3 r2 = C - A;
 VEC3 norm = r1.cross(r2).normalized();
 return norm;
}

VEC2 Rectangle::getUV(const VEC3 p, int& valid)
{
  VEC3 ad = D-A;
  VEC3 dc = C-D;
  float u = (p-A).cross(ad).norm()/(ad.norm() * dc.norm()); // "X" distance
  float v = (p-D).cross(dc).norm()/(dc.norm() * ad.norm()); // "Y" distance
  valid = 1;
  return VEC2(u,v);
}

void Rectangle::getBounds(VEC3& lbound, VEC3& ubound)
{
 VEC3 min_tmp = A.array().min(B.array());
 min_tmp = min_tmp.array().min(C.array());
 lbound = min_tmp.array().min(D.array());

 VEC3 max_tmp = A.array().max(B.array());
 max_tmp = max_tmp.array().max(C.array());
 ubound = max_tmp.array().max(D.array());
}

VEC3 Rectangle::samplePoint()
{
  std::random_device rd;
  std::mt19937 generator(rd());
  std::uniform_real_distribution<double> uniform(0.0, 1.0);

  float x = uniform(generator);
  float y = uniform(generator);
  VEC3 sample = A + x * (B-A) + y * (D-A);
  return sample;
}

RectPrismV2::RectPrismV2(VEC3 a, VEC3 b, VEC3 c, VEC3 d, VEC3 e, VEC3 f, VEC3 g, VEC3 h,
          VEC3 col, std::string material, bool in_motion,
          int texframe, std::string shader)
{
  A = a;
  B = b;
  C = c;
  D = d;
  E = e;
  F = f;
  G = g;
  H = h;
  faces.push_back(std::make_shared<Rectangle>(a,b,c,d,col));
  faces.push_back(std::make_shared<Rectangle>(e,f,g,h,col));
  faces.push_back(std::make_shared<Rectangle>(a,b,f,e,col));
  faces.push_back(std::make_shared<Rectangle>(d,a,e,h,col));
  faces.push_back(std::make_shared<Rectangle>(b,c,g,f,col));
  faces.push_back(std::make_shared<Rectangle>(c,d,h,g,col));
  length = (b-a).norm();
  width = (d-a).norm();
  height = (e-a).norm();
  color = col;
  reflect_params.material = material;
  dims = 3;
  motion = in_motion;
  model = shader;
  center = (A + B + C + D + E + F + G + H)/8;
  name = "rectprism";
  if (texframe >= 0) tex_frame=texframe;
}

bool RectPrismV2::intersect(const VEC3 ray, const VEC3 start, float& t, bool& inside)
{
  float tmin = FLT_MAX;
  float t_tmp;
  bool inside_tmp;
  for (std::shared_ptr<Rectangle> face : faces)
  {
    bool hit = face->intersect(ray, start, t_tmp, inside_tmp);
    if (hit == true)
    {
      if (t_tmp < tmin)
      {
        tmin = t_tmp;
        inside = inside_tmp;
      }
    }
  }
  if (tmin < FLT_MAX)
  {
    t = tmin;
    return true;
  }
  return false;
}

bool RectPrismV2::intersectShadow(const VEC3 ray, const VEC3 start, const float t_max)
{
  for (std::shared_ptr<Rectangle> face : faces)
  {
    bool hit = face->intersectShadow(ray, start, t_max);
    if (hit == true)
    {
      // if (start[2] > 21)
      // {
      //   std::cout << "Point: " << start << std::endl;
      //   std::cout << "Ray: " << ray << std::endl;
      //   std::cout << "Occluding shape center: " << face->center << std::endl;
      //   std::cout << "A: " << face->A << std::endl;
      //   std::cout << "B: " << face->B << std::endl;
      //   std::cout << "C: " << face->C << std::endl;
      //   std::cout << "D: " << face->D << std::endl;
      // }
      return true;
    }
  }
  return false;
}

VEC3 RectPrismV2::getNorm(const VEC3 point)
{
  float eps = 1e-3;
  VEC3 normbot = -(F-E).cross(H-E).normalized();
  VEC3 normright = (E-A).cross(D-A).normalized();
  VEC3 normfront = (B-A).cross(E-A).normalized();

  float pa_bot = abs((point - A).normalized().dot(normbot));
  float pg_bot = abs((point - G).normalized().dot(normbot));
  if (pa_bot <= eps || pg_bot  <= eps)
  {
    return normbot;
  }

  float pa_right= abs((point - A).normalized().dot(normright));
  float pg_right = abs((point - G).normalized().dot(normright));
  if (pa_right <= eps || pg_right <= eps)
  {
    return normright;
  }

  float pa_front = abs((point - A).normalized().dot(normfront));
  float pg_front = abs((point - G).normalized().dot(normfront));
  if (pa_front <= eps || pg_front <= eps)
  {
    return normfront;
  }
  // If made it this far, then point is not on surface of prism
  printf("RectPrism getNorm: point is not on prism!\n");
  std::cout << "Point: " << point << std::endl;
  std::cout << "A: " << A << std::endl;
  std::cout << "B: " << B << std::endl;
  std::cout << "C: " << C << std::endl;
  std::cout << "D: " << D << std::endl;
  std::cout << "E: " << E << std::endl;
  std::cout << "F: " << F << std::endl;
  std::cout << "G: " << G << std::endl;
  std::cout << "H: " << H << std::endl;

  std::cout << "P-A botnorm check: " << abs((point - A).normalized().dot(normbot)) << std::endl;
  std::cout << "P-A normright check: " << abs((point - A).normalized().dot(normright)) << std::endl;
  std::cout << "P-A normfront check: " << abs((point - A).normalized().dot(normfront)) << std::endl;
  std::cout << "P-G botnorm check: " << abs((point - G).normalized().dot(normbot)) << std::endl;
  std::cout << "P-G normright check: " << abs((point - G).normalized().dot(normright)) << std::endl;
  std::cout << "P-G normfront check: " << abs((point - G).normalized().dot(normfront)) << std::endl;

  // Just return the closest one
  float min_side = std::min({pa_bot, pg_bot, pa_right, pg_right, pa_front, pg_front});
  if (pa_bot == min_side || pg_bot == min_side)
  {
    return normbot;
  }
  if (pa_right == min_side || pg_right == min_side)
  {
    return normright;
  }
  return normfront;
}

void RectPrismV2::getBounds(VEC3& lbound, VEC3& ubound)
{
  VEC3 min_tmp = A.array().min(B.array());
  min_tmp = min_tmp.array().min(C.array());
  min_tmp = min_tmp.array().min(D.array());
  min_tmp = min_tmp.array().min(E.array());
  min_tmp = min_tmp.array().min(F.array());
  min_tmp = min_tmp.array().min(G.array());
  min_tmp = min_tmp.array().min(H.array());
  lbound = min_tmp;

  VEC3 max_tmp = A.array().max(B.array());
  max_tmp = max_tmp.array().max(C.array());
  max_tmp = max_tmp.array().max(D.array());
  max_tmp = max_tmp.array().max(E.array());
  max_tmp = max_tmp.array().max(F.array());
  max_tmp = max_tmp.array().max(G.array());
  max_tmp = max_tmp.array().max(H.array());
  ubound = max_tmp;
}

VEC2 RectPrismV2::getUV(const VEC3 p, int& valid)
{
  // Get UV of TOP face!
  VEC2 uv = faces[0]->getUV(p, valid);
  return uv;
}

RectPrism::RectPrism(VEC3 a, VEC3 b, VEC3 c, VEC3 d, VEC3 e, VEC3 f, VEC3 g, VEC3 h,
          VEC3 col, std::string material, bool in_motion,
          int texframe, std::string shader)
{
  A = a;
  B = b;
  C = c;
  D = d;
  E = e;
  F = f;
  G = g;
  H = h;
  length = (b-a).norm();
  width = (d-a).norm();
  height = (e-a).norm();
  color = col;
  reflect_params.material = material;
  dims = 3;
  motion = in_motion;
  model = shader;
  center = (A + B + C + D + E + F + G + H)/8;
  name = "rectprism";
  if (texframe >= 0) tex_frame=texframe;

  // Pre-compute change of basis
  VEC3 w = (e-a).normalized();
  VEC3 u = (b-a).normalized();
  VEC3 v = (d-a).normalized();
  MATRIX4 cob; cob << u.transpose(), 0, v.transpose(), 0, w.transpose(), 0,
                      0, 0, 0, 1;
  MATRIX4 origin = MatrixXd::Identity(4,4);
  origin(0,3) = -center[0];
  origin(1,3) = -center[1];
  origin(2,3) = -center[2];
  objM = cob * origin;

  // Set bounds
  getObjBounds(lbound, ubound);
  getBounds(lbound, ubound);
}

bool RectPrism::intersect(const VEC3 ray, const VEC3 start, float& t, bool& inside)
{
  // Same as bounding volume
  float eps = 1e-6;
  VEC3 inv_ray = ray.cwiseInverse();
  float tmin;
  float tmax;
  inside = false;
  if (isinf(inv_ray[0]))
  {
   if (!(start[0] >= lbound[0] && start[0] <= ubound[0]))
   {
     return false;
   }
   tmin = FLT_MIN;
   tmax = FLT_MAX;
  }
  else if (ray[0] < 0)
  {
   tmin = (ubound[0] - start[0]) * inv_ray[0];
   tmax = (lbound[0] - start[0]) * inv_ray[0];
  }
  else
  {
   tmin = (lbound[0] - start[0]) * inv_ray[0];
   tmax = (ubound[0] - start[0]) * inv_ray[0];
  }
  float tymin;
  float tymax;
  if (isinf(inv_ray[1]))
  {
    if (!(start[1] >= lbound[1] && start[1] <= ubound[1]))
    {
      return false;
    }
   tymin = FLT_MIN;
   tymax = FLT_MAX;
  }
  else if (ray[1] < 0)
  {
   tymin = (ubound[1] - start[1]) * inv_ray[1];
   tymax = (lbound[1] - start[1]) * inv_ray[1];
  }
  else
  {
   tymin = (lbound[1] - start[1]) * inv_ray[1];
   tymax = (ubound[1] - start[1]) * inv_ray[1];
  }
  if (tmin > tymax || tymin > tmax)
  {
   return false;
  }
  if (tymin > tmin) tmin = tymin;
  if (tymax < tmax) tmax = tymax;

  float tzmin;
  float tzmax;
  if (isinf(inv_ray[2]))
  {
    if (!(start[2] >= lbound[2] && start[2] <= ubound[2]))
    {
      return false;
    }
   tzmin=FLT_MIN;
   tzmax=FLT_MAX;
  }
  else if (ray[2] < 0)
  {
   tzmin = (ubound[2] - start[2]) * inv_ray[2];
   tzmax = (lbound[2] - start[2]) * inv_ray[2];
  }
  else
  {
   tzmin = (lbound[2] - start[2]) * inv_ray[2];
   tzmax = (ubound[2] - start[2]) * inv_ray[2];
  }
  if (tmin > tzmax || tzmin > tmax)
  {
   return false;
  }
  if (tzmin > tmin) tmin = tzmin;
  if (tzmax < tmax) tmax = tzmax;
  // Note: need to rescale norms???
  if (tmin < eps && tmax > eps)
  {
    t = tmax * ray.norm()/ray.norm();
    inside = true;
    return true;
  }
  if (tmax <= eps)
  {
    return false;
  }
  t = tmin * ray.norm()/ray.norm();
  return true;
}

bool RectPrism::intersectShadow(const VEC3 ray, const VEC3 start, const float t_max)
{
  // Same as bounding volume intersection
  float eps = 1e-6;
  VEC3 inv_ray = ray.cwiseInverse();
  float tmin;
  float tmax;
  if (abs(ray[0]) < eps)
  {
    if (!(start[0] > lbound[0] && start[0] < ubound[0]))
    {
      return false;
    }
   tmin = FLT_MIN;
   tmax = FLT_MAX;
  }
  else if (ray[0] < 0)
  {
   tmin = (ubound[0] - start[0]) * inv_ray[0];
   tmax = (lbound[0] - start[0]) * inv_ray[0];
  }
  else
  {
   tmin = (lbound[0] - start[0]) * inv_ray[0];
   tmax = (ubound[0] - start[0]) * inv_ray[0];
  }
  float tymin;
  float tymax;
  if (abs(ray[1]) < eps)
  {
    if (!(start[1] > lbound[1] && start[1] < ubound[1]))
    {
      return false;
    }
   tymin = FLT_MIN;
   tymax = FLT_MAX;
  }
  else if (ray[1] < 0)
  {
   tymin = (ubound[1] - start[1]) * inv_ray[1];
   tymax = (lbound[1] - start[1]) * inv_ray[1];
  }
  else
  {
   tymin = (lbound[1] - start[1]) * inv_ray[1];
   tymax = (ubound[1] - start[1]) * inv_ray[1];
  }
  if (tmin > tymax || tymin > tmax)
  {
   return false;
  }
  if (tymin > tmin) tmin = tymin;
  if (tymax < tmax) tmax = tymax;

  float tzmin;
  float tzmax;
  if (abs(ray[2]) < eps)
  {
    if (!(start[2] > lbound[2] && start[2] < ubound[2]))
    {
      return false;
    }
   tzmin=FLT_MIN;
   tzmax=FLT_MAX;
  }
  else if (ray[2] < 0)
  {
   tzmin = (ubound[2] - start[2]) * inv_ray[2];
   tzmax = (lbound[2] - start[2]) * inv_ray[2];
  }
  else
  {
   tzmin = (lbound[2] - start[2]) * inv_ray[2];
   tzmax = (ubound[2] - start[2]) * inv_ray[2];
  }
  if (tmin > tzmax || tzmin > tmax)
  {
   return false;
  }
  if (tzmin > tmin) tmin = tzmin;
  if (tzmax < tmax) tmax = tzmax;

  if (tmin < eps && tmax > eps)
  {
    return (t_max > tmax);
  }
  if (tmax <= eps)
  {
    return false;
  }
  // Final check: bounding volume isn't behind and intersection isn't past boundary
  return (t_max > tmin);
}

bool RectPrism::intersectShadowBad(const VEC3 ray, const VEC3 start, const float t_max)
{
  // Same as bounding volume intersection
  VEC4 objray; objray << ray, 1;
  objray = objM * objray;
  VEC4 objstart; objstart << start, 1;
  objstart = objM * objstart;
  float eps = 1e-4;
  VEC4 inv_ray = objray.cwiseInverse();
  float tmin;
  float tmax;
  if (abs(ray[0]) < eps)
  {
    if (!(objstart[0] > lbound[0] && objstart[0] < ubound[0]))
    {
      return false;
    }
   tmin = FLT_MIN;
   tmax = FLT_MAX;
  }
  else if (ray[0] < 0)
  {
   tmin = (ubound[0] - objstart[0]) * inv_ray[0];
   tmax = (lbound[0] - objstart[0]) * inv_ray[0];
  }
  else
  {
   tmin = (lbound[0] - objstart[0]) * inv_ray[0];
   tmax = (ubound[0] - objstart[0]) * inv_ray[0];
  }
  float tymin;
  float tymax;
  if (abs(ray[1]) < eps)
  {
    if (!(objstart[1] > lbound[1] && objstart[1] < ubound[1]))
    {
      return false;
    }
   tymin = FLT_MIN;
   tymax = FLT_MAX;
  }
  else if (ray[1] < 0)
  {
   tymin = (ubound[1] - objstart[1]) * inv_ray[1];
   tymax = (lbound[1] - objstart[1]) * inv_ray[1];
  }
  else
  {
   tymin = (lbound[1] - objstart[1]) * inv_ray[1];
   tymax = (ubound[1] - objstart[1]) * inv_ray[1];
  }
  if (tmin > tymax || tymin > tmax)
  {
   return false;
  }
  if (tymin > tmin) tmin = tymin;
  if (tymax < tmax) tmax = tymax;

  float tzmin;
  float tzmax;
  if (abs(ray[2]) < eps)
  {
    if (!(objstart[2] > lbound[2] && objstart[2] < ubound[2]))
    {
      return false;
    }
   tzmin=FLT_MIN;
   tzmax=FLT_MAX;
  }
  else if (ray[2] < 0)
  {
   tzmin = (ubound[2] - objstart[2]) * inv_ray[2];
   tzmax = (lbound[2] - objstart[2]) * inv_ray[2];
  }
  else
  {
   tzmin = (lbound[2] - objstart[2]) * inv_ray[2];
   tzmax = (ubound[2] - objstart[2]) * inv_ray[2];
  }
  if (tmin > tzmax || tzmin > tmax)
  {
   return false;
  }
  if (tzmin > tmin) tmin = tzmin;
  if (tzmax < tmax) tmax = tzmax;

  if (tmin < eps && tmax > eps)
  {
    return (t_max > tmax);
  }
  if (tmax <= eps)
  {
    return false;
  }
  // Final check: bounding volume isn't behind and intersection isn't past boundary
  return (t_max > tmin);
}

VEC3 RectPrism::getNorm(const VEC3 point)
{
  // Use object coordinates again (gives us quadrants)
  // VEC4 p_transf; p_transf << point, 1;
  // VEC4 pObj = objM * p_transf;
  //
  // // std::cout << "Checking norm for point: " << point << std::endl;
  // // std::cout << "Obj coords: " << pObj << std::endl;
  // // std::cout << "Height: " << height << std::endl;
  // // std::cout << "Length: " << length << std::endl;
  // // std::cout << "Width: " << width << std::endl;
  //
  // // TODO: just take closest side based on ratio (-0.5 - 0.5)
  // float y = abs(pObj[1]/height);
  // float x = abs(pObj[0]/length);
  // float z = abs(pObj[2]/width);
  //
  // float tmp = std::min({x,y,z});
  // if (tmp == x)
  // {
  //   if (pObj[0] < 0)
  //   {
  //     return(-(E-A).cross(D-A).normalized());
  //   }
  //   return((E-A).cross(D-A).normalized());
  // }
  // if (tmp == y)
  // {
  //   if (pObj[0] < 0)
  //   {
  //     return(-(F-E).cross(H-E).normalized());
  //   }
  //   return((F-E).cross(H-E).normalized());
  // }
  // if (tmp == z)
  // {
  //   if (pObj[0] < 0)
  //   {
  //     return(-(B-A).cross(E-A).normalized());
  //   }
  //   return((B-A).cross(E-A).normalized());
  // }

  // CHECK DOT PRODUCT AGAINST CANDIDATE NORMALS
  // NOTE: WE DONT HAVE TO RETURN IN THE RIGHT DIRECTION BC THE RAY TRACER WILL ALWAYS FLIP PROPERLY
  float eps = 1e-3;
  VEC3 normbot = -(F-E).cross(H-E).normalized();
  VEC3 normright = (E-A).cross(D-A).normalized();
  VEC3 normfront = (B-A).cross(E-A).normalized();

  float pa_bot = abs((point - A).normalized().dot(normbot));
  float pg_bot = abs((point - G).normalized().dot(normbot));
  if (pa_bot <= eps || pg_bot  <= eps)
  {
    return normbot;
  }

  float pa_right= abs((point - A).normalized().dot(normright));
  float pg_right = abs((point - G).normalized().dot(normright));
  if (pa_right <= eps || pg_right <= eps)
  {
    return normright;
  }

  float pa_front = abs((point - A).normalized().dot(normfront));
  float pg_front = abs((point - G).normalized().dot(normfront));
  if (pa_front <= eps || pg_front <= eps)
  {
    return normfront;
  }
  // If made it this far, then point is not on surface of prism
  printf("RectPrism getNorm: point is not on prism!\n");
  std::cout << "Point: " << point << std::endl;
  std::cout << "A: " << A << std::endl;
  std::cout << "B: " << B << std::endl;
  std::cout << "C: " << C << std::endl;
  std::cout << "D: " << D << std::endl;
  std::cout << "E: " << E << std::endl;
  std::cout << "F: " << F << std::endl;
  std::cout << "G: " << G << std::endl;
  std::cout << "H: " << H << std::endl;

  std::cout << "P-A botnorm check: " << abs((point - A).normalized().dot(normbot)) << std::endl;
  std::cout << "P-A normright check: " << abs((point - A).normalized().dot(normright)) << std::endl;
  std::cout << "P-A normfront check: " << abs((point - A).normalized().dot(normfront)) << std::endl;
  std::cout << "P-G botnorm check: " << abs((point - G).normalized().dot(normbot)) << std::endl;
  std::cout << "P-G normright check: " << abs((point - G).normalized().dot(normright)) << std::endl;
  std::cout << "P-G normfront check: " << abs((point - G).normalized().dot(normfront)) << std::endl;

  // Just return the closest one
  float min_side = std::min({pa_bot, pg_bot, pa_right, pg_right, pa_front, pg_front});
  if (pa_bot == min_side || pg_bot == min_side)
  {
    return normbot;
  }
  if (pa_right == min_side || pg_right == min_side)
  {
    return normright;
  }
  return normfront;
}

void RectPrism::getBounds(VEC3& lbound, VEC3& ubound)
{
  VEC3 min_tmp = A.array().min(B.array());
  min_tmp = min_tmp.array().min(C.array());
  min_tmp = min_tmp.array().min(D.array());
  min_tmp = min_tmp.array().min(E.array());
  min_tmp = min_tmp.array().min(F.array());
  min_tmp = min_tmp.array().min(G.array());
  min_tmp = min_tmp.array().min(H.array());
  lbound = min_tmp;

  VEC3 max_tmp = A.array().max(B.array());
  max_tmp = max_tmp.array().max(C.array());
  max_tmp = max_tmp.array().max(D.array());
  max_tmp = max_tmp.array().max(E.array());
  max_tmp = max_tmp.array().max(F.array());
  max_tmp = max_tmp.array().max(G.array());
  max_tmp = max_tmp.array().max(H.array());
  ubound = max_tmp;
}

void RectPrism::getObjBounds(VEC3& lbound, VEC3& ubound)
{
  VEC4 objA; objA << A, 1;
  VEC4 objB; objB << B, 1;
  VEC4 objC; objC << C, 1;
  VEC4 objD; objD << D, 1;
  VEC4 objE; objE << E, 1;
  VEC4 objF; objF << F, 1;
  VEC4 objG; objG << G, 1;
  VEC4 objH; objH << H, 1;

  objA = objM * objA;
  objB = objM * objB;
  objC = objM * objC;
  objD = objM * objD;
  objE = objM * objE;
  objF = objM * objF;
  objG = objM * objG;
  objH = objM * objH;

  VEC4 min_tmp = objA.array().min(objB.array());
  min_tmp = min_tmp.array().min(objC.array());
  min_tmp = min_tmp.array().min(objD.array());
  min_tmp = min_tmp.array().min(objE.array());
  min_tmp = min_tmp.array().min(objF.array());
  min_tmp = min_tmp.array().min(objG.array());
  min_tmp = min_tmp.array().min(objH.array());
  lbound = min_tmp.head<3>();

  VEC4 max_tmp = objA.array().max(objB.array());
  max_tmp = max_tmp.array().max(objC.array());
  max_tmp = max_tmp.array().max(objD.array());
  max_tmp = max_tmp.array().max(objE.array());
  max_tmp = max_tmp.array().max(objF.array());
  max_tmp = max_tmp.array().max(objG.array());
  max_tmp = max_tmp.array().max(objH.array());
  ubound = max_tmp.head<3>();
}

VEC2 RectPrism::getUV(const VEC3 p, int& valid)
{
  // Use rectangle defined by ABCD
  VEC3 ad = D-A;
  VEC3 dc = C-D;

  if (abs(ad.cross(dc).dot(p)) <= 1e-5)
  {
    valid = 1;
    float u = (p-A).cross(ad).norm()/(ad.norm() * dc.norm()); // "X" distance
    float v = (p-D).cross(dc).norm()/(dc.norm() * ad.norm()); // "Y" distance
    return VEC2(u,v);
  }
  else
  {
    valid = 0;
    return VEC2(-1,-1);
  }

}

// ****************************
// ***** Derived Shapes ***************
// ****************************
//
RectPrismWithCylinder::RectPrismWithCylinder(VEC3 a, VEC3 b, VEC3 c, VEC3 d, VEC3 e, VEC3 f, VEC3 g, VEC3 h,
          VEC3 col, std::string material, bool in_motion,
          int texframe, std::string shader)
{
  A = a;
  B = b;
  C = c;
  D = d;
  E = e;
  F = f;
  G = g;
  H = h;
  length = (b-a).norm();
  width = (d-a).norm();
  height = (e-a).norm();
  color = col;
  reflect_params.material = material;
  dims = 3;
  motion = in_motion;
  model = shader;
  center = (A + B + C + D + E + F + G + H)/8;
  name = "RectPrismWithCylinder";
  if (texframe >= 0) tex_frame=texframe;

  // Set bounds
  getBounds(lbound, ubound);

  // Pre-compute change of basis
  VEC3 w = (e-a).normalized();
  VEC3 u = (b-a).normalized();
  VEC3 v = (d-a).normalized();
  MATRIX4 cob; cob << u.transpose(), 0, v.transpose(), 0, w.transpose(), 0,
                      0, 0, 0, 1;
  MATRIX4 origin = MatrixXd::Identity(4,4);
  origin(0,3) = -center[0];
  origin(1,3) = -center[1];
  origin(2,3) = -center[2];
  objM = cob * origin;
}

bool RectPrismWithCylinder::intersect(const VEC3 ray, const VEC3 start, float& t, bool& inside)
{
  // Same as bounding volume
  float eps = 1e-4;
  VEC3 inv_ray = ray.cwiseInverse();
  float tmin;
  float tmax;
  inside = false;
  if (abs(ray[0]) < eps)
  {
    if (!(start[0] >= lbound[0] && start[0] <= ubound[0]))
    {
      return false;
    }
   tmin = FLT_MIN;
   tmax = FLT_MAX;
  }
  else if (ray[0] < 0)
  {
   tmin = (ubound[0] - start[0]) * inv_ray[0];
   tmax = (lbound[0] - start[0]) * inv_ray[0];
  }
  else
  {
   tmin = (lbound[0] - start[0]) * inv_ray[0];
   tmax = (ubound[0] - start[0]) * inv_ray[0];
  }
  float tymin;
  float tymax;
  if (abs(ray[1]) < eps)
  {
    if (!(start[1] >= lbound[1] && start[1] <= ubound[1]))
    {
      return false;
    }
   tymin = FLT_MIN;
   tymax = FLT_MAX;
  }
  else if (ray[1] < 0)
  {
   tymin = (ubound[1] - start[1]) * inv_ray[1];
   tymax = (lbound[1] - start[1]) * inv_ray[1];
  }
  else
  {
   tymin = (lbound[1] - start[1]) * inv_ray[1];
   tymax = (ubound[1] - start[1]) * inv_ray[1];
  }
  if (tmin > tymax || tymin > tmax)
  {
   return false;
  }
  if (tymin > tmin) tmin = tymin;
  if (tymax < tmax) tmax = tymax;

  float tzmin;
  float tzmax;
  if (abs(ray[2]) < eps)
  {
    if (!(start[2] >= lbound[2] && start[2] <= ubound[2]))
    {
      return false;
    }
   tzmin=FLT_MIN;
   tzmax=FLT_MAX;
  }
  else if (ray[2] < 0)
  {
   tzmin = (ubound[2] - start[2]) * inv_ray[2];
   tzmax = (lbound[2] - start[2]) * inv_ray[2];
  }
  else
  {
   tzmin = (lbound[2] - start[2]) * inv_ray[2];
   tzmax = (ubound[2] - start[2]) * inv_ray[2];
  }
  if (tmin > tzmax || tzmin > tmax)
  {
   return false;
  }
  if (tzmin > tmin) tmin = tzmin;
  if (tzmax < tmax) tmax = tzmax;
  if (tmax <= eps)
  {
    return false;
  }
  if (tmin < eps && tmax > eps)
  {
    t = tmax;
    inside = true;
  }
  t = tmin;

  // Check intersection against cylinders
  float tcyl = FLT_MAX;
  bool inside_cyl;
  bool hit;
  bool cap_hit;
  std::shared_ptr<Cylinder> hit_cyl;
  for (int i = 0; i < holes.size(); i++)
  {
    std::shared_ptr<Cylinder> cyl = holes[i];
    float t_tmp;
    bool inside_tmp = false;
    bool hit_tmp = cyl->intersect(ray, start, t_tmp, inside_tmp);
    if (hit_tmp == true)
    {
      hit = true;
      if (t_tmp <= tcyl)
      {
        lastHit = i;
        hit_cyl = cyl;
        inside_cyl = inside_tmp;
        tcyl = t_tmp;
      }
    }
    hit_tmp = cyl->intersectCap(ray, start, t_tmp, inside_tmp);
    if (hit_tmp == true)
    {
      hit = true;
      if (t_tmp <= tcyl)
      {
        hit_cyl = cyl;
        cap_hit = true;
        inside_cyl = inside_tmp;
        tcyl = t_tmp;
      }
    }
  }
  if (hit == true)
  {
    if (tcyl <= t) // EVERYTHING FOR THIS
    {
      if (cap_hit == true)
      {
        return false;
      }
      inside = inside_cyl;
      t = tcyl;
      color = hit_cyl->color;
    }
  }
  lastHit = -1;
  return true;
}

bool RectPrismWithCylinder::intersectShadow(const VEC3 ray, const VEC3 start, const float t_max)
{
  // Same as bounding volume intersection
  float eps = 1e-4;
  VEC3 inv_ray = ray.cwiseInverse();
  float tmin;
  float tmax;
  // Edge cases: pretty close to parallel
  if (abs(ray[0]) < eps)
  {
    if (!(start[0] > lbound[0] && start[0] < ubound[0]))
    {
      return false;
    }
   tmin = FLT_MIN;
   tmax = FLT_MAX;
  }
  else if (ray[0] < 0)
  {
   tmin = (ubound[0] - start[0]) * inv_ray[0];
   tmax = (lbound[0] - start[0]) * inv_ray[0];
  }
  else
  {
   tmin = (lbound[0] - start[0]) * inv_ray[0];
   tmax = (ubound[0] - start[0]) * inv_ray[0];
  }
  float tymin;
  float tymax;
  if (abs(ray[1]) < eps)
  {
    if (!(start[1] > lbound[1] && start[1] < ubound[1]))
    {
      return false;
    }
   tymin = FLT_MIN;
   tymax = FLT_MAX;
  }
  else if (ray[1] < 0)
  {
   tymin = (ubound[1] - start[1]) * inv_ray[1];
   tymax = (lbound[1] - start[1]) * inv_ray[1];
  }
  else
  {
   tymin = (lbound[1] - start[1]) * inv_ray[1];
   tymax = (ubound[1] - start[1]) * inv_ray[1];
  }
  if (tmin > tymax || tymin > tmax)
  {
   return false;
  }
  if (tymin > tmin) tmin = tymin;
  if (tymax < tmax) tmax = tymax;

  float tzmin;
  float tzmax;
  if (abs(ray[2]) < eps)
  {
    if (!(start[2] > lbound[2] && start[2] < ubound[2]))
    {
      return false;
    }
   tzmin=FLT_MIN;
   tzmax=FLT_MAX;
  }
  else if (ray[2] < 0)
  {
   tzmin = (ubound[2] - start[2]) * inv_ray[2];
   tzmax = (lbound[2] - start[2]) * inv_ray[2];
  }
  else
  {
   tzmin = (lbound[2] - start[2]) * inv_ray[2];
   tzmax = (ubound[2] - start[2]) * inv_ray[2];
  }
  if (tmin > tzmax || tzmin > tmax)
  {
   return false;
  }
  if (tzmin > tmin) tmin = tzmin;
  if (tzmax < tmax) tmax = tzmax;
  float t;
  if (tmax <= eps)
  {
    return false;
  }
  if (tmin < eps && tmax > eps)
  {
    if (tmax >= t_max) return false;
    t = tmax;
  }
  t = tmin;

  // Check intersection against cylinders
  float tcyl = FLT_MAX;
  bool inside_cyl;
  bool hit;
  bool cap_hit;
  std::shared_ptr<Cylinder> hit_cyl;
  for (std::shared_ptr<Cylinder> cyl: holes)
  {
    float t_tmp = FLT_MIN;
    bool inside_tmp = false;
    bool hit_tmp = cyl->intersect(ray, start, t_tmp, inside_tmp);
    if (hit_tmp == true && t_tmp > eps && t_tmp < t_max)
    {
      hit = true;
      if (t_tmp <= tcyl)
      {
        hit_cyl = cyl;
        tcyl = t_tmp;
      }
    }
    hit_tmp = cyl->intersectCap(ray, start, t_tmp, inside_tmp);
    if (hit_tmp == true && t_tmp > eps && t_tmp < t_max)
    {
      hit = true;
      if (t_tmp <= tcyl)
      {
        hit_cyl = cyl;
        cap_hit = true;
        tcyl = t_tmp;
      }
    }
  }
  if (hit == true)
  {
    if (tcyl <= t && tcyl > eps && tcyl < t_max) // EVERYTHING FOR THIS
    {
      if (cap_hit == true)
      {
        return false;
      }
    }
  }
  return true;
}

VEC3 RectPrismWithCylinder::getNorm(const VEC3 point)
{
  // Check with last hit first
  if (lastHit >= 0)
  {
    return holes[lastHit]->getNorm(point);
  }
  // CHECK DOT PRODUCT AGAINST CANDIDATE NORMALS
  // NOTE: WE DONT HAVE TO RETURN IN THE RIGHT DIRECTION BC THE RAY TRACER WILL ALWAYS FLIP PROPERLY
  float eps = 1e-3;
  VEC3 normbot = -(F-E).cross(H-E).normalized();
  VEC3 normright = (E-A).cross(D-A).normalized();
  VEC3 normfront = (B-A).cross(E-A).normalized();

  if ((point - A).dot(normbot) <= eps)
  {
    return normbot;
  }
  if ((point - A).dot(normright) <= eps)
  {
    return normright;
  }
  if ((point - A).dot(normfront) <= eps)
  {
    return normfront;
  }
  // If made it this far, then point is not on surface of prism
  printf("RectPrismWithCylinder getNorm: point is not on prism!\n");
  throw "RectPrismWithCylinder getNorm: point is not on prism!";
}

RectPrismWithHoles::RectPrismWithHoles(VEC3 a, VEC3 b, VEC3 c, VEC3 d, VEC3 e, VEC3 f, VEC3 g, VEC3 h,
                                        VEC3 col, std::string material, bool in_motion,
                                        int texframe, std::string shader)
{
  A = a;
  B = b;
  C = c;
  D = d;
  E = e;
  F = f;
  G = g;
  H = h;
  length = (b-a).norm();
  width = (d-a).norm();
  height = (e-a).norm();
  color = col;
  reflect_params.material = material;
  dims = 3;
  motion = in_motion;
  model = shader;
  center = (A + B + C + D + E + F + G + H)/8;
  name = "rectprism";
  if (texframe >= 0) tex_frame=texframe;

  // Set bounds
  getBounds(lbound, ubound);

  // Pre-compute change of basis
  VEC3 w = (e-a).normalized();
  VEC3 u = (b-a).normalized();
  VEC3 v = (d-a).normalized();
  MATRIX4 cob; cob << u.transpose(), 0, v.transpose(), 0, w.transpose(), 0,
                      0, 0, 0, 1;
  MATRIX4 origin = MatrixXd::Identity(4,4);
  origin(0,3) = -center[0];
  origin(1,3) = -center[1];
  origin(2,3) = -center[2];
  objM = cob * origin;
}

void RectPrismWithHoles::getBounds(VEC3& lbound, VEC3& ubound)
{
  VEC3 min_tmp = A.array().min(B.array());
  min_tmp = min_tmp.array().min(C.array());
  min_tmp = min_tmp.array().min(D.array());
  min_tmp = min_tmp.array().min(E.array());
  min_tmp = min_tmp.array().min(F.array());
  min_tmp = min_tmp.array().min(G.array());
  min_tmp = min_tmp.array().min(H.array());
  lbound = min_tmp;

  VEC3 max_tmp = A.array().max(B.array());
  max_tmp = max_tmp.array().max(C.array());
  max_tmp = max_tmp.array().max(D.array());
  max_tmp = max_tmp.array().max(E.array());
  max_tmp = max_tmp.array().max(F.array());
  max_tmp = max_tmp.array().max(G.array());
  max_tmp = max_tmp.array().max(H.array());
  ubound = max_tmp;
}

bool RectPrismWithHoles::intersect(const VEC3 ray, const VEC3 start, float& t, bool& inside)
{
  lastHit = -1;
  float eps = 1e-4;
  // Standard intersect check
  // Same as bounding volume
  VEC3 inv_ray = ray.cwiseInverse();
  float tmin;
  float tmax;
  inside = false;
  if (isinf(inv_ray[0]))
  {
   tmin = FLT_MIN;
   tmax = FLT_MAX;
  }
  else if (ray[0] < 0)
  {
   tmin = (ubound[0] - start[0]) * inv_ray[0];
   tmax = (lbound[0] - start[0]) * inv_ray[0];
  }
  else
  {
   tmin = (lbound[0] - start[0]) * inv_ray[0];
   tmax = (ubound[0] - start[0]) * inv_ray[0];
  }
  float tymin;
  float tymax;
  if (isinf(inv_ray[1]))
  {
   tymin = FLT_MIN;
   tymax = FLT_MAX;
  }
  else if (ray[1] < 0)
  {
   tymin = (ubound[1] - start[1]) * inv_ray[1];
   tymax = (lbound[1] - start[1]) * inv_ray[1];
  }
  else
  {
   tymin = (lbound[1] - start[1]) * inv_ray[1];
   tymax = (ubound[1] - start[1]) * inv_ray[1];
  }
  if (tmin > tymax || tymin > tmax)
  {
   return false;
  }
  if (tymin > tmin) tmin = tymin;
  if (tymax < tmax) tmax = tymax;

  float tzmin;
  float tzmax;
  if (isinf(inv_ray[2]))
  {
   tzmin=FLT_MIN;
   tzmax=FLT_MAX;
  }
  else if (ray[2] < 0)
  {
   tzmin = (ubound[2] - start[2]) * inv_ray[2];
   tzmax = (lbound[2] - start[2]) * inv_ray[2];
  }
  else
  {
   tzmin = (lbound[2] - start[2]) * inv_ray[2];
   tzmax = (ubound[2] - start[2]) * inv_ray[2];
  }
  if (tmin > tzmax || tzmin > tmax)
  {
   return false;
  }
  if (tzmin > tmin) tmin = tzmin;
  if (tzmax < tmax) tmax = tzmax;

  if (tmin < eps && tmax > eps)
  {
    t = tmax;
    inside = true;
  }
  else
  {
    if (tmax <= eps)
    {
      return false;
    }
    t = tmin;
  }
  bool inside_tmp;
  float tmin_tmp = FLT_MAX;
  float t_tmp;
  std::shared_ptr<GeoPrimitive> hit_shape;
  // If inside, then check for regular min intersection
  if (inside == true)
  {
    for (int i = 0; i < holes.size(); i++)
    {
      std::shared_ptr<GeoPrimitive> hole = holes[i];
      bool hit = hole->intersect(ray, start, t_tmp, inside_tmp);
      if (hit == true && t_tmp <= tmin_tmp)
      {
        hit_shape = hole;
        tmin_tmp = t_tmp;
        lastHit = i;
      }
    }
    // If hit, then need to check that not caps
    if (lastHit != -1)
    {
      VEC3 checknorm = getNorm(ray + t * start);
      VEC3 shapenorm = hit_shape->getNorm(ray + tmin_tmp * start);
      if (checknorm.isApprox(shapenorm) || checknorm.isApprox(-shapenorm))
      {
        lastHit = -1;
        return false;
      }
      else if (tmin_tmp > t)
      {
        lastHit = -1;
        return true;
      }
      else
      {
        color = hit_shape->color;
        t = tmin_tmp;
        return true;
      }
    }
    return true;
  }
  else
  {
    for (int i = 0; i < holes.size(); i++)
    {
      std::shared_ptr<GeoPrimitive> hole = holes[i];
      bool hit = hole->intersectMax(ray, start, t_tmp);
      // Keep lowest double intersection
      if (hit == true && t_tmp < tmin_tmp)
      {
        hit_shape = hole;
        tmin_tmp = t_tmp;
        lastHit = i;
      }
    }
    // If hit, then need to check that not caps
    if (lastHit != -1)
    {
      VEC3 checknorm = getNorm(ray + t * start);
      VEC3 shapenorm = hit_shape->getNorm(ray + tmin_tmp * start);
      if (checknorm.isApprox(shapenorm) || checknorm.isApprox(-shapenorm))
      {
        lastHit = -1;
        return false;
      }
      else
      {
        color = hit_shape->color;
        t = tmin_tmp;
        return true;
      }
    }
    return true;
  }
}

bool RectPrismWithHoles::intersectShadow(const VEC3 ray, const VEC3 start, const float t_max)
{
  lastHit = -1;
  float eps = 1e-4;
  // Standard intersect check
  // Same as bounding volume
  VEC3 inv_ray = ray.cwiseInverse();
  float tmin;
  float tmax;
  float t;
  bool inside = false;
  if (isinf(inv_ray[0]))
  {
   tmin = FLT_MIN;
   tmax = FLT_MAX;
  }
  else if (ray[0] < 0)
  {
   tmin = (ubound[0] - start[0]) * inv_ray[0];
   tmax = (lbound[0] - start[0]) * inv_ray[0];
  }
  else
  {
   tmin = (lbound[0] - start[0]) * inv_ray[0];
   tmax = (ubound[0] - start[0]) * inv_ray[0];
  }
  float tymin;
  float tymax;
  if (isinf(inv_ray[1]))
  {
   tymin = FLT_MIN;
   tymax = FLT_MAX;
  }
  else if (ray[1] < 0)
  {
   tymin = (ubound[1] - start[1]) * inv_ray[1];
   tymax = (lbound[1] - start[1]) * inv_ray[1];
  }
  else
  {
   tymin = (lbound[1] - start[1]) * inv_ray[1];
   tymax = (ubound[1] - start[1]) * inv_ray[1];
  }
  if (tmin > tymax || tymin > tmax)
  {
   return false;
  }
  if (tymin > tmin) tmin = tymin;
  if (tymax < tmax) tmax = tymax;

  float tzmin;
  float tzmax;
  if (isinf(inv_ray[2]))
  {
   tzmin=FLT_MIN;
   tzmax=FLT_MAX;
  }
  else if (ray[2] < 0)
  {
   tzmin = (ubound[2] - start[2]) * inv_ray[2];
   tzmax = (lbound[2] - start[2]) * inv_ray[2];
  }
  else
  {
   tzmin = (lbound[2] - start[2]) * inv_ray[2];
   tzmax = (ubound[2] - start[2]) * inv_ray[2];
  }
  if (tmin > tzmax || tzmin > tmax)
  {
   return false;
  }
  if (tzmin > tmin) tmin = tzmin;
  if (tzmax < tmax) tmax = tzmax;

  if (tmin < eps && tmax > eps)
  {
    if (tmax >= t_max)
    {
      return false;
    }
    t = tmax;
    inside = true;
  }
  else
  {
    if (tmax <= eps || tmin >= t_max)
    {
      return false;
    }
    t = tmin;
  }
  bool inside_tmp;
  float tmin_tmp = FLT_MAX;
  float t_tmp;
  std::shared_ptr<GeoPrimitive> hit_shape;
  bool ever_hit = false;
  // If inside, then check for regular min intersection
  if (inside == true)
  {
    for (int i = 0; i < holes.size(); i++)
    {
      std::shared_ptr<GeoPrimitive> hole = holes[i];
      bool hit = hole->intersect(ray, start, t_tmp, inside_tmp);
      if (hit == true && t_tmp <= tmin_tmp)
      {
        hit_shape = hole;
        tmin_tmp = t_tmp;
        ever_hit = true;
      }
    }
    // If hit, then need to check that not caps
    if (ever_hit == true)
    {
      VEC3 checknorm = getNorm(ray + t * start);
      VEC3 shapenorm = hit_shape->getNorm(ray + tmin_tmp * start);
      if (checknorm.isApprox(shapenorm) || checknorm.isApprox(-shapenorm))
      {
        return false;
      }
      else
      {
        return true;
      }
    }
    return true;
  }
  else
  {
    for (int i = 0; i < holes.size(); i++)
    {
      std::shared_ptr<GeoPrimitive> hole = holes[i];
      bool hit = hole->intersectMax(ray, start, t_tmp);
      // Keep lowest double intersection
      if (hit == true && t_tmp < tmin_tmp)
      {
        hit_shape = hole;
        tmin_tmp = t_tmp;
        ever_hit = true;
      }
    }
    // If hit, then need to check that not caps
    if (ever_hit == true)
    {
      VEC3 checknorm = getNorm(ray + t * start);
      VEC3 shapenorm = hit_shape->getNorm(ray + tmin_tmp * start);
      if (checknorm.isApprox(shapenorm) || checknorm.isApprox(-shapenorm))
      {
        return false;
      }
      else
      {
        return true;
      }
    }
    return true;
  }
}

VEC3 RectPrismWithHoles::getNorm(const VEC3 point)
{
  if (lastHit != -1)
  {
    return(holes[lastHit]->getNorm(point));
  }
  else
  {
    // Use object coordinates again (gives us quadrants)
    VEC4 p_transf; p_transf << point, 1;
    VEC4 pObj = objM * p_transf;

    float eps = 1e-3;
    if (abs(pObj[1] - height/2) <= eps)
    {
      if (pObj[1] < 0)
      {
        return(-(F-E).cross(H-E).normalized());
      }
      return((F-E).cross(H-E).normalized());
    }
    if (abs(pObj[0] - length/2) <= eps)
    {
      if (pObj[0] < 0)
      {
        return(-(E-A).cross(D-A).normalized());
      }
      return((E-A).cross(D-A).normalized());
    }
    if (abs(pObj[2] - width/2) <= eps)
    {
      if (pObj[2] < 0)
      {
        return(-(B-A).cross(E-A).normalized());
      }
      return((B-A).cross(E-A).normalized());
    }
    // If made it this far, then point is not on surface of prism
    printf("RectPrismWithHoles getNorm: point is not on prism!\n");
    throw "RectPrismWithHoles getNorm: point is not on prism!";
  }
}

Checkerboard::Checkerboard(VEC3 a, VEC3 b, VEC3 c, VEC3 d, VEC3 col1, VEC3 col2, float S_square,
                            std::string material, bool in_motion, std::string shader)
{
 A = a;
 B = b;
 C = c;
 D = d;
 length = (b-a).norm();
 width = (d-a).norm();
 color = col1;
 color1 = col1;
 color2 = col2;
 S = S_square;
 dims = 2;
 motion = in_motion;
 model = shader;
 reflect_params.material = material;
 center = (A + B + C + D)/4;
 name="checkerboard";
}

bool Checkerboard::intersect(const VEC3 ray, const VEC3 start, float& t, bool& inside)
{
 float eps = 1e-3;
 inside = false;
 // Check for EDGE INTERSECTIONS: whenever viewing ray is perpendicular to normal
 if (getNorm(start).dot(ray) == 0)
 {
  float t_tmp;
  bool hit = false;
  hit = segmentIntersect(A, B, ray, start, t_tmp);
  if (hit == true) {return true;}

  hit = segmentIntersect(A, D, ray, start, t_tmp);
  if (hit == true) {return true;}

  hit = segmentIntersect(B, C, ray, start, t_tmp);
  if (hit == true) {return true;}

  hit = segmentIntersect(C, D, ray, start, t_tmp);
  if (hit == true) {return true;}

  return false;
 }
 // Hardcode solution
 VEC3 norm = getNorm(start).normalized();
 float dn = ray.dot(norm); // Parallel edge case: should be taken care of with above but just in case
 if (dn == 0)
 {
  return false;
 }
 float t_final = (A - start).dot(norm)/dn;
 if (t_final <= eps)
 {
  return false;
 }
 VEC3 point = start + t_final * ray;
 VEC3 V_hit = point - A;
 VEC3 V1 = B-A;
 VEC3 V2 = D-A;
 float check1 = V1.normalized().dot(V_hit);
 float check2 = V2.normalized().dot(V_hit);
 if (0 <= check1 && check1 <= V1.norm() && 0 <= check2 && check2 <= V2.norm())
 {
  t = t_final;
  // Change color based on hit point
  int i = check1 / S;
  int j = check2 / S;
  if (i % 2 == 0)
  {
   if (j % 2 == 0)
   {
    color = color1;
   }
   if (j % 2 == 1)
   {
    color = color2;
   }
  }
  if (i % 2 == 1)
  {
   if (j % 2 == 0)
   {
    color = color2;
   }
   if (j % 2 == 1)
   {
    color = color1;
   }
  }
  return true;
 }
 return false;
}


CheckerboardWithHole::CheckerboardWithHole(VEC3 a, VEC3 b, VEC3 c, VEC3 d, VEC3 col1, VEC3 col2, float S_square,
                              std::shared_ptr<Rectangle> shape, std::string material, bool in_motion, std::string shader)
{
 A = a;
 B = b;
 C = c;
 D = d;
 length = (b-a).norm();
 width = (d-a).norm();
 color = col1;
 color1 = col1;
 color2 = col2;
 S = S_square;
 reflect_params.material = material;
 dims = 2;
 motion = in_motion;
 model = shader;
 hole = shape;
 center = (A + B + C + D)/4;
 name="checkboardhole";
}

bool CheckerboardWithHole::intersect(const VEC3 ray, const VEC3 start, float& t, bool& inside)
{
 float eps = 1e-3;
 inside = false;
 // Check for EDGE INTERSECTIONS: whenever viewing ray is perpendicular to normal
 if (getNorm(start).dot(ray) == 0)
 {
  float t_tmp;
  bool hit = false;
  hit = segmentIntersect(A, B, ray, start, t_tmp);
  if (hit == true) {return true;}

  hit = segmentIntersect(A, D, ray, start, t_tmp);
  if (hit == true) {return true;}

  hit = segmentIntersect(B, C, ray, start, t_tmp);
  if (hit == true) {return true;}

  hit = segmentIntersect(C, D, ray, start, t_tmp);
  if (hit == true) {return true;}
  return false;
 }
 // Hardcode solution
 VEC3 norm = getNorm(start).normalized();
 float dn = ray.dot(norm); // Parallel edge case: should be taken care of with above but just in case
 if (dn == 0)
 {
  return false;
 }
 float t_final = (A - start).dot(norm)/dn;
 if (t_final <= eps)
 {
  return false;
 }
 VEC3 point = start + t_final * ray;
 VEC3 V_hit = point - A;
 VEC3 V1 = B-A;
 VEC3 V2 = D-A;
 float check1 = V1.normalized().dot(V_hit);
 float check2 = V2.normalized().dot(V_hit);
 if (0 <= check1 && check1 <= V1.norm() && 0 <= check2 && check2 <= V2.norm())
 {
  // Check against hole boundaries
  float t_tmp;
  bool holecheck = hole->intersect(ray, start, t_tmp, inside);
  if (holecheck == true)
  {
   return false;
  }
  t = t_final;
  // Change color based on hit point
  int i = check1 / S;
  int j = check2 / S;
  if (i % 2 == 0)
  {
   if (j % 2 == 0)
   {
    color = color1;
   }
   if (j % 2 == 1)
   {
    color = color2;
   }
  }
  if (i % 2 == 1)
  {
   if (j % 2 == 0)
   {
    color = color2;
   }
   if (j % 2 == 1)
   {
    color = color1;
   }
  }
  return true;
 }
 return false;
}

bool CheckerboardWithHole::intersectShadow(const VEC3 ray, const VEC3 start, const float t_max)
{
 // Check for EDGE INTERSECTIONS: whenever viewing ray is perpendicular to normal
 // if (getNorm(start).dot(ray) == 0)
 // {
 //  float t_tmp;
 //  bool hit = false;
 //  hit = segmentIntersect(A, B, ray, start, t_tmp);
 //  if (hit == true) {return true;}
 //
 //  hit = segmentIntersect(A, D, ray, start, t_tmp);
 //  if (hit == true) {return true;}
 //
 //  hit = segmentIntersect(B, C, ray, start, t_tmp);
 //  if (hit == true) {return true;}
 //
 //  hit = segmentIntersect(C, D, ray, start, t_tmp);
 //  if (hit == true) {return true;}
 //
 //  return false;
 // }

 float eps = 1e-3;
 // Hardcode solution
 VEC3 norm = getNorm(start).normalized();
 float dn = ray.dot(norm); // Parallel edge case: should be taken care of with above but just in case
 if (dn == 0)
 {
  return false;
 }
 float t_final = (A - start).dot(norm)/dn;
 if (t_final <= eps)
 {
  return false;
 }
 VEC3 point = start + t_final * ray;
 VEC3 V_hit = point - A;
 VEC3 V1 = B-A;
 VEC3 V2 = D-A;
 float check1 = V1.normalized().dot(V_hit);
 float check2 = V2.normalized().dot(V_hit);
 if (0 <= check1 && check1 <= V1.norm() && 0 <= check2 && check2 <= V2.norm() && t_final < t_max)
 {
  // Check against hole boundaries
  bool holecheck = hole->intersectShadow(ray, start, t_max);
  if (holecheck == true)
  {
   return false;
  }
  return true;
 }
 return false;
}

VEC2 CheckerboardWithHole::getUV(const VEC3 p, int& valid)
{
  // Edge case: point is outside of bounds or in hole
  VEC3 V_hit = p - A;
  VEC3 V1 = B-A;
  VEC3 V2 = D-A;
  float check1 = V1.normalized().dot(V_hit);
  float check2 = V2.normalized().dot(V_hit);
  if (0 <= check1 && check1 <= V1.norm() && 0 <= check2 && check2 <= V2.norm())
  {
   // Check against hole boundaries
   bool holecheck = hole->intersectShadow(VEC3(1,1,1), p - VEC3(1,1,1), FLT_MAX);
   if (holecheck == true)
   {
    valid = 0;
    return VEC2(-1,-1);
   }
   // Identify bounding square of point: find global UV first
   VEC3 ad = D-A;
   VEC3 dc = C-D;
   float u = (p-A).cross(ad).norm()/(ad.norm() * dc.norm()); // "X" distance
   float v = (p-D).cross(dc).norm()/(dc.norm() * ad.norm()); // "Y" distance

   // NOTE: texture inds start from UPPER LEFT
   // Convert to mini UV by computing mini UV distance in terms of global UV
   float miniu_dist = S/length;
   float miniv_dist = S/width;

   float miniu = u/miniu_dist - int(u/miniu_dist);
   float miniv = v/miniv_dist - int(v/miniv_dist);

   // Clamp below
   if (miniu < 0) miniu=0;
   if (miniv < 0) miniv=0;


   // Check that UV is actually valid
   // if (miniu < 0 || miniu > 1 || miniv < 0 || miniv > 1)
   // {
   //   printf("UV BAD: %f, %f\n", miniu, miniv);
   //   throw;
   // }

   // Check whether in border: +- borderwidth/2*S
   if ((miniu <= borderwidth/(2*S) || miniu >= 1-borderwidth/(2*S)) ||
          (miniv <= borderwidth/(2*S) || miniv >= 1-borderwidth/(2*S)))
   {
     valid = 2;
     return VEC2(miniu, miniv);
   }
   else
   {
     valid = 1;
     return VEC2(miniu, miniv);
   }
  }
  else
  {
    valid = 0;
    return VEC2(-1,-1);
  }
}

CheckerCylinder::CheckerCylinder(VEC3 v1, VEC3 v2, float r, VEC3 col, float s,
                                  std::string material, bool in_motion, std::string shader)
{
 c1 = v1;
 c2 = v2;
 axis = (v2 - v1).normalized();
 radius = r;
 color = col;
 reflect_params.material = material;
 dims = 3;
 motion = in_motion;
 model = shader;
 S = s;
 center = (v1 + v2)/2;
 name = "checkercylinder";

 // Object matrix
 MATRIX4 cob = buildCOB(axis);
 MATRIX4 origin = MatrixXd::Identity(4,4);
 origin(0,3) = -c1[0];
 origin(1,3) = -c1[1];
 origin(2,3) = -c1[2];
 objM = cob * origin;
}

VEC2 CheckerCylinder::getUV(const VEC3 p, int& valid)
{
  // Convert p to cylindrical coordinates: requires rotating cylinder to origin
  // SAME procedure as Mcam except now we send c0 to origin, and axis to positive z-axis!
  VEC4 p_transf; p_transf << p, 1;
  p_transf = objM * p_transf;
  VEC3 p_obj = p_transf.head<3>(); // Object coordinates
  // Cylindrical coordinates == UV
  float u = 0;
  if (p[0] != 0)
  {
    u = (atan2(p_obj[1], p_obj[0]) + M_PI)/(2 * M_PI);
  }
  float v = p_obj[2]/axis.norm();

  // Convert to mini UV by computing mini UV distance in terms of global UV
  float miniu_dist = S/(2 * M_PI * radius);
  float miniv_dist = S/axis.norm();

  float miniu = u/miniu_dist - int(u/miniu_dist);
  float miniv = v/miniv_dist - int(v/miniv_dist);

  if (miniu > 1 || miniu < 0 || miniv > 1 || miniv < 0)
  {
    printf("CheckerCylinder getUV: UV out of bounds. %f, %f\n", miniu, miniv);
    throw;
  }

  // Check whether in border: +- borderwidth/2*S
  if ((miniu <= borderwidth/(2*S) || miniu >= 1-borderwidth/(2*S)) ||
        (miniv <= borderwidth/(2*S) || miniv >= 1-borderwidth/(2*S)))
  {
    // printf("UV in border: %f, %f\n", miniu, miniv);
    // printf("Global UV: %f, %f\n", u, v);
    valid = 2;
    return VEC2(miniu, miniv);
  }
  else
  {
   valid = 1;
   return VEC2(miniu, miniv);
  }
}

BoundingVolume::BoundingVolume(std::vector<int> inds, std::vector<std::shared_ptr<GeoPrimitive>> shapes, bool is_leaf)
{
 indices = inds;
 leaf = is_leaf;
 lbound = VEC3(FLT_MAX, FLT_MAX, FLT_MAX);
 ubound = VEC3(FLT_MIN, FLT_MIN, FLT_MIN);
 setBounds(shapes);
}

// Set boundary values for box
void BoundingVolume::setBounds(std::vector<std::shared_ptr<GeoPrimitive>> shapes)
{
 for (int ind : indices)
 {
  VEC3 lbound_tmp;
  VEC3 ubound_tmp;
  shapes[ind]->getBounds(lbound_tmp, ubound_tmp);
  lbound = lbound.array().min(lbound_tmp.array());
  ubound = ubound.array().max(ubound_tmp.array());
 }
 // Always adjust slightly outside the primitive
 lbound -= VEC3(1e-2, 1e-2, 1e-2);
 ubound += VEC3(1e-2, 1e-2, 1e-2);
}

bool BoundingVolume::intersect(const VEC3 ray, const VEC3 inv_ray, const VEC3 start)
{
 // Note: divide by zero is OKAY
 // intersections: check for negative directions
 float tmin;
 float tmax;
 if (isinf(inv_ray[0]))
 {
   if (!(start[0] >= lbound[0] && start[0] <= ubound[0]))
   {
     return false;
   }
  tmin = FLT_MIN;
  tmax = FLT_MAX;
 }
 else if (ray[0] < 0)
 {
  tmin = (ubound[0] - start[0]) * inv_ray[0];
  tmax = (lbound[0] - start[0]) * inv_ray[0];
 }
 else
 {
  tmin = (lbound[0] - start[0]) * inv_ray[0];
  tmax = (ubound[0] - start[0]) * inv_ray[0];
 }
 float tymin;
 float tymax;
 if (isinf(inv_ray[1]))
 {
   if (!(start[1] >= lbound[1] && start[1] <= ubound[1]))
   {
     return false;
   }
  tymin = FLT_MIN;
  tymax = FLT_MAX;
 }
 else if (ray[1] < 0)
 {
  tymin = (ubound[1] - start[1]) * inv_ray[1];
  tymax = (lbound[1] - start[1]) * inv_ray[1];
 }
 else
 {
  tymin = (lbound[1] - start[1]) * inv_ray[1];
  tymax = (ubound[1] - start[1]) * inv_ray[1];
 }
 if (tmin > tymax || tymin > tmax)
 {
  return false;
 }
 if (tymin > tmin) tmin = tymin;
 if (tymax < tmax) tmax = tymax;

 float tzmin;
 float tzmax;
 if (isinf(inv_ray[2]))
 {
   if (!(start[2] >= lbound[2] && start[2] <= ubound[2]))
   {
     return false;
   }
  tzmin=FLT_MIN;
  tzmax=FLT_MAX;
 }
 else if (ray[2] < 0)
 {
  tzmin = (ubound[2] - start[2]) * inv_ray[2];
  tzmax = (lbound[2] - start[2]) * inv_ray[2];
 }
 else
 {
  tzmin = (lbound[2] - start[2]) * inv_ray[2];
  tzmax = (ubound[2] - start[2]) * inv_ray[2];
 }
 if (tmin > tzmax || tzmin > tmax)
 {
  return false;
 }
 if (tzmin > tmin) tmin = tzmin;
 if (tzmax < tmax) tmax = tzmax;

 // Final check: bounding volume isn't behind
 return (tmax > 0);
}

// ****************************
// ***** LIGHTS ***************
// ****************************
pointLight::pointLight(const VEC3 c, const VEC3 col)
{
 center = c;
 color = col;
}

VEC3 pointLight::sampleRay(const VEC3 point)
{
 return(center-point);
}

sphereLight::sphereLight(const VEC3 c, const float r, const VEC3 col, std::string material, bool in_motion)
{
 // BAD DESIGN: keep for now just to see if shit works
 light = true;
 Sphere::center = c;
 LightPrimitive::center = c;
 Sphere::color = col;
 LightPrimitive::color = col;
 radius = r;
 reflect_params.material = material;
 motion = in_motion;
 name = "spherelight";
}

VEC3 sphereLight::sampleRay(const VEC3 point)
{
 std::random_device rd;
 std::mt19937 generator(rd());
 std::uniform_real_distribution<double> uniform(0.0, 1.0);

 double theta = 2 * M_PI * uniform(generator);
 double phi = acos(1 - 2 * uniform(generator));
 VEC3 tmp = radius * VEC3(sin(phi) * cos(theta), sin(phi) * sin(theta), cos(phi)) + Sphere::center;
 // baxis adds an additional condition
 int sample_limit = 20;
 if (baxis.isApprox(VEC3(0,0,0)))
 {
   while ((tmp - Sphere::center).dot(point - Sphere::center) < 0)
   {
    if (sample_limit < 0)
    {
      printf("spherelight sampleRay: too many missed samples!\n");
      throw "spherelight sampleRay: too many missed samples!";
    }
    // Check mirror image of point
    VEC3 rev_tmp = -radius * VEC3(sin(phi) * cos(theta), sin(phi) * sin(theta), cos(phi)) + Sphere::center;
    if ((rev_tmp - Sphere::center).dot(point - Sphere::center) >= 0)
    {
      tmp = rev_tmp;
      break;
    }
    theta = 2 * M_PI * uniform(generator);
    phi = acos(1 - 2 * uniform(generator));
    tmp = radius * VEC3(sin(phi) * cos(theta), sin(phi) * sin(theta), cos(phi)) + Sphere::center;
    sample_limit--;
   }
 }
 else
 {
   while ((tmp - Sphere::center).dot(point - Sphere::center) < 0 || (tmp - Sphere::center).dot(baxis) < 0)
   {
    if (sample_limit < 0)
    {
      printf("spherelight sampleRay: too many missed samples!\n");
      throw "spherelight sampleRay: too many missed samples!";
    }
    // Check mirror image of point
    VEC3 rev_tmp = -radius * VEC3(sin(phi) * cos(theta), sin(phi) * sin(theta), cos(phi)) + Sphere::center;
    if ((rev_tmp - Sphere::center).dot(point - Sphere::center) >= 0 && (rev_tmp - Sphere::center).dot(baxis) >= 0)
    {
      tmp = rev_tmp;
      break;
    }
    theta = 2 * M_PI * uniform(generator);
    phi = acos(1 - 2 * uniform(generator));
    tmp = radius * VEC3(sin(phi) * cos(theta), sin(phi) * sin(theta), cos(phi)) + Sphere::center;
    sample_limit--;
   }
 }
 return(tmp);
}

rectangleLight::rectangleLight(VEC3 a, VEC3 b, VEC3 c, VEC3 d, VEC3 col, std::string material, bool in_motion)
{
 // BAD DESIGN: keep for now just to see if shit works
 light = true;
 A = a;
 B = b;
 C = c;
 D = d;
 Rectangle::center = (a + b + c + d)/4;
 LightPrimitive::center = (a + b + c + d)/4;
 Rectangle::color = col;
 LightPrimitive::color = col;
 reflect_params.material = material;
 motion = in_motion;
 name = "rectanglelight";
}

VEC3 rectangleLight::sampleRay(const VEC3 point)
{
 VEC3 sample = samplePoint();
 return(sample-point);
}
