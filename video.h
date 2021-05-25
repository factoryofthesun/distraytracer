#ifndef VIDEOS
#define VIDEOS

#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "Eigen/Geometry"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <iostream>
#include <float.h>
#include "helpers.h"

class Video {
public:
  ~Video(){}
  Video();
  Video(std::string vid_path);
  void loadVideo(std::string vid_path);
  std::vector<std::vector<VEC3>> frames;
private:
  std::string path;
};



#endif
