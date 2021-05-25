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
#include "video.h"

Video::Video()
{
}

Video::Video(std::string vid_path)
{
  loadVideo(vid_path);
}

void Video::loadVideo(std::string vid_path)
{
  path = vid_path;
  std::vector<fs::path> f_frames = get_all(vid_path, ".jpg");
  int width, height, n;
  for (fs::path f : f_frames)
  {
    unsigned char* frame_data = stbi_load(f.string().c_str(), &width, &height, &n, 0);
    // Load frame data into vector
    std::vector<VEC3> frame_tmp;
    for (int i = 0; i < width*height; i++)
    {
      // Note: STBI data likely in 0-255 range -- convert back
      frame_tmp.push_back(VEC3(frame_data[i*n], frame_data[i*n + 1], frame_data[i*n + 2])/255.0);
    }
    frames.push_back(frame_tmp);
  }
}
