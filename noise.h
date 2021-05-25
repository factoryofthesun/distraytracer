// Helper functions for 3D, 3D, and 4D (time-based) noise

#include <cstdio>
#include <cmath>
#include <cstdlib>
using namespace std;

#define MAX_PRIME 10
int primeIndex = 0;
int denom = 1073741823;

int primes[MAX_PRIME][3] = {
  { 995615039, 600173719, 701464987 },
  { 831731269, 162318869, 136250887 },
  { 174329291, 946737083, 245679977 },
  { 362489573, 795918041, 350777237 },
  { 457025711, 880830799, 909678923 },
  { 787070341, 177340217, 593320781 },
  { 405493717, 291031019, 391950901 },
  { 458904767, 676625681, 424452397 },
  { 531736441, 939683957, 810651871 },
  { 997169939, 842027887, 423882827 }
};

double cosInterpolate(double a, double b, double x) {  // cosine interpolation
  double angle = x * M_PI;
  double f = (1 - cos(angle)) * 0.5;
  return  a*(1-f) + b*f;
}

double Noise3D(int i, int x, int y, int z) {
  int n = x + y * 57 + z * pow(57,2);
  n = (n << 13) ^ n;
  int a = primes[i][0];
  int b = primes[i][1];
  int c = primes[i][2];
  int t = (n * (n*n*a + b)+c) & 0x7fffffff;
  return 1.0 - (double)(t)/denom;
}

double Noise2D(int i, int x, int y) {
  int n = x + y * 57;
  n = (n << 13) ^ n;
  int a = primes[i][0];
  int b = primes[i][1];
  int c = primes[i][2];
  int t = (n * (n*n*a + b)+c) & 0x7fffffff;
  return 1.0 - (double)(t)/denom;
}

double Smoothed3D(int i, int x, int y, int z) {
  double alpha = 9.0/18;
  double beta = 2.0/(8 * 18);
  double gamma = 4.0/(6 * 18);
  double delta = 3.0/(12 * 18);

  double corners = Noise3D(i, x-1, y-1, z-1) + Noise3D(i, x+1, y-1, z-1) +
                   Noise3D(i, x-1, y+1, z-1) + Noise3D(i, x+1, y+1, z-1) +
                   Noise3D(i, x-1, y-1, z+1) + Noise3D(i, x+1, y-1, z+1) +
                   Noise3D(i, x-1, y+1, z+1) + Noise3D(i, x+1, y+1, z+1);
  double sides = Noise3D(i, x-1, y, z) + Noise3D(i, x+1, y, z) + Noise3D(i, x, y+1, z) +
                  Noise3D(i, x, y-1, z) + Noise3D(i, x, y, z-1) + Noise3D(i, x, y, z+1);
  double dgsides = Noise3D(i, x-1, y, z-1) + Noise3D(i, x+1, y, z-1) +
                   Noise3D(i, x, y+1, z-1) + Noise3D(i, x, y-1, z-1) +
                   Noise3D(i, x-1, y, z+1) + Noise3D(i, x+1, y, z+1) +
                   Noise3D(i, x, y+1, z+1) + Noise3D(i, x-1, y-1, z) + Noise3D(i, x, y-1, z+1) +
                   Noise3D(i, x+1, y-1, z) + Noise3D(i, x-1, y+1, z) + Noise3D(i, x+1, y+1, z);
  double center = Noise3D(i, x, y, z);
  return alpha * center + beta * corners + gamma * sides + delta * dgsides;
}

double Smoothed2D(int i, int x, int y) {
  double corners = (Noise2D(i, x-1, y-1) + Noise2D(i, x+1, y-1) +
                    Noise2D(i, x-1, y+1) + Noise2D(i, x+1, y+1)) / 16,
         sides = (Noise2D(i, x-1, y) + Noise2D(i, x+1, y) + Noise2D(i, x, y-1) +
                  Noise2D(i, x, y+1)) / 8,
         center = Noise2D(i, x, y) / 4;
  return corners + sides + center;
}

double InterpolatedNoise3D(int i, double x, double y, double z) {
  int integer_X = x;
  double fractional_X = x - integer_X;
  int integer_Y = y;
  double fractional_Y = y - integer_Y;
  int integer_Z = z;
  double fractional_Z = z - integer_Z;

  double v1 = Smoothed3D(i, integer_X, integer_Y, integer_Z);
  double v2 = Smoothed3D(i, integer_X+1, integer_Y, integer_Z);
  double v3 = Smoothed3D(i, integer_X, integer_Y+1, integer_Z);
  double v4 = Smoothed3D(i, integer_X+1, integer_Y+1, integer_Z);
  double v5 = Smoothed3D(i, integer_X, integer_Y, integer_Z+1);
  double v6 = Smoothed3D(i, integer_X+1, integer_Y, integer_Z+1);
  double v7 = Smoothed3D(i, integer_X, integer_Y+1, integer_Z+1);
  double v8 = Smoothed3D(i, integer_X+1, integer_Y+1, integer_Z+1);

  double w1 = cosInterpolate(v5, v6, fractional_X);
  double w2 = cosInterpolate(v7, v8, fractional_X);
  double w3 = cosInterpolate(v1, v2, fractional_X);
  double w4 = cosInterpolate(v3, v4, fractional_X);

  double i1 = cosInterpolate(w3, w4, fractional_Y);
  double i2 = cosInterpolate(w1, w2, fractional_Y);

  return cosInterpolate(i1, i2, fractional_Z);
}

double InterpolatedNoise2D(int i, double x, double y) {
  int integer_X = x;
  double fractional_X = x - integer_X;
  int integer_Y = y;
  double fractional_Y = y - integer_Y;

  double v1 = Smoothed2D(i, integer_X, integer_Y);
  double v2 = Smoothed2D(i, integer_X + 1, integer_Y);
  double v3 = Smoothed2D(i, integer_X, integer_Y + 1);
  double v4 = Smoothed2D(i, integer_X + 1, integer_Y + 1);
  double i1 = cosInterpolate(v1, v2, fractional_X);
  double i2 = cosInterpolate(v3, v4, fractional_X);
  return cosInterpolate(i1, i2, fractional_Y);
}

double ValueNoise_3D(double x, double y, double z, int numOctaves=4, double persistence=0.5) {
  double total = 0;
  double frequency = pow(2, numOctaves);
  double amplitude = pow(persistence, numOctaves);
  for (int i = 0; i < numOctaves; ++i) {
    frequency /= 2;
    amplitude /= persistence;
    total += InterpolatedNoise3D((primeIndex + i) % MAX_PRIME,
              x * frequency, y * frequency, z * frequency) * amplitude;
  }
  // return total / frequency;
  return total;
}

double ValueNoise_2D(double x, double y, int numOctaves=4, double persistence=0.5) {
  double total = 0;
  double frequency = pow(2, numOctaves);
  double amplitude = pow(persistence, numOctaves);
  for (int i = 0; i < numOctaves; ++i) {
    frequency /= 2;
    amplitude /= persistence;
    total += InterpolatedNoise2D((primeIndex + i) % MAX_PRIME,
                x * frequency, y * frequency) * amplitude;
  }
  return total;
}
