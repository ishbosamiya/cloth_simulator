#include "math.hpp"

int solveQuadratic(double a, double b, double c, double r_x[2])
{
  // http://en.wikipedia.org/wiki/Quadratic_formula#Floating_point_implementation
  double d = b * b - 4 * a * c;
  if (d < 0) {
    r_x[0] = -b / (2 * a);
    return 0;
  }
  double q = -(b + sgn(b) * sqrt(d)) / 2;
  int i = 0;
  if (abs(a) > 1e-12 * abs(q))
    r_x[i++] = q / a;
  if (abs(q) > 1e-12 * abs(c))
    r_x[i++] = c / q;
  if (i == 2 && r_x[0] > r_x[1])
    swap(r_x[0], r_x[1]);
  return i;
}

double newtonsMethod(double a, double b, double c, double d, double x0, int init_dir)
{
  if (init_dir != 0) {
    // quadratic approximation around x0, assuming y' = 0
    double y0 = d + x0 * (c + x0 * (b + x0 * a));
    double ddy0 = 2 * b + x0 * (6 * a);
    x0 += init_dir * sqrt(abs(2 * y0 / ddy0));
  }
  for (int iter = 0; iter < 100; iter++) {
    double y = d + x0 * (c + x0 * (b + x0 * a));
    double dy = c + x0 * (2 * b + x0 * 3 * a);
    if (dy == 0) {
      return x0;
    }
    double x1 = x0 - y / dy;
    if (abs(x0 - x1) < 1e-6) {
      return x0;
    }
    x0 = x1;
  }
  return x0;
}

int solveCubic(double a, double b, double c, double d, double r_x[3])
{
  double xc[2];
  int ncrit = solveQuadratic(3 * a, 2 * b, c, xc);
  if (ncrit == 0) {
    r_x[0] = newtonsMethod(a, b, c, d, xc[0], 0);
    return 1;
  }
  else if (ncrit == 1) {  // cubic is actually quadratic
    return solveQuadratic(b, c, d, r_x);
  }
  else {
    double yc[2] = {d + xc[0] * (c + xc[0] * (b + xc[0] * a)),
                    d + xc[1] * (c + xc[1] * (b + xc[1] * a))};
    int i = 0;
    if (yc[0] * a >= 0) {
      r_x[i++] = newtonsMethod(a, b, c, d, xc[0], -1);
    }
    if (yc[0] * yc[1] <= 0) {
      int closer = abs(yc[0]) < abs(yc[1]) ? 0 : 1;
      r_x[i++] = newtonsMethod(a, b, c, d, xc[closer], closer == 0 ? 1 : -1);
    }
    if (yc[1] * a <= 0) {
      r_x[i++] = newtonsMethod(a, b, c, d, xc[1], 1);
    }
    return i;
  }
}
