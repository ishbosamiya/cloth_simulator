#include "math.hpp"

/* Solving Cubic Equation
 * Referenced "To Solve a Real Cubic Equation" by W. Kahan from
 * University of California, Berkley - 1989 */

static double disc(double a, double b, double c)
{
  return (b * b) - (a * c);
}

static void qdrtc(
    double A, double B, double C, double &r_x1, double &r_y1, double &r_x2, double &r_y2)
{
  double b = -B * 0.5;
  double q = disc(A, b, C);
  if (q < 0.0) {
    r_x1 = b / A;
    r_x2 = r_x1;
    r_y1 = sqrt(-q) / A;
    r_y2 = -r_y1;
  }
  else {
    r_y1 = 0.0;
    r_y2 = 0.0;
    double r = b + (sgn(b) * sqrt(q));
    if (r == 0.0) {
      r_x1 = C / A;
      r_x2 = -r_x1;
    }
    else {
      r_x1 = C / r;
      r_x2 = r / A;
    }
  }
}

static void eval(double X,
                 double A,
                 double B,
                 double C,
                 double D,
                 double &r_Q,
                 double &r_Q_dash,
                 double &r_B1,
                 double &r_C1)
{
  double q0 = A * X;
  r_B1 = q0 + B;
  r_C1 = (r_B1 * X) + C;
  r_Q_dash = (q0 + r_B1) * X + r_C1;
  r_Q = (r_C1 * X) + D;
}

static void qbc(double A,
                double B,
                double C,
                double D,
                double &r_X,
                double &r_X1,
                double &r_Y1,
                double &r_X2,
                double &r_Y2)
{
  double b1, c2;
  if (A == 0.0) {
    r_X = numeric_limits<double>::infinity();
    A = B;
    b1 = C;
    c2 = D;
    qdrtc(A, b1, c2, r_X1, r_Y1, r_X2, r_Y2);
    return;
  }
  if (D == 0.0) {
    r_X = 0;
    b1 = B;
    c2 = C;
    qdrtc(A, b1, c2, r_X1, r_Y1, r_X2, r_Y2);
    return;
  }
  r_X = -(B / A) / 3;
  double q, q_dash;
  eval(r_X, A, B, C, D, q, q_dash, b1, c2);
  double t = q / A;
  double r = cbrt(abs(t));
  double s = sgn(t);
  t = -q_dash / A;
  if (t > 0.0) {
    r = 1.324718 * max(r, sqrt(t));
  }
  double x0 = r_X - (s * r);
  if (x0 == r_X) {
    qdrtc(A, b1, c2, r_X1, r_Y1, r_X2, r_Y2);
    return;
  }
  do {
    r_X = x0;
    eval(r_X, A, B, C, D, q, q_dash, b1, c2);
    if (q_dash == 0.0) {
      x0 = r_X;
    }
    else {
      x0 = r_X - (q / q_dash) / (1 + numeric_limits<double>::epsilon());
    }
  } while (s * x0 <= s * r_X);

  if (abs(A) * r_X * r_X > abs(D / r_X)) {
    c2 = -D / r_X;
    b1 = (c2 - C) / r_X;
  }

  qdrtc(A, b1, c2, r_X1, r_Y1, r_X2, r_Y2);
  return;
}

int solveCubic(double a, double b, double c, double d, double r_x[3])
{
  double x, x1, x2, y1, y2;
  qbc(a, b, c, d, x, x1, y1, x2, y2);
  int num_sol = 0;
  r_x[num_sol++] = x;
  if (y1 == 0.0) {
    r_x[num_sol++] = x1;
  }
  if (y2 == 0.0) {
    r_x[num_sol++] = x2;
  }
  return num_sol;
}

/* *** END *** (Solving Cubic Equation) */
