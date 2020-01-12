#ifndef MISC_HPP
#define MISC_HPP

#include <vector>
#include <limits>
#include <algorithm>

using namespace std;

// i+1 and i-1 modulo 3
// This way of computing it tends to be faster than using %
#define NEXT(i) ((i) < 2 ? (i) + 1 : (i)-2)
#define PREV(i) ((i) > 0 ? (i)-1 : (i) + 2)

const double infinity = numeric_limits<double>::infinity();

template<typename T> T sqr(const T &x)
{
  return x * x;
}

template<typename T> T clamp(const T &x, const T &a, const T &b)
{
  return min(max(x, a), b);
}

template<typename T> T min(const T &a, const T &b, const T &c)
{
  return min(a, min(b, c));
}
template<typename T> T min(const T &a, const T &b, const T &c, const T &d)
{
  return min(min(a, b), min(c, d));
}

template<typename T> T max(const T &a, const T &b, const T &c)
{
  return max(a, max(b, c));
}
template<typename T> T max(const T &a, const T &b, const T &c, const T &d)
{
  return max(max(a, b), max(c, d));
}

template<typename T> inline int find(const T *x, T *const *xs, int n = 3)
{
  for (int i = 0; i < n; i++) {
    if (xs[i] == x) {
      return i;
    }
  }

  return -1;
}

template<typename T> inline int find(const T &x, const T *xs, int n = 3)
{
  for (int i = 0; i < n; i++) {
    if (xs[i] == x) {
      return i;
    }
  }

  return -1;
}

template<typename T> inline int find(const T &x, const vector<T> &xs)
{
  for (int i = 0; i < xs.size(); i++) {
    if (xs[i] == x) {
      return i;
    }
  }

  return -1;
}

template<typename T> inline bool is_in(const T *x, T *const *xs, int n = 3)
{
  return find(x, xs, n) != -1;
}

template<typename T> inline bool is_in(const T &x, const T *xs, int n = 3)
{
  return find(x, xs, n) != -1;
}

template<typename T> inline bool is_in(const T &x, const vector<T> &xs)
{
  return find(x, xs) != -1;
}

template<typename T> inline void include(const T &x, vector<T> &xs)
{
  if (!is_in(x, xs)) {
    xs.push_back(x);
  }
}

template<typename T> inline void remove(int i, vector<T> &xs)
{
  xs[i] = xs.back();
  xs.pop_back();
}

template<typename T> inline void exclude(const T &x, vector<T> &xs)
{
  int i = find(x, xs);
  if (i != -1) {
    remove(i, xs);
  }
}

template<typename T> inline void replace(const T &v0, const T &v1, T vs[3])
{
  int i = find(v0, vs);
  if (i != -1) {
    vs[i] = v1;
  }
}

template<typename T> inline void replace(const T &x0, const T &x1, vector<T> &xs)
{
  int i = find(x0, xs);
  if (i != -1) {
    xs[i] = x1;
  }
}

template<typename T> inline bool subset(const vector<T> &xs, const vector<T> &ys)
{
  for (int i = 0; i < xs.size(); i++) {
    if (!is_in(xs[i], ys)) {
      return false;
    }
  }

  return true;
}

template<typename T> inline void append(vector<T> &xs, const vector<T> &ys)
{
  xs.insert(xs.end(), ys.begin(), ys.end());
}

#endif
