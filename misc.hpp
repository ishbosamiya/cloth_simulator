#ifndef MISC_HPP
#define MISC_HPP

// i+1 and i-1 modulo 3
// This way of computing it tends to be faster than using %
#define NEXT(i) ((i) < 2 ? (i) + 1 : (i)-2)
#define PREV(i) ((i) > 0 ? (i)-1 : (i) + 2)

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

template<typename T> inline int find(const T &x, const std::vector<T> &xs)
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

template<typename T> inline bool is_in(const T &x, const std::vector<T> &xs)
{
  return find(x, xs) != -1;
}

template<typename T> inline void include(const T &x, std::vector<T> &xs)
{
  if (!is_in(x, xs)) {
    xs.push_back(x);
  }
}

template<typename T> inline void remove(int i, std::vector<T> &xs)
{
  xs[i] = xs.back();
  xs.pop_back();
}

template<typename T> inline void exclude(const T &x, std::vector<T> &xs)
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

template<typename T> inline void replace(const T &x0, const T &x1, std::vector<T> &xs)
{
  int i = find(x0, xs);
  if (i != -1) {
    xs[i] = x1;
  }
}

template<typename T> inline bool subset(const std::vector<T> &xs, const std::vector<T> &ys)
{
  for (int i = 0; i < xs.size(); i++) {
    if (!is_in(xs[i], ys)) {
      return false;
    }
  }

  return true;
}

template<typename T> inline void append(std::vector<T> &xs, const std::vector<T> &ys)
{
  xs.insert(xs.end(), ys.begin(), ys.end());
}

#endif
