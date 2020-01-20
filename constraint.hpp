#ifndef CONSTRAINT_HPP
#define CONSTRAINT_HPP

#include <vector>
#include <iostream>

#include "math.hpp"

using namespace std;

class Constraint {
 private:
 protected:
  double *stiffness;

 public:
  Constraint(double *stiffness) : stiffness(stiffness)
  {
  }

  Constraint(const Constraint &other) : stiffness(other.stiffness)
  {
  }

  virtual ~Constraint()
  {
  }

  virtual void evaluateWeightedLaplacian(vector<EigenSparseMatrixTriplet> &r_laplacian_triplets)
  {
    cout << "warning: reach <Constraint> base class virtual function." << endl;
  }

  virtual void evaluateDVector(unsigned int index, const EigenVecX &x, EigenVecX &r_d)
  {
    cout << "warning: reach <Constraint> base class virtual function." << endl;
  }
  virtual void evaluateJMatrix(unsigned int index, vector<EigenSparseMatrixTriplet> &r_j_triplets)
  {
    cout << "warning: reach <Constraint> base class virtual function." << endl;
  }

  inline const double &getStiffness()
  {
    return (*stiffness);
  }
};

class SpringConstraint : public Constraint {
 private:
  unsigned int p1, p2; /* indices of spring end points */
  double rest_length;  /* rest length of spring */
 public:
  SpringConstraint(double *stiffness) : Constraint(stiffness)
  {
  }

  SpringConstraint(const SpringConstraint &other)
      : Constraint(other), p1(other.p1), p2(other.p2), rest_length(other.rest_length)
  {
  }

  SpringConstraint(double *stiffness, unsigned int p1, unsigned int p2, double rest_length)
      : Constraint(stiffness), p1(p1), p2(p2), rest_length(rest_length)
  {
  }

  virtual ~SpringConstraint()
  {
  }

  virtual void evaluateWeightedLaplacian(vector<EigenSparseMatrixTriplet> &r_laplacian_triplets);

  virtual void evaluateDVector(unsigned int index, const EigenVecX &x, EigenVecX &r_d);
  virtual void evaluateJMatrix(unsigned int index, vector<EigenSparseMatrixTriplet> &r_j_triplets);
};

#endif