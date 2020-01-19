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
  Constraint(double *stiffness);
  Constraint(const Constraint &other);
  virtual ~Constraint();

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
  SpringConstraint(double *stiffness);
  SpringConstraint(const SpringConstraint &other);
  virtual ~SpringConstraint();

  virtual void evaluateWeightedLaplacian(vector<EigenSparseMatrixTriplet> &r_laplacian_triplets);

  virtual void evaluateDVector(unsigned int index, const EigenVecX &x, EigenVecX &r_d);
  virtual void evaluateJMatrix(unsigned int index, vector<EigenSparseMatrixTriplet> &r_j_triplets);
};

#endif
