#include "constraint.hpp"

void SpringConstraint::evaluateWeightedLaplacian(
    vector<EigenSparseMatrixTriplet> &r_laplacian_triplets)
{
  double ks = *(stiffness);
  r_laplacian_triplets.push_back(EigenSparseMatrixTriplet(3 * p1 + 0, 3 * p1 + 0, ks));
  r_laplacian_triplets.push_back(EigenSparseMatrixTriplet(3 * p1 + 1, 3 * p1 + 1, ks));
  r_laplacian_triplets.push_back(EigenSparseMatrixTriplet(3 * p1 + 2, 3 * p1 + 2, ks));

  r_laplacian_triplets.push_back(EigenSparseMatrixTriplet(3 * p1 + 0, 3 * p2 + 0, ks));
  r_laplacian_triplets.push_back(EigenSparseMatrixTriplet(3 * p1 + 1, 3 * p2 + 1, ks));
  r_laplacian_triplets.push_back(EigenSparseMatrixTriplet(3 * p1 + 2, 3 * p2 + 2, ks));

  r_laplacian_triplets.push_back(EigenSparseMatrixTriplet(3 * p2 + 0, 3 * p1 + 0, ks));
  r_laplacian_triplets.push_back(EigenSparseMatrixTriplet(3 * p2 + 1, 3 * p1 + 1, ks));
  r_laplacian_triplets.push_back(EigenSparseMatrixTriplet(3 * p2 + 2, 3 * p1 + 2, ks));

  r_laplacian_triplets.push_back(EigenSparseMatrixTriplet(3 * p2 + 0, 3 * p2 + 0, ks));
  r_laplacian_triplets.push_back(EigenSparseMatrixTriplet(3 * p2 + 1, 3 * p2 + 1, ks));
  r_laplacian_triplets.push_back(EigenSparseMatrixTriplet(3 * p2 + 2, 3 * p2 + 2, ks));
}

void SpringConstraint::evaluateDVector(unsigned int index, const EigenVecX &x, EigenVecX &r_d)
{
  EigenVec3 x_ij = x.block_vector(p1) - x.block_vector(p2);
  EigenVec3 di = x_ij.normalized() * rest_length;

  r_d.block_vector(index) = di;
}

void SpringConstraint::evaluateJMatrix(unsigned int index,
                                       vector<EigenSparseMatrixTriplet> &r_j_triplets)
{
  double ks = *(stiffness);

  r_j_triplets.push_back(EigenSparseMatrixTriplet(3 * p1 + 0, 3 * index + 0, ks));
  r_j_triplets.push_back(EigenSparseMatrixTriplet(3 * p1 + 1, 3 * index + 1, ks));
  r_j_triplets.push_back(EigenSparseMatrixTriplet(3 * p1 + 2, 3 * index + 2, ks));

  r_j_triplets.push_back(EigenSparseMatrixTriplet(3 * p2 + 0, 3 * index + 0, -ks));
  r_j_triplets.push_back(EigenSparseMatrixTriplet(3 * p2 + 1, 3 * index + 1, -ks));
  r_j_triplets.push_back(EigenSparseMatrixTriplet(3 * p2 + 2, 3 * index + 2, -ks));
}
