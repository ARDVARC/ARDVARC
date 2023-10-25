#include <stdio.h>
#include <math.h>
#include "ardvarc_utils.h"

//-----------------------------------------------------
void print_matrix(const float* A, int m, int n) {
  // prints matrix as 2-dimensional tablei -- this is how we
  // usually think of matrices.
   int i, j;
   for (i = 0; i < m; i++) {
      for (j = 0; j < n; j++) {
          printf("%8.4f", MATRIX_ELEMENT(A, m, n, i, j));
      }
      printf("\n");
   }
}

//-----------------------------------------------------
int lindex(int m, int n, int i, int j) {
  // Function returning the linear index into matrix of
  // dimensions m, n. Assumes row-major indexing (C-style).
  return j + i*n;
}

void zeros(int m, int n, float *A) {
  int i, j;
  for (j = 0; j < n; j++) {
    for (i = 0; i < m; i++) {
      MATRIX_ELEMENT(A, m, n, i, j) = 0.0;
    }
  }
}

//-----------------------------------------------------
void linspace(float x0, float x1, int N, float *v) {
  // Returns vector v with N values from x0 to x1
  int i;
  float dx;

  dx = (x1-x0)/(N-1);
  for (i = 0; i < N; i++) {
    v[i] = x0 + i*dx;
  }
}

void covar_gen() {
  // adding two matrices
  for (i = 0; i < r; ++i)
    for (j = 0; j < c; ++j) {
      sum[i][j] = a[i][j] + b[i][j];
  }
}