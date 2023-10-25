#ifndef MATRIX_UTILS_H
#define MATRIX_UTILS_H

// These fcns are meant to make it easier to deal with
// matrices in C on the Beaglebone.

// Macros to extract matrix index and element.
#define MATRIX_IDX(n, i, j) j + i*n
#define MATRIX_ELEMENT(A, m, n, i, j) A[ MATRIX_IDX(n, i, j) ]

// Min and max macros for scalars.
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

// Function prototypes
void print_matrix(const float* A, int m, int n);
int lindex(int m, int n, int i, int j);
void zeros(int m, int n, float *A);
void linspace(float x0, float x1, int N, float *v);

#endif