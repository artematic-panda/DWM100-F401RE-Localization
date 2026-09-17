#pragma once

#define MAX_ITER 100U

/* transpose():
FUNCTION:
    Takes in a row major matrix and turns it into a column major matrix
DEFS:
    matrix:         The row major matrix
    transpose_mat:  The new matrix to be written
    rows:           The number of rows in the matrix
    cols:           The number of columns in the matrix
*/
void transpose(float* matrix, float* transpose_mat, uint8_t rows, uint8_t cols);
   

/* dot_product():
FUNCTION:
    Returns the dot product of two vectors, A and B
    Requires that A and B are the same size
DEFS:
    A:      The first vector in the dot product
    B:      The second vector in the dot product
    size:   The size of the A and B (should be the same for both)
*/
float dot_product(float* A, float* B, uint8_t size);

/* norm():
FUNCTION:
    Returns the l2 norm of a vector A
DEFS:
    A:      The vector to be l2 normed
    size:   The length of the A
*/
float norm(float* A, uint8_t size);

/* scale_vector():
FUNCTION:
    scales the input vector by a scalar
DEFS:
    scalar: The scalar
    A:      The vector to be scaled
    result: Resulting scaled vector
    size:   The length of the A (same as result)
*/
void scale_vector(float scalar, float* A, float* result, uint8_t size);

/* add_vectors():
FUNCTION:
    scales the input vector by a scalar
DEFS:
    result: The resulting sum
    A:      The first vector in the sum
    B:      The second vector in the sum
    size:   The size of the A and B (should be the same for both)
*/
void add_vectors(float* result, float* A, float* B, uint8_t size);

/* matrix_mult():
FUNCTION
    Performs matrix multiplication between two matrices and stores it in a result matrix
    Results are stored in row major order
    result = m * n
DEFS:
    m:          The first matrix in the operation in row major order
    n:          The second matrix in the operation in row major order
    result:     The result of the matrix multiplication in row order form
    m_rows:     The number of rows in the m matix
    n_cols:     The number of columns in the n matrix
    shared:     The number of columns in m matrix / number of rows in n matrix (SAME VALUE)
*/
void matrix_mult(float* m, float* n, float* result,
                 uint8_t m_rows, uint8_t n_cols, uint8_t shared);

/* inv_3x3():
FUNCTION:
    Calculates the inverse of a 3x3 matrix given in row major order
DEFS:
    matrix:     The matrix that you want to calculate the inverse of
    inverse:    The output of the function, the inverse of matrix
*/
void inv_3x3(float* matrix, float* inverse);

/* inv_4x4():
FUNCTION:
    Calculates the inverse of a 4x4 matrix given in row major order
    returns 1 if bad
DEFS:
    matrix:     The 4x4 matrix that you want to calculate the inverse of, must be in row major order
    inverse:    The inverse of matrix in row major order
*/
char inv_4x4(float* matrix, float* inverse);

/* gram_Schmidt():
FUNCTION:
    Gram Schmidt orthonormalization of column vector matrix process.
    Returns orthonormal matrix to original matrix buffer with column vectors.
DEFS:
    matrix:     input matrix
    rows:       number of rows in input matrix
    cols:       number of columns in input matrix
*/
void gram_Schmidt(float* matrix, uint8_t rows, uint8_t cols);

/*
    To recover the geometric coordinates of the anchors we need to find the eigenvectors and eigenvalues.
    of the M matrix. To accomplish this, this program uses the power iteration method with Gram-Schmidt 
    for numerical stability and uniqueness of eigenvectors.
    Since M is a symmetric (diagonalizable) (NxN) matrix, we know we have N unique eigenvectors.
    Then, we know that we have a valid eigenbasis with rank N, which means any vector can be expressed
    as a linear combination of the eigenbasis. As such, (M^k)V inherently converges to the largest
    eigenvalue/eigenvector pair as k->inf. (as long as V is not orthogonal to the eigenvectors).
    https://ergodic.ugr.es/cphys/lecciones/fortran/power_method.pdf
    https://www.cs.unc.edu/techreports/96-043.pdf
*/
/* find_Eigens()
FUNCTION:
    find the eigenvalues and eigenvectors of an input matrix
DEFS:
    matrix:      input symmetric square matrix
    V:           Orthonormal matrix with # rows matching matrix dim. This will become the eigenvector matrix
    rows:        # of rows in V (and dim of matrix)
    cols:        # of cols in V
    eigenvalues: eigenvalues of the matrix. It will be length cols
*/
void find_Eigens(float* matrix, float* V, uint8_t rows, uint8_t cols, float* eigenvalues);

 // Helper: print a matrix in row-major order
void print_matrix(float* mat, uint8_t rows, uint8_t cols);

