#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "matrix.h"
#include "main.h"

/* transpose():
FUNCTION:
    Takes in a row major matrix and turns it into a column major matrix
DEFS:
    matrix:         The row major matrix
    transpose_mat:  The new matrix to be written
    rows:           The number of rows in the matrix
    cols:           The number of columns in the matrix
*/
void transpose(float* matrix, float* transpose_mat, uint8_t rows, uint8_t cols){
    for (uint8_t i = 0; i < rows; i++) {
        for (uint8_t j = 0; j < cols; j++) {
            transpose_mat[j * rows + i] = matrix[i * cols + j];
        }
    }
}

/* dot_product():
FUNCTION:
    Returns the dot product of two vectors, A and B
    Requires that A and B are the same size
DEFS:
    A:      The first vector in the dot product
    B:      The second vector in the dot product
    size:   The size of the A and B (should be the same for both)
*/
float dot_product(float* A, float* B, uint8_t size){
    float result = 0.0;
    for(uint8_t i = 0; i < size; i++){
        result += A[i] * B[i];
    }
    return result;
}

/* norm():
FUNCTION:
    Returns the l2 norm of a vector A
DEFS:
    A:      The vector to be l2 normed
    size:   The length of the A
*/
float norm(float* A, uint8_t size){
    return sqrtf(dot_product(A, A, size));
}


/* scale_vector():
FUNCTION:
    scales the input vector by a scalar
DEFS:
    scalar: The scalar
    A:      The vector to be scaled
    result: Resulting scaled vector
    size:   The length of the A (same as result)
*/
void scale_vector(float scalar, float* A, float* result, uint8_t size){
    for(int i = 0; i < size; i++) {
        result[i] = A[i] * scalar;
    }
}

/* add_vectors():
FUNCTION:
    scales the input vector by a scalar
DEFS:
    result: The resulting sum
    A:      The first vector in the sum
    B:      The second vector in the sum
    size:   The size of the A and B (should be the same for both)
*/
void add_vectors(float* result, float* A, float* B, uint8_t size){
    for(int i = 0; i < size; i++) {
        result[i] = A[i] + B[i];
    }
}

/* matrix_mult():
FUNCTION:
    Performs matrix multiplication between two matrices and stores it in a result matrix
    Results are stored in row major order
    result = m * n
DEFS:
    m:          The first matrix in the operation
    n:          The second matrix in the operation
    result:     The result of the matrix multiplication in row order form
    m_rows:     The number of rows in the m matix
    n_cols:     The number of columns in the n matrix
    shared:     The number of columns in m matrix / number of rows in n matrix (SAME VALUE)
*/
void matrix_mult(float* m, float* n, float* result,
                 uint8_t m_rows, uint8_t n_cols, uint8_t shared){

        float temp[n_cols * shared];
        transpose(n, temp, shared, n_cols);
        //loop through each row of the first matrix
        for(uint8_t i = 0; i < m_rows; i++){
        
            //loop through each col of the second matrix
            for(uint8_t j = 0; j < n_cols; j ++){
                result[(i*n_cols) + j] = dot_product(&m[i*shared], &temp[j*shared], shared);
            }
        }
    
}

/* inv_3x3():
FUNCTION:
    Calculates the inverse of a 3x3 matrix given in row major order
DEFS:
    matrix:     The 3x3 matrix that you want to calculate the inverse of, must be in row major order
    inverse:    The inverse of matrix in row major order
*/
void inv_3x3(float* matrix, float* inverse){
//MATH CAN BE SIMPLIFIED TO SAVE ON TIME/MEMORY
    /*
    matrix  =   [a, b, c]
                [d, e, f]
                [g, h, i]
    */
    float a, b, c;
    float d, e, f;
    float g, h, i;

    a = matrix[0]; b = matrix[1]; c = matrix[2];
    d = matrix[3]; e = matrix[4]; f = matrix[5];
    g = matrix[6]; h = matrix[7]; i = matrix[8];

    float det = ((a*((e*i) - (f*h))) - (b*((d*i) - (f*g))) + (c*((d*h) - (e*g))));
    if (det == 0) return;

    /*
    cof matrix    =    [(ei-hf) -(di-fg) (dh-eg)]
                        [-(bi-ch) (ai-cg) -(ah-bg)]
                        [(bf-ce) -(af-cd) (ae-bd)]
    */
    float cofactor[9] = {(e*i - h*f), -1*(d*i - f*g), (d*h - e*g),
                         -1*(b*i - c*h), (a*i - c*g), -1*(a*h - b*g),
                         (b*f - c*e), -1*(a*f - c*d), (a*e - b*d)
                        };
    
    //do the transform to get adj of matrix
    float adj[9] = {cofactor[0], cofactor[3], cofactor[6],
                    cofactor[1], cofactor[4], cofactor[7],
                    cofactor[2], cofactor[5], cofactor[8]
                   };

    //multiply by 1/det to get inverse
    for(uint8_t i = 0; i < 9; i++){
         inverse[i] = adj[i] * (1.0f/det);
    }

    
}

/* inv_4x4():
FUNCTION:
    Calculates the inverse of a 4x4 matrix given in row major order
DEFS:
    matrix:     The 4x4 matrix that you want to calculate the inverse of, must be in row major order
    inverse:    The inverse of matrix in row major order
*/
char inv_4x4(float* matrix, float* inverse){
    const int n = 4;
    float aug[32];

    // Build augmented matrix [A | I]
    for(int i = 0; i < n; i++){
        for(int j = 0; j < n; j++){
            aug[i*2*n + j]     = matrix[i*n + j];
            aug[i*2*n + j + n] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // Gauss-Jordan with partial pivoting
    for(int i = 0; i < n; i++){

        // --- Find pivot row (max absolute value in column i) ---
        int pivot_row = i;
        float max_val = fabsf(aug[i*2*n + i]);

        for(int r = i + 1; r < n; r++){
            float val = fabsf(aug[r*2*n + i]);
            if(val > max_val){
                max_val = val;
                pivot_row = r;
            }
        }

        // --- If pivot row is not current row, swap the rows ---
        if(pivot_row != i){
            for(int c = 0; c < 2*n; c++){
                float temp = aug[i*2*n + c];
                aug[i*2*n + c] = aug[pivot_row*2*n + c];
                aug[pivot_row*2*n + c] = temp;
            }
        }

        // --- Now pivot is aug[i][i], ensure it’s not zero ---
        float pivot = aug[i*2*n + i];
        if(fabsf(pivot) < 1e-8f){
            // Matrix is singular or extremely close to singular
            // (you can return an error or set inverse to zero)
            return 1;
        }

        // --- Normalize pivot row ---
        for(int j = 0; j < 2*n; j++){
            aug[i*2*n + j] /= pivot;
        }

        // --- Eliminate all other rows ---
        for(int r = 0; r < n; r++){
            if(r == i) continue;
            float factor = aug[r*2*n + i];
            for(int c = 0; c < 2*n; c++){
                aug[r*2*n + c] -= factor * aug[i*2*n + c];
            }
        }
    }

    // Extract inverse from augmented matrix
    for(int i = 0; i < n; i++){
        for(int j = 0; j < n; j++){
            inverse[i*n + j] = aug[i*2*n + (j+n)];
        }
    }
    return 0;
}

/* gram_Schmidt():
FUNCTION:
    Gram Schmidt orthonormalization of column vector matrix process.
    Returns orthonormal matrix to original matrix buffer with column vectors.
DEFS:
    matrix:     input matrix
    rows:       number of rows in input matrix
    cols:       number of columns in input matrix
*/
void gram_Schmidt(float* matrix, uint8_t rows, uint8_t cols) {
    float E[cols][rows];
    transpose(matrix, E[0], rows, cols); // Transpose to get row vectors
    float E0_norm = norm(E[0], rows); // l2 norm of first vector
    for (int r = 0; r < rows; r++) {
        E[0][r] /= E0_norm;
    } // normalize first vector in matrix
    for (int v = 1; v < cols; v++) {
        for (int vp = 0; vp < v; vp++) {
            // projection dot product of vectors prior to 'v' in E to vector 'v'
            float scalar = dot_product(E[v], E[vp], rows) / dot_product(E[vp], E[vp], rows);
            float sE_vp[rows];
            scale_vector(-scalar, E[vp], sE_vp, rows);
            // subtract projection from 'v'
            add_vectors(E[v], E[v], sE_vp, rows);
        }
        // normalize vector 'v'
        float v_norm = norm(E[v], rows);
        for (int r = 0; r < rows; r++) {
            E[v][r] /= v_norm;
        }
    }
    // return orthonormal matrix in original input buffer
    transpose(E[0], matrix, cols, rows);
}

/*
    To recover the geometric coordinates of the anchors we need to find the eigenvectors and eigenvalues
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
    matrix:      input symmetric square matrix M
    V:           Orthonormal matrix with # rows matching matrix dim. This will become the eigenvector matrix
    rows:        # of rows in V (and dim of matrix)
    cols:        # of cols in V
    eigenvalues: eigenvalues of the matrix. It will be length cols
*/
void find_Eigens(float* matrix, float* V, uint8_t rows, uint8_t cols, float* eigenvalues) {
    // Gram-Schmidt process to converge V to M's eigenvectors, following a convergence criteria
    float total_convergence;
    float V_T[cols][rows];
    uint16_t iter = 0;
    do {
        float V_copy[cols][rows]; // for monitoring convergence
        transpose(V, V_copy[0], rows, cols);
        // the power iteration steps
        matrix_mult(matrix, V, V, rows, cols, rows); // Safe to have V as destination as it's internally copied
        gram_Schmidt(V, rows, cols); // works on col vectors
        // check convergence
        total_convergence = 0.0f;
        transpose(V, V_T[0], rows, cols);
        for (int v = 0; v < cols; v++) {
            float diff1[rows];
            float diff2[rows];
            // add literal
            add_vectors(diff1, V_T[v], V_copy[v], rows);
            // add opposite
            scale_vector(-1.0, V_T[v], V_T[v], rows);
            add_vectors(diff2, V_T[v], V_copy[v], rows);
            total_convergence += MIN(norm(diff1, rows), norm(diff2, rows));
        }
        iter++;
    } while ((total_convergence > 5E-4) && (iter < MAX_ITER));
    // retrieve eigenvalue diagonal matrix
    float VTM[cols][rows];
    matrix_mult(V_T[0], matrix, VTM[0], cols, rows, rows);
    float diagonal[cols][cols];
    matrix_mult(VTM[0], V, diagonal[0], cols, cols, rows);
    // make all eigenvalues positive; store eigenvalues
    transpose(V, V_T[0], rows, cols);
    for (int i = 0; i < cols; i++) {
        eigenvalues[i] = fabs(diagonal[i][i]);
        if (diagonal[i][i] < 0.0f) {
            scale_vector(-1.0f, V_T[i], V_T[i], rows);
        }
    }
    // write to original V matrix
    transpose(V_T[0], V, cols, rows);
}

// Helper: print a matrix in row-major order
void print_matrix(float* mat, uint8_t rows, uint8_t cols) {
    for (uint8_t i = 0; i < rows; i++) {
        printf("\r");
        for (uint8_t j = 0; j < cols; j++) {
            printf("%10.5f ", mat[i * cols + j]);
        }
        printf("\n");
    }
    printf("\n");
}

