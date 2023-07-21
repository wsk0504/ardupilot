/*
 *  N dimensional matrix operations
 */

#pragma once

#include "math.h"
#include <stdint.h>
#include "vectorN.h"

template <typename T, uint8_t N>
class VectorN;


template <typename T, uint8_t N>
class MatrixN {
  
    friend class VectorN<T,N>;

public:
    // constructor from zeros
    MatrixN(void) {
        memset(v, 0, sizeof(v));        
    }

    // constructor from 4 diagonals
    MatrixN(const float d[N]) {
        memset(v, 0, sizeof(v));
        for (uint8_t i = 0; i < N; i++) {
            v[i][i] = d[i];
        }
    }

    // multiply two vectors to give a matrix, in-place
    void mult(const VectorN<T,N> &A, const VectorN<T,N> &B);

    // subtract B from the matrix
    MatrixN<T,N> &operator -=(const MatrixN<T,N> &B);

    // add B to the matrix
    MatrixN<T,N> &operator +=(const MatrixN<T,N> &B);
    
    // Matrix symmetry routine
    void force_symmetry(void);

private:
    T v[N][N];
};

// JH 07_12_23 constructor and power operator for 2x2 matrix

template <typename T>
class MatrixN<T,2> {
public:
    MatrixN() {
        for (uint8_t i = 0; i < 2; i++) {
            for (uint8_t j = 0; j < 2; j++) {
                v[i][j] = 0;
            }
        }
    }
    MatrixN(T a_, T b_, T c_, T d_) {
        v[0][0] = a_;
        v[0][1] = b_;
        v[1][0] = c_;
        v[1][1] = d_;
    }
VectorN<T, 2> operator *(const VectorN<T, 2>& vec) const {
    VectorN<T, 2> result;
    for (uint8_t i = 0; i < 2; i++) {
        T sum = 0;
        for (uint8_t j = 0; j < 2; j++) {
            sum += v[i][j] * vec[j]; // assuming vec[j] gives the j-th element of the vector
        }
        result[i] = sum; // assuming result[i] is used to set the i-th element of the vector
    }
    return result;
}
MatrixN<T, 2> operator +(const MatrixN<T, 2>& other) const {
    MatrixN<T, 2> result;
    for (uint8_t i = 0; i < 2; i++) {
        for (uint8_t j = 0; j < 2; j++) {
            result[i][j] = v[i][j] + other[i][j]; 
        }
    }
    return result;
}
MatrixN<T, 2> operator -(const MatrixN<T, 2>& other) const {
    MatrixN<T, 2> result;
    for (uint8_t i = 0; i < 2; i++) {
        for (uint8_t j = 0; j < 2; j++) {
            result[i][j] = v[i][j] - other[i][j]; 
        }
    }
    return result;
}
MatrixN<T, 2> operator *(const T& scalar) const {
    MatrixN<T, 2> result;
    for (uint8_t i = 0; i < 2; i++) {
        for (uint8_t j = 0; j < 2; j++) {
            result[i][j] = v[i][j] * scalar; 
        }
    }
    return result;
}
MatrixN<T,2> operator *(const MatrixN<T,2> &m) const {
    MatrixN<T,2> result;
    for (uint8_t i = 0; i < 2; i++) {
        for (uint8_t j = 0; j < 2; j++) {
            result.v[i][j] = 0;
            for (uint8_t k = 0; k < 2; k++) {
                result.v[i][j] += v[i][k] * m.v[k][j];
            }
        }
    }
    return result;
}
MatrixN<T,2> power(int n) const {
    if (n == 0) {
        MatrixN<T,2> identity (1.0f,0.0f,0.0f,1.0f);
        return identity;
    } else if (n == 1) {
        return *this;
    } else {
        MatrixN<T,2> result = *this;
        for (int i = 1; i < n; i++) {
            result = result * (*this);
        }
        return result;
    }
}



T* operator[](uint8_t i) {
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 2);
#endif
        return v[i];
    }

    const T* operator[](uint8_t i) const {
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 2);
#endif
        return v[i];
    }

private:
    T v[2][2];
};

template <typename T>
class MatrixN<T,3> {
public:
    MatrixN() {
        for (uint8_t i = 0; i < 3; i++) {
            for (uint8_t j = 0; j < 3; j++) {
                v[i][j] = 0;
            }
        }
    }
    MatrixN(T a_, T b_, T c_, T d_, T e_, T f_, T g_, T h_, T i_) {
        v[0][0] = a_;
        v[0][1] = b_;
        v[0][2] = c_;
        v[1][0] = d_;
        v[1][1] = e_;
        v[1][2] = f_;
        v[2][0] = g_;
        v[2][1] = h_;
        v[2][2] = i_;
    }
MatrixN<T, 3> operator *(const T& scalar) const {
    MatrixN<T, 3> result;
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            result[i][j] = v[i][j] * scalar; 
        }
    }
    return result;
}

VectorN<T, 3> operator *(const VectorN<T, 3>& vec) const {
    VectorN<T, 3> result;
    for (uint8_t i = 0; i < 3; i++) {
        T sum = 0;
        for (uint8_t j = 0; j < 3; j++) {
            sum += v[i][j] * vec[j]; // assuming vec[j] gives the j-th element of the vector
        }
        result[i] = sum; // assuming result[i] is used to set the i-th element of the vector
    }
    return result;
}    

MatrixN<T, 3> operator +(const MatrixN<T, 3>& other) const {
    MatrixN<T, 3> result;
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            result[i][j] = v[i][j] + other[i][j]; 
        }
    }
    return result;
}

    T* operator[](uint8_t i) {
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 3);
#endif
        return v[i];
    }

    const T* operator[](uint8_t i) const {
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 3);
#endif
        return v[i];
    }


private:
    T v[3][3];
};
