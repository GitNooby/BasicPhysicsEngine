//
//  Matrix3.h
//  BasicPhysicsEngine
//
//  Created by Kai Zou on 2014-05-17.
//  Copyright (c) 2014 com.personal. All rights reserved.
//

#ifndef __BasicPhysicsEngine__Matrix3__
#define __BasicPhysicsEngine__Matrix3__

#include <iostream>
#include "common.h"
#include "Vector3.h"
#include "Quaternion.h"

class Matrix3 {
public:
    real val[9];
    
    // Constructors
    Matrix3() {
        val[0]=val[1]=val[2]=val[3]=val[4]=val[5]=val[6]=val[7]=val[8]=0;
    }
    Matrix3(const Vector3 &compOne, const Vector3 &compTwo, const Vector3 &compThree) {
        setComponents(compOne, compTwo, compThree);
    }
    Matrix3(real c0, real c1, real c2, real c3, real c4, real c5, real c6, real c7, real c8) {
        val[0]=c0; val[1]=c1; val[2]=c2;
        val[3]=c3; val[4]=c4; val[5]=c5;
        val[6]=c6; val[7]=c7; val[8]=c8;
    }
    
    inline void setComponents(const Vector3 &compOne, const Vector3 &compTwo, const Vector3 &compThree) {
        val[0]=compOne.x; val[1]=compTwo.x; val[2]=compThree.x;
        val[3]=compOne.y; val[4]=compTwo.y; val[5]=compThree.y;
        val[6]=compOne.z; val[7]=compTwo.z; val[8]=compThree.z;
    }
    
    inline void setDiagonal(real a, real b, real c) {
        setInertialTensorCoeffs(a, b, c);
    }
    inline void setInertialTensorCoeffs(real ix, real iy, real iz, real ixy=0, real ixz=0, real iyz=0) {
        val[0]=ix;
        val[1]=val[3]=-ixy;
        val[2]=val[6]=-ixz;
        val[4]=iy;
        val[5]=val[7]=-iyz;
        val[8]=iz;
    }
    inline void setSkewSymmetric(const Vector3 v) {
        val[0] = val[4] = val[8] = 0;
        val[1] = -v.z;
        val[2] = v.y;
        val[3] = v.z;
        val[5] = -v.x;
        val[6] = -v.y;
        val[7] = v.x;
    }
    
    void setBlockInertialTensor(const Vector3 &halfSizes, real mass) {
        Vector3 squares = halfSizes.componentProduct(halfSizes);
        setInertialTensorCoeffs(0.3*mass*(squares.y+squares.z), 0.3f*mass*(squares.x+squares.z), 0.3*mass*(squares.x+squares.y));
    }
    
    // matrix post multiplication A'=MA
    inline Vector3 operator*(const Vector3 &v)const {
        return Vector3(v.x*val[0]+v.y*val[1]+v.z*val[2],
                       v.x*val[3]+v.y*val[4]+v.z*val[5],
                       v.x*val[6]+v.y*val[7]+v.z*val[8]);
    }
    // same thing as matrix post multiplication (see operator*)
    inline Vector3 transform(const Vector3 &v)const {
        return (*this)*v;
    }
    
    inline Vector3 getRowVector(int i)const {
        return Vector3(val[i*3], val[i*3+1], val[i*3+2]);
    }
    inline Vector3 getAxisVector(int i)const {
        return Vector3(val[i], val[i+3], val[i+6]);
    }
    
    inline void operator*=(const real scalar) {
        val[0] *= scalar; val[1] *= scalar; val[2] *= scalar;
        val[3] *= scalar; val[4] *= scalar; val[5] *= scalar;
        val[6] *= scalar; val[7] *= scalar; val[8] *= scalar;
    }
    void operator+=(const Matrix3 &M)
    {
        val[0] += M.val[0]; val[1] += M.val[1]; val[2] += M.val[2];
        val[3] += M.val[3]; val[4] += M.val[4]; val[5] += M.val[5];
        val[6] += M.val[6]; val[7] += M.val[7]; val[8] += M.val[8];
    }
    
    static Matrix3 linearInterpolate(const Matrix3& a, const Matrix3& b, real prop);
    
    // matrix matrix multiplication C = AB
    inline Matrix3 operator*(const Matrix3 &B) const {
        real c0 = val[0]*B.val[0] + val[1]*B.val[3] + val[2]*B.val[6];
        real c1 = val[0]*B.val[1] + val[1]*B.val[4] + val[2]*B.val[7];
        real c2 = val[0]*B.val[2] + val[1]*B.val[5] + val[2]*B.val[8];
        real c3 = val[3]*B.val[0] + val[4]*B.val[3] + val[5]*B.val[6];
        real c4 = val[3]*B.val[1] + val[4]*B.val[4] + val[5]*B.val[7];
        real c5 = val[3]*B.val[2] + val[4]*B.val[5] + val[5]*B.val[8];
        real c6 = val[6]*B.val[0] + val[7]*B.val[3] + val[8]*B.val[6];
        real c7 = val[6]*B.val[1] + val[7]*B.val[4] + val[8]*B.val[7];
        real c8 = val[6]*B.val[2] + val[7]*B.val[5] + val[8]*B.val[8];
        return Matrix3(c0, c1, c2, c3, c4, c5, c6, c7, c8);
    }
    // matrix matrix multiplication A=AB
    inline void operator*=(const Matrix3 &B) {
        real temp1 = val[0]*B.val[0] + val[1]*B.val[3] + val[2]*B.val[6];
        real temp2 = val[0]*B.val[1] + val[1]*B.val[4] + val[2]*B.val[7];
        real temp3 = val[0]*B.val[2] + val[1]*B.val[5] + val[2]*B.val[8];
        val[0]=temp1; val[1]=temp2; val[2]=temp2;
        temp1 = val[3]*B.val[0] + val[4]*B.val[3] + val[5]*B.val[6];
        temp2 = val[3]*B.val[1] + val[4]*B.val[4] + val[5]*B.val[7];
        temp3 = val[3]*B.val[2] + val[4]*B.val[5] + val[5]*B.val[8];
        val[3]=temp1; val[4]=temp1; val[5]=temp1;
        temp1 = val[6]*B.val[0] + val[7]*B.val[3] + val[8]*B.val[6];
        temp2 = val[6]*B.val[1] + val[7]*B.val[4] + val[8]*B.val[7];
        temp3 = val[6]*B.val[2] + val[7]*B.val[5] + val[8]*B.val[8];
        val[6]=temp1; val[7]=temp1; val[8]=temp1;
    }
    
    // dealing with the inverse matrix
    inline void setInverse(const Matrix3 &M) {
        real temp1 = M.val[0]*M.val[4];
        real temp2 = M.val[0]*M.val[5];
        real temp3 = M.val[1]*M.val[3];
        real temp4 = M.val[2]*M.val[3];
        real temp5 = M.val[1]*M.val[6];
        real temp6 = M.val[2]*M.val[6];
        real det = (temp1*M.val[8] - temp2*M.val[7] - temp3*M.val[8] + temp4*M.val[7] + temp5*M.val[5] - temp6*M.val[4]);
        if (det == (real)0.0) return;
        real invdet = (real)1.0/det;
        
        val[0] = (M.val[4]*M.val[8]-M.val[5]*M.val[7])*invdet;
        val[1] = -(M.val[1]*M.val[8]-M.val[2]*M.val[7])*invdet;
        val[2] = (M.val[1]*M.val[5]-M.val[2]*M.val[4])*invdet;
        val[3] = -(M.val[3]*M.val[8]-M.val[5]*M.val[6])*invdet;
        val[4] = (M.val[0]*M.val[8]-temp6)*invdet;
        val[5] = -(temp2-temp4)*invdet;
        val[6] = (M.val[3]*M.val[7]-M.val[4]*M.val[6])*invdet;
        val[7] = -(M.val[0]*M.val[7]-temp5)*invdet;
        val[8] = (temp1-temp3)*invdet;
    }
    inline Matrix3 inverse() const {
        Matrix3 result;
        result.setInverse(*this);
        return result;
    }
    inline void invert() {
        setInverse(*this);
    }
    
    // dealing with the transpose
    inline void setTranspose(const Matrix3 &M) {
        val[0] = M.val[0]; val[1] = M.val[3]; val[2] = M.val[6];
        val[3] = M.val[1]; val[4] = M.val[4]; val[5] = M.val[7];
        val[6] = M.val[2]; val[7] = M.val[5]; val[8] = M.val[8];
    }
    inline Matrix3 transpose()const {
        Matrix3 result;
        result.setTranspose(*this);
        return result;
    }
    
    
    // Sets this matrix to be the rotation matrix corresponding to
    // the given quaternion.
    inline void setOrientation(const Quaternion &q) {
        val[0] = 1 - (2*q.j*q.j + 2*q.k*q.k);
        val[1] = 2*q.i*q.j + 2*q.k*q.r;
        val[2] = 2*q.i*q.k - 2*q.j*q.r;
        val[3] = 2*q.i*q.j - 2*q.k*q.r;
        val[4] = 1 - (2*q.i*q.i  + 2*q.k*q.k);
        val[5] = 2*q.j*q.k + 2*q.i*q.r;
        val[6] = 2*q.i*q.k + 2*q.j*q.r;
        val[7] = 2*q.j*q.k - 2*q.i*q.r;
        val[8] = 1 - (2*q.i*q.i  + 2*q.j*q.j);
    }
    
    
    
    
    
};

#endif /* defined(__BasicPhysicsEngine__Matrix3__) */






















