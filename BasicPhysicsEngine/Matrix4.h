//
//  Matrix4.h
//  BasicPhysicsEngine
//
//  Created by Kai Zou on 2014-05-17.
//  Copyright (c) 2014 com.personal. All rights reserved.
//

#ifndef __BasicPhysicsEngine__Matrix4__
#define __BasicPhysicsEngine__Matrix4__

#include <iostream>
#include "common.h"
#include "Vector3.h"
#include "Quaternion.h"

class Matrix4 {
public:
    real val[12];

    Matrix4() {
        val[1]=val[2]=val[3]=val[4]=val[6]=val[7]=val[8]=val[9]=val[11]=0;
        val[0]=val[5]=val[10]=1;
    }
    
    void setDiagonal(real a, real b, real c) {
        val[0] = a; val[5] = b; val[10] = c;
    }
    
    
    // matrix vector post multiplication
    inline Vector3 operator*(const Vector3 &v)const {
        real x = v.x*val[0]+v.y*val[1]+v.z*val[2] + val[3];
        real y = v.x*val[4]+v.y*val[5]+v.z*val[6] + val[7];
        real z = v.x*val[8]+v.y*val[9]+v.z*val[10] + val[11];
        return Vector3(x, y, z);
    }
    inline Vector3 transform(const Vector3 &v)const {
        return (*this)*v;
    }
    
    // matrix matrix multiplication
    inline Matrix4 operator*(const Matrix4 &M) const {
        Matrix4 result;
        result.val[0] = (M.val[0]*val[0]) + (M.val[4]*val[1]) + (M.val[8]*val[2]);
        result.val[4] = (M.val[0]*val[4]) + (M.val[4]*val[5]) + (M.val[8]*val[6]);
        result.val[8] = (M.val[0]*val[8]) + (M.val[4]*val[9]) + (M.val[8]*val[10]);
        result.val[1] = (M.val[1]*val[0]) + (M.val[5]*val[1]) + (M.val[9]*val[2]);
        result.val[5] = (M.val[1]*val[4]) + (M.val[5]*val[5]) + (M.val[9]*val[6]);
        result.val[9] = (M.val[1]*val[8]) + (M.val[5]*val[9]) + (M.val[9]*val[10]);
        result.val[2] = (M.val[2]*val[0]) + (M.val[6]*val[1]) + (M.val[10]*val[2]);
        result.val[6] = (M.val[2]*val[4]) + (M.val[6]*val[5]) + (M.val[10]*val[6]);
        result.val[10] = (M.val[2]*val[8]) + (M.val[6]*val[9]) + (M.val[10]*val[10]);
        result.val[3] = (M.val[3]*val[0]) + (M.val[7]*val[1]) + (M.val[11]*val[2]) + val[3];
        result.val[7] = (M.val[3]*val[4]) + (M.val[7]*val[5]) + (M.val[11]*val[6]) + val[7];
        result.val[11] = (M.val[3]*val[8]) + (M.val[7]*val[9]) + (M.val[11]*val[10]) + val[11];
        return result;
    }
    
    inline real getDeterminant()const {
        return -val[8]*val[5]*val[2]+val[4]*val[9]*val[2]+val[8]*val[1]*val[6]-val[0]*val[9]*val[6]-val[4]*val[1]*val[10]+val[0]*val[5]*val[10];
    }
    
    inline void setInverse(const Matrix4 &m) {
        // Make sure the determinant is non-zero.
        real det = getDeterminant();
        if (det == 0) return;
        det = ((real)1.0)/det;
        
        val[0] = (-m.val[9]*m.val[6]+m.val[5]*m.val[10])*det;
        val[4] = (m.val[8]*m.val[6]-m.val[4]*m.val[10])*det;
        val[8] = (-m.val[8]*m.val[5]+m.val[4]*m.val[9])*det;
        
        val[1] = (m.val[9]*m.val[2]-m.val[1]*m.val[10])*det;
        val[5] = (-m.val[8]*m.val[2]+m.val[0]*m.val[10])*det;
        val[9] = (m.val[8]*m.val[1]-m.val[0]*m.val[9])*det;
        
        val[2] = (-m.val[5]*m.val[2]+m.val[1]*m.val[6])*det;
        val[6] = (+m.val[4]*m.val[2]-m.val[0]*m.val[6])*det;
        val[10] = (-m.val[4]*m.val[1]+m.val[0]*m.val[5])*det;
        
        val[3] = (m.val[9]*m.val[6]*m.val[3]
                   -m.val[5]*m.val[10]*m.val[3]
                   -m.val[9]*m.val[2]*m.val[7]
                   +m.val[1]*m.val[10]*m.val[7]
                   +m.val[5]*m.val[2]*m.val[11]
                   -m.val[1]*m.val[6]*m.val[11])*det;
        val[7] = (-m.val[8]*m.val[6]*m.val[3]
                   +m.val[4]*m.val[10]*m.val[3]
                   +m.val[8]*m.val[2]*m.val[7]
                   -m.val[0]*m.val[10]*m.val[7]
                   -m.val[4]*m.val[2]*m.val[11]
                   +m.val[0]*m.val[6]*m.val[11])*det;
        val[11] =(m.val[8]*m.val[5]*m.val[3]
                   -m.val[4]*m.val[9]*m.val[3]
                   -m.val[8]*m.val[1]*m.val[7]
                   +m.val[0]*m.val[9]*m.val[7]
                   +m.val[4]*m.val[1]*m.val[11]
                   -m.val[0]*m.val[5]*m.val[11])*det;
    }
    inline Matrix4 inverse()const {
        Matrix4 result;
        result.setInverse(*this);
        return result;
    }
    inline void invert() {
        setInverse(*this);
    }
    
    inline Vector3 transformDirection(const Vector3 &vector) const {
        return Vector3(vector.x*val[0]+vector.y*val[1]+vector.z*val[2],
                       vector.x*val[4]+vector.y*val[5]+vector.z*val[6],
                       vector.x*val[8]+vector.y*val[9]+vector.z*val[10]);
    }
    inline Vector3 transformInverseDirection(const Vector3 &vector) const
    {
        return Vector3(vector.x*val[0] + vector.y*val[4]+vector.z*val[8],
                       vector.x*val[1]+vector.y*val[5]+vector.z*val[9],
                       vector.x*val[2]+vector.y*val[6]+vector.z*val[10]);
    }
    inline Vector3 transformInverse(const Vector3 &vector) const
    {
        Vector3 tmp = vector;
        tmp.x -= val[3];
        tmp.y -= val[7];
        tmp.z -= val[11];
        return Vector3(tmp.x*val[0]+tmp.y*val[4]+tmp.z*val[8],
                       tmp.x*val[1]+tmp.y*val[5]+tmp.z*val[9],
                       tmp.x*val[2]+tmp.y*val[6]+tmp.z*val[10]);
    }
    
    inline Vector3 getAxisVector(int i) const {
        return Vector3(val[i], val[i+4], val[i+8]);
    }
    
    inline void setOrientationAndPos(const Quaternion &q, const Vector3 &pos) {
        val[0] = 1 - (2*q.j*q.j + 2*q.k*q.k);
        val[1] = 2*q.i*q.j + 2*q.k*q.r;
        val[2] = 2*q.i*q.k - 2*q.j*q.r;
        val[3] = pos.x;
        
        val[4] = 2*q.i*q.j - 2*q.k*q.r;
        val[5] = 1 - (2*q.i*q.i  + 2*q.k*q.k);
        val[6] = 2*q.j*q.k + 2*q.i*q.r;
        val[7] = pos.y;
        
        val[8] = 2*q.i*q.k + 2*q.j*q.r;
        val[9] = 2*q.j*q.k - 2*q.i*q.r;
        val[10] = 1 - (2*q.i*q.i  + 2*q.j*q.j);
        val[11] = pos.z;
    }
    
    inline void fillGLArray(float array[16]) const {
        array[0] = (float)val[0];
        array[1] = (float)val[4];
        array[2] = (float)val[8];
        array[3] = (float)0;
        
        array[4] = (float)val[1];
        array[5] = (float)val[5];
        array[6] = (float)val[9];
        array[7] = (float)0;
        
        array[8] = (float)val[2];
        array[9] = (float)val[6];
        array[10] = (float)val[10];
        array[11] = (float)0;
        
        array[12] = (float)val[3];
        array[13] = (float)val[7];
        array[14] = (float)val[11];
        array[15] = (float)1;
    }

};

#endif /* defined(__BasicPhysicsEngine__Matrix4__) */
