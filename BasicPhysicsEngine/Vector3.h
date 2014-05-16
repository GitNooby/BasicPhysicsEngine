//
//  Vector3.h
//  BasicPhysicsEngine
//
//  Created by Kai Zou on 2014-05-15.
//  Copyright (c) 2014 com.personal. All rights reserved.
//

#ifndef __BasicPhysicsEngine__Vector3__
#define __BasicPhysicsEngine__Vector3__

#include <iostream>
#include "common.h"

class Vector3 {
public:
    real x, y, z;
private:
    real pad;
public:
    Vector3(){ x=0; y=0; z=0; }
    Vector3(real x, real y, real z){ this->x=x; this->y=y; this->z=z; }
    void invert() { x=-x; y=-y; z=-z; }
    real magnitude() const {
        return sqrt(x*x + y*y + z*z);
    }
    real magnitudeSquared() const {
        return x*x+y*y+z*z;
    }
    void normalize() {
        real len = magnitude();
        if (len > 0) {
            (*this) *= ((real)1)/len; // overloaded *=
        }
    }
    
    void clearVector() {
        x = y = z = 0;
    }
    
    // component-wise multiplication of two vectors
    Vector3 componentProduct(const Vector3 &vect) const {
        return Vector3(x*vect.x, y*vect.y, z*vect.z);
    }
    void componentProductUpdate(const Vector3 &vect) {
        x *= vect.x; y *= vect.y; z *= vect.z;
    }
    
    // dot product
    real dotProduct(const Vector3 &vect) const {
        return x*vect.x + y*vect.y + z*vect.z;
    }
    real operator*(const Vector3 &vect) const {
        return x*vect.x + y*vect.y + z*vect.z;
    }
    
    // cross product
    Vector3 crossProduct(const Vector3 &vect) const {
        return Vector3(y*vect.z - z*vect.y, z*vect.x - x*vect.z, x*vect.y - y*vect.x);
    }
    void operator%=(const Vector3 &vect) {
        *this = crossProduct(vect);
    }
    Vector3 operator%(const Vector3 &vect) const {
        return Vector3(y*vect.z - z*vect.y, z*vect.x - x*vect.z, x*vect.y - y*vect.x);
    }
    
    // generates orthonormal basis vectors from inputs
    bool generateOrthonormalBasis(Vector3 *A, Vector3 *B, Vector3 *C) {
        // returns true if orthonormal bases can be generated, false otherwise
        A->normalize();                                 // 1. normalize A
        *C = *A % *B;                                   // 2. find C from AxB
        if (C->magnitudeSquared() == 0.0) return false; // 3. if |C|==0, return error
        C->normalize();                                 // 4. normalize C
        *B = *C % *A;                                   // 5. generate B normalized from CxA
        return true;
    }
    
    // a short hand for: V1=V1+V2*s
    void addScaledVector(const Vector3 &vect, real scalar) {
        x = x + vect.x * scalar;
        y = y + vect.y * scalar;
        z = z + vect.z * scalar;
    }
    
    /* Overloaded operators */
    // vector scalar multiplication
    void operator*=(const real scalar) {
        x *= scalar; y *= scalar; z *= scalar;
    }
    // vector scalar multiplication
    Vector3 operator*(const real scalar) const {
        return Vector3(x*scalar, y*scalar, z*scalar);
    }
    // vector vector addition
    void operator+=(const Vector3 &vect) {
        x+=vect.x; y+=vect.y; z+=vect.z;
    }
    // vector vector addition
    Vector3 operator+(const Vector3 &vect) const {
        return Vector3(x+vect.x, y+vect.y, z+vect.z);
    }
    // vector vector subtraction
    void operator-=(const Vector3 &vect) {
        x-=vect.x; y-=vect.y; z-=vect.z;
    }
    // vector vector subtraction
    Vector3 operator-(const Vector3 &vect) const {
        return Vector3(x-vect.x, y-vect.y, z-vect.z);
    }
    
};

#endif /* defined(__BasicPhysicsEngine__Vector3__) */
