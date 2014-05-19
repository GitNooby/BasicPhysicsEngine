//
//  Quaternion.h
//  BasicPhysicsEngine
//
//  Created by Kai Zou on 2014-05-17.
//  Copyright (c) 2014 com.personal. All rights reserved.
//

#ifndef __BasicPhysicsEngine__Quaternion__
#define __BasicPhysicsEngine__Quaternion__

#include <iostream>
#include "common.h"
#include "Vector3.h"

class Quaternion {
public:
    union {
        struct {
            real r; real i; real j; real k;
        };
        real data[4];
    };
    
    Quaternion() {r=1;i=j=k=0;}
    Quaternion(const real r, const real i, const real j, const real k) {
        this->r=r; this->i=i; this->j=j; this->k=k;
    }
    
    inline void normalize() {
        real d = r*r+i*i+j*j+k*k;
        // Check for zero length quaternion, and use the no-rotation
        // quaternion in that case.
        if (d < DBL_EPSILON) {
            r = 1;
            return;
        }
        d = ((real)1.0)/sqrt(d);
        r *= d;
        i *= d;
        j *= d;
        k *= d;
    }
    
    inline void operator *=(const Quaternion &multiplier) {
        Quaternion q = *this;
        r = q.r*multiplier.r - q.i*multiplier.i - q.j*multiplier.j - q.k*multiplier.k;
        i = q.r*multiplier.i + q.i*multiplier.r + q.j*multiplier.k - q.k*multiplier.j;
        j = q.r*multiplier.j + q.j*multiplier.r + q.k*multiplier.i - q.i*multiplier.k;
        k = q.r*multiplier.k + q.k*multiplier.r + q.i*multiplier.j - q.j*multiplier.i;
    }
    
    inline void addScaledVector(const Vector3& vector, real scale) {
        Quaternion q(0,
                     vector.x * scale,
                     vector.y * scale,
                     vector.z * scale);
        q *= *this;
        r += q.r * ((real)0.5);
        i += q.i * ((real)0.5);
        j += q.j * ((real)0.5);
        k += q.k * ((real)0.5);
    }
    
    inline void rotateByVector(const Vector3& vector) {
        Quaternion q(0, vector.x, vector.y, vector.z);
        (*this) *= q;
    }
};


#endif /* defined(__BasicPhysicsEngine__Quaternion__) */
