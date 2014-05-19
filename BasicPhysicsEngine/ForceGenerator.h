//
//  ForceGenerator.h
//  BasicPhysicsEngine
//
//  Created by Kai Zou on 2014-05-18.
//  Copyright (c) 2014 com.personal. All rights reserved.
//

#ifndef __BasicPhysicsEngine__ForceGenerator__
#define __BasicPhysicsEngine__ForceGenerator__

#include <iostream>
#include "RigidBody.h"

class ForceGenerator {
    virtual void updateForce(RigidBody *body, real duration)=0;
};

class Gravity : public ForceGenerator {
    Vector3 gravity;
public:
    Gravity(const Vector3 &g);
    virtual void updateForce(RigidBody *body, real duration);
};

class Spring : public ForceGenerator {
    Vector3 localConnectionPoint;     // connection point on object A in local coord
    Vector3 otherConnectionPoint; // connection point on object B in local coord
    RigidBody *other;
    real springConstant, restLength;
public:
    Spring(const Vector3 &localConnectionPt, RigidBody *otherBody, const Vector3 &otherConnectionPt, real springConstant, real restLength);
    virtual void updateForce(RigidBody *body, real duration);
};

#endif /* defined(__BasicPhysicsEngine__ForceGenerator__) */
