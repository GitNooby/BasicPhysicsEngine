//
//  particle.h
//  BasicPhysicsEngine
//
//  Created by Kai Zou on 2014-05-15.
//  Copyright (c) 2014 com.personal. All rights reserved.
//

#ifndef __BasicPhysicsEngine__particle__
#define __BasicPhysicsEngine__particle__

#include <iostream>
#include "Vector3.h"

class Particle {
protected:
    Vector3 position;
    Vector3 velocity;
    Vector3 acceleration;
    real dampingFactor;
    real inverseMass;
    Vector3 netForce;
public:
    // particle operators
    void integrate(real deltaTime);
    void clearNetForce();
    void sumForce(const Vector3 &f);

    // mass getters and setters
    void setMass(const real mass);
    real getMass() const;
    void setInverseMass(const real im);
    real getInverseMass() const;
    bool hasFiniteMass() const;
    
    // damping factor getters and setters
    void setDamping(const real damping);
    real getDamping() const;
    
    // position getters and setters
    void setPosition(const Vector3 &p);
    void setPosition(const real x, const real y, const real z);
    void getPosition(Vector3 *p);
    Vector3 getPosition() const;
    
    // velocity getters and setters
    void setVelocity(const Vector3 &v);
    void setVelocity(const real x, const real y, const real z);
    void getVelocity(Vector3 *v);
    Vector3 getVelocity();
    
    // acceleration getters and setters
    void setAcceleration(const Vector3 &a);
    void setAcceleration(const real x, const real y, const real z);
    void getAcceleration(Vector3 *a);
    Vector3 getAcceleration() const;
    
    
};

#endif /* defined(__BasicPhysicsEngine__particle__) */







































