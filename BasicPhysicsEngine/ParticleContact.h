//
//  ParticleContact.h
//  BasicPhysicsEngine
//
//  Created by Kai Zou on 2014-05-16.
//  Copyright (c) 2014 com.personal. All rights reserved.
//

#ifndef __BasicPhysicsEngine__ParticleContact__
#define __BasicPhysicsEngine__ParticleContact__

#include <iostream>
#include "Particle.h"

class ParticleContact {
public:
    Particle *particle[2];
    real restitutionCoeff;
    Vector3 contactNormal;
    real penetration;
    Vector3 particleMovement[2];
protected:
    void resolveContact(real deltaTime);
    real computeSeparatingVelocity() const;
private:
    void resolveVelocity(real deltaTime);
    void resolveInterpenetration(real deltaTime);
};

#endif /* defined(__BasicPhysicsEngine__ParticleContact__) */
