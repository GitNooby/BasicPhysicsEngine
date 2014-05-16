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

class ParticleContactResolver;

class ParticleContact {
    friend class ParticleContactResolver;
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

class ParticleContactResolver {
protected:
    unsigned iterations;
    unsigned iterationsUsed;
public:
    ParticleContactResolver(unsigned iterations);
    void setIterations(unsigned iterations);
    void resolveContacts(ParticleContact *contactArray, unsigned numContacts, real deltaTime);
};

class ParticleContactGenerator {
public:
    virtual unsigned addContact(ParticleContact *contact, unsigned limit)=0;
};

#endif /* defined(__BasicPhysicsEngine__ParticleContact__) */
