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

// An object to track a particular contact between two particles
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

// The engine used to resolve contacts
class ParticleContactResolver {
protected:
    unsigned iterations;
    unsigned iterationsUsed;
public:
    ParticleContactResolver(unsigned iterations);
    void setIterations(unsigned iterations);
    void resolveContacts(ParticleContact *contactArray, unsigned numContacts, real deltaTime);
};

// The base class engine used to detect types collisions
class ParticleContactGenerator {
public:
    virtual unsigned addContact(ParticleContact *contact, unsigned limit)=0;
};

// Base class for generating contacts between 2 particles
class ParticleLink : public ParticleContactGenerator {
public:
    Particle* particle[2];
protected:
    real currentLength() const;
public:
    virtual unsigned addContact(ParticleContact *contact, unsigned limit)const=0;
};

// Rope class, subclass of ParticleLink
class ParticleRope : public ParticleLink {
public:
    real maxLength;
    real restitution;
public:
    virtual unsigned addContact(ParticleContact *contact, unsigned limit)const;
};

// Rigid stick, subclass of ParticleLink
class ParticleStick : public ParticleLink {
public:
    real stickLength;
    virtual unsigned addContact(ParticleContact *contact, unsigned limit)const;
};



#endif /* defined(__BasicPhysicsEngine__ParticleContact__) */






















