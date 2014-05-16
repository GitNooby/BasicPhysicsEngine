//
//  ParticleForceGenerator.h
//  BasicPhysicsEngine
//
//  Created by Kai Zou on 2014-05-15.
//  Copyright (c) 2014 com.personal. All rights reserved.
//

#ifndef __BasicPhysicsEngine__ParticleForceGenerator__
#define __BasicPhysicsEngine__ParticleForceGenerator__

#include <iostream>
#include "common.h"
#include "Particle.h"
#include "Vector3.h"
#include <vector>

// Abstract class for different types of force generators to inheirate
class ParticleForceGenerator {
public:
    virtual void updateForce(Particle *p, real deltaTime)=0;
};

// Gravity generator
class ParticleGravity : public ParticleForceGenerator {
    Vector3 gravity;
public:
    ParticleGravity(const Vector3 &gravity);
    virtual void updateForce(Particle *p, real deltaTime);
};

// Drag force generator
class ParticleDrag : public ParticleForceGenerator {
    real k1, k2;
public:
    ParticleDrag(real k1, real k2);
    virtual void updateForce(Particle *p, real deltaTime);
};

// Spring force with one end fixed
class ParticleAnchoredSpring : public ParticleForceGenerator {
protected:
    Vector3 *anchor;
    real springConst, restLength;
public:
    ParticleAnchoredSpring();
    ParticleAnchoredSpring(Vector3 *anchor, real springConst, real restLength);
    const Vector3* getAnchor() const {return anchor;}
    void init(Vector3 *anchor, real springConst, real restLength);
    virtual void updateForce(Particle *p, real deltaTime);
};

// Bungee force with one end fixed
class ParticleAnchoredBungee : public ParticleAnchoredSpring {
public:
    virtual void updateForce(Particle *p, real duration);
};

// Spring force between two particles
class ParticleSpring : public ParticleForceGenerator {
    Particle *otherParticle;
    real springConst, restLength;
public:
    ParticleSpring(Particle *otherParticle, real springConst, real restLength);
    virtual void updateForce(Particle *p, real deltaTime);
};

// Bungee force between two particles
class ParticleBungee : public ParticleForceGenerator {
    Particle* otherParticle;
    real springConst, restLength;
public:
    ParticleBungee(Particle *other, real springConst, real restLength);
    virtual void updateForce(Particle *p, real deltaTime);
};

//// Fake stiff spring force with one end fixed
//class ParticleFakeSpring : public ParticleForceGenerator {
//    Vector3 *anchor;
//    real springConst, damping;
//public:
//    ParticleFakeSpring(Vector3 *anchor, real springConst, real damping);
//    virtual void updateForce(Particle *p, real deltaTime);
//};

// Holds all the force generators and the particles they apply to
class ParticleForceRegistry {
protected:
    struct ParticleForceRegistration {
        Particle *particle;
        ParticleForceGenerator *forceGenerator;
    };
    typedef std::vector<ParticleForceRegistration> ForceRegistry;
    ForceRegistry forceRegistrations;
public:
    void add(Particle *p, ParticleForceGenerator *fg);
    void remove(Particle *p, ParticleForceGenerator *fg);
    void clearForceRegistry();
    void updateForces(real deltaTime);
};


#endif /* defined(__BasicPhysicsEngine__ParticleForceGenerator__) */
