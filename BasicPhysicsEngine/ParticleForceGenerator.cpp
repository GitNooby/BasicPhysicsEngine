//
//  ParticleForceGenerator.cpp
//  BasicPhysicsEngine
//
//  Created by Kai Zou on 2014-05-15.
//  Copyright (c) 2014 com.personal. All rights reserved.
//

#include "ParticleForceGenerator.h"


void ParticleForceRegistry::updateForces(real deltaTime) {
    ForceRegistry::iterator it = forceRegistrations.begin();
    for (; it != forceRegistrations.end(); it++) {
        it->forceGenerator->updateForce(it->particle, deltaTime);
    }
}
void ParticleForceRegistry::add(Particle *p, ParticleForceGenerator *fg) {
    ParticleForceRegistry::ParticleForceRegistration aRegistration;
    aRegistration.particle = p;
    aRegistration.forceGenerator = fg;
    this->forceRegistrations.push_back(aRegistration);
}

ParticleGravity::ParticleGravity(const Vector3 &gravity) {
    ParticleGravity::gravity = gravity;
}
void ParticleGravity::updateForce(Particle *p, real deltaTime) {
    if (p->hasFiniteMass() == false) return;
    p->sumForce(gravity*p->getMass());
}

ParticleDrag::ParticleDrag(real k1, real k2) {
    ParticleDrag::k1 = k1; ParticleDrag::k2 = k2;
}
void ParticleDrag::updateForce(Particle *p, real deltaTime) {
    Vector3 force;
    p->getVelocity(&force);
    real dragCoeff = force.magnitude();
    dragCoeff = k1*dragCoeff + k2*dragCoeff*dragCoeff;
    force.normalize();
    force *= -dragCoeff;
    p->sumForce(force);
}

ParticleAnchoredSpring::ParticleAnchoredSpring() {
}
ParticleAnchoredSpring::ParticleAnchoredSpring(Vector3 *anchor, real springConst, real restLength) {
    this->init(anchor, springConst, restLength);
}
void ParticleAnchoredSpring::init(Vector3 *anchor, real springConst, real restLength) {
    ParticleAnchoredSpring::anchor = anchor;
    ParticleAnchoredSpring::springConst = springConst;
    ParticleAnchoredSpring::restLength = restLength;
}
void ParticleAnchoredSpring::updateForce(Particle *p, real deltaTime) {
    // compute spring vector
    Vector3 force;
    p->getPosition(&force);
    force -= *anchor;
    // compute force magnitude
    real magnitude = force.magnitude();
    magnitude = (restLength - magnitude) * springConst;
    // compute the force and apply
    force.normalize();
    force *= magnitude;
    p->sumForce(force);
}

void ParticleAnchoredBungee::updateForce(Particle *p, real deltaTime){
    // compute bungee vector
    Vector3 force;
    p->getPosition(&force);
    force -= *anchor;
    // compute force mangitude
    real magnitude = force.magnitude();
    if (magnitude < restLength) return;
    magnitude = magnitude - restLength;
    magnitude *= springConst;
    // compute the force and apply
    force.normalize();
    force *= -magnitude;
    p->sumForce(force);
}

ParticleSpring::ParticleSpring(Particle *other, real sc, real rl)
: otherParticle(other), springConst(sc), restLength(rl) {
}
void ParticleSpring::updateForce(Particle* particle, real duration) {
    // Calculate the vector of the spring
    Vector3 force;
    particle->getPosition(&force);
    force -= otherParticle->getPosition();
    // Calculate the magnitude of the force
    real magnitude = force.magnitude();
    magnitude = abs(magnitude - restLength);
    magnitude *= springConst;
    // Calculate the final force and apply it
    force.normalize();
    force *= -magnitude;
    particle->sumForce(force);
}

ParticleBungee::ParticleBungee(Particle *other, real sc, real rl)
: otherParticle(other), springConst(sc), restLength(rl) {
}
void ParticleBungee::updateForce(Particle* particle, real duration)
{
    // Calculate the vector of the spring
    Vector3 force;
    particle->getPosition(&force);
    force -= otherParticle->getPosition();
    // Check if the bungee is compressed
    real magnitude = force.magnitude();
    if (magnitude <= restLength) return;
    // Calculate the magnitude of the force
    magnitude = springConst * (restLength - magnitude);
    // Calculate the final force and apply it
    force.normalize();
    force *= -magnitude;
    particle->sumForce(force);
}

//ParticleFakeSpring::ParticleFakeSpring(Vector3 *anchor, real springConst, real damping) {
//    ParticleFakeSpring::anchor = anchor;
//    ParticleFakeSpring::springConst = springConst;
//    ParticleFakeSpring::damping = damping;
//}
//void ParticleFakeSpring::updateForce(Particle* p, real deltaTime) {
//    if (!p->hasFiniteMass()) return;
//    // Calculate the relative position of the particle to the anchor
//    Vector3 position = p->getPosition();
//    position -= *anchor;
//    // Calculate the constants and check they are in bounds.
//    real gamma = 0.5f * sqrt(4 * springConst - damping*damping);
//    if (gamma == 0.0f) return;
//    Vector3 c = position * (damping / (2.0f * gamma)) +
//    p->getVelocity() * (1.0f / gamma);
//    // Calculate the target position
//    Vector3 target = position * cos(gamma * deltaTime) +
//    c * sin(gamma * deltaTime);
//    target *= exp(-0.5f * deltaTime * damping);
//    // Calculate the resulting acceleration and therefore the force
//    Vector3 accel = (target - position) * ((real)1.0 / (deltaTime*deltaTime)) -
//    p->getVelocity() * ((real)1.0/deltaTime);
//    p->sumForce(accel * p->getMass());
//}
































