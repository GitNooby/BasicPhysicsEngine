//
//  particle.cpp
//  BasicPhysicsEngine
//
//  Created by Kai Zou on 2014-05-15.
//  Copyright (c) 2014 com.personal. All rights reserved.
//

#include "Particle.h"

void Particle::integrate(real deltaTime) {
    if (inverseMass <= 0.0) return;
    if (deltaTime < 0.0) return;
    
    position.addScaledVector(velocity, deltaTime);
    
    Vector3 newAcceleration = acceleration;
    newAcceleration.addScaledVector(netForce, inverseMass);
    
    velocity.addScaledVector(newAcceleration, deltaTime);
    velocity *= pow(dampingFactor, deltaTime); // for stability
    this->clearNetForce();
}
void Particle::clearNetForce() {
    netForce.clearVector();
}
void Particle::sumForce(const Vector3 &f) {
    netForce += f;
}

// mass getters and setters
void Particle::setMass(const real mass) {
    assert(mass != 0);
    this->inverseMass = ((real)1.0)/mass;
}
real Particle::getMass() const {
    if (inverseMass == 0) return REAL_MAX;
    return ((real)1.0)/inverseMass;
}
void Particle::setInverseMass(const real im) {
    inverseMass = im;
}
real Particle::getInverseMass() const {
    return inverseMass;
}
bool Particle::hasFiniteMass() const {
    return inverseMass >= 0.0;
}

// damping factor getters and setters
void Particle::setDamping(const real damping) {
    dampingFactor = damping;
}
real Particle::getDamping() const {
    return dampingFactor;
}

// position getters and setters
void Particle::setPosition(const Vector3 &p) {
    position = p;
}
void Particle::setPosition(const real x, const real y, const real z) {
    position.x=x; position.y=y; position.z=z;
}
void Particle::getPosition(Vector3 *p) {
    *p = position;
}
Vector3 Particle::getPosition() const {
    return position;
}

// velocity getters and setters
void Particle::setVelocity(const Vector3 &v) {
    velocity = v;
}
void Particle::setVelocity(const real x, const real y, const real z) {
    velocity.x=x; velocity.y=y; velocity.z=z;
}
void Particle::getVelocity(Vector3 *v) {
    *v = velocity;
}
Vector3 Particle::getVelocity() {
    return velocity;
}

// acceleration getters and setters
void Particle::setAcceleration(const Vector3 &a) {
    acceleration = a;
}
void Particle::setAcceleration(const real x, const real y, const real z) {
    acceleration.x=x; acceleration.y=y; acceleration.z=z;
}
void Particle::getAcceleration(Vector3 *a) {
    *a = acceleration;
}
Vector3 Particle::getAcceleration() const {
    return acceleration;
}





















