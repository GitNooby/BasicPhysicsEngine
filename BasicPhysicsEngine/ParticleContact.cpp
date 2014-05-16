//
//  ParticleContact.cpp
//  BasicPhysicsEngine
//
//  Created by Kai Zou on 2014-05-16.
//  Copyright (c) 2014 com.personal. All rights reserved.
//

#include "ParticleContact.h"


void ParticleContact::resolveContact(real deltaTime) {
    this->resolveVelocity(deltaTime);
    this->resolveInterpenetration(deltaTime);
}
real ParticleContact::computeSeparatingVelocity() const {
    Vector3 relativeVelocity = this->particle[0]->getVelocity();
    if (this->particle[1] != nullptr) relativeVelocity -= this->particle[1]->getVelocity();
    return relativeVelocity * contactNormal;
}
void ParticleContact::resolveVelocity(real deltaTime) {
    real separatingVelocity = this->computeSeparatingVelocity();
    if (separatingVelocity >= 0) {
        return;
    }
    real newSeparatingVelocity = -separatingVelocity * this->restitutionCoeff;
    
    // check the velocity buildup due to acceleration only
    Vector3 accelerationCausedVelocity = this->particle[0]->getAcceleration();
    if (this->particle[1]) {
        accelerationCausedVelocity -= this->particle[1]->getAcceleration();
    }
    real accelerationCausedSeparationVelocity = accelerationCausedVelocity * contactNormal * deltaTime;
    
    // if we have a closing velocity due to acceleration, remove it from the separating velocity
    if (accelerationCausedSeparationVelocity < 0) {
        newSeparatingVelocity += restitutionCoeff * accelerationCausedSeparationVelocity;
        if (newSeparatingVelocity < 0) newSeparatingVelocity = 0;
    }
    
    real deltaVelocity = newSeparatingVelocity - separatingVelocity;
    
    // apply change in velocity to each object in proportion to their inverse mass
    real totalInverseMass = particle[0]->getInverseMass();
    if (particle[1] != nullptr) totalInverseMass += particle[1]->getInverseMass();
    if (totalInverseMass <= 0) return;
    
    real impulse = deltaVelocity / totalInverseMass;
    Vector3 impulsePerInverseMass = contactNormal * impulse;
    
    // apply the impulses
    particle[0]->setVelocity(particle[0]->getVelocity() + impulsePerInverseMass * particle[0]->getInverseMass());
    if (particle[1] != nullptr) {
        particle[1]->setVelocity(particle[1]->getVelocity() + impulsePerInverseMass * -particle[1]->getInverseMass());
    }
}

void ParticleContact::resolveInterpenetration(real deltaTime) {
    if (penetration <= 0) return;
    
    real totalInverseMass = particle[0]->getInverseMass();
    if (particle[1]) totalInverseMass += particle[1]->getInverseMass();
    if (totalInverseMass <= 0) return;
    
    // Find the amount of penetration resolution per unit of inverse mass
    Vector3 movePerIMass = contactNormal * (penetration / totalInverseMass);
    
    // Calculate the the movement amounts
    particleMovement[0] = movePerIMass * particle[0]->getInverseMass();
    if (particle[1]) {
        particleMovement[1] = movePerIMass * -particle[1]->getInverseMass();
    } else {
        particleMovement[1].clearVector();
    }
    
    // Apply the penetration resolution
    particle[0]->setPosition(particle[0]->getPosition() + particleMovement[0]);
    if (particle[1]) {
        particle[1]->setPosition(particle[1]->getPosition() + particleMovement[1]);
    }
}





















