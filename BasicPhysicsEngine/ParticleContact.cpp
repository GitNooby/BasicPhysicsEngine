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
    
    // if we have a closing velocity due to acceleration, remove it from the separating velocity (this is to deal with resting contacts)
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

ParticleContactResolver::ParticleContactResolver(unsigned iterations) {
    this->iterations = iterations;
}
void ParticleContactResolver::setIterations(unsigned iterations) {
    this->iterations = iterations;
}
void ParticleContactResolver::resolveContacts(ParticleContact *contactArray, unsigned numContacts, real deltaTime) {
    unsigned i;
    this->iterationsUsed = 0;
    while (iterationsUsed < iterations) {
        // find the contact with the largest closing velocity
        real max = REAL_MAX;
        unsigned maxIndex = numContacts;
        for (i=0; i<numContacts; i++) {
            real sepVel = contactArray[i].computeSeparatingVelocity();
            if (sepVel < max && (sepVel<0||contactArray[i].penetration>0)) {
                max=sepVel;
                maxIndex=i;
            }
        }
        
        // worth resolving?
        if (maxIndex == numContacts) break;
        
        // resolve this contact
        contactArray[maxIndex].resolveContact(deltaTime);
        
        // update interpenetrations for all particles
        Vector3 *move = contactArray[maxIndex].particleMovement;
        for (i=0; i<numContacts; i++) {
            if (contactArray[i].particle[0] == contactArray[maxIndex].particle[0]) {
                contactArray[i].penetration -= move[0] * contactArray[i].contactNormal;
            }
            else if (contactArray[i].particle[0] == contactArray[maxIndex].particle[1]) {
                contactArray[i].penetration -= move[1] * contactArray[i].contactNormal;
            }
            
            if (contactArray[i].particle[1]) {
                if (contactArray[i].particle[1] == contactArray[maxIndex].particle[0]) {
                    contactArray[i].penetration += move[0] * contactArray[i].contactNormal;
                }
                else if (contactArray[i].particle[1] == contactArray[maxIndex].particle[1]) {
                    contactArray[i].penetration += move[1] * contactArray[i].contactNormal;
                }
            }
        }
        iterationsUsed++;
    }
}



real ParticleLink::currentLength() const {
    return (particle[0]->getPosition()-particle[1]->getPosition()).magnitude();
}


unsigned ParticleRope::addContact(ParticleContact *contact, unsigned limit)const {
    real length = this->currentLength();
    if (length < maxLength) return 0;
    contact->particle[0] = particle[0];
    contact->particle[1] = particle[1];
    
    Vector3 contactNormal = particle[1]->getPosition()-particle[0]->getPosition();
    contactNormal.normalize();
    contact->contactNormal = contactNormal;
    contact->penetration = length-maxLength;
    contact->restitutionCoeff = restitution;
    return 1;
}



unsigned ParticleStick::addContact(ParticleContact *contact, unsigned limit)const {
    real currentLen = this->currentLength();
    if (currentLen == this->stickLength) return 0;
    contact->particle[0] = particle[0];
    contact->particle[1] = particle[1];
    Vector3 contactNormal = particle[1]->getPosition()-particle[0]->getPosition();
    contactNormal.normalize();
    if (currentLen > this->stickLength) {
        contact->contactNormal = contactNormal;
        contact->penetration = currentLen - this->stickLength;
    } else {
        contact->contactNormal = contactNormal * -1;
        contact->penetration = this->stickLength - currentLen;
    }
    contact->restitutionCoeff = 0;
    return 1;
}










