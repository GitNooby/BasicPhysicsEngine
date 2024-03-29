//
//  RigidBody.cpp
//  BasicPhysicsEngine
//
//  Created by Kai Zou on 2014-05-18.
//  Copyright (c) 2014 com.personal. All rights reserved.
//

#include "RigidBody.h"

/**
 * Internal function that checks the validity of an inverse inertia tensor.
 */
static inline void _checkInverseInertiaTensor(const Matrix3 &iitWorld) {
    // TODO: Perform a validity check in an assert.
}

/**
 * Internal function to do an intertia tensor transform by a quaternion.
 * Note that the implementation of this function was created by an
 * automated code-generator and optimizer.
 */
static inline void _transformInertiaTensor(Matrix3 &iitWorld,
                                           const Quaternion &q,
                                           const Matrix3 &iitBody,
                                           const Matrix4 &rotmat) {
    real t4 = rotmat.val[0]*iitBody.val[0]+
    rotmat.val[1]*iitBody.val[3]+
    rotmat.val[2]*iitBody.val[6];
    real t9 = rotmat.val[0]*iitBody.val[1]+
    rotmat.val[1]*iitBody.val[4]+
    rotmat.val[2]*iitBody.val[7];
    real t14 = rotmat.val[0]*iitBody.val[2]+
    rotmat.val[1]*iitBody.val[5]+
    rotmat.val[2]*iitBody.val[8];
    real t28 = rotmat.val[4]*iitBody.val[0]+
    rotmat.val[5]*iitBody.val[3]+
    rotmat.val[6]*iitBody.val[6];
    real t33 = rotmat.val[4]*iitBody.val[1]+
    rotmat.val[5]*iitBody.val[4]+
    rotmat.val[6]*iitBody.val[7];
    real t38 = rotmat.val[4]*iitBody.val[2]+
    rotmat.val[5]*iitBody.val[5]+
    rotmat.val[6]*iitBody.val[8];
    real t52 = rotmat.val[8]*iitBody.val[0]+
    rotmat.val[9]*iitBody.val[3]+
    rotmat.val[10]*iitBody.val[6];
    real t57 = rotmat.val[8]*iitBody.val[1]+
    rotmat.val[9]*iitBody.val[4]+
    rotmat.val[10]*iitBody.val[7];
    real t62 = rotmat.val[8]*iitBody.val[2]+
    rotmat.val[9]*iitBody.val[5]+
    rotmat.val[10]*iitBody.val[8];
    
    iitWorld.val[0] = t4*rotmat.val[0]+
    t9*rotmat.val[1]+
    t14*rotmat.val[2];
    iitWorld.val[1] = t4*rotmat.val[4]+
    t9*rotmat.val[5]+
    t14*rotmat.val[6];
    iitWorld.val[2] = t4*rotmat.val[8]+
    t9*rotmat.val[9]+
    t14*rotmat.val[10];
    iitWorld.val[3] = t28*rotmat.val[0]+
    t33*rotmat.val[1]+
    t38*rotmat.val[2];
    iitWorld.val[4] = t28*rotmat.val[4]+
    t33*rotmat.val[5]+
    t38*rotmat.val[6];
    iitWorld.val[5] = t28*rotmat.val[8]+
    t33*rotmat.val[9]+
    t38*rotmat.val[10];
    iitWorld.val[6] = t52*rotmat.val[0]+
    t57*rotmat.val[1]+
    t62*rotmat.val[2];
    iitWorld.val[7] = t52*rotmat.val[4]+
    t57*rotmat.val[5]+
    t62*rotmat.val[6];
    iitWorld.val[8] = t52*rotmat.val[8]+
    t57*rotmat.val[9]+
    t62*rotmat.val[10];
}

/**
 * Inline function that creates a transform matrix from a
 * position and orientation.
 */
static inline void _calculateTransformMatrix(Matrix4 &transformMatrix,
                                             const Vector3 &position,
                                             const Quaternion &orientation) {
    transformMatrix.val[0] = 1-2*orientation.j*orientation.j-
    2*orientation.k*orientation.k;
    transformMatrix.val[1] = 2*orientation.i*orientation.j -
    2*orientation.r*orientation.k;
    transformMatrix.val[2] = 2*orientation.i*orientation.k +
    2*orientation.r*orientation.j;
    transformMatrix.val[3] = position.x;
    
    transformMatrix.val[4] = 2*orientation.i*orientation.j +
    2*orientation.r*orientation.k;
    transformMatrix.val[5] = 1-2*orientation.i*orientation.i-
    2*orientation.k*orientation.k;
    transformMatrix.val[6] = 2*orientation.j*orientation.k -
    2*orientation.r*orientation.i;
    transformMatrix.val[7] = position.y;
    
    transformMatrix.val[8] = 2*orientation.i*orientation.k -
    2*orientation.r*orientation.j;
    transformMatrix.val[9] = 2*orientation.j*orientation.k +
    2*orientation.r*orientation.i;
    transformMatrix.val[10] = 1-2*orientation.i*orientation.i-
    2*orientation.j*orientation.j;
    transformMatrix.val[11] = position.z;
}


void RigidBody::calculateDerivedData() {
    orientation.normalize();
    
    // Calculate the transform matrix for the body.
    _calculateTransformMatrix(transformMatrix, position, orientation);
    
    // Calculate the inertiaTensor in world space.
    _transformInertiaTensor(inverseInertiaTensorWorld, orientation, inverseInertiaTensor, transformMatrix);
}

void RigidBody::integrate(real duration) {
    if (!isAwake) return;
    
    // Calculate linear acceleration from force inputs.
    lastFrameAcceleration = acceleration;
    lastFrameAcceleration.addScaledVector(forceAccum, inverseMass);
    
    // Calculate angular acceleration from torque inputs.
    Vector3 angularAcceleration =
    inverseInertiaTensorWorld.transform(torqueAccum);
    
    // Adjust velocities
    // Update linear velocity from both acceleration and impulse.
    velocity.addScaledVector(lastFrameAcceleration, duration);
    
    // Update angular velocity from both acceleration and impulse.
    rotation.addScaledVector(angularAcceleration, duration);
    
    // Impose drag.
    velocity *= pow(linearDamping, duration);
    rotation *= pow(angularDamping, duration);
    
    // Adjust positions
    // Update linear position.
    position.addScaledVector(velocity, duration);
    
    // Update angular position.
    orientation.addScaledVector(rotation, duration);
    
    // Normalise the orientation, and update the matrices with the new
    // position and orientation
    calculateDerivedData();
    
    // Clear accumulators.
    clearAccumulators();
    
    // Update the kinetic energy store, and possibly put the body to
    // sleep.
    if (canSleep) {
        real currentMotion = velocity.dotProduct(velocity) +
        rotation.dotProduct(rotation);
        
        real bias = pow(0.5, duration);
        motion = bias*motion + (1-bias)*currentMotion;
        
        if (motion < 0.01) setAwake(false);
        else if (motion > 10 * 0.01) motion = 10 * 0.01;
    }
}

void RigidBody::setMass(const real mass) {
    assert(mass != 0);
    RigidBody::inverseMass = ((real)1.0)/mass;
}

real RigidBody::getMass() const {
    if (inverseMass == 0) {
        return REAL_MAX;
    } else {
        return ((real)1.0)/inverseMass;
    }
}

void RigidBody::setInverseMass(const real inverseMass) {
    RigidBody::inverseMass = inverseMass;
}

real RigidBody::getInverseMass() const {
    return inverseMass;
}

bool RigidBody::hasFiniteMass() const {
    return inverseMass >= 0.0f;
}

void RigidBody::setInertiaTensor(const Matrix3 &inertiaTensor) {
    inverseInertiaTensor.setInverse(inertiaTensor);
    _checkInverseInertiaTensor(inverseInertiaTensor);
}

void RigidBody::getInertiaTensor(Matrix3 *inertiaTensor) const {
    inertiaTensor->setInverse(inverseInertiaTensor);
}

Matrix3 RigidBody::getInertiaTensor() const {
    Matrix3 it;
    getInertiaTensor(&it);
    return it;
}

void RigidBody::getInertiaTensorWorld(Matrix3 *inertiaTensor) const {
    inertiaTensor->setInverse(inverseInertiaTensorWorld);
}

Matrix3 RigidBody::getInertiaTensorWorld() const {
    Matrix3 it;
    getInertiaTensorWorld(&it);
    return it;
}

void RigidBody::setInverseInertiaTensor(const Matrix3 &inverseInertiaTensor) {
    _checkInverseInertiaTensor(inverseInertiaTensor);
    RigidBody::inverseInertiaTensor = inverseInertiaTensor;
}

void RigidBody::getInverseInertiaTensor(Matrix3 *inverseInertiaTensor) const {
    *inverseInertiaTensor = RigidBody::inverseInertiaTensor;
}

Matrix3 RigidBody::getInverseInertiaTensor() const {
    return inverseInertiaTensor;
}

void RigidBody::getInverseInertiaTensorWorld(Matrix3 *inverseInertiaTensor) const {
    *inverseInertiaTensor = inverseInertiaTensorWorld;
}

Matrix3 RigidBody::getInverseInertiaTensorWorld() const {
    return inverseInertiaTensorWorld;
}

void RigidBody::setDamping(const real linearDamping, const real angularDamping) {
    RigidBody::linearDamping = linearDamping;
    RigidBody::angularDamping = angularDamping;
}

void RigidBody::setLinearDamping(const real linearDamping) {
    RigidBody::linearDamping = linearDamping;
}

real RigidBody::getLinearDamping() const {
    return linearDamping;
}

void RigidBody::setAngularDamping(const real angularDamping) {
    RigidBody::angularDamping = angularDamping;
}

real RigidBody::getAngularDamping() const {
    return angularDamping;
}

void RigidBody::setPosition(const Vector3 &position) {
    RigidBody::position = position;
}

void RigidBody::setPosition(const real x, const real y, const real z) {
    position.x = x;
    position.y = y;
    position.z = z;
}

void RigidBody::getPosition(Vector3 *position) const {
    *position = RigidBody::position;
}

Vector3 RigidBody::getPosition() const {
    return position;
}

void RigidBody::setOrientation(const Quaternion &orientation) {
    RigidBody::orientation = orientation;
    RigidBody::orientation.normalize();
}

void RigidBody::setOrientation(const real r, const real i, const real j, const real k) {
    orientation.r = r;
    orientation.i = i;
    orientation.j = j;
    orientation.k = k;
    orientation.normalize();
}

void RigidBody::getOrientation(Quaternion *orientation) const {
    *orientation = RigidBody::orientation;
}

Quaternion RigidBody::getOrientation() const {
    return orientation;
}

void RigidBody::getOrientation(Matrix3 *matrix) const {
    getOrientation(matrix->val);
}

void RigidBody::getOrientation(real matrix[9]) const {
    matrix[0] = transformMatrix.val[0];
    matrix[1] = transformMatrix.val[1];
    matrix[2] = transformMatrix.val[2];
    
    matrix[3] = transformMatrix.val[4];
    matrix[4] = transformMatrix.val[5];
    matrix[5] = transformMatrix.val[6];
    
    matrix[6] = transformMatrix.val[8];
    matrix[7] = transformMatrix.val[9];
    matrix[8] = transformMatrix.val[10];
}

void RigidBody::getTransform(Matrix4 *transform) const {
    memcpy(transform, &transformMatrix.val, sizeof(Matrix4));
}

void RigidBody::getTransform(real matrix[16]) const {
    memcpy(matrix, transformMatrix.val, sizeof(real)*12);
    matrix[12] = matrix[13] = matrix[14] = 0;
    matrix[15] = 1;
}

void RigidBody::getGLTransform(float matrix[16]) const {
    matrix[0] = (float)transformMatrix.val[0];
    matrix[1] = (float)transformMatrix.val[4];
    matrix[2] = (float)transformMatrix.val[8];
    matrix[3] = 0;
    
    matrix[4] = (float)transformMatrix.val[1];
    matrix[5] = (float)transformMatrix.val[5];
    matrix[6] = (float)transformMatrix.val[9];
    matrix[7] = 0;
    
    matrix[8] = (float)transformMatrix.val[2];
    matrix[9] = (float)transformMatrix.val[6];
    matrix[10] = (float)transformMatrix.val[10];
    matrix[11] = 0;
    
    matrix[12] = (float)transformMatrix.val[3];
    matrix[13] = (float)transformMatrix.val[7];
    matrix[14] = (float)transformMatrix.val[11];
    matrix[15] = 1;
}

Matrix4 RigidBody::getTransform() const {
    return transformMatrix;
}

Vector3 RigidBody::getPointInLocalSpace(const Vector3 &point) const {
    return transformMatrix.transformInverse(point);
}

Vector3 RigidBody::getPointInWorldSpace(const Vector3 &point) const {
    return transformMatrix.transform(point);
}

Vector3 RigidBody::getDirectionInLocalSpace(const Vector3 &direction) const {
    return transformMatrix.transformInverseDirection(direction);
}

Vector3 RigidBody::getDirectionInWorldSpace(const Vector3 &direction) const {
    return transformMatrix.transformDirection(direction);
}


void RigidBody::setVelocity(const Vector3 &velocity) {
    RigidBody::velocity = velocity;
}

void RigidBody::setVelocity(const real x, const real y, const real z) {
    velocity.x = x;
    velocity.y = y;
    velocity.z = z;
}

void RigidBody::getVelocity(Vector3 *velocity) const {
    *velocity = RigidBody::velocity;
}

Vector3 RigidBody::getVelocity() const {
    return velocity;
}

void RigidBody::addVelocity(const Vector3 &deltaVelocity) {
    velocity += deltaVelocity;
}

void RigidBody::setRotation(const Vector3 &rotation) {
    RigidBody::rotation = rotation;
}

void RigidBody::setRotation(const real x, const real y, const real z) {
    rotation.x = x;
    rotation.y = y;
    rotation.z = z;
}

void RigidBody::getRotation(Vector3 *rotation) const {
    *rotation = RigidBody::rotation;
}

Vector3 RigidBody::getRotation() const {
    return rotation;
}

void RigidBody::addRotation(const Vector3 &deltaRotation) {
    rotation += deltaRotation;
}

void RigidBody::setAwake(const bool awake) {
    if (awake) {
        isAwake= true;
        
        // Add a bit of motion to avoid it falling asleep immediately.
        motion = 0.01*2.0f;
    } else {
        isAwake = false;
        velocity.clearVector();
        rotation.clearVector();
    }
}

void RigidBody::setCanSleep(const bool canSleep) {
    RigidBody::canSleep = canSleep;
    if (!canSleep && !isAwake) setAwake();
}

void RigidBody::getLastFrameAcceleration(Vector3 *acceleration) const {
    *acceleration = lastFrameAcceleration;
}

Vector3 RigidBody::getLastFrameAcceleration() const {
    return lastFrameAcceleration;
}

void RigidBody::clearAccumulators() {
    forceAccum.clearVector();
    torqueAccum.clearVector();
}

void RigidBody::addForce(const Vector3 &force) {
    forceAccum += force;
    isAwake = true;
}

void RigidBody::addForceAtBodyPoint(const Vector3 &force, const Vector3 &point) {
    // Convert to coordinates relative to center of mass.
    Vector3 pt = getPointInWorldSpace(point);
    addForceAtPoint(force, pt);
    
}

void RigidBody::addForceAtPoint(const Vector3 &force, const Vector3 &point) {
    // Convert to coordinates relative to center of mass.
    Vector3 pt = point;
    pt -= position;
    
    forceAccum += force;
    torqueAccum += pt % force;
    
    isAwake = true;
}

void RigidBody::addTorque(const Vector3 &torque) {
    torqueAccum += torque;
    isAwake = true;
}

void RigidBody::setAcceleration(const Vector3 &acceleration) {
    RigidBody::acceleration = acceleration;
}

void RigidBody::setAcceleration(const real x, const real y, const real z) {
    acceleration.x = x;
    acceleration.y = y;
    acceleration.z = z;
}

void RigidBody::getAcceleration(Vector3 *acceleration) const {
    *acceleration = RigidBody::acceleration;
}

Vector3 RigidBody::getAcceleration() const {
    return acceleration;
}

