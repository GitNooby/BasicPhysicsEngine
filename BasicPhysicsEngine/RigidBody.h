//
//  RigidBody.h
//  BasicPhysicsEngine
//
//  Created by Kai Zou on 2014-05-18.
//  Copyright (c) 2014 com.personal. All rights reserved.
//

#ifndef __BasicPhysicsEngine__RigidBody__
#define __BasicPhysicsEngine__RigidBody__

#include <iostream>
#include "common.h"
#include "Vector3.h"
#include "Quaternion.h"
#include "Matrix3.h"
#include "Matrix4.h"

class RigidBody
{
protected:
    real inverseMass;
    Matrix3 inverseInertiaTensor;
    real linearDamping;
    real angularDamping;
    Vector3 position;
    Quaternion orientation;
    Vector3 velocity;
    Vector3 rotation;
    Vector3 acceleration;
    Matrix3 inverseInertiaTensorWorld;

    real motion;
    bool isAwake;
    bool canSleep;
    
    /* Holds a transform matrix for converting body space into
       world space and vice versa */
    Matrix4 transformMatrix;
    
    Vector3 forceAccum;
    Vector3 torqueAccum;
    Vector3 lastFrameAcceleration;
public:
    void calculateDerivedData();
    void integrate(real duration);
    
    // mass setters, getters, and queries
    void setMass(const real mass);
    real getMass() const;
    void setInverseMass(const real inverseMass);
    real getInverseMass() const;
    bool hasFiniteMass() const;
    
    // inertiaTensor setters, getters, and queries
    void setInertiaTensor(const Matrix3 &inertiaTensor);
    void getInertiaTensor(Matrix3 *inertiaTensor) const;
    Matrix3 getInertiaTensor() const;
    void getInertiaTensorWorld(Matrix3 *inertiaTensor) const;
    Matrix3 getInertiaTensorWorld() const;
    void setInverseInertiaTensor(const Matrix3 &inverseInertiaTensor);
    void getInverseInertiaTensor(Matrix3 *inverseInertiaTensor) const;
    Matrix3 getInverseInertiaTensor() const;
    void getInverseInertiaTensorWorld(Matrix3 *inverseInertiaTensor) const;
    Matrix3 getInverseInertiaTensorWorld() const;
    
    // linear and angluar damping getters, setters, query
    void setDamping(const real linearDamping, const real angularDamping);
    void setLinearDamping(const real linearDamping);
    real getLinearDamping() const;
    void setAngularDamping(const real angularDamping);
    real getAngularDamping() const;
    
    // position getters and setters
    void setPosition(const Vector3 &position);
    void setPosition(const real x, const real y, const real z);
    void getPosition(Vector3 *position) const;
    Vector3 getPosition() const;
    
    // orientation getters, setters, and queries
    void setOrientation(const Quaternion &orientation);
    void setOrientation(const real r, const real i, const real j, const real k);
    void getOrientation(Quaternion *orientation) const;
    Quaternion getOrientation() const;
    void getOrientation(Matrix3 *matrix) const;
    void getOrientation(real matrix[9]) const;
    
    // velocity getters, setters, and queries
    void setVelocity(const Vector3 &velocity);
    void setVelocity(const real x, const real y, const real z);
    void getVelocity(Vector3 *velocity) const;
    Vector3 getVelocity() const;
    void addVelocity(const Vector3 &deltaVelocity);
    
    // rotation (angular velocity) setters, getters, and queries
    void setRotation(const Vector3 &rotation);
    void setRotation(const real x, const real y, const real z);
    void getRotation(Vector3 *rotation) const;
    Vector3 getRotation() const;
    void addRotation(const Vector3 &deltaRotation);
    
    // accerlation setters, getters, and queries
    void setAcceleration(const Vector3 &acceleration); // set body's constant acceleration
    void setAcceleration(const real x, const real y, const real z);
    void getAcceleration(Vector3 *acceleration) const;
    Vector3 getAcceleration() const;
    
    // gets the current acceleration
    void getLastFrameAcceleration(Vector3 *linearAcceleration) const;
    Vector3 getLastFrameAcceleration() const;
    
    // transform representing body's position and orientation, transformation turns this matrix from local space to world space
    void getTransform(Matrix4 *transform) const;
    void getTransform(real matrix[16]) const;
    void getGLTransform(float matrix[16]) const;
    Matrix4 getTransform() const;
    
    // converts point from world space to local space
    Vector3 getPointInLocalSpace(const Vector3 &point) const;
    // converts point from local space to world space
    Vector3 getPointInWorldSpace(const Vector3 &point) const;
    // converts direction from world space to local space
    Vector3 getDirectionInLocalSpace(const Vector3 &direction) const;
    // converts direction from local space to world space
    Vector3 getDirectionInWorldSpace(const Vector3 &direction) const;
    
    // determines if body responds to integration
    bool getAwake() const { return isAwake; }
    void setAwake(const bool awake=true);
    bool getCanSleep() const { return canSleep; }
    void setCanSleep(const bool canSleep=true);
    
    void clearAccumulators();
    void addForce(const Vector3 &force); // add given force to center of mass
    void addForceAtPoint(const Vector3 &force, const Vector3 &point); // add force at point on body in world coordinates
    void addForceAtBodyPoint(const Vector3 &force, const Vector3 &point); // add force at point on body in body coordinates
    void addTorque(const Vector3 &torque); // add torque expressed in world coordinates
    
};

#endif /* defined(__BasicPhysicsEngine__RigidBody__) */
