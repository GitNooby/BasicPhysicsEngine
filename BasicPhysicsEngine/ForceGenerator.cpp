//
//  ForceGenerator.cpp
//  BasicPhysicsEngine
//
//  Created by Kai Zou on 2014-05-18.
//  Copyright (c) 2014 com.personal. All rights reserved.
//

#include "ForceGenerator.h"

Gravity::Gravity(const Vector3 &g) {
    Gravity::gravity = g;
}
void Gravity::updateForce(RigidBody *body, real duration) {
    if (body->hasFiniteMass() == false) return;
    body->addForce(gravity * body->getMass());
}

Spring::Spring(const Vector3 &localConnectionPt, RigidBody *otherBody, const Vector3 &otherConnectionPt, real springConstant, real restLength) {
    Spring::localConnectionPt = localConnectionPt;
    Spring::other = otherBody;
    Spring::otherConnectionPt = otherConnectionPt;
    Spring::springConstant = springConstant;
    Spring::restLength;
}
void Spring::updateForce(RigidBody *body, real duration) {
    // find the ends of spring connections in world space
    Vector3 lws = body->getPointInWorldSpace(connectionPoint);
    Vector3 ows = body->getPointInWorldSpace(otherConnectionPt);
    Vector3 force = lws - ows;
    // find force mangitude
    real mangitude = force.mangitude();
    mangitude = abs(mangitude-restLength);
    mangitude *= springConstant;
    // find force and apply
    force.normalize();
    force *= -mangitude;
    body->addForceAtPoint(force, lws);
}