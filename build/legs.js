require=(function e(t,n,r){function s(o,u){if(!n[o]){if(!t[o]){var a=typeof require=="function"&&require;if(!u&&a)return a(o,!0);if(i)return i(o,!0);var f=new Error("Cannot find module '"+o+"'");throw f.code="MODULE_NOT_FOUND",f}var l=n[o]={exports:{}};t[o][0].call(l.exports,function(e){var n=t[o][1][e];return s(n?n:e)},l,l.exports,e,t,n,r)}return n[o].exports}var i=typeof require=="function"&&require;for(var o=0;o<r.length;o++)s(r[o]);return s})({1:[function(require,module,exports){
if(typeof THREE === 'undefined')
  THREE = require('../third_party/three.js');

if(typeof Ammo === 'undefined')
  Ammo = require('../third_party/ammo.js');

SkeletalFigure = require('../skeletal_figure.js');

var isZero = function(x) {
  var e = 0.0000000001;
  return -e < x && x < e;
};

var clamp = function(x, min, max) {
  return Math.min(Math.max(x, min), max);
};

/** Computes the torque to bring the current orientation to the target 
 *  orientation. Target angular velocity is always zero.
 *
 * \param  Object cParams Control parameters.
 * \param  Quaternion curQ Current orientation.
 * \param  Quaternion targetQ Target orientation.
 * \param  Vector curOmega Current angular velocity.
 * \param  Vector torque Vector to update with the resulting torque (optional).
 * \return Vector Torque.
 *
 * \note the resulting reference frame is the same as that of the parameters.
 *       Also make sure that the parameters are in the same reference frame.
 */
var computeRelTorque = function(cParams, curQ, targetQ, curOmega, torque) {
  var tq = torque;
  if(tq === undefined)
    tq = new THREE.Vector3();

  /* starts by computing the proportional part */
  /* qDelta = curQ^-1 * targetQ */
  var qDelta = targetQ;
  qDelta.multiplyQuaternions(curQ.conjugate(), targetQ);
  tq.set(qDelta.x, qDelta.y, qDelta.z);

  var sinHalfAngle = tq.length();

  /* this could potentially be slightly greater than 1 due to numerical
   * errors, which is bad for computing the asin later */
  sinHalfAngle = clamp(sinHalfAngle, -1, 1);

  /* avoids division by zero if the orientations match too closely */
  if(!isZero(sinHalfAngle))
  {
    var angle = 2*Math.asin(sinHalfAngle);
    var sign = (qDelta.w < 0) ? -1 : 1;
    tq.multiplyScalar(1/sinHalfAngle * angle * cParams.pdGains[0] * sign);
  }
  else
  {
    tq.set(0, 0, 0);
  }

  /* adds the derivative part */
  tq.add(curOmega.multiplyScalar(cParams.pdGains[1]));

  if(tq.length() > cParams.torqueLimit)
    tq.multiplyScalar(cParams.torqueLimit/tq.length());

  return tq;
};

var Gait = function() {
};

Gait.prototype.setContactForLeg = function(i, contact) {
  this.inContact[i] = contact;
  this.checkForTransitions();
};

Gait.prototype.update = function(timeStep) {
  this.stateTime += timeStep;
  this.checkForTransitions();
};

Gait.prototype.isInContact = function(i) {
  return this.inContact[i];
};

Gait.prototype.isStanceLeg = function(i) {
  return this.stateSwingLeg[this.state] != i;
};

Gait.prototype.advanceState = function() {
  this.state += 1;
  this.state %= this.stateTargets.length;
  this.stateTime = 0;
};

Gait.prototype.checkForTransitions = function() {
  while(this.transitions[this.state]())
    this.advanceState();
};

Gait.prototype.getAnglesForLeg = function(i) {
  if(this.isStanceLeg(i))
    return this.stateTargets[this.state][1];
  else
    return this.stateTargets[this.state][0];
};

var Joint = function(skJoint, controlParams, absAngle) {
  this.model = skJoint.model;
  this.bodyA = skJoint.parentSegment.body;
  this.bodyB = skJoint.childSegment.body;
  this.controlParams = controlParams;

  this.absAngle = absAngle;
  this.targetQ = new THREE.Quaternion();
  this.auxEuler = new THREE.Euler();
  this.torque = new THREE.Vector3();
  this.btTorque = new Ammo.btVector3();
};

Joint.prototype.computeTargetQFromRelAngles = function(pitch, roll) {
  this.auxEuler.set(pitch, roll, 0, 'XYZ');
  this.targetQ.setFromEuler(this.auxEuler).conjugate();
};

Joint.prototype.computeRelTorque = function(curQ, targetQ, curOmega) {
  return computeRelTorque(this.controlParams, curQ, targetQ, curOmega, this.torque);
};

Joint.prototype.computeTorque = function(charFrame) {
  if(!this.absAngle) {
    /* computes the torque in bodyA's frame */
    this.computeRelTorque(this.model.getRelativeOrientation(), 
                          this.targetQ, 
                          this.model.getRelativeVelocity());

    /* converts the resulting torque to world frame */;
    this.torque.applyQuaternion(this.bodyA.getOrientation());
  } else {
    /* computes the torque in character frame */
    this.computeRelTorque(this.bodyB.getOrientation().conjugate().multiply(charFrame), 
                          this.targetQ,
                          this.bodyB.getAngularVelocity().applyQuaternion(charFrame.conjugate()));
    /* revert the character frame to its original value */
    charFrame.conjugate();

    /* converts the resulting torque to world frame */;
    this.torque.applyQuaternion(charFrame);
  }
};

Joint.prototype.applyTorque = function() {
  this.btTorque.setValue(this.torque.x, this.torque.y, this.torque.z);
  /* applies the equal and opposite torques to both objects */
  this.bodyA.body.applyTorque(this.btTorque);
  this.btTorque.op_mul(-1);
  this.bodyB.body.applyTorque(this.btTorque);
};

var Leg = function(initialAngles, rootSkJoint, controlParams) {
  this.trunk = rootSkJoint.parentSegment.body;
  this.segments = [];
  this.joints = [];
  this.time = 0;
  this.controlParams = controlParams;
  this.footPos = new THREE.Vector3();
  this.charFrame = new THREE.Quaternion();

  /* adds the joints */
  var absAngle = true;
  skJoint = rootSkJoint;
  var xAxis = new THREE.Vector3(1, 0, 0);
  var i = 0;
  var angle = 0;
  while(true)
  {
    this.segments.push(skJoint.childSegment.body);
    angle += initialAngles[i];
    skJoint.childSegment.body.rotateAxis(xAxis, angle);
    skJoint.childSegment.body.snapTo((new THREE.Vector3()).fromArray(skJoint.snapPointChild),
                                     skJoint.parentSegment.body,
                                     (new THREE.Vector3()).fromArray(skJoint.snapPointParent));

    this.joints.push(new Joint(skJoint, this.controlParams.joints[i], absAngle));

    absAngle = false;
    if(skJoint.childSegment.childrenJoints.length == 0)
      break;
    skJoint = skJoint.childSegment.childrenJoints[0];
    i++;
  };
};

Leg.prototype.computeTorques = function(targetAngles, fbP, fbD) {
  /* computes the character orientation (heading of trunk) */
  this.charFrame.copy(this.trunk.getHeading());

  /* computes the torques for all the joints */
  for(var i = 0; i < this.joints.length; i++) {
    var fbGains = this.joints[i].controlParams.fbGains;
    var pitch = fbP.y * fbGains[0] + fbD.y * fbGains[1] + targetAngles[i];
    var roll = -fbP.x * fbGains[0] - fbD.x * fbGains[1];
    this.joints[i].computeTargetQFromRelAngles(pitch, roll);
    this.joints[i].computeTorque(this.charFrame);
  }
};

Leg.prototype.applyTorques = function() {
  for(var i = 0; i < this.joints.length; i++)
    this.joints[i].applyTorque();
};

Leg.prototype.getFootPos = function() {
  var footSegment = this.segments[this.segments.length - 1];
  this.footPos.set(0, 0, 0);
  footSegment.toWorldFrame(this.footPos, this.footPos);
  return this.footPos;
};

var LegFrame = function(gait, rootSkSgmt, controlParams) {
  this.heading = 0;
  this.targetQ = new THREE.Quaternion();

  this.targetVel = new THREE.Vector3(0, 0, 0);
  this.targetVelWorld = new THREE.Vector3(0, 0, 0);
  
  this.fbP = new THREE.Vector3();
  this.fbD = new THREE.Vector3();
  this.standWorldPos = new THREE.Vector3();
  this.COMPos = new THREE.Vector3();
  this.COMVel = new THREE.Vector3();

  this.gait = gait;
  this.controlParams = controlParams;
  this.trunk = rootSkSgmt.body;
  this.legs = [];
  for(var i = 0; i < rootSkSgmt.childrenJoints.length; i++)
    this.legs.push(new Leg(this.gait.getAnglesForLeg(i),
                           rootSkSgmt.childrenJoints[i],
                           controlParams));

  this.vts = [
    this.orientationVT = new THREE.Vector3()
  ];
  this.vtTotal = new THREE.Vector3();

  this.vfs = [
    this.cmVelVF = new THREE.Vector3()
  ];
  this.vfTotal = new THREE.Vector3();

  this.auxVect3 = new THREE.Vector3();
  this.auxQuat = new THREE.Quaternion();
};

LegFrame.prototype.update = function(timeStep) {
  /* advances the gait */
  this.gait.update(timeStep);

  /* computes the center of mass position and velocity */
  this.computeCOMPosNVel();

  /* computes the feedback angles */
  this.computeFeedbackAngles();

  var numStanceLegs = 0;
  var zero = this.auxVect3.set(0, 0, 0);
  /* computes the tracking torques for all the legs */
  for(var i = 0; i < this.legs.length; i++) {
    if(this.gait.isStanceLeg(i)) {
      this.legs[i].computeTorques(this.gait.getAnglesForLeg(i),
                                  zero, zero);
      /* resets the tracking torque for the stance hips */
      this.legs[i].joints[0].torque.set(0, 0, 0);

      /* and counts the stance legs */
      numStanceLegs++;
    } else {
      this.legs[i].computeTorques(this.gait.getAnglesForLeg(i),
                                  this.fbP, this.fbD);
    }
  }

  /* computes the trunk's orientation virtual torque */
  this.computeOrientationVT();

  /* computes the velocity tracking virtual force */
  this.computeCMVelVF();

  /* if there are no stance legs, no virtual torques/forces are added */
  if(numStanceLegs != 0)
  {
    /* adds all the virtual forces and torques */
    this.addVFTs(numStanceLegs);
  }

  /* applies the torques */
  for(var i = 0; i < this.legs.length; i++)
    this.legs[i].applyTorques();
};

LegFrame.prototype.addVFToJoint = function(vf, p, j)
{
  var axes = j.model.getAxes();
  var b = j.model.getPosition();
  var aux = this.auxVect3;
  b.subVectors(p, b);
  for(var i = 0; i < axes.length; i++)
  {
    var a = axes[i];
    aux.crossVectors(a, b);
    j.torque.add(a.multiplyScalar(aux.dot(vf)))
  }
};

LegFrame.prototype.addVFTs = function(numStanceLegs) {
  /* computes the total virtual force applied to the CM */
  this.vfTotal.set(0, 0, 0);
  for(var i = 0; i < this.vfs.length; i++)
    this.vfTotal.add(this.vfs[i]);

  /* computes the total virtual torque */
  this.vtTotal.set(0, 0, 0);
  for(var i = 0; i < this.vts.length; i++)
    this.vtTotal.add(this.vts[i]);

  /* splits the virtual force equally among the stance legs */
  this.vfTotal.multiplyScalar(1/numStanceLegs);

  /* splits the virtual torque equally among the stance legs */
  this.vtTotal.multiplyScalar(1/numStanceLegs);

  /* adds the total virtual force and torque to each stance leg */
  for(var i = 0; i < this.legs.length; i++) {
    var leg = this.legs[i];
    if(this.gait.isStanceLeg(i)) {
      for(var j = 0; j < leg.joints.length; j++) {
        this.addVFToJoint(this.vfTotal, this.COMPos, leg.joints[j]);
        leg.joints[j].torque.add(this.vtTotal);
      }
    }
  }
};

LegFrame.prototype.computeOrientationVT = function() {
  this.targetQ.set(0, 0, Math.sin(this.heading/2), Math.cos(this.heading/2));

  this.auxQuat.copy(this.trunk.getOrientation());

  computeRelTorque(this.controlParams.orientationVT,
                   this.auxQuat,
                   this.targetQ,
                   this.trunk.getAngularVelocity().applyQuaternion(this.trunk.getOrientation().conjugate()),
                   this.orientationVT);
  this.orientationVT.applyQuaternion(this.trunk.getOrientation());
  return this.orientationVT;
};

LegFrame.prototype.computeCMVelVF = function() {
  /* computes the target velocity in world coordinates */
  this.targetVelWorld.copy(this.targetVel);
  this.targetVelWorld.applyQuaternion(this.trunk.getOrientation());

  /* computes the difference in velocity and multiplies by a constant factor */
  this.cmVelVF.subVectors(this.COMVel, this.targetVelWorld);
  this.cmVelVF.z = 0;
  this.cmVelVF.multiplyScalar(this.controlParams.positionVF.pdGains[1]);
}

LegFrame.prototype.computeCOMPosNVel = function() {
  var aux = this.auxVect3.set(0, 0, 0);
  var totalMass = 0;

  this.trunk.toWorldFrame(aux, aux);
  aux.multiplyScalar(this.trunk.mass);
  this.COMPos.copy(aux);

  aux = this.trunk.getLinearVelocity();
  aux.multiplyScalar(this.trunk.mass);
  this.COMVel.copy(aux);

  totalMass += this.trunk.mass;

  for(var i = 0; i < this.legs.length; i++) {
    var leg = this.legs[i]
    for (var i = 0; i < leg.segments.length; i++) {
      aux.set(0, 0, 0);
      leg.segments[i].toWorldFrame(aux, aux);
      aux.multiplyScalar(leg.segments[i].mass);
      this.COMPos.add(aux);

      aux.set(0, 0, 0);
      aux = leg.segments[i].getLinearVelocity();
      aux.multiplyScalar(leg.segments[i].mass);
      this.COMVel.add(aux);

      totalMass += leg.segments[i].mass;
    };
  };

  this.COMPos.multiplyScalar(1/totalMass);
  this.COMVel.multiplyScalar(1/totalMass);
};

LegFrame.prototype.computeFeedbackAngles = function() {
  var numStanceLegs = 0;
  this.standWorldPos.set(0, 0, 0);
  for(var i = 0; i < this.legs.length; i++)
  {
    if(this.gait.isStanceLeg(i))
    {
      numStanceLegs++;
      this.standWorldPos.add(this.legs[i].getFootPos());    
    }
  }

  if(numStanceLegs > 0)
  {
    this.fbP.copy(this.COMPos);
    this.fbD.copy(this.COMVel);
    this.standWorldPos.multiplyScalar(1/numStanceLegs);
    this.fbP.sub(this.standWorldPos);
    this.fbP.z = 0;
    this.fbP.applyQuaternion(this.trunk.getHeading().conjugate());

    this.fbD.z = 0;
    this.fbD.applyQuaternion(this.trunk.getHeading().conjugate());
  }
  else
  {
    this.fbP.set(0, 0, 0);
    this.fbD.set(0, 0, 0);
  }
};

module.exports.Gait = Gait;
module.exports.Joint = Joint;
module.exports.LegFrame = LegFrame;
},{"../skeletal_figure.js":4,"../third_party/ammo.js":undefined,"../third_party/three.js":undefined}],2:[function(require,module,exports){
'use strict';

var extend = function(extended, base) {
  extended.prototype = Object.create(base.prototype);
  extended.prototype.base = base;
  extended.prototype.constructor = extended;
};

var Logger = function() {
  this.time = 0;
  this.info = {};
};

Logger.prototype.log = function(msg) {
  throw 'Not implemented';
};

Logger.prototype.step = function(timeStep) {
  this.time += timeStep;
  for(var key in this.info)
    this.log('[' + this.time.toFixed(3) + '] ' + key + ': ' + this.info[key]);
};

Logger.prototype.setInfo = function(key, value) {
  this.info[key] = value;
}

var ConsoleLogger = function() {
  this.base.call(this);
};

extend(ConsoleLogger, Logger);

ConsoleLogger.prototype.log = function(msg) {
  console.log(msg);
};

var NullLogger = function() {
  this.base.call(this);
};

extend(NullLogger, Logger);

NullLogger.prototype.log = function(msg) {
  /* does nothing */
};

if ( typeof module === 'object' ) {
  module.exports.ConsoleLogger = ConsoleLogger;
  module.exports.NullLogger = NullLogger;
}
},{}],3:[function(require,module,exports){
if(typeof THREE === 'undefined')
  THREE = require('../third_party/three.js');

if(typeof Ammo === 'undefined')
  Ammo = require('../third_party/ammo.js');

var three2BulletTransform = function(threeT, bulletT) {
  t = bulletT
  if(bulletT === undefined)
    t = new Ammo.btTransform();

  var p = new THREE.Vector3();
  p.setFromMatrixPosition(threeT);
  var q = new THREE.Quaternion();
  q.setFromRotationMatrix(threeT);

  var o = new Ammo.btVector3(p.x, p.y, p.z);
  var r = new Ammo.btQuaternion(q.x, q.y, q.z, q.w);
  t.setIdentity();
  t.setOrigin(o);
  t.setRotation(r);

  Ammo.destroy(o);
  Ammo.destroy(r);

  return t;
};

var bullet2ThreeTransform = function(bulletT, threeT) {
  t = threeT
  if(threeT === undefined)
    t = new THREE.Matrix4();

  var p = bullet2ThreeTransform.p;
  var q = bullet2ThreeTransform.q;

  p.set(bulletT.getOrigin().x(),
        bulletT.getOrigin().y(),
        bulletT.getOrigin().z());
  q.set(bulletT.getRotation().x(),
        bulletT.getRotation().y(),
        bulletT.getRotation().z(),
        bulletT.getRotation().w());

  t.identity();
  t.makeRotationFromQuaternion(q);
  t.setPosition(p);

  return t;
};

bullet2ThreeTransform.p = new THREE.Vector3();
bullet2ThreeTransform.q = new THREE.Quaternion();

var extend = function(extended, base) {
  extended.prototype = Object.create(base.prototype);
  extended.prototype.base = base;
  extended.prototype.constructor = extended;
};

var CollisionGroup = {
  NONE: 0,
  BONE: 1,
  GROUND: 2
};

var RigidBody = function(mass, size) {
  this.mass = mass;
  this.size = size;
  this.collidesWith = CollisionGroup.GROUND;
  this.transform = new THREE.Matrix4();
  this.transformAux = new THREE.Matrix4();
  this.q = new THREE.Quaternion();
  this.vel = new THREE.Vector3();
  this.omega = new THREE.Vector3();
  this.btTransformAux = new Ammo.btTransform();

  /* TODO: use a pool instead of this */
  this.btVecAux = new Ammo.btVector3();
  this.btVecAux2 = new Ammo.btVector3();

  this.auxMat4 = new THREE.Matrix4();
};

RigidBody.prototype.detach = function(scene) {
  scene.remove(this.visual);
  scene.world.bodies.splice(scene.world.bodies.indexOf(this), 1);
  scene.world.removeRigidBody(this.body);
};

RigidBody.prototype.destroy = function() {
  Ammo.destroy(this.btTransformAux);
  Ammo.destroy(this.btVecAux);
  Ammo.destroy(this.btVecAux2);
  Ammo.destroy(this.body);
};

RigidBody.prototype.translate = function(x, y, z) {
  this.transformAux.makeTranslation(x, y, z);
  this.transform.multiplyMatrices(this.transformAux, this.transform);
};

RigidBody.prototype.rotateAxis = function(axis, theta, pivot) {
  if(pivot !== undefined)
    this.translate(-pivot.x, -pivot.y, -pivot.z);
  this.transformAux.makeRotationAxis(axis, theta);
  this.transform.multiplyMatrices(this.transformAux, this.transform);
  if(pivot !== undefined)
    this.translate(pivot.x, pivot.y, pivot.z);
  if(this.body !== undefined) {
    three2BulletTransform(this.transform, this.btTransformAux);
    this.body.setCenterOfMassTransform(this.btTransformAux);
  }
};

RigidBody.prototype.toWorldFrame = function(localPoint, worldPoint) {
  var p = worldPoint;
  if(worldPoint === undefined)
    p = new THREE.Vector3();

  if(localPoint === undefined)
    p.set(0, 0, 0);
  else
    p.copy(localPoint);

  var t = this.btTransform;
  if(this.body !== undefined)
    t = this.body.getCenterOfMassTransform();

  bullet2ThreeTransform(t, this.auxMat4);

  p.applyMatrix4(this.auxMat4);

  return p;
};

RigidBody.prototype.toLocalFrame = function(worldPoint, localPoint) {
  var p = localPoint;
  if(localPoint === undefined)
    p = new THREE.Vector3();

  p.copy(worldPoint);

  var t = this.btTransform;
  if(this.body !== undefined)
    t = this.body.getCenterOfMassTransform();

  bullet2ThreeTransform(t, this.auxMat4);

  this.auxMat4.getInverse(this.auxMat4);
  p.applyMatrix4(this.auxMat4);

  return p;
};

RigidBody.prototype.getLinearVelocity = function(localPoint) {
  var vel = this.body.getLinearVelocity();
  this.vel.set(vel.x(), vel.y(), vel.z());

  if(localPoint !== undefined) {
    var omega = this.getAngularVelocity();
    this.vel.add(omega.cross(localPoint));
  }

  return this.vel;
};

RigidBody.prototype.getAngularVelocity = function() {
  var omega = this.body.getAngularVelocity();
  this.omega.set(omega.x(), omega.y(), omega.z());

  return this.omega;
};

RigidBody.prototype.getOrientation = function() {
  var q = this.body.getCenterOfMassTransform().getRotation();
  this.q.set(q.x(), q.y(), q.z(), q.w());

  return this.q;
};

RigidBody.prototype.getHeading = function() {
  /* uses the swing/twist decomposition of the orientation of the trunk to get
   * a twist around a Z axis and a swing that aligns the rotation angle with the
   * Z axis
   * the idea is that the swing is done before the twist, and so:
   * rot = twist * swing */
  var rot = this.getOrientation();
  rot.x = 0;
  rot.y = 0;
  rot.normalize();
  return rot;
};

RigidBody.prototype.snapTo = function(snapPoint, bodyB, snapPointB) {
  /* converts the local snap point of this object to world frame */
  var worldSnapPoint = new THREE.Vector4(snapPoint.x, 
                                         snapPoint.y, 
                                         snapPoint.z, 
                                         1);
  worldSnapPoint.applyMatrix4(this.transform);

  /* converts the local snap point of object B to world frame */
  var worldSnapPointB = new THREE.Vector4(snapPointB.x, 
                                          snapPointB.y, 
                                          snapPointB.z, 
                                          1);
  worldSnapPointB.applyMatrix4(bodyB.transform);

  /* translates the difference between the two snap points in world frame */
  this.translate(worldSnapPointB.x - worldSnapPoint.x, 
                 worldSnapPointB.y - worldSnapPoint.y, 
                 worldSnapPointB.z - worldSnapPoint.z);
};

RigidBody.prototype.applyForce = function(force, relPos) {
  this.btVecAux.setValue(force.x, force.y, force.z);
  if(relPos === undefined)
    this.btVecAux2.setValue(0, 0, 0);
  else
    this.btVecAux2.setValue(relPos.x, relPos.y, relPos.z);
  this.body.applyForce(this.btVecAux, this.btVecAux2);
};

RigidBody.prototype.buildAndInsert = function(scene) {
  this.buildRigidBody();
  this.body.setActivationState(4);
  scene.world.addRigidBody(this.body, this.collisionGroup, this.collidesWith);
  scene.world.bodies.push(this);

  this.buildVisual();
  scene.add(this.visual);
};

RigidBody.prototype.updateVisual = function() {
  if( this.body.getMotionState() && this.visual !== undefined ) {
    this.body.getMotionState().getWorldTransform(this.btTransformAux);

    this.visual.position.set(this.btTransformAux.getOrigin().x(), 
                             this.btTransformAux.getOrigin().y(), 
                             this.btTransformAux.getOrigin().z());
    this.visual.quaternion.set(this.btTransformAux.getRotation().x(), 
                               this.btTransformAux.getRotation().y(), 
                               this.btTransformAux.getRotation().z(), 
                               this.btTransformAux.getRotation().w());
  }
};

RigidBody.prototype.buildRigidBody = function() {
  throw "Unsupported operation";
};

RigidBody.prototype.buildVisual = function() {
  throw "Unsupported operation";
};

var Bone = function(mass, size) {
  this.base.call(this, mass, size);
  this.collisionGroup = CollisionGroup.BONE;
};

extend(Bone, RigidBody);

Bone.prototype.buildRigidBody = function() {
  /* sets up the motion state from the current transform */
  var t = new Ammo.btTransform();
  three2BulletTransform(this.transform, t);
  var motionState = new Ammo.btDefaultMotionState(t);
  
  var localInertia = new Ammo.btVector3(0, 0, 0);
  var halfExtents = new Ammo.btVector3(0.9*this.size.x, 0.9*this.size.y, 0.9*this.size.z);
  var shape = new Ammo.btBoxShape(halfExtents);
  shape.calculateLocalInertia(this.mass,localInertia);
  var rbInfo = new Ammo.btRigidBodyConstructionInfo(this.mass, motionState, 
                                                    shape, localInertia);
  this.body = new Ammo.btRigidBody(rbInfo);

  Ammo.destroy(t);
  Ammo.destroy(halfExtents);
  Ammo.destroy(localInertia);
  Ammo.destroy(rbInfo);
};

Bone.prototype.buildVisual = function() {
  var mesh = new THREE.Mesh(new THREE.BoxGeometry(1.8*this.size.x, 1.8*this.size.y, 1.8*this.size.z), 
                            new THREE.MeshLambertMaterial({color: 0x66a5ff}));
  mesh.receiveShadow = true;
  mesh.castShadow = true;

  this.visual = new THREE.Object3D();
  this.visual.add(mesh);
};

var Foot = function(mass, size) {
  this.base.call(this, mass, size);
  this.collisionGroup = CollisionGroup.BONE;
};

extend(Foot, RigidBody);

Foot.prototype.buildRigidBody = function() {
  /* sets up the motion state from the current transform */
  var t = new Ammo.btTransform();
  three2BulletTransform(this.transform, t);
  var motionState = new Ammo.btDefaultMotionState(t);
  
  var localInertia = new Ammo.btVector3(0, 0, 0);
  var halfExtents = new Ammo.btVector3(0.9*this.size.x, 0.9*this.size.y, 0.9*this.size.z);
  var shape = new Ammo.btCompoundShape();

  var foot = new Ammo.btBoxShape(halfExtents);

  t.setRotation(new Ammo.btQuaternion(0, 0, Math.sin(Math.PI/4), Math.cos(Math.PI/4)));
  
  var heel = new Ammo.btSphereShape(2.1*this.size.y);
  t.setOrigin(new Ammo.btVector3(0, 0, 0.75 * this.size.z));
  shape.addChildShape(t, heel);

  halfExtents.setValue(1.1*this.size.y, 0.9*this.size.x, 1.1*this.size.y);
  var toe = new Ammo.btCylinderShape(halfExtents);
  t.setOrigin(new Ammo.btVector3(0, -0.7*this.size.y, -0.5*this.size.z));
  shape.addChildShape(t, toe);
  
  shape.calculateLocalInertia(this.mass,localInertia);
  var rbInfo = new Ammo.btRigidBodyConstructionInfo(this.mass, motionState, 
                                                    shape, localInertia);
  this.body = new Ammo.btRigidBody(rbInfo);

  Ammo.destroy(t);
  Ammo.destroy(halfExtents);
  Ammo.destroy(localInertia);
  Ammo.destroy(rbInfo);
};

Foot.prototype.buildVisual = function() {
  var mesh = new THREE.Mesh(new THREE.BoxGeometry(1.8*this.size.x, 1.8*this.size.y, 1.8*this.size.z), 
                            new THREE.MeshLambertMaterial({color: 0x66a5ff}));
  mesh.receiveShadow = true;
  mesh.castShadow = true;

  var heelMesh = new THREE.Mesh(new THREE.SphereGeometry(2.1*this.size.y), 
                                new THREE.MeshLambertMaterial({color: 0x66a5ff}));
  heelMesh.receiveShadow = true;
  heelMesh.castShadow = true;
  heelMesh.position.set(0, 0, 0.75 * this.size.z);
  heelMesh.rotation.z = Math.PI/2;

  var toeMesh = new THREE.Mesh(new THREE.CylinderGeometry(1.1*this.size.y, 1.1*this.size.y, 1.8*this.size.x), 
                               new THREE.MeshLambertMaterial({color: 0x66a5ff}));
  toeMesh.receiveShadow = true;
  toeMesh.castShadow = true;
  toeMesh.position.set(0, -0.7*this.size.y, -0.5 * this.size.z);
  toeMesh.rotation.z = Math.PI/2;

  this.visual = new THREE.Object3D();
  this.visual.add(mesh);
  this.visual.add(heelMesh);
  this.visual.add(toeMesh);
};

var Ground = function(size) {
  this.base.call(this, 0, size);
  this.collisionGroup = CollisionGroup.GROUND;
  this.collidesWith = CollisionGroup.BONE | CollisionGroup.GROUND;
};

extend(Ground, RigidBody);

Ground.prototype.buildRigidBody = function() {
  /* sets up the motion state from the current transform */
  var t = new Ammo.btTransform();
  three2BulletTransform(this.transform, t);
  var motionState = new Ammo.btDefaultMotionState(t);
  
  var localInertia = new Ammo.btVector3(0, 0, 0);
  var halfExtents = new Ammo.btVector3(this.size.x, this.size.y, this.size.z);
  var shape = new Ammo.btBoxShape(halfExtents);
  shape.calculateLocalInertia(this.mass,localInertia);
  var rbInfo = new Ammo.btRigidBodyConstructionInfo(this.mass, motionState, 
                                                    shape, localInertia);
  this.body = new Ammo.btRigidBody(rbInfo);
  this.body.setFriction(3);

  Ammo.destroy(t);
  Ammo.destroy(halfExtents);
  Ammo.destroy(localInertia);
  Ammo.destroy(rbInfo);
};

Ground.prototype.buildVisual = function() {
  var mesh = new THREE.Mesh(new THREE.BoxGeometry(2*this.size.x, 2*this.size.y, 2*this.size.z, 99, 99), 
                            new THREE.MeshLambertMaterial({color: 0xeeeeee}));
  mesh.receiveShadow = true;
  mesh.castShadow = false;
  var lightColor = new THREE.Color(0xcccccc);
  var darkColor = new THREE.Color(0xaaaaaa);
  for(var i = 0; i < mesh.geometry.faces.length; i++) {
    var color = Math.floor(i / 2) % 2 ? lightColor : darkColor;
    mesh.geometry.faces[i].vertexColors[0] = color;
    mesh.geometry.faces[i].vertexColors[1] = color;
    mesh.geometry.faces[i].vertexColors[2] = color;
  }
  mesh.material.vertexColors = THREE.VertexColors;

  this.visual = new THREE.Object3D();
  this.visual.add(mesh);
};

var Joint = function(bodyA, bodyB, pivotInA, pivotInB) {
  this.bodyA = bodyA;
  this.bodyB = bodyB;
  this.pivotInA = pivotInA;
  this.pivotInB = pivotInB;
  this.targetAngle = [0, 0, 0];

  this.curQ = new THREE.Quaternion();
  this.curOmega = new THREE.Vector3();

  this.pivotWorldA = new THREE.Vector3();
  this.pivotWorldB = new THREE.Vector3();
};

Joint.prototype.detach = function(scene) {
  scene.world.removeConstraint(this.c);
};

Joint.prototype.destroy = function() {
  Ammo.destroy(this.c);
};

Joint.prototype.getPosition = function() {
  this.bodyA.toWorldFrame(this.pivotInA, this.pivotWorldA);
  this.bodyB.toWorldFrame(this.pivotInB, this.pivotWorldA);
  this.pivotWorldA.lerp(this.pivotWorldA, 0.5);
  return this.pivotWorldA;
};

Joint.prototype.getRelativeOrientation = function() {
  var qA = this.bodyA.getOrientation();
  var qB = this.bodyB.getOrientation();
  this.curQ.multiplyQuaternions(qB.conjugate(), qA);

  return this.curQ;
};

Joint.prototype.getRelativeVelocity = function() {
  var omegaA = this.bodyA.body.getAngularVelocity();
  var omegaB = this.bodyB.body.getAngularVelocity();

  this.curOmega.set(omegaB.x(), omegaB.y(), omegaB.z());

  /* avoids creating a new auxiliary vector to do this simple operation */
  this.curOmega.x -= omegaA.x();
  this.curOmega.y -= omegaA.y();
  this.curOmega.z -= omegaA.z();

  return this.curOmega.applyQuaternion(this.bodyA.getOrientation().conjugate());
};

Joint.prototype.buildAndInsert = function(scene) {
  throw "Unsupported operation";
};

Joint.prototype.getAxes = function() {
  throw "Unsupported operation";
};

var HingeJoint = function(bodyA, bodyB, pivotInA, pivotInB, lowerLimit, higherLimit) {
  this.base.call(this, bodyA, bodyB, pivotInA, pivotInB);
  this.lowerLimit = lowerLimit;
  this.higherLimit = higherLimit;
  /* TODO: axis is hardcoded */
  this.axis = new THREE.Vector3(1, 0, 0);
  this.axes = [new THREE.Vector3()];
}

extend(HingeJoint, Joint);

HingeJoint.prototype.buildAndInsert = function(scene) {
  var btAxis = new Ammo.btVector3(this.axis.x, this.axis.y, this.axis.z);
  var btPivotInA = new Ammo.btVector3(this.pivotInA.x,
                                      this.pivotInA.y,
                                      this.pivotInA.z);
  var btPivotInB = new Ammo.btVector3(this.pivotInB.x,
                                      this.pivotInB.y,
                                      this.pivotInB.z);
  this.c = new Ammo.btHingeConstraint(this.bodyA.body,
                                      this.bodyB.body,
                                      btPivotInA,
                                      btPivotInB,
                                      btAxis, btAxis, true);

  this.c.setLimit(this.lowerLimit, this.higherLimit);

  scene.world.addConstraint(this.c, true);

  Ammo.destroy(btAxis);
  Ammo.destroy(btPivotInA);
  Ammo.destroy(btPivotInB);
};

HingeJoint.prototype.getAxes = function() {
  var a = this.axes[0];
  a.copy(this.axis);
  a.applyQuaternion(this.bodyA.getOrientation());
  return this.axes;
};

var BallSocketJoint = function(bodyA, bodyB, pivotInA, pivotInB, angularLowerLimit, angularUpperLimit) {
  this.base.call(this, bodyA, bodyB, pivotInA, pivotInB);
  this.angularLowerLimit = angularLowerLimit;
  this.angularUpperLimit = angularUpperLimit;
  this.axes = [new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()];
}

extend(BallSocketJoint, Joint);

BallSocketJoint.prototype.buildAndInsert = function(scene) {
  var frameInA = new Ammo.btTransform();
  var frameInB = new Ammo.btTransform();
  var btVector3 = new Ammo.btVector3();

  frameInA.setIdentity();
  btVector3.setValue(this.pivotInA.x, this.pivotInA.y, this.pivotInA.z);
  frameInA.setOrigin(btVector3);

  frameInB.setIdentity();
  btVector3.setValue(this.pivotInB.x, this.pivotInB.y, this.pivotInB.z);
  frameInB.setOrigin(btVector3);

  this.c = new Ammo.btGeneric6DofConstraint(this.bodyA.body, 
                                            this.bodyB.body, 
                                            frameInA, 
                                            frameInB, 
                                            true);

  btVector3.setValue(this.angularLowerLimit[0],
                     this.angularLowerLimit[1],
                     this.angularLowerLimit[2]);
  this.c.setAngularLowerLimit(btVector3);

  btVector3.setValue(this.angularUpperLimit[0],
                     this.angularUpperLimit[1],
                     this.angularUpperLimit[2]);
  this.c.setAngularUpperLimit(btVector3);

  scene.world.addConstraint(this.c, true);

  Ammo.destroy(btVector3);
  Ammo.destroy(frameInB);
  Ammo.destroy(frameInA);
};

BallSocketJoint.prototype.getAxes = function() {
  var a;
  for(var i = 0; i < this.axes.length; i++)
  {
    a = this.c.getAxis(i);
    this.axes[i].set(a.x(), a.y(), a.z());
  }
  return this.axes;
};

module.exports.Joint = Joint;
module.exports.HingeJoint = HingeJoint;
module.exports.BallSocketJoint = BallSocketJoint;
module.exports.Bone = Bone;
module.exports.Foot = Foot;
module.exports.Ground = Ground;
},{"../third_party/ammo.js":undefined,"../third_party/three.js":undefined}],4:[function(require,module,exports){
Function.prototype.construct = function(argArray) {
  var constr = this;
  var inst = Object.create(constr.prototype);
  constr.apply(inst, argArray);
  return inst;
};

var SkeletalJoint = function(model, modelParams, parentSegment, childSegment,
                             snapPointParent, snapPointChild) {
  this.model = model.construct([parentSegment.body, childSegment.body,
                              (new THREE.Vector3()).fromArray(snapPointParent),
                              (new THREE.Vector3()).fromArray(snapPointChild)].concat(modelParams));
  this.parentSegment = parentSegment;
  this.childSegment = childSegment;
  this.snapPointParent = snapPointParent;
  this.snapPointChild = snapPointChild;
};

var SkeletalSegment = function(body, parent) {
  this.body = body;
  this.parent = parent;
  this.childrenJoints = [];
};

SkeletalSegment.prototype.snap = function(jointModel, jointParams, childSegment,
                                          snapPointParent, snapPointChild) {
  childSegment.parent = this;
  var newJoint = new SkeletalJoint(jointModel, jointParams, this, childSegment,
                                   snapPointParent, snapPointChild);
  this.childrenJoints.push(newJoint);
  return this;
};

SkeletalSegment.prototype.buildAndInsert = function(scene)
{
  this.body.buildAndInsert(scene);
  for (var i = 0; i < this.childrenJoints.length; i++) {
    var skJoint = this.childrenJoints[i];
    skJoint.childSegment.body.snapTo((new THREE.Vector3()).fromArray(skJoint.snapPointChild),
                                     skJoint.parentSegment.body,
                                     (new THREE.Vector3()).fromArray(skJoint.snapPointParent));
    skJoint.childSegment.buildAndInsert(scene);
    skJoint.model.buildAndInsert(scene);
  };
};

SkeletalSegment.fromJSON = function(json) {
  var scaleVec3 = function(v, s) {
    return [v[0] * s[0], v[1] * s[1], v[2] * s[2]]
  }

  var body = json.model.construct([json.mass,
                                  (new THREE.Vector3()).fromArray(json.size)].concat(json.modelParams));
  var segment = new SkeletalSegment(body);
  if(json.children !== undefined) {
    for (var i = 0; i < json.children.length; i++) {
      segment.snap(json.children[i].model,
                   json.children[i].modelParams,
                   SkeletalSegment.fromJSON(json.children[i].child),
                   scaleVec3(json.children[i].snapPointParent, json.size),
                   scaleVec3(json.children[i].snapPointChild, json.children[i].child.size));
    };
  }
  return segment;
};

module.exports.SkeletalJoint = SkeletalJoint;
module.exports.SkeletalSegment = SkeletalSegment;
},{}],"legs":[function(require,module,exports){
module.exports.Control = require('./control/control.js');
module.exports.Model = require('./model/model.js');
module.exports.SkeletalFigure = require('./skeletal_figure.js');
module.exports.Log = require('./log.js');
},{"./control/control.js":1,"./log.js":2,"./model/model.js":3,"./skeletal_figure.js":4}]},{},[]);
