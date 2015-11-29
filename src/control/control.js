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
  throw 'Operation not supported';
};

Gait.prototype.update = function(timeStep) {
  throw 'Operation not supported';
};

Gait.prototype.isStanceLeg = function(i) {
  return this.stateStanceLeg[this.state] == i;
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