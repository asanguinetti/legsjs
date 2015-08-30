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

/** Computes the torque to bring the current orientation to the target 
 *  orientation. Target angular velocity is always zero.
 *
 * \param  Quaternion curQ Current orientation.
 * \param  Quaternion targetQ Target orientation.
 * \param  Vector curOmega Current angular velocity.
 * \return Vector Torque.
 *
 * \note the resulting reference frame is the same as that of the parameters.
 *       Also make sure that the parameters are in the same reference frame.
 */
Joint.prototype.computeRelTorque = function(curQ, targetQ, curOmega) {
  /* starts by computing the proportional part */
  /* qDelta = curQ^-1 * targetQ */
  var qDelta = targetQ;
  qDelta.multiplyQuaternions(curQ.conjugate(), targetQ);
  this.torque.set(qDelta.x, qDelta.y, qDelta.z);

  var sinHalfAngle = this.torque.length();

  /* this could potentially be slightly greater than 1 due to numerical
   * errors, which is bad for computing the asin later */
  sinHalfAngle = clamp(sinHalfAngle, -1, 1);

  /* avoids division by zero if the orientations match too closely */
  if(!isZero(sinHalfAngle))
  {
    var angle = 2*Math.asin(sinHalfAngle);
    var sign = (qDelta.w < 0) ? -1 : 1;
    this.torque.multiplyScalar(1/sinHalfAngle * angle * this.controlParams.pdGains[0] * sign);
  }
  else
  {
    this.torque.set(0, 0, 0);
  }

  /* adds the derivative part */
  this.torque.add(curOmega.multiplyScalar(this.controlParams.pdGains[1]));

  if(this.torque.length() > this.controlParams.torqueLimit)
    this.torque.multiplyScalar(this.controlParams.torqueLimit/this.torque.length());
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
  var dir = new THREE.Vector3(0, 0, 0);
  dir.copy(this.torque);
  dir.normalize();

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
  var charFrame = new THREE.Quaternion();
  charFrame.copy(this.trunk.getHeading());

  /* computes the torques for all the joints */
  for(var i = 0; i < this.joints.length; i++) {
    var fbGains = this.joints[i].controlParams.fbGains;
    var pitch = fbP.y * fbGains[0] + fbD.y * fbGains[1] + targetAngles[i];
    var roll = -fbP.x * fbGains[0] - fbD.x * fbGains[1];
    this.joints[i].computeTargetQFromRelAngles(pitch, roll);
    this.joints[i].computeTorque(charFrame);
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
  this.LFTransform = new THREE.Matrix4();
  this.LFEuler = new THREE.Euler();
  this.torqueLF = new THREE.Vector3();
  this.torqueSwing = new THREE.Vector3();
  
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
};

LegFrame.prototype.update = function(timeStep) {
  /* advances the gait */
  this.gait.update(timeStep);

  this.computeFeedbackAngles();

  var zero = new THREE.Vector3();
  for(var i = 0; i < this.legs.length; i++)
  {
    if(this.gait.isStanceLeg(i))
      this.legs[i].computeTorques(this.gait.getAnglesForLeg(i),
                                  zero, zero);
    else
      this.legs[i].computeTorques(this.gait.getAnglesForLeg(i),
                                  this.fbP, this.fbD);
  }

  this.applyNetTorque();
};

LegFrame.prototype.computeCOMPosNVel = function() {
  var aux = new THREE.Vector3();
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
    this.computeCOMPosNVel();
    this.fbP.copy(this.COMPos);
    this.fbD.copy(this.COMVel);
    this.standWorldPos.multiplyScalar(1/numStanceLegs);
    this.fbP.sub(this.standWorldPos);
    this.fbP.z = 0;
    this.fbP.applyQuaternion(this.trunk.getHeading().conjugate());

    this.fbD.applyQuaternion(this.trunk.getHeading().conjugate());
  }
  else
  {
    this.fbP.set(0, 0, 0);
    this.fbD.set(0, 0, 0);
  }
};

LegFrame.prototype.applyNetTorque = function() {
  var numberStanceLegs = 0;
  this.computeTorqueLF();
  var dir = new THREE.Vector3();
  dir.copy(this.torqueLF);
  dir.normalize();
  this.torqueSwing.set(0, 0, 0);
  for(var i = 0; i < this.legs.length; i++) {
    if(this.gait.isStanceLeg(i))
      numberStanceLegs++;
    else
      this.torqueSwing.add(this.legs[i].joints[0].torque);
  }

  for(var i = 0; i < this.legs.length; i++)
  {
    if(this.gait.isStanceLeg(i))
    {
      this.legs[i].joints[0].torque.subVectors(this.torqueLF, this.torqueSwing);
      this.legs[i].joints[0].torque.divideScalar(numberStanceLegs);
    }
    this.legs[i].applyTorques();
  }
};

LegFrame.prototype.computeTorqueLF = function() {
  var charFrame = new THREE.Quaternion();

  /* gets the heading as a twist/swing decomposition of the orientation of the
   * trunk, where orientation = heading * swing */
  charFrame.copy(this.trunk.getHeading());
  var omega = this.trunk.getAngularVelocity();

  /* we want the swing to be the identity without affecting the heading
   * so in the trunk's local frame, we need to induce a rotation of -swing
   * so qDelta = -swing = -orientation * heading */
  var qDelta = this.trunk.getOrientation();
  qDelta.conjugate();
  qDelta.multiply(charFrame);
  this.torqueLF.set(qDelta.x, qDelta.y, qDelta.z);

  var sinHalfAngle = this.torqueLF.length();

  /* this could potentially be slightly greater than 1 due to numerical
   * errors, which is bad for computing the asin later */
  sinHalfAngle = clamp(sinHalfAngle, -1, 1);

  /* avoids division by zero if the orientations match too closely */
  if(!isZero(sinHalfAngle))
  {
    var angle = 2*Math.asin(sinHalfAngle);
    var sign = (qDelta.w < 0) ? -1 : 1;
    this.torqueLF.multiplyScalar(1/sinHalfAngle * angle * this.controlParams.legFrame.pdGains[0] * sign);
  }
  else
  {
    this.torqueLF.set(0, 0, 0);
  }

  this.torqueLF.applyQuaternion(this.trunk.getOrientation());

  /* adds the derivative part */
  this.torqueLF.add(omega.multiplyScalar(-this.controlParams.legFrame.pdGains[1]));

  if(this.torqueLF.length() > this.controlParams.legFrame.torqueLimit)
    this.torqueLF.multiplyScalar(this.controlParams.legFrame.torqueLimit/this.torqueLF.length());
  return this.torqueLF;
};

module.exports.Gait = Gait;
module.exports.Joint = Joint;
module.exports.LegFrame = LegFrame;