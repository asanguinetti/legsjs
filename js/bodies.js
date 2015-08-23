if(typeof THREE === 'undefined')
  THREE = require('three');

if(typeof Ammo === 'undefined')
  Ammo = require('./ammo.js');

if(typeof SkeletalFigure === 'undefined')
  SkeletalFigure = require('./skeletal_figure.js');

var isZero = function(x) {
  var e = 0.0000000001;
  return -e < x && x < e;
};

var clamp = function(x, min, max) {
  return Math.min(Math.max(x, min), max);
};

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

  var p = new THREE.Vector3(bulletT.getOrigin().x(), 
                            bulletT.getOrigin().y(), 
                            bulletT.getOrigin().z());
  var q = new THREE.Quaternion(bulletT.getRotation().x(), 
                               bulletT.getRotation().y(), 
                               bulletT.getRotation().z(), 
                               bulletT.getRotation().w());

  t.identity();
  t.makeRotationFromQuaternion(q);
  t.setPosition(p);

  return t;
};

var signedAngleTo = function(a, b) {
  return Math.atan2(a.x*b.y - a.y*b.x, a.x*b.x + a.y*b.y);
};

var extend = function(extended, base) {
  extended.prototype = Object.create(base.prototype);
  extended.prototype.base = base;
  extended.prototype.constructor = extended;
};

var CollisionGroup = {
  NONE: 0,
  BONE: 1,
  GROUND: 2,
  MUSCLE: 4
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

var BipedWalkingGait = function() {
  this.state = 0;
  this.stateTime = 0;

  /* swh, swk, swa, sth, stk, sta */
  var liftTargets = [
    [0.5, -1.1, 0.6+Math.PI/2], [0, -0.05, Math.PI/2]
  ];
  var strikeTargets = [
    [-0.1, -0.05, 0.15+Math.PI/2], [0, -0.1, Math.PI/2]
  ];

  this.stateTargets = [
    liftTargets,
    strikeTargets,
    liftTargets,
    strikeTargets
  ];

  this.stateStanceLeg = [
    0, 0, 1, 1
  ];
};

extend(BipedWalkingGait, Gait);

BipedWalkingGait.prototype.setContactForLeg = function(i, contact) {
  if((this.state == 1 && i == 1) || (this.state == 3 && i == 0)) {
    if(contact) {
      this.state += 1;
      this.state %= 4;
      this.stateTime = 0;
    }
  }
};

BipedWalkingGait.prototype.update = function(timeStep) {
  this.stateTime += timeStep;
  if(this.state == 0 || this.state == 2) {
    if(this.stateTime > 0.3) {
      this.state += 1;
      this.state %= 4;
      this.stateTime = 0;
    }
  }
};

var QuadrupedWalkingGait = function() {
  this.state = 1;
  this.stateTime = 0;

  /* swh, swk, swa, swt, sth, stk, sta, stt */
  this.liftTargets = [];
  this.strikeTargets = [];

  this.stateTargets = [
    this.liftTargets,
    this.strikeTargets,
    this.liftTargets,
    this.strikeTargets
  ];

  this.stateStanceLeg = [
    0, 0, 1, 1
  ];
};

extend(QuadrupedWalkingGait, Gait);

QuadrupedWalkingGait.prototype.setContactForLeg = function(i, contact) {
  if((this.state == 1 && i == 1) || (this.state == 3 && i == 0)) {
    if(contact) {
      this.state += 1;
      this.state %= 4;
      this.stateTime = 0;
    }
  }
};

QuadrupedWalkingGait.prototype.update = function(timeStep) {
  this.stateTime += timeStep;
  if(this.state == 0 || this.state == 2) {
    if(this.stateTime > 0.3) {
      this.state += 1;
      this.state %= 4;
      this.stateTime = 0;
    }
  }
};

var QuadrupedWalkingGaitRear = function() {
  this.base.call(this);
  /* swh, swk, swa, swt */
  this.liftTargets[0] = [0.45, -1.6, 1.1, Math.PI/2];
  /* sth, stk, sta, stt */
  this.liftTargets[1] = [0.1, -0.65, 0.5, Math.PI/2 - 0.5];
  /* swh, swk, swa, swt */
  this.strikeTargets[0] = [0.1, -0.65, 0.5, Math.PI/2 - 0.5];
  /* sth, stk, sta, stt */
  this.strikeTargets[1] = [0.1, -0.65, 0.5, Math.PI/2 - 0.5];
};

extend(QuadrupedWalkingGaitRear, QuadrupedWalkingGait);

var QuadrupedWalkingGaitFront = function() {
  this.base.call(this);
  /* swh, swk, swa, swt */
  this.liftTargets[0] = [-0.4, 1.4, -1.7, Math.PI/2];
  /* sth, stk, sta, stt */
  this.liftTargets[1] = [-0.15, 0.2, -0.1, Math.PI/2 - 0.2];
  /* swh, swk, swa, swt */
  this.strikeTargets[0] = [-0.15, 0.2, -0.1, Math.PI/2 - 0.2];
  /* sth, stk, sta, stt */
  this.strikeTargets[1] = [-0.15, 0.2, -0.1, Math.PI/2 - 0.2];

  this.stateStanceLeg = [
    1, 1, 0, 0
  ];
};

extend(QuadrupedWalkingGaitFront, QuadrupedWalkingGait);

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
  var mat4 = new THREE.Matrix4();
  
  var p = worldPoint;
  if(worldPoint === undefined)
    p = new THREE.Vector3();

  p.copy(localPoint);

  var t = this.btTransform;
  if(this.body !== undefined)
    t = this.body.getCenterOfMassTransform();

  bullet2ThreeTransform(t, mat4);

  p.applyMatrix4(mat4);

  return p;
};

RigidBody.prototype.toLocalFrame = function(worldPoint, localPoint) {
  var mat4 = new THREE.Matrix4();
  
  var p = localPoint;
  if(localPoint === undefined)
    p = new THREE.Vector3();

  p.copy(worldPoint);

  var t = this.btTransform;
  if(this.body !== undefined)
    t = this.body.getCenterOfMassTransform();

  bullet2ThreeTransform(t, mat4);

  mat4.getInverse(mat4);
  p.applyMatrix4(mat4);

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
  var q = this.body.getCenterOfMassTransform().getRotation()
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

RigidBody.prototype.getEulerRotation = function() {
  var t = this.btTransform;
  if(this.body !== undefined)
  {
    t = this.body.getCenterOfMassTransform();
  }
  bullet2ThreeTransform(t, this.transformAux);

  var euler = new THREE.Euler();
  euler.setFromRotationMatrix(this.transformAux, 'XYZ');
  return euler.toVector3();
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
}

RigidBody.prototype.buildAndInsert = function(scene) {
  this.buildRigidBody();
  this.body.setActivationState(4);
  scene.world.addRigidBody(this.body, this.collisionGroup, this.collidesWith);
  scene.world.bodies.push(this);

  this.buildVisual();
  scene.add(this.visual);
}

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
}

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

Bone.prototype = new RigidBody;
Bone.prototype.base = RigidBody;
Bone.prototype.constructor = Bone;

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
  this.heel = new Bone(mass, new THREE.Vector3(size.x, size.y, size.z - size.x));
  this.heel.translate(0, 0, size.x);
  this.forefoot = new Bone(mass, new THREE.Vector3(size.x, size.x, size.x));
  this.forefoot.snapTo(new THREE.Vector3(0, 0, 0),
                       this.heel,
                       new THREE.Vector3(0, 0, -this.heel.size.z - this.forefoot.size.x));
  this.collisionGroup = CollisionGroup.BONE;
};

Foot.prototype = new RigidBody;
Foot.prototype.base = RigidBody;
Foot.prototype.constructor = Foot;

Foot.prototype.buildRigidBody = function() {
  var t = new Ammo.btTransform();

  /* sets up the motion state from the current transform */
  three2BulletTransform(this.transform, t);
  var motionState = new Ammo.btDefaultMotionState(t);

  var shape = new Ammo.btCompoundShape(true);
  var heelShape = new Ammo.btBoxShape(new Ammo.btVector3(0.9*this.heel.size.x,
                                                         0.9*this.heel.size.y,
                                                         0.9*this.heel.size.z));
  var forefootShape = new Ammo.btSphereShape(0.9*this.forefoot.size.x);

  /* adds the heel part */
  three2BulletTransform(this.heel.transform, t);
  shape.addChildShape(t, heelShape);

  /* adds the forefoot */
  three2BulletTransform(this.forefoot.transform, t);
  shape.addChildShape(t, forefootShape);

  var localInertia = new Ammo.btVector3(0, 0, 0);
  shape.calculateLocalInertia(this.mass, localInertia);

  var rbInfo = new Ammo.btRigidBodyConstructionInfo(this.mass, motionState, 
                                                    shape, localInertia);
  
  this.body = new Ammo.btRigidBody(rbInfo);
};

Foot.prototype.buildVisual = function() {
  var heel = new THREE.Mesh(new THREE.BoxGeometry(1.8*this.heel.size.x,
                                                  1.8*this.heel.size.y,
                                                  1.8*this.heel.size.z), 
                            new THREE.MeshLambertMaterial({color: 0x66a5ff}));
  heel.applyMatrix(this.heel.transform);
  heel.receiveShadow = true;
  heel.castShadow = false;

  var forefoot = new THREE.Mesh(new THREE.SphereGeometry(0.9*this.forefoot.size.x), 
                                new THREE.MeshLambertMaterial({color: 0x66a5ff}));
  forefoot.applyMatrix(this.forefoot.transform);
  forefoot.receiveShadow = true;
  forefoot.castShadow = false;

  this.visual = new THREE.Object3D();
  this.visual.add(heel);
  this.visual.add(forefoot);
};

var Ground = function(size) {
  this.base.call(this, 0, size);
  this.collisionGroup = CollisionGroup.GROUND;
  this.collidesWith = CollisionGroup.BONE;
};

Ground.prototype = new RigidBody;
Ground.prototype.base = RigidBody;
Ground.prototype.constructor = Ground;

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

var Trunk = function(mass, size) {
  this.base.call(this, mass, size);
  this.collisionGroup = CollisionGroup.BONE;

  var dir = new THREE.Vector3(this.size.x, 0, -this.size.z);
  var shoulderLength = 0.5*dir.length();

  this.backbone = new Bone(mass, new THREE.Vector3(0.2, this.size.y, 0.2));

  this.shoulderLeft = new Bone(mass, new THREE.Vector3(0.2, shoulderLength, 0.2));
  this.shoulderLeft.rotateAxis(new THREE.Vector3(0, 0, 1), Math.PI/2);
  this.shoulderLeft.rotateAxis(new THREE.Vector3(0, 1, 0), Math.atan2(-dir.z, dir.x));
  this.shoulderLeft.snapTo(new THREE.Vector3(0, this.shoulderLeft.size.y, 0),
                           this.backbone,
                           new THREE.Vector3(0, this.backbone.size.y, 0));

  this.shoulderRight = new Bone(mass, new THREE.Vector3(0.2, shoulderLength, 0.2));
  this.shoulderRight.rotateAxis(new THREE.Vector3(0, 0, 1), Math.PI/2);
  this.shoulderRight.rotateAxis(new THREE.Vector3(0, 1, 0), Math.atan2(-dir.z, -dir.x));
  this.shoulderRight.snapTo(new THREE.Vector3(0, this.shoulderRight.size.y, 0),
                            this.backbone,
                            new THREE.Vector3(0, this.backbone.size.y, 0));

  this.hipLeft = new Bone(mass, new THREE.Vector3(0.2, shoulderLength, 0.2));
  this.hipLeft.rotateAxis(new THREE.Vector3(0, 0, 1), Math.PI/2);
  this.hipLeft.rotateAxis(new THREE.Vector3(0, 1, 0), Math.atan2(-dir.z, dir.x));
  this.hipLeft.snapTo(new THREE.Vector3(0, this.hipLeft.size.y, 0),
                      this.backbone,
                      new THREE.Vector3(0, -this.backbone.size.y, 0));

  this.hipRight = new Bone(mass, new THREE.Vector3(0.2, shoulderLength, 0.2));
  this.hipRight.rotateAxis(new THREE.Vector3(0, 0, 1), Math.PI/2);
  this.hipRight.rotateAxis(new THREE.Vector3(0, 1, 0), Math.atan2(-dir.z, -dir.x));
  this.hipRight.snapTo(new THREE.Vector3(0, this.hipRight.size.y, 0),
                       this.backbone,
                       new THREE.Vector3(0, -this.backbone.size.y, 0));
};

Trunk.prototype = new RigidBody;
Trunk.prototype.base = RigidBody;
Trunk.prototype.constructor = Trunk;

Trunk.prototype.buildRigidBody = function() {
  var t = new Ammo.btTransform();

  /* sets up the motion state from the current transform */
  three2BulletTransform(this.transform, t);
  var motionState = new Ammo.btDefaultMotionState(t);

  var shape = new Ammo.btCompoundShape(true);
  var backboneShape = new Ammo.btBoxShape(new Ammo.btVector3(0.9*this.backbone.size.x,
                                                             0.9*this.backbone.size.y,
                                                             0.9*this.backbone.size.z));
  var shoulderShape = new Ammo.btBoxShape(new Ammo.btVector3(0.9*this.shoulderLeft.size.x,
                                                             0.9*this.shoulderLeft.size.y,
                                                             0.9*this.shoulderLeft.size.z));

  /* adds the backbone part */
  three2BulletTransform(this.backbone.transform, t);
  shape.addChildShape(t, backboneShape);

  /* adds the shoulders and hip bones */
  three2BulletTransform(this.shoulderLeft.transform, t);
  shape.addChildShape(t, shoulderShape);

  three2BulletTransform(this.shoulderRight.transform, t);
  shape.addChildShape(t, shoulderShape);

  three2BulletTransform(this.hipLeft.transform, t);
  shape.addChildShape(t, shoulderShape);

  three2BulletTransform(this.hipRight.transform, t);
  shape.addChildShape(t, shoulderShape);

  var localInertia = new Ammo.btVector3(0, 0, 0);
  shape.calculateLocalInertia(this.mass, localInertia);

  var rbInfo = new Ammo.btRigidBodyConstructionInfo(this.mass, motionState, 
                                                    shape, localInertia);
  
  this.body = new Ammo.btRigidBody(rbInfo);
};

Trunk.prototype.buildVisual = function() {
  var visual = new THREE.Object3D();
  
  var segments = [this.backbone, 
                  this.shoulderLeft, this.shoulderRight, 
                  this.hipLeft, this.hipRight];
  segments.forEach(function(segment) {
    var mesh = new THREE.Mesh(new THREE.BoxGeometry(1.8*segment.size.x, 
                                                    1.8*segment.size.y, 
                                                    1.8*segment.size.z), 
                              new THREE.MeshLambertMaterial({color: 0x66a5ff}));
    mesh.applyMatrix(segment.transform);
    mesh.receiveShadow = true;
    mesh.castShadow = true;
    visual.add(mesh);
  });

  this.visual = visual;
};

var Joint = function(skJoint, controlParams, 
                     angularLowerLimit, angularUpperLimit, absAngle) {
  this.bodyA = skJoint.parentSegment.body;
  this.bodyB = skJoint.childSegment.body;
  this.pivotInA = (new THREE.Vector3()).fromArray(skJoint.snapPointParent);
  this.pivotInB = (new THREE.Vector3()).fromArray(skJoint.snapPointChild);
  this.targetAngle = [0, 0, 0];
  this.controlParams = controlParams;

  this.absAngle = absAngle;
  this.targetQ = new THREE.Quaternion();
  this.auxEuler = new THREE.Euler();
  this.curQ = new THREE.Quaternion();
  this.curOmega = new THREE.Vector3();
  this.torque = new THREE.Vector3();
  this.btTorque = new Ammo.btVector3();

  var zero = new THREE.Vector3(0, 0, 0);
  var unit = new THREE.Vector3(1, 0, 0);
  this.torqueArrow = new THREE.ArrowHelper(unit, zero, 0, 0x00ff00);

  this.angularLowerLimit = angularLowerLimit;
  this.angularUpperLimit = angularUpperLimit;

  /* TODO: add support for setting the joint axis */
  this.axis = [new THREE.Vector3(1, 0, 0), 
               new THREE.Vector3(0, 1, 0),
               new THREE.Vector3(0, 0, 1)];
};

Joint.prototype.getPosition = function() {
  var pivotWorldA = this.bodyA.toWorldFrame(this.pivotInA);
  var pivotWorldB = this.bodyB.toWorldFrame(this.pivotInB);
  pivotWorldA.lerp(pivotWorldB, 0.5);
  return pivotWorldA;
};

Joint.prototype.getAxis = function() {
  var axis;
  for(var i = 0; i < 3; i++) {
    axis = this.c.getAxis(i);
    this.axis[i].set(axis.x(), axis.y(), axis.z());
  }
  return this.axis;
};

Joint.prototype.getTorqueForVirtualForce = function(point, force) {
  // torque = (axis x (point - anchor)) * force
  var a = this.getAxis();
  var b;
  var torque = new THREE.Vector3();
  for(var i = 0; i < a.length; i++) {
    if(this.angularLowerLimit.getComponent(i) > this.angularUpperLimit.getComponent(i)) {
      b = this.getPosition();
      b.subVectors(point, b);
      b.crossVectors(a[i], b);
      torque.setComponent(i, b.dot(force));
    }
  }
  return torque;
};

Joint.prototype.computeTargetQFromRelAngles = function(pitch, roll) {
  this.auxEuler.set(pitch, roll, 0, 'XYZ');
  this.targetQ.setFromEuler(this.auxEuler).conjugate();
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
    this.computeRelTorque(this.getRelativeOrientation(), 
                          this.targetQ, 
                          this.getRelativeVelocity());

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
  this.torqueArrow.position.copy(this.getPosition());
  this.torqueArrow.setLength(this.torque.length()*0.1);
  this.torqueArrow.setDirection(dir);

  this.btTorque.setValue(this.torque.x, this.torque.y, this.torque.z);
  /* applies the equal and opposite torques to both objects */
  this.bodyA.body.applyTorque(this.btTorque);
  this.btTorque.op_mul(-1);
  this.bodyB.body.applyTorque(this.btTorque);
};

Joint.prototype.buildAndInsert = function(scene) {
  var frameInA = new Ammo.btTransform();
  var frameInB = new Ammo.btTransform();
  frameInA.setIdentity();
  frameInA.setOrigin(new Ammo.btVector3(this.pivotInA.x,
                                        this.pivotInA.y,
                                        this.pivotInA.z));
  frameInB.setIdentity();
  frameInB.setOrigin(new Ammo.btVector3(this.pivotInB.x,
                                        this.pivotInB.y,
                                        this.pivotInB.z));

  this.c = new Ammo.btGeneric6DofConstraint(this.bodyA.body, 
                                            this.bodyB.body, 
                                            frameInA, 
                                            frameInB, 
                                            true);

  /* TODO: add support for setting the joint axis */
  this.c.setAngularLowerLimit(new Ammo.btVector3(this.angularLowerLimit.x,
                                                 this.angularLowerLimit.y,
                                                 this.angularLowerLimit.z));
  this.c.setAngularUpperLimit(new Ammo.btVector3(this.angularUpperLimit.x,
                                                 this.angularUpperLimit.y,
                                                 this.angularUpperLimit.z));

  scene.world.addConstraint(this.c, true);

  scene.add(this.torqueArrow);
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

    var angularLowerLimit = new THREE.Vector3(1, -Math.PI/4, 0);
    var angularUpperLimit = new THREE.Vector3(0, Math.PI/4, 0);

    if(skJoint.type == SkeletalFigure.SkeletalJoint.types.HINGE)
    {
      angularLowerLimit = new THREE.Vector3(1, 0, 0);
      angularUpperLimit = new THREE.Vector3(0, 0, 0);
    }

    this.joints.push(new Joint(skJoint,
                               this.controlParams.joints[i],
                               angularLowerLimit,
                               angularUpperLimit,
                               absAngle));
    absAngle = false;
    if(skJoint.childSegment.childrenJoints.length == 0)
      break;
    skJoint = skJoint.childSegment.childrenJoints[0];
    i++;
  };
};

Leg.prototype.buildAndInsert = function(scene) {
  /* buils and inserts the leg segments */
  this.segments.forEach(function(segment) {
    segment.buildAndInsert(scene);
  });

  /* buils and inserts the leg joints */
  this.joints.forEach(function(joint) {
    joint.buildAndInsert(scene);
  });
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

  var zero = new THREE.Vector3(0, 0, 0);
  var unit = new THREE.Vector3(1, 0, 0);
  this.torqueLFArrow = new THREE.ArrowHelper(unit, zero, 0, 0xff0000);

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
  this.torqueLFArrow.setLength(this.torqueLF.length() * 0.1);
  var dir = new THREE.Vector3();
  dir.copy(this.torqueLF);
  dir.normalize();
  this.torqueLFArrow.position.copy(this.trunk.toWorldFrame(new THREE.Vector3(0, 0, 0)));
  this.torqueLFArrow.setDirection(dir);
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

LegFrame.prototype.buildAndInsert = function(scene) {
  /* builds and inserts the trunk */
  /* FIXME: this will cause problems if the trunk has already been built */
  this.trunk.buildAndInsert(scene);
  /* buils and inserts the legs */
  this.legs.forEach(function(leg) {
    leg.buildAndInsert(scene);
  });

  scene.add(this.torqueLFArrow);
};

module.exports.Bone = Bone;
module.exports.Joint = Joint;
module.exports.Gait = Gait;
module.exports.LegFrame = LegFrame;