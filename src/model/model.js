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

  if(localPoint === undefined)
    p.set(0, 0, 0);
  else
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
};

Bone.prototype.buildVisual = function() {
  var mesh = new THREE.Mesh(new THREE.BoxGeometry(1.8*this.size.x, 1.8*this.size.y, 1.8*this.size.z), 
                            new THREE.MeshLambertMaterial({color: 0x66a5ff}));
  mesh.receiveShadow = true;
  mesh.castShadow = true;

  this.visual = new THREE.Object3D();
  this.visual.add(mesh);
};

var Ground = function(size) {
  this.base.call(this, 0, size);
  this.collisionGroup = CollisionGroup.GROUND;
  this.collidesWith = CollisionGroup.BONE;
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

  var zero = new THREE.Vector3(0, 0, 0);
  var unit = new THREE.Vector3(1, 0, 0);
};

Joint.prototype.getPosition = function() {
  var pivotWorldA = this.bodyA.toWorldFrame(this.pivotInA);
  var pivotWorldB = this.bodyB.toWorldFrame(this.pivotInB);
  pivotWorldA.lerp(pivotWorldB, 0.5);
  return pivotWorldA;
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

var HingeJoint = function(bodyA, bodyB, pivotInA, pivotInB, lowerLimit, higherLimit) {
  this.base.call(this, bodyA, bodyB, pivotInA, pivotInB);
  this.lowerLimit = lowerLimit;
  this.higherLimit = higherLimit;
}

extend(HingeJoint, Joint);

HingeJoint.prototype.buildAndInsert = function(scene) {
  var xAxis = new Ammo.btVector3(1, 0, 0);
  this.c = new Ammo.btHingeConstraint(this.bodyA.body,
                                      this.bodyB.body,
                                      new Ammo.btVector3(this.pivotInA.x,
                                                         this.pivotInA.y,
                                                         this.pivotInA.z),
                                      new Ammo.btVector3(this.pivotInB.x,
                                                         this.pivotInB.y,
                                                         this.pivotInB.z),
                                      xAxis, xAxis, true);

  this.c.setLimit(this.lowerLimit, this.higherLimit);

  scene.world.addConstraint(this.c, true);
};

var BallSocketJoint = function(bodyA, bodyB, pivotInA, pivotInB, angularLowerLimit, angularUpperLimit) {
  this.base.call(this, bodyA, bodyB, pivotInA, pivotInB);
  this.angularLowerLimit = angularLowerLimit;
  this.angularUpperLimit = angularUpperLimit;
}

extend(BallSocketJoint, Joint);

BallSocketJoint.prototype.buildAndInsert = function(scene) {
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

  this.c.setAngularLowerLimit(new Ammo.btVector3(this.angularLowerLimit[0],
                                                 this.angularLowerLimit[1],
                                                 this.angularLowerLimit[2]));
  this.c.setAngularUpperLimit(new Ammo.btVector3(this.angularUpperLimit[0],
                                                 this.angularUpperLimit[1],
                                                 this.angularUpperLimit[2]));

  scene.world.addConstraint(this.c, true);
};

module.exports.Joint = Joint;
module.exports.HingeJoint = HingeJoint;
module.exports.BallSocketJoint = BallSocketJoint;
module.exports.Bone = Bone;
module.exports.Ground = Ground;