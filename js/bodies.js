var three2BulletTransform = function(threeT, bulletT) {
  t = bulletT
  if(bulletT === undefined)
    t = new Ammo.btTransform();

  var p = new THREE.Vector3();
  p.setFromMatrixPosition(threeT);
  var q = new THREE.Quaternion();
  q.setFromRotationMatrix(threeT);

  t.setIdentity();
  t.setOrigin(new Ammo.btVector3(p.x, p.y, p.z));
  t.setRotation(new Ammo.btQuaternion(q.x, q.y, q.z, q.w));

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
  extended.prototype = base.prototype;
  extended.prototype.base = base;
  extended.prototype.constructor = extended;
};

/**
 * Solves a planar inverse kinematic problem with 3 links and 3 rotational joints.
 * \param list q1 List of target joint angles starting from the closest to the 
 *                base (1st solution).
 * \param list q2 List of target joint angles starting from the closest to the 
 *                base (2nd solution).
 * \param list l List of link lengths conforming the chain, starting from 
 *               the closest to the base.
 * \param list p End-effector position and orientation.
 * \return list List of joint angles starting from the closest to the base.
 */
var ikSolve = function(q1, q2, l, p) {
  var w = [];

  /* gets the position and angle of the wrist from the following equations.
   * w_x = p_x - l_3*sin(phi)
   * w_y = p_y + l_3*cos(phi)
   * alpha = atan(-w_y/w_x) 
   */
  w[0] = p[0] - l[2]*Math.sin(p[2]);
  w[1] = p[1] + l[2]*Math.cos(p[2]);
  w[2] = Math.atan2(w[0], -w[1]);

  /* gets the squared length of the wrist's position vector because it is used
   * many times later
   * we also cap it to the squared length of the arm */
  w[3] = Math.min(w[0]*w[0] + w[1]*w[1], Math.pow(l[0] + l[1], 2));

  /* gets q_2 as the supplement of beta which is itself calculated from the
   * cosine theorem 
   * q_2 = PI - beta
   * l_1^2 + l_2^2 - 2*l_1*l_2*cos(beta) = w_x^2 + w_y^2
   */
  q1[1] = Math.PI - Math.acos((l[0]*l[0] + l[1]*l[1] - w[3])/(2*l[0]*l[1]));
  q2[1] = -q1[1]


  /* similarly:
   * q_1 = alpha - gamma
   * l_1^2 + w_x^2 + w_y^2 - 2*l_1*sqrt(w_x^2 + w_y^2)*cos(gamma) = l_2^2
   */
   q1[0] = w[2] - Math.acos((l[0]*l[0] - l[1]*l[1] + w[3])/(2*l[0]*Math.sqrt(w[3])));
   q2[0] = 2*w[2] - q1[0];

   /* finally:
    * q_3 = phi - q_2 - q_1
    */
   q1[2] = p[2] - q1[1] - q1[0];
   q2[2] = p[2] - q2[1] - q2[0];
};

var CollisionGroup = {
  NONE: 0,
  BONE: 1,
  GROUND: 2,
  MUSCLE: 4
}

var Gait = function(phase) {
  this.phase = phase;
  /* FIXME: strike and take off positions are inverted */
  this.swingCycle = 0.4;
  this.period = 3;
  this.targeFootPos = [0, 0, -Math.PI/8];
  this.time = this.phase * this.period;
  this.normalizedTime = this.phase;
};

Gait.prototype.update = function(timeStep) {
  this.time += timeStep;
  this.time %= this.period;
  this.normalizedTime = this.time / this.period;
  if(this.normalizedTime < this.swingCycle) {
    /* renormalizes the time for the swing cycle */
    this.normalizedTime /= this.swingCycle;
    /* linear interpolation between take off and strike positions */
    this.targeFootPos[0] = this.normalizedTime*this.strikePosition + 
                           (1 - this.normalizedTime)*this.takeOffPosition;
    /* height uses an inverse parabola with roots at 0 and 1 */
    this.targeFootPos[1] = (this.maxSwingHeight*27/4)*this.normalizedTime*this.normalizedTime*(1 - this.normalizedTime) - this.stanceHeight;
    /* linear interpolation between take off and strike angles */
    this.targeFootPos[2] = this.normalizedTime*this.strikeAngle + 
                           (1 - this.normalizedTime)*this.takeOffAngle;
  } else {
    /* renormalizes the time for the stance cycle */
    this.normalizedTime = (this.normalizedTime - this.swingCycle)/(1 - this.swingCycle);
    /* linear interpolation between strike and take off positions */
    this.targeFootPos[0] = this.normalizedTime*this.takeOffPosition + 
                           (1 - this.normalizedTime)*this.strikePosition;
    /* height uses an inverse parabola with roots at 0 and 1 */
    this.targeFootPos[1] = -this.stanceHeight;
    /* linear interpolation between strike and take off angles */
    this.targeFootPos[2] = this.normalizedTime*this.takeOffAngle + 
                           (1 - this.normalizedTime)*this.strikeAngle;
  }
}

var FrontLegGait = function(phase) {
  this.base.call(this, phase);
  this.strikePosition = 1;
  this.takeOffPosition = -1;
  this.strikeAngle = Math.PI/4;
  this.takeOffAngle = -Math.PI/4;
  this.maxSwingHeight = 1;
  this.stanceHeight = 4.5;
};

extend(FrontLegGait, Gait);

var RearLegGait = function(phase) {
  this.base.call(this, phase);
  this.strikePosition = 1;
  this.takeOffPosition = -3;
  this.strikeAngle = Math.PI/4;
  this.takeOffAngle = -Math.PI/4;
  this.maxSwingHeight = 1;
  this.stanceHeight = 4;
};

extend(RearLegGait, Gait);

var RigidBody = function(mass, size) {
  this.mass = mass;
  this.size = size;
  this.collidesWith = CollisionGroup.GROUND;
  this.transform = new THREE.Matrix4();
  this.transformAux = new THREE.Matrix4();
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
};

RigidBody.prototype.toWorldFrame = function(localPoint, worldPoint) {
  var mat4 = new THREE.Matrix4();
  
  var p = worldPoint;
  if(worldPoint === undefined)
    p = new THREE.Vector3();

  var t = this.btTransform;
  if(this.body !== undefined)
  {
    t = this.btTransformAux;
    this.body.getMotionState().getWorldTransform(t);
  }
  bullet2ThreeTransform(t, mat4);

  p.applyMatrix4(mat4);

  return p;
};

RigidBody.prototype.getLinearVelocity = function(localPoint) {
  var vel = this.body.getLinearVelocity();

  if(localPoint !== undefined) {
    var lp = new Ammo.btVector3(localPoint.x, 
                                localPoint.y, 
                                localPoint.z);
    vel.op_add(this.body.getAngularVelocity().cross(lp));
  }

  return new THREE.Vector3(vel.x(), vel.y(), vel.z());
};

RigidBody.prototype.getEulerRotation = function() {
  var mat4 = new THREE.Matrix4();

  var t = this.btTransform;
  if(this.body !== undefined)
  {
    t = this.btTransformAux;
    this.body.getMotionState().getWorldTransform(t);
  }
  bullet2ThreeTransform(t, mat4);

  var euler = new THREE.Euler();
  euler.setFromRotationMatrix(mat4, 'XYZ');
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
  var mesh = new THREE.Mesh(new THREE.BoxGeometry(2*this.size.x, 2*this.size.y, 2*this.size.z), 
                            new THREE.MeshLambertMaterial({color: 0xeeeeee}));
  mesh.receiveShadow = true;
  mesh.castShadow = false;

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

var Joint = function(bodyA, pivotInA, bodyB, pivotInB) {
  this.bodyA = bodyA;
  this.bodyB = bodyB;
  this.pivotInA = pivotInA;
  this.pivotInB = pivotInB;
  this.targetAngle = 0;

  /* TODO: add support for setting the joint axis */
  this.axis = new THREE.Vector3(1, 0, 0);
};

Joint.prototype.getPosition = function() {
  var pivotWorldA = this.bodyA.toWorldFrame(this.pivotInA);
  var pivotWorldB = this.bodyB.toWorldFrame(this.pivotInB);
  pivotWorldA.lerp(pivotWorldB, 0.5);
  return pivotWorldA;
};

Joint.prototype.getAxis = function() {
  var axis = this.c.getAxis(0);
  this.axis.set(axis.x(), axis.y(), axis.z());
  return this.axis;
};

Joint.prototype.getTorqueForVirtualForce = function(point, force) {
  // torque = (axis x (point - anchor)) * force
  var a = this.getAxis();
  var b = this.getPosition();
  b.subVectors(point, b);
  b.crossVectors(a, b);
  return b.dot(force);
};

Joint.prototype.update = function(extraTorque) {
  var jointAxis = new Ammo.btVector3(this.c.getAxis(0));

  /* calculates the relative angular velocity of the two objects */
  var deltaOmega = new Ammo.btVector3(this.bodyB.body.getAngularVelocity());
  deltaOmega.op_sub(this.bodyA.body.getAngularVelocity())

  /* calculates the projection of the relative angular velocity along the joint axis */
  var jointVel = deltaOmega.dot(jointAxis);

  /* calculates the torque to apply using PD */
  var torqueScalar = 2000*(this.targetAngle + this.c.getAngle(0)) + 60*(0 - jointVel);
  torqueScalar += extraTorque;

  /* applies the equal and opposite torques to both objects */
  jointAxis.op_mul(torqueScalar);
  this.bodyB.body.applyTorque(jointAxis);
  jointAxis.op_mul(-1);
  this.bodyA.body.applyTorque(jointAxis);
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
  this.c.setAngularLowerLimit(new Ammo.btVector3(1, 0, 0));
  this.c.setAngularUpperLimit(new Ammo.btVector3(0, 0, 0));

  scene.world.addConstraint(this.c, true);
};

var Leg = function(trunk, pivot, segments, gait) {
  this.trunk = trunk;
  this.pivot = pivot;
  this.segments = segments;
  this.gait = gait;
  this.joints = [];
  this.time = 0;
  this.q1 = [];
  this.q2 = [];

  /* TODO: place the segments in the right orientation using IK */

  /* adds the shoulder/hip joint */
  segments[0].snapTo(new THREE.Vector3(0, 0, segments[0].size.z), trunk, pivot);
  this.joints.push(
      new Joint(trunk,
                pivot,
                segments[0],
                new THREE.Vector3(0, 0, segments[0].size.z))
  );

  /* adds the rest of the joints */
  for(var i = 1; i < segments.length; i++) {
    segments[i].snapTo(new THREE.Vector3(0, 0, segments[i].size.z),
                       segments[i-1],
                       new THREE.Vector3(0, 0, -segments[i-1].size.z));
    this.joints.push(
      new Joint(segments[i - 1],
                new THREE.Vector3(0, 0, -segments[i-1].size.z),
                segments[i],
                new THREE.Vector3(0, 0, segments[i].size.z))
    );
  }
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

Leg.prototype.update = function(timeStep) {
  this.gait.update(timeStep);
  ikSolve(this.q1, this.q2,
          [2*this.segments[0].size.z,
           2*this.segments[1].size.z,
           2*this.segments[2].size.z],
          this.gait.targeFootPos);

  /* calculates the force to help keeping the hip and shoulders height */
  var pivotPosWorld = this.trunk.toWorldFrame(this.pivot);
  var pivotVelWorld = this.trunk.getLinearVelocity(this.pivot);
  var force = new THREE.Vector3(0, 0, -5*(this.gait.stanceHeight - pivotPosWorld.z) - 5*(0 - pivotVelWorld.z));

  for(var i = 0; i < this.joints.length; i++) {
    this.joints[i].targetAngle = this.q[i];
    this.joints[i].update(this.joints[i].getTorqueForVirtualForce(pivotPosWorld, force));
  };
};

var FrontLeg = function(trunk, pivot, segments, gait) {
  this.base.call(this, trunk, pivot, segments, gait);
  this.q = this.q1;
};

FrontLeg.prototype = Leg.prototype;
FrontLeg.prototype.base = Leg;
FrontLeg.prototype.constructor = FrontLeg;

var RearLeg = function(trunk, pivot, segments, gait) {
  this.base.call(this, trunk, pivot, segments, gait);
  this.q = this.q2;
};

RearLeg.prototype = Leg.prototype;
RearLeg.prototype.base = Leg;
RearLeg.prototype.constructor = RearLeg;