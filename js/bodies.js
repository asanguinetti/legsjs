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

var CollisionGroup = {
  NONE: 0,
  BONE: 1,
  GROUND: 2,
  MUSCLE: 4
}

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
  this.heel = new Bone(mass, size);
  this.forefoot = new Bone(mass, new THREE.Vector3(0.2, 0.2, 0.2));
  this.forefoot.snapTo(new THREE.Vector3(0, 0, 0),
                       this.heel,
                       new THREE.Vector3(0, 0, -size.z - size.x));
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

var Muscle = function(bodyA, snapPointA, bodyB, snapPointB, restLength, delta, phase) {
  this.base.call(this, 1, new THREE.Vector3(0.1, 0.5*restLength, 0.1));

  this.restLength = restLength;
  this.delta = delta;
  this.phase = phase;

  this.bodyA = bodyA;
  this.snapPointA = snapPointA;
  this.bodyB = bodyB;
  this.snapPointB = snapPointB;

  this.collisionGroup = CollisionGroup.MUSCLE;
  this.collidesWith = CollisionGroup.NONE;
};

Muscle.prototype = new RigidBody;
Muscle.prototype.base = RigidBody;
Muscle.prototype.constructor = Muscle;

Muscle.prototype.buildAndInsert = function(scene) {
  /* converts the local snap point of object A to world frame */
  var worldSnapPointA = new THREE.Vector4(this.snapPointA.x, 
                                          this.snapPointA.y, 
                                          this.snapPointA.z, 
                                          1);
  worldSnapPointA.applyMatrix4(this.bodyA.transform);

  /* converts the local snap point of object B to world frame */
  var worldSnapPointB = new THREE.Vector4(this.snapPointB.x, 
                                          this.snapPointB.y, 
                                          this.snapPointB.z, 
                                          1);
  worldSnapPointB.applyMatrix4(this.bodyB.transform);

  var fwd = new THREE.Vector3();
  fwd.subVectors(worldSnapPointB, worldSnapPointA);

  var center = new THREE.Vector4();
  center.addVectors(worldSnapPointA, fwd.multiplyScalar(0.5));

  var q = new THREE.Quaternion();
  q.setFromUnitVectors(new THREE.Vector3(0, -1, 0), fwd.normalize());
  
  this.transform.identity();
  this.transform.makeRotationFromQuaternion(q);
  this.transform.setPosition(center);

  RigidBody.prototype.buildAndInsert.call(this, scene);

  this.c1 = new Ammo.btPoint2PointConstraint(this.bodyA.body, 
                                             this.body,
                                             new Ammo.btVector3(this.snapPointA.x,
                                                                this.snapPointA.y,
                                                                this.snapPointA.z),
                                             new Ammo.btVector3(0, 0.5*this.restLength, 0));
  this.c2 = new Ammo.btPoint2PointConstraint(this.bodyB.body, 
                                             this.body,
                                             new Ammo.btVector3(this.snapPointB.x,
                                                                this.snapPointB.y,
                                                                this.snapPointB.z),
                                             new Ammo.btVector3(0, -0.5*this.restLength, 0));
  scene.world.addConstraint(this.c1, true);
  scene.world.addConstraint(this.c2, true);
  this.time = 0;
  this.update(0);
}

Muscle.prototype.buildRigidBody = function() {
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

Muscle.prototype.buildVisual = function() {
  var mesh = new THREE.Mesh(new THREE.CylinderGeometry(this.size.x, 
                                                       this.size.z,
                                                       1), 
                            new THREE.MeshLambertMaterial({color: 0xff8888}));
  mesh.receiveShadow = false;
  mesh.castShadow = false;

  this.visual = new THREE.Object3D();
  this.visual.add(mesh);
};

Muscle.prototype.update = function(timeDelta, showVisual) {
  this.time += timeDelta;
  this.curLength = this.restLength + this.delta * Math.sin(this.time + this.phase);
  this.c1.setPivotB(new Ammo.btVector3(0, 0.5*this.curLength, 0));
  this.c2.setPivotB(new Ammo.btVector3(0, -0.5*this.curLength, 0));
  this.visual.scale.y = this.curLength;

  /* computes the color of the muscle */
  var r = (this.curLength - this.restLength) / this.delta;
  this.visual.children[0].material.color = new THREE.Color( 0.5 + 0.5*r, 0.5 - 0.5*r, 0 );
  this.visual.visible = showVisual;
}