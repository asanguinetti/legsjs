var RigidBody = function(mass, size) {
  this.mass = mass;
  this.size = size;
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
  scene.world.addRigidBody(this.body);
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
};

Bone.prototype = new RigidBody;
Bone.prototype.base = RigidBody;
Bone.prototype.constructor = Bone;

Bone.prototype.buildRigidBody = function() {
  /* sets up the motion state from the current transform */
  var p = new THREE.Vector3();
  p.setFromMatrixPosition(this.transform);
  var q = new THREE.Quaternion();
  q.setFromRotationMatrix(this.transform);

  var t = new Ammo.btTransform();
  t.setIdentity();
  t.setOrigin(new Ammo.btVector3(p.x, p.y, p.z));
  t.setRotation(new Ammo.btQuaternion(q.x, q.y, q.z, q.w));
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
};

Ground.prototype = new RigidBody;
Ground.prototype.base = RigidBody;
Ground.prototype.constructor = Ground;

Ground.prototype.buildRigidBody = function() {
  /* sets up the motion state from the current transform */
  var p = new THREE.Vector3();
  p.setFromMatrixPosition(this.transform);
  var q = new THREE.Quaternion();
  q.setFromRotationMatrix(this.transform);

  var t = new Ammo.btTransform();
  t.setIdentity();
  t.setOrigin(new Ammo.btVector3(p.x, p.y, p.z));
  t.setRotation(new Ammo.btQuaternion(q.x, q.y, q.z, q.w));
  var motionState = new Ammo.btDefaultMotionState(t);
  
  var localInertia = new Ammo.btVector3(0, 0, 0);
  var halfExtents = new Ammo.btVector3(this.size.x, this.size.y, this.size.z);
  var shape = new Ammo.btBoxShape(halfExtents);
  shape.calculateLocalInertia(this.mass,localInertia);
  var rbInfo = new Ammo.btRigidBodyConstructionInfo(this.mass, motionState, 
                                                    shape, localInertia);
  this.body = new Ammo.btRigidBody(rbInfo);
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
};

Trunk.prototype = new RigidBody;
Trunk.prototype.base = RigidBody;
Trunk.prototype.constructor = Trunk;

Trunk.prototype.buildRigidBody = function() {
  var t = new Ammo.btTransform();

  /* sets up the motion state from the current transform */
  var p = new THREE.Vector3();
  p.setFromMatrixPosition(this.transform);
  var q = new THREE.Quaternion();
  q.setFromRotationMatrix(this.transform);

  t.setIdentity();
  t.setOrigin(new Ammo.btVector3(p.x, p.y, p.z));
  t.setRotation(new Ammo.btQuaternion(q.x, q.y, q.z, q.w));
  var motionState = new Ammo.btDefaultMotionState(t);

  var shape = new Ammo.btCompoundShape(true);
  var dir = new Ammo.btVector3(this.size.x, 0, -this.size.z);
  var shoulderLength = 0.5 * dir.length();
  var backboneShape = new Ammo.btBoxShape(new Ammo.btVector3(0.2, this.size.y, 0.2));
  var shoulderShape = new Ammo.btBoxShape(new Ammo.btVector3(0.2, shoulderLength, 0.2));

  /* adds the backbone part */
  t.setIdentity();
  shape.addChildShape(t, backboneShape);

  /* adds the shoulders and hip bones */
  t.getBasis().setEulerZYX(0, Math.atan2(-dir.z(), dir.x()), Math.PI/2);
  t.setOrigin(new Ammo.btVector3(0.5*dir.x(),this.size.y,0.5*dir.z()));
  shape.addChildShape(t, shoulderShape);

  t.getBasis().setEulerZYX(0, Math.atan2(-dir.z(), -dir.x()), Math.PI/2);
  t.setOrigin(new Ammo.btVector3(-0.5*dir.x(),this.size.y,0.5*dir.z()));
  shape.addChildShape(t, shoulderShape);

  t.getBasis().setEulerZYX(0, Math.atan2(-dir.z(), dir.x()), Math.PI/2);
  t.setOrigin(new Ammo.btVector3(0.5*dir.x(),-this.size.y,0.5*dir.z()));
  shape.addChildShape(t, shoulderShape);

  t.getBasis().setEulerZYX(0, Math.atan2(-dir.z(), -dir.x()), Math.PI/2);
  t.setOrigin(new Ammo.btVector3(-0.5*dir.x(),-this.size.y,0.5*dir.z()));
  shape.addChildShape(t, shoulderShape);

  var localInertia = new Ammo.btVector3(0, 0, 0);
  shape.calculateLocalInertia(this.mass, localInertia);

  var rbInfo = new Ammo.btRigidBodyConstructionInfo(this.mass, motionState, 
                                                    shape, localInertia);
  
  this.body = new Ammo.btRigidBody(rbInfo);
};

Trunk.prototype.buildVisual = function() {
  var mesh = new THREE.Mesh(new THREE.BoxGeometry(2*this.size.x, 2*this.size.y, 2*this.size.z), 
                            new THREE.MeshLambertMaterial({color: 0x66a5ff}));
  mesh.receiveShadow = true;
  mesh.castShadow = true;

  this.visual = new THREE.Object3D();
  this.visual.add(mesh);
};