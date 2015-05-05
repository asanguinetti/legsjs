var assert = require('assert')
var THREE = require('three');
var bodies = require('../js/bodies.js');


var common = {};


var isNearZero = function(x) {
  var e = 0.0001;
  return -e < x && x < e;
};


var dirEqual = function(v, dir) {
  return v.angleTo(dir) == 0;
};


exports.setUp = function(callback)
{
  var scene = new THREE.Scene();
  var collisionConfiguration = new Ammo.btDefaultCollisionConfiguration();
  var dispatcher = new Ammo.btCollisionDispatcher(collisionConfiguration);
  var overlappingPairCache = new Ammo.btDbvtBroadphase();
  var solver = new Ammo.btSequentialImpulseConstraintSolver();
  scene.world = new Ammo.btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
  scene.world.setGravity(new Ammo.btVector3(0,0,-20));
  scene.world.bodies = [];

  this.bodyA = new bodies.Bone(0, new THREE.Vector3(0.2, 0.2, 1.0));
  this.bodyB = new bodies.Bone(0, new THREE.Vector3(0.2, 0.2, 1.0));

  this.bodyA.translate(0, 0, this.bodyA.size.z + this.bodyB.size.z);

  this.bodyA.buildAndInsert(scene);
  this.bodyB.buildAndInsert(scene);

  this.pivotInA = new THREE.Vector3(0, 0, -this.bodyA.size.z);
  this.pivotInB = new THREE.Vector3(0, 0, this.bodyB.size.z);

  this.pGain = 300;
  this.dGain = 10;

  this.angularLowerLimit = new THREE.Vector3(1, 0, 0);
  this.angularUpperLimit = new THREE.Vector3(0, 0, 0);

  callback();
};


/* defines a group of tests for relative joints */
exports.relative = {
  setUp: function(callback) {
    this.joint = new bodies.Joint(this.bodyA, this.pivotInA,
                                  this.bodyB, this.pivotInB,
                                  this.pGain, this.dGain, 
                                  this.angularLowerLimit, this.angularUpperLimit,
                                  false);
    callback();
  }
};


/* defines a group of tests for absolute joints */
exports.absolute = {
  setUp: function(callback) {
    this.joint = new bodies.Joint(this.bodyA, this.pivotInA,
                                  this.bodyB, this.pivotInB,
                                  this.pGain, this.dGain, 
                                  this.angularLowerLimit, this.angularUpperLimit,
                                  true);
    callback();
  }
};


common.alignedChildTest = function(test)
{
  var charFrame = new THREE.Quaternion();

  /* The target relative orientation is the neutral quaternion and both segments
   * have exactly the same orientation at this point, this means we expect no 
   * torque at all. */
  this.joint.targetQ = new THREE.Quaternion();
  this.joint.computeTorque(charFrame);
  test.ok(isNearZero(this.joint.torque.length()),
          'Torque expected to be zero since the target relative orientation is zero');

  /* Now we set a target relative orientation as some positive rotation along the X axis
   * that means that if the two segments had this relative orientation, a positive
   * rotation along the X axis would put the child segment in the same orientation as
   * the parent segment.
   * Since in the current status, both the parent and child segment have the same
   * orientation, in order to get to the target orientation, a negative rotation on
   * the X axis would have to be applied to the child segment. Which is equivalent
   * to apply the opposite rotation on the parent segment.
   */
  this.joint.targetQ.set(Math.sin(Math.PI/8), 0, 0, Math.cos(Math.PI/8));
  this.joint.targetQ.normalize();
  this.joint.computeTorque(charFrame);
  test.ok(dirEqual(this.joint.torque, new THREE.Vector3(1, 0, 0)),
          'Torque expected to be in the direction of the X axis');

  test.done();
};


common.rotatedChildTest = function(test)
{
  var charFrame = new THREE.Quaternion();

  /* rotate the child segment as if it had reached its target orientation */
  this.joint.bodyB.rotateAxis(new THREE.Vector3(1, 0, 0),
                              -Math.PI/4,
                              this.joint.pivotInB);
  this.joint.targetQ.set(Math.sin(Math.PI/8), 0, 0, Math.cos(Math.PI/8));
  this.joint.targetQ.normalize();
  this.joint.computeTorque(charFrame);
  test.ok(isNearZero(this.joint.torque.length()), 
          'Torque expected to be zero since the target relative orientation is the same as the current one');

  /* now try to make it go back to the original orientation */
  /* the torque should be in the negative X axis this time */
  this.joint.targetQ = new THREE.Quaternion();
  this.joint.computeTorque(charFrame);
  test.ok(dirEqual(this.joint.torque, new THREE.Vector3(-1, 0, 0)), 
          'Torque expected to be in the direction of the -X axis');

  test.done();
};


common.rotatedParentTest = function(test)
{
  var charFrame = new THREE.Quaternion();

  /* rotate the parent segment as if it had reached its target orientation */
  this.joint.bodyA.rotateAxis(new THREE.Vector3(1, 0, 0),
                              Math.PI/4,
                              this.joint.pivotInB);
  this.joint.targetQ.set(Math.sin(Math.PI/8), 0, 0, Math.cos(Math.PI/8));
  this.joint.targetQ.normalize();
  this.joint.computeTorque(charFrame);
  if(this.joint.absAngle)
    test.ok(dirEqual(this.joint.torque, new THREE.Vector3(1, 0, 0)),
            'Torque expected to be in the direction of the X axis');
  else
    test.ok(isNearZero(this.joint.torque.length()), 
            'Torque expected to be zero since the target relative orientation is the same as the current one');

  /* now try to make it go back to the original orientation */
  /* the torque should be in the X axis this time */
  this.joint.targetQ = new THREE.Quaternion();
  this.joint.computeTorque(charFrame);
  if(this.joint.absAngle)
    test.ok(isNearZero(this.joint.torque.length()), 
            'Torque expected to be zero since the target relative orientation is the same as the current one');
  else
    test.ok(dirEqual(this.joint.torque, new THREE.Vector3(-1, 0, 0)), 
            'Torque expected to be in the direction of the X axis');

  test.done();
};


common.angularVelocityTest = function(test)
{
  var charFrame = new THREE.Quaternion();

  this.joint.targetQ = new THREE.Quaternion();

  /* sets the angular velocity along the X axis to the child segment */
  /* the torque should be in the X axis since it's applied to the parent segment */
  this.joint.bodyB.body.setAngularVelocity(new Ammo.btVector3(10, 0, 0));
  this.joint.computeTorque(charFrame);
  test.ok(dirEqual(this.joint.torque, new THREE.Vector3(1, 0, 0)), 
          'Torque expected to be in the direction of the X axis');

  /* sets the angular velocity along the -X axis to the child segment */
  /* the torque should be in the -X axis since it's applied to the parent segment */
  this.joint.bodyB.body.setAngularVelocity(new Ammo.btVector3(-10, 0, 0));
  this.joint.computeTorque(charFrame);
  test.ok(dirEqual(this.joint.torque, new THREE.Vector3(-1, 0, 0)), 
          'Torque expected to be in the direction of the -X axis');

  /* sets an angular velocity along the X axis to the parent segment */
  /* the torque should remain in the -X axis since the parent's velocity is contributing
   * to increase relative velocity */
  this.joint.bodyA.body.setAngularVelocity(new Ammo.btVector3(5, 0, 0));
  this.joint.computeTorque(charFrame);
  test.ok(dirEqual(this.joint.torque, new THREE.Vector3(-1, 0, 0)), 
          'Torque expected to be in the direction of the -X axis');

  /* sets an angular velocity along the -X axis to the parent segment */
  /* the torque should remain in the -X axis since the parent's velocity is smaller
   * than the child's */
  this.joint.bodyA.body.setAngularVelocity(new Ammo.btVector3(-5, 0, 0));
  this.joint.computeTorque(charFrame);
  test.ok(dirEqual(this.joint.torque, new THREE.Vector3(-1, 0, 0)), 
          'Torque expected to be in the direction of the -X axis');

  /* sets a bigger angular velocity along the -X axis to the parent segment */
  /* the torque should change to the X axis since the parent's velocity is now bigger
   * than the child's (unless the joint uses an absolute frame reference) */
  this.joint.bodyA.body.setAngularVelocity(new Ammo.btVector3(-15, 0, 0));
  this.joint.computeTorque(charFrame);
  if(this.joint.absAngle)
    test.ok(dirEqual(this.joint.torque, new THREE.Vector3(-1, 0, 0)), 
            'Torque expected to be in the direction of the -X axis');  
  else
    test.ok(dirEqual(this.joint.torque, new THREE.Vector3(1, 0, 0)), 
            'Torque expected to be in the direction of the X axis');

  /* makes both velocities equal and opposite. */
  this.joint.bodyA.body.setAngularVelocity(new Ammo.btVector3(-10, 0, 0));
  this.joint.computeTorque(charFrame);
  if(this.joint.absAngle)
    test.ok(dirEqual(this.joint.torque, new THREE.Vector3(-1, 0, 0)), 
            'Torque expected to be in the direction of the -X axis');  
  else
    test.ok(isNearZero(this.joint.torque.length()),
            'Torque expected to be zero since the both parent and child velocities are equal and opposite');

  test.done();
};


common.rotatedCharFrameTest = function(test)
{
  /* rotates the character frame 90º on the Z axis */
  var charFrame = new THREE.Quaternion(0, 0, Math.sin(Math.PI/4), Math.cos(Math.PI/4));
  charFrame.normalize();

  /* the relative joint should need no torque since it has nothing to do with the
   * character frame */
  /* the absolute joint, on the other, hand would require a torque along the Z axis
   * too, but since the joint torque is the one applied to the parent, it will be
   * on the -Z axis */
  this.joint.targetQ = new THREE.Quaternion();
  this.joint.computeTorque(charFrame);
  if(this.joint.absAngle)
    test.ok(dirEqual(this.joint.torque, new THREE.Vector3(0, 0, -1)),
            'Torque expected in the direction of the -Z axis');
  else
    test.ok(isNearZero(this.joint.torque.length()),
            'Torque expected to be zero since the target relative orientation is zero');

  /* for the absolute joint the torque will try to make a composed rotation */
  this.joint.targetQ.set(Math.sin(Math.PI/8), 0, 0, Math.cos(Math.PI/8));
  this.joint.targetQ.normalize();
  this.joint.computeTorque(charFrame);
  /* targetQ has been updated with the character frame when computing the torque */
  var dir = new THREE.Vector3();
  dir.fromArray(this.joint.targetQ.clone().multiply(charFrame.clone().conjugate()).toArray());
  if(this.joint.absAngle)
    test.ok(dirEqual(this.joint.torque, dir),
            'Torque expected to be aligned with ' + dir.toArray());
  else
    test.ok(dirEqual(this.joint.torque, new THREE.Vector3(1, 0, 0)),
            'Torque expected to be in the direction of the X axis');

  test.done();
};


/* merges the common test in the relative and absolute groups */
for(var test in common) {
  exports.relative[test] = common[test];
  exports.absolute[test] = common[test];
}