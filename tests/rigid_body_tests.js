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


var dirNearEqual = function(v, dir) {
  return isNearZero(v.angleTo(dir));
};


var quatNearEqual = function(q1, q2) {
  var v1 = new THREE.Vector3(q1.x, q1.y, q1.z);
  var v2 = new THREE.Vector3(q2.x, q2.y, q2.z);
  v1.multiplyScalar(q1.w);
  v2.multiplyScalar(q2.w);
  return isNearZero(q1.w - q2.w) &&
         (isNearZero(q1.w - 1) || isNearZero(q1.w + 1) || dirNearEqual(v1, v2));
}


exports.setUp = function(callback)
{
  var scene = new THREE.Scene();
  var collisionConfiguration = new Ammo.btDefaultCollisionConfiguration();
  var dispatcher = new Ammo.btCollisionDispatcher(collisionConfiguration);
  var overlappingPairCache = new Ammo.btDbvtBroadphase();
  var solver = new Ammo.btSequentialImpulseConstraintSolver();
  scene.world = new Ammo.btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
  scene.world.setGravity(new Ammo.btVector3(0,0,-10));
  scene.world.bodies = [];

  this.rb = new bodies.Bone(0, new THREE.Vector3(0.5, 0.5, 0.5));

  this.rb.buildAndInsert(scene);

  callback();
};


exports.getHeadingTest = function(test)
{
  var expected = new THREE.Quaternion();
  var expectedSwing = new THREE.Quaternion();
  var swing = new THREE.Quaternion();

  /* x = 0, y = 0, z = 0 */
  var heading = this.rb.getHeading();
  test.ok(heading.equals(expected),
          'The initial heading is expected to be the identity');

  /* x = 0, y = 0, z = PI/2 */
  this.rb.rotateAxis(new THREE.Vector3(0, 0, 1), Math.PI/2);
  expected.setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI/2);
  heading = this.rb.getHeading();
  test.ok(quatNearEqual(heading, expected),
          'Expected heading to be PI/2 degrees along the Z axis');

  /* x = PI/2, y = 0, z = 0 */
  this.rb.rotateAxis(new THREE.Vector3(0, 0, 1), -Math.PI/2);
  this.rb.rotateAxis(new THREE.Vector3(1, 0, 0), Math.PI/2);
  expected.set(0, 0, 0, 1);
  heading = this.rb.getHeading();
  test.ok(quatNearEqual(heading, expected),
          'Expected heading to be the identity');

  expectedSwing.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI/2);

  swing.copy(heading);
  swing.conjugate();
  swing.multiply(this.rb.getOrientation());
  test.ok(quatNearEqual(swing, expectedSwing),
          'Expected swing to be PI/2 degrees along the X axis');

  /* x = PI/2, y = 0, z = PI/2 */
  this.rb.rotateAxis(new THREE.Vector3(0, 0, 1), Math.PI/2);
  expected.setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI/2);
  heading = this.rb.getHeading();
  test.ok(quatNearEqual(heading, expected),
          'Expected heading to be PI/2 degrees along the Z axis');

  swing.copy(heading);
  swing.conjugate();
  swing.multiply(this.rb.getOrientation());
  test.ok(quatNearEqual(swing, expectedSwing),
          'Expected swing to remain the same');

  /* x = PI/2, y = 0, z = PI */
  this.rb.rotateAxis(new THREE.Vector3(0, 0, 1), Math.PI/2);
  expected.setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI);
  heading = this.rb.getHeading();
  test.ok(quatNearEqual(heading, expected),
          'Expected heading to be PI degrees along the Z axis');

  swing.copy(heading);
  swing.conjugate();
  swing.multiply(this.rb.getOrientation());
  test.ok(quatNearEqual(swing, expectedSwing),
          'Expected swing to remain the same');

  /* x = PI, y = PI/2, z = 0 */
  this.rb.rotateAxis(new THREE.Vector3(0, 0, 1), -Math.PI);
  this.rb.rotateAxis(new THREE.Vector3(1, 0, 0), Math.PI/2);
  this.rb.rotateAxis(new THREE.Vector3(0, 1, 0), Math.PI/2);
  heading = this.rb.getHeading();
  test.ok(isNearZero(heading.x) && isNearZero(heading.y),
          'Expected heading to be something along the Z axis only');

  swing.copy(heading);
  swing.conjugate();
  swing.multiply(this.rb.getOrientation());
  test.ok(isNearZero(swing.z),
          'Expected swing to be on the XY plane only');

  expectedSwing.copy(swing);

  /* x = PI, y = PI/2, z = PI/2 */
  this.rb.rotateAxis(new THREE.Vector3(0, 0, 1), Math.PI/2);
  heading = this.rb.getHeading();
  test.ok(isNearZero(heading.x) && isNearZero(heading.y),
          'Expected heading to be something along the Z axis only');

  swing.copy(heading);
  swing.conjugate();
  swing.multiply(this.rb.getOrientation());
  test.ok(isNearZero(swing.z),
          'Expected swing to be on the XY plane only');
  test.ok(quatNearEqual(swing, expectedSwing),
          'Expected swing to remain the same');

  var euler = new THREE.Euler();
  euler.setFromQuaternion(this.rb.getOrientation(), 'XYZ');
  expected.setFromAxisAngle(new THREE.Vector3(0, 0, 1), euler.z);
  heading = this.rb.getHeading();
  test.ok(quatNearEqual(heading, expected),
          'Using Euler angles decomposition should give the same result');

  test.done();
};