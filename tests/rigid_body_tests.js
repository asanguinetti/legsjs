var assert = require('assert')
var THREE = require('../src/third_party/three.js');
var model = require('../src/model/model.js');
var testUtils = require('./test_utils.js');


var common = {};


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

  this.rb = new model.Bone(0, new THREE.Vector3(0.5, 0.5, 0.5));

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
  test.ok(testUtils.quatNearEqual(heading, expected),
          'Expected heading to be PI/2 degrees along the Z axis');

  /* x = PI/2, y = 0, z = 0 */
  this.rb.rotateAxis(new THREE.Vector3(0, 0, 1), -Math.PI/2);
  this.rb.rotateAxis(new THREE.Vector3(1, 0, 0), Math.PI/2);
  expected.set(0, 0, 0, 1);
  heading = this.rb.getHeading();
  test.ok(testUtils.quatNearEqual(heading, expected),
          'Expected heading to be the identity');

  expectedSwing.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI/2);

  swing.copy(heading);
  swing.conjugate();
  swing.multiply(this.rb.getOrientation());
  test.ok(testUtils.quatNearEqual(swing, expectedSwing),
          'Expected swing to be PI/2 degrees along the X axis');

  /* x = PI/2, y = 0, z = PI/2 */
  this.rb.rotateAxis(new THREE.Vector3(0, 0, 1), Math.PI/2);
  expected.setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI/2);
  heading = this.rb.getHeading();
  test.ok(testUtils.quatNearEqual(heading, expected),
          'Expected heading to be PI/2 degrees along the Z axis');

  swing.copy(heading);
  swing.conjugate();
  swing.multiply(this.rb.getOrientation());
  test.ok(testUtils.quatNearEqual(swing, expectedSwing),
          'Expected swing to remain the same');

  /* x = PI/2, y = 0, z = PI */
  this.rb.rotateAxis(new THREE.Vector3(0, 0, 1), Math.PI/2);
  expected.setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI);
  heading = this.rb.getHeading();
  test.ok(testUtils.quatNearEqual(heading, expected),
          'Expected heading to be PI degrees along the Z axis');

  swing.copy(heading);
  swing.conjugate();
  swing.multiply(this.rb.getOrientation());
  test.ok(testUtils.quatNearEqual(swing, expectedSwing),
          'Expected swing to remain the same');

  /* x = PI, y = PI/2, z = 0 */
  this.rb.rotateAxis(new THREE.Vector3(0, 0, 1), -Math.PI);
  this.rb.rotateAxis(new THREE.Vector3(1, 0, 0), Math.PI/2);
  this.rb.rotateAxis(new THREE.Vector3(0, 1, 0), Math.PI/2);
  heading = this.rb.getHeading();
  test.ok(testUtils.isNearZero(heading.x) && testUtils.isNearZero(heading.y),
          'Expected heading to be something along the Z axis only');

  swing.copy(heading);
  swing.conjugate();
  swing.multiply(this.rb.getOrientation());
  test.ok(testUtils.isNearZero(swing.z),
          'Expected swing to be on the XY plane only');

  expectedSwing.copy(swing);

  /* x = PI, y = PI/2, z = PI/2 */
  this.rb.rotateAxis(new THREE.Vector3(0, 0, 1), Math.PI/2);
  heading = this.rb.getHeading();
  test.ok(testUtils.isNearZero(heading.x) && testUtils.isNearZero(heading.y),
          'Expected heading to be something along the Z axis only');

  swing.copy(heading);
  swing.conjugate();
  swing.multiply(this.rb.getOrientation());
  test.ok(testUtils.isNearZero(swing.z),
          'Expected swing to be on the XY plane only');
  test.ok(testUtils.quatNearEqual(swing, expectedSwing),
          'Expected swing to remain the same');

  var euler = new THREE.Euler();
  euler.setFromQuaternion(this.rb.getOrientation(), 'XYZ');
  expected.setFromAxisAngle(new THREE.Vector3(0, 0, 1), euler.z);
  heading = this.rb.getHeading();
  test.ok(testUtils.quatNearEqual(heading, expected),
          'Using Euler angles decomposition should give the same result');

  test.done();
};