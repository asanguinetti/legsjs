var assert = require('assert')
var THREE = require('three');
var bodies = require('../js/bodies.js');
var testUtils = require('./test_utils.js');


var extend = function(extended, base) {
  extended.prototype = Object.create(base.prototype);
  extended.prototype.base = base;
  extended.prototype.constructor = extended;
};

exports.setUp = function(callback)
{
  var DummyGait = function(phase) {
    this.base.call(this, phase);
  };

  extend(DummyGait, bodies.Gait);

  DummyGait.prototype.update = function(timeStep) {
    this.targeFootPos[0] = 0;
    this.targeFootPos[1] = -7.0;
    this.targeFootPos[2] = 0;
  };

  var TestGait = function() {
  };

  TestGait.prototype.setContactForLeg = function(i, contact) {
  };

  TestGait.prototype.update = function(timeStep) {
  };

  TestGait.prototype.isStanceLeg = function(i) {
    return i == 0;
  };

  TestGait.prototype.getAnglesForLeg = function(i) {
    return [0, 0, 0];
  };

  var scene = new THREE.Scene();
  var collisionConfiguration = new Ammo.btDefaultCollisionConfiguration();
  var dispatcher = new Ammo.btCollisionDispatcher(collisionConfiguration);
  var overlappingPairCache = new Ammo.btDbvtBroadphase();
  var solver = new Ammo.btSequentialImpulseConstraintSolver();
  scene.world = new Ammo.btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
  scene.world.setGravity(new Ammo.btVector3(0,0,-10));
  scene.world.bodies = [];

  /* trunk */
  var mass = 1;
  var trunk = new bodies.Bone(mass*70, new THREE.Vector3(1, 1, 2));
  trunk.translate(0, 0, 6.4 + trunk.size.z + 0.5);
  trunk.buildAndInsert(scene);

  var pdGains = {
    legFrame: [6000, 1200],
    joints: [
      [1500, 300],
      [1500, 300],
      [1500, 300]
    ],
  };

  var fbGains = [
    [0.10, 0.10],
    [0, 0],
    [0, 0]
  ];

  this.stanceLeg = new bodies.RearLeg(trunk, 
                                      new THREE.Vector3(trunk.size.x, 0, -trunk.size.z),
                                      [new bodies.Bone(mass*5, new THREE.Vector3(0.2, 0.2, 1.3)),
                                       new bodies.Bone(mass*4, new THREE.Vector3(0.2, 0.2, 1.3)),
                                       new bodies.Bone(mass, new THREE.Vector3(0.3, 0.1, 0.6))],
                                      new DummyGait(0.5),
                                      pdGains);

  this.swingLeg = new bodies.RearLeg(trunk, 
                                     new THREE.Vector3(-trunk.size.x, 0, -trunk.size.z),
                                     [new bodies.Bone(mass*5, new THREE.Vector3(0.2, 0.2, 1.3)),
                                      new bodies.Bone(mass*4, new THREE.Vector3(0.2, 0.2, 1.3)),
                                      new bodies.Bone(mass, new THREE.Vector3(0.3, 0.1, 0.6))],
                                     new DummyGait(0.0),
                                     pdGains);

  var legs = [this.stanceLeg, this.swingLeg];

  for(var i = 0; i < legs.length; i++) {
    legs[i].buildAndInsert(scene);
  };

  this.legFrame = new bodies.LegFrame(new TestGait(), trunk, legs, pdGains, fbGains);

  callback();
};

exports.basicTest = function(test)
{
  this.legFrame.computeFeedbackAngles();
  test.ok(testUtils.isNearZero(this.legFrame.fbD.length()),
          'The derivative feedback is expected to be zero since the CM is not moving');

  test.ok(testUtils.dirEqual(this.legFrame.fbP, new THREE.Vector3(-1, 0, 0)),
          'The proportional feedback is expected to point in the direction of the -X axis');

  test.done();
};

exports.rotatedFrameTest = function(test)
{
  rotations = [
    Math.PI/2,
    Math.PI/4 - Math.PI/2,
    Math.PI - Math.PI/4,
    -Math.PI/2
  ];

  for(var k = 0; k < rotations.length; k++)
  {
    this.legFrame.trunk.rotateAxis(new THREE.Vector3(0, 0, 1), rotations[ k ]);
    for(var i = 0; i < this.legFrame.legs.length; i++)
      for(var j = 0; j < this.legFrame.legs[ i ].segments.length; j++)
        this.legFrame.legs[ i ].segments[ j ].rotateAxis(new THREE.Vector3(0, 0, 1), rotations[ k ]);

    /* the expected feedback angles should be roughly the same for any orientation of the frame */
    this.legFrame.computeFeedbackAngles();
    test.ok(testUtils.isNearZero(this.legFrame.fbD.length()),
            'The derivative feedback is expected to be zero since the CM is not moving');

    test.ok(testUtils.dirNearEqual(this.legFrame.fbP, new THREE.Vector3(-1, 0, 0)),
            'The proportional feedback is expected to point in the direction of the -X axis');
  }

  test.done();
};

exports.rotatedSwingLegTest = function(test)
{
  /* the expected feedback angles should be roughly the same for any orientation of the swing leg */
  var zero = new THREE.Vector3(0, 0, 0);

  this.swingLeg.segments[0].rotateAxis(new THREE.Vector3(1, 0, 0), Math.PI/4, zero);
  for(var i = 1; i < this.swingLeg.segments.length; i++)
  {
    this.swingLeg.segments[i].rotateAxis(new THREE.Vector3(1, 0, 0), Math.PI/4, zero);
    this.swingLeg.segments[i].snapTo(new THREE.Vector3(0, 0, this.swingLeg.segments[i].size.z),
                                     this.swingLeg.segments[i-1],
                                     new THREE.Vector3(0, 0, -this.swingLeg.segments[i-1].size.z));
  }

  this.legFrame.computeFeedbackAngles();
  test.ok(testUtils.isNearZero(this.legFrame.fbD.length()),
          'The derivative feedback is expected to be zero since the CM is not moving');

  test.ok(testUtils.dirEqual(this.legFrame.fbP, new THREE.Vector3(-1, 0, 0)),
          'The proportional feedback is expected to point in the direction of the -X axis');

  this.swingLeg.segments[0].rotateAxis(new THREE.Vector3(0, 1, 0), Math.PI/4, zero);
  for(var i = 1; i < this.swingLeg.segments.length; i++)
  {
    this.swingLeg.segments[i].rotateAxis(new THREE.Vector3(0, 1, 0), Math.PI/4, zero);
    this.swingLeg.segments[i].snapTo(new THREE.Vector3(0, 0, this.swingLeg.segments[i].size.z),
                                     this.swingLeg.segments[i-1],
                                     new THREE.Vector3(0, 0, -this.swingLeg.segments[i-1].size.z));
  }

  this.legFrame.computeFeedbackAngles();
  test.ok(testUtils.isNearZero(this.legFrame.fbD.length()),
          'The derivative feedback is expected to be zero since the CM is not moving');

  test.ok(testUtils.dirEqual(this.legFrame.fbP, new THREE.Vector3(-1, 0, 0)),
          'The proportional feedback is expected to point in the direction of the -X axis');

  test.done();
}