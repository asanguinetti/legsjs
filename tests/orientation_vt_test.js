var assert = require('assert')
var THREE = require('../src/third_party/three.js');
var control = require('../src/control/control.js');
var testUtils = require('./test_utils.js');
var SkeletalFigure = require('../src/skeletal_figure.js')
var TestSkeletalBiped = require('./test_skeletal_biped.js')

exports.setUp = function(callback)
{
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

  var skeletalFigure = SkeletalFigure.SkeletalSegment.fromJSON(TestSkeletalBiped.SkeletalBiped);

  var trunk = skeletalFigure.body;
  trunk.translate(0, 0, 6.4 + trunk.size.z + 0.5);

  var controlParams = {
    orientationVT: {
      torqueLimit: 1000,
      pdGains: [2000, -200],
      fbGains: [0, 0]
    },
    joints: [
      {
        torqueLimit: 1000,
        pdGains: [1500, 300],
        fbGains: [0.10, 0.10]
      },
      {
        torqueLimit: 1000,
        pdGains: [1500, 300],
        fbGains: [0, 0]
      },
      {
        torqueLimit: 1000,
        pdGains: [1500, 300],
        fbGains: [0, 0]
      }
    ]
  }

  this.legFrame = new control.LegFrame(new TestGait(),
                                       skeletalFigure,
                                       controlParams);
  skeletalFigure.buildAndInsert(scene);

  this.swingLeg = this.legFrame.legs[1];

  callback();
};

exports.basicOrientationTest = function(test)
{
  this.legFrame.computeOrientationVT();
  test.ok(testUtils.isNearZero(this.legFrame.orientationVT.length()),
          'The orientational virtual torque is expected to be zero since the trunk is perfectly aligned');
  test.done();
};

exports.rotatedTest = function(test)
{
  rotations = [
    Math.PI/2,
    Math.PI/4 - Math.PI/2,
    Math.PI - Math.PI/4,
    -Math.PI/2 - Math.PI
  ];

  for(var k = 0; k < rotations.length; k++)
  {
    this.legFrame.trunk.rotateAxis(new THREE.Vector3(0, 0, 1), rotations[ k ]);
    for(var i = 0; i < this.legFrame.legs.length; i++)
      for(var j = 0; j < this.legFrame.legs[ i ].segments.length; j++)
        this.legFrame.legs[ i ].segments[ j ].rotateAxis(new THREE.Vector3(0, 0, 1), rotations[ k ]);

    /* the expected orientational virtual torques should be roughly the same for any orientation of the frame */
    var curHeading = this.legFrame.trunk.getHeading();
    this.legFrame.heading = Math.atan2(curHeading.z, curHeading.w)*2;
    this.legFrame.computeOrientationVT();
    test.ok(testUtils.isNearZero(this.legFrame.orientationVT.length()),
            'The orientational virtual torque is expected to be zero since the trunk is perfectly aligned');
  }

  test.done();
};

exports.inclinedXTest = function(test)
{
  this.legFrame.trunk.rotateAxis(new THREE.Vector3(1, 0, 0), Math.PI/8);

  rotations = [
    Math.PI/2,
    Math.PI/4 - Math.PI/2,
    Math.PI - Math.PI/4,
    -Math.PI/2 - Math.PI
  ];

  this.legFrame.computeOrientationVT();
  var refTq = this.legFrame.orientationVT.clone();

  for(var k = 0; k < rotations.length; k++)
  {
    this.legFrame.trunk.rotateAxis(new THREE.Vector3(0, 0, 1), rotations[ k ]);
    for(var i = 0; i < this.legFrame.legs.length; i++)
      for(var j = 0; j < this.legFrame.legs[ i ].segments.length; j++)
        this.legFrame.legs[ i ].segments[ j ].rotateAxis(new THREE.Vector3(0, 0, 1), rotations[ k ]);

    /* the expected orientational virtual torques should be roughly the same for any orientation of the frame */
    var curHeading = this.legFrame.trunk.getHeading();
    this.legFrame.heading = Math.atan2(curHeading.z, curHeading.w)*2;
    this.legFrame.computeOrientationVT();
    test.ok(testUtils.isNearZero(refTq.length() - this.legFrame.orientationVT.length()),
            'The orientational virtual torque is expected to be the same, regardless of the heading as long as the target heading matches the current');

    /* the orientationVT is in world coordinates
     * the trunk is rotated on the X axis on the trunk's reference frame 
     * as the trunk is rotated along the Z axis, the direction of orientationVT 
     * also rotates, that is why we need to convert it to local coordinates 
     * before comparing it with the reference torque */
    this.legFrame.orientationVT.applyQuaternion(this.legFrame.trunk.getOrientation().conjugate())
    test.ok(testUtils.dirNearEqual(refTq, this.legFrame.orientationVT),
            'The orientational virtual torque is expected to be the same, regardless of the heading as long as the target heading matches the current');
  }

  test.done();
};

exports.inclinedYTest = function(test)
{
  this.legFrame.trunk.rotateAxis(new THREE.Vector3(0, 1, 0), Math.PI/8);

  rotations = [
    Math.PI/2,
    Math.PI/4 - Math.PI/2,
    Math.PI - Math.PI/4,
    -Math.PI/2 - Math.PI
  ];

  this.legFrame.computeOrientationVT();
  var refTq = this.legFrame.orientationVT.clone();

  for(var k = 0; k < rotations.length; k++)
  {
    this.legFrame.trunk.rotateAxis(new THREE.Vector3(0, 0, 1), rotations[ k ]);
    for(var i = 0; i < this.legFrame.legs.length; i++)
      for(var j = 0; j < this.legFrame.legs[ i ].segments.length; j++)
        this.legFrame.legs[ i ].segments[ j ].rotateAxis(new THREE.Vector3(0, 0, 1), rotations[ k ]);

    /* the expected orientational virtual torques should be roughly the same for any orientation of the frame */
    var curHeading = this.legFrame.trunk.getHeading();
    this.legFrame.heading = Math.atan2(curHeading.z, curHeading.w)*2;
    this.legFrame.computeOrientationVT();
    test.ok(testUtils.isNearZero(refTq.length() - this.legFrame.orientationVT.length()),
            'The orientational virtual torque is expected to be the same, regardless of the heading as long as the target heading matches the current');

    /* the orientationVT is in world coordinates
     * the trunk is rotated on the Y axis on the trunk's reference frame 
     * as the trunk is rotated along the Z axis, the direction of orientationVT 
     * also rotates, that is why we need to convert it to local coordinates 
     * before comparing it with the reference torque */
    this.legFrame.orientationVT.applyQuaternion(this.legFrame.trunk.getOrientation().conjugate())
    test.ok(testUtils.dirNearEqual(refTq, this.legFrame.orientationVT),
            'The orientational virtual torque is expected to be the same, regardless of the heading as long as the target heading matches the current');
  }

  test.done();
};

exports.velocityTest = function(test)
{
  this.legFrame.trunk.body.setAngularVelocity(new Ammo.btVector3(10, 5, 2));

  rotations = [
    Math.PI/2,
    Math.PI/4 - Math.PI/2,
    Math.PI - Math.PI/4,
    -Math.PI/2 - Math.PI
  ];

  this.legFrame.computeOrientationVT();
  var refTq = this.legFrame.orientationVT.clone();

  for(var k = 0; k < rotations.length; k++)
  {
    this.legFrame.trunk.rotateAxis(new THREE.Vector3(0, 0, 1), rotations[ k ]);
    for(var i = 0; i < this.legFrame.legs.length; i++)
      for(var j = 0; j < this.legFrame.legs[ i ].segments.length; j++)
        this.legFrame.legs[ i ].segments[ j ].rotateAxis(new THREE.Vector3(0, 0, 1), rotations[ k ]);

    /* the expected orientational virtual torques should be roughly the same for any orientation of the frame */
    var curHeading = this.legFrame.trunk.getHeading();
    this.legFrame.heading = Math.atan2(curHeading.z, curHeading.w)*2;
    this.legFrame.computeOrientationVT();
    test.ok(testUtils.isNearZero(refTq.length() - this.legFrame.orientationVT.length()),
            'The orientational virtual torque is expected to be the same, regardless of the heading as long as the target heading matches the current');

    test.ok(testUtils.dirNearEqual(refTq, this.legFrame.orientationVT),
            'The orientational virtual torque is expected to be the same, regardless of the heading as long as the target heading matches the current');
  }

  test.done();
};