var extend = function(extended, base) {
  extended.prototype = Object.create(base.prototype);
  extended.prototype.base = base;
  extended.prototype.constructor = extended;
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

  this.stateSwingLeg = [
    1, 1, 0, 0
  ];

  this.inContact = [
    false, false
  ];

  var self = this;

  var timedTransition = function() {
    return 0.3 < self.stateTime;
  }

  var contactTransition = function() {
    return self.inContact[self.stateSwingLeg[self.state]];
  }

  this.transitions = [
    timedTransition,
    contactTransition,
    timedTransition,
    contactTransition
  ];
};

extend(QuadrupedWalkingGait, Legs.Control.Gait);

var QuadrupedWalkingGaitRear = function() {
  this.base.call(this);
  /* swh, swk, swa, swt */
  this.liftTargets[0] = [0.5, -1.6, 1.1, Math.PI/2];
  /* sth, stk, sta, stt */
  this.liftTargets[1] = [0.1, -0.65, 0.7, Math.PI/2 - 0.5];
  /* swh, swk, swa, swt */
  this.strikeTargets[0] = [0.1, -0.65, 0.7, Math.PI/2 - 0.5];
  /* sth, stk, sta, stt */
  this.strikeTargets[1] = [0.1, -0.65, 0.7, Math.PI/2 - 0.5];
};

extend(QuadrupedWalkingGaitRear, QuadrupedWalkingGait);

var QuadrupedWalkingGaitFront = function() {
  this.base.call(this);
  /* swh, swk, swa, swt */
  this.liftTargets[0] = [-0.45, 1.6, -1.8, Math.PI/2];
  /* sth, stk, sta, stt */
  this.liftTargets[1] = [-0.18, 0.3, -0.3, Math.PI/2 - 0.2];
  /* swh, swk, swa, swt */
  this.strikeTargets[0] = [-0.18, 0.3, -0.3, Math.PI/2 - 0.2];
  /* sth, stk, sta, stt */
  this.strikeTargets[1] = [-0.18, 0.3, -0.3, Math.PI/2 - 0.2];

  this.stateSwingLeg = [
    0, 0, 1, 1
  ];
};

extend(QuadrupedWalkingGaitFront, QuadrupedWalkingGait);