var extend = function(extended, base) {
  extended.prototype = Object.create(base.prototype);
  extended.prototype.base = base;
  extended.prototype.constructor = extended;
};

var BipedWalkingGait = function() {
  this.state = 0;
  this.stateTime = 0;

  /* swh, swk, swa, sth, stk, sta */
  var liftTargets = [
    [0.5, -1.2, 0.6+Math.PI/2], [0, -0.05, Math.PI/2]
  ];
  var strikeTargets = [
    [-0.1, -0.05, 0.15+Math.PI/2], [0, -0.1, Math.PI/2]
  ];

  this.stateTargets = [
    liftTargets,
    strikeTargets,
    liftTargets,
    strikeTargets
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

extend(BipedWalkingGait, Legs.Control.Gait);
