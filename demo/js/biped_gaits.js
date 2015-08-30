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
    [0.5, -1.1, 0.6+Math.PI/2], [0, -0.05, Math.PI/2]
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

  this.stateStanceLeg = [
    0, 0, 1, 1
  ];
};

extend(BipedWalkingGait, Legs.Control.Gait);

BipedWalkingGait.prototype.setContactForLeg = function(i, contact) {
  if((this.state == 1 && i == 1) || (this.state == 3 && i == 0)) {
    if(contact) {
      this.state += 1;
      this.state %= 4;
      this.stateTime = 0;
    }
  }
};

BipedWalkingGait.prototype.update = function(timeStep) {
  this.stateTime += timeStep;
  if(this.state == 0 || this.state == 2) {
    if(this.stateTime > 0.3) {
      this.state += 1;
      this.state %= 4;
      this.stateTime = 0;
    }
  }
};
