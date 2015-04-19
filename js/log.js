'use strict';

var Logger = function() {
  this.time = 0;
  this.info = {};
};

Logger.prototype.log = function(msg) {
  throw 'Not implemented';
};

Logger.prototype.step = function(timeStep) {
  this.time += timeStep;
  for(var key in this.info)
    this.log('[' + this.time.toFixed(3) + '] ' + key + ': ' + this.info[key]);
};

Logger.prototype.setInfo = function(key, value) {
  this.info[key] = value;
}

var ConsoleLogger = function() {
  this.base.call(this);
};

extend(ConsoleLogger, Logger);

ConsoleLogger.prototype.log = function(msg) {
  console.log(msg);
};

var NullLogger = function() {
  this.base.call(this);
};

extend(NullLogger, Logger);

NullLogger.prototype.log = function(msg) {
  /* does nothing */
};