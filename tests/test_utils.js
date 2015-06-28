var THREE = require('three');

module.exports = {
  isNearZero: function(x) {
    var e = 0.0001;
    return -e < x && x < e;
  },

  dirEqual: function(v, dir) {
    return v.angleTo(dir) == 0;
  },

  dirNearEqual: function(v, dir) {
    return this.isNearZero(v.angleTo(dir));
  },

  quatNearEqual: function(q1, q2) {
    var v1 = new THREE.Vector3(q1.x, q1.y, q1.z);
    var v2 = new THREE.Vector3(q2.x, q2.y, q2.z);
    v1.multiplyScalar(q1.w);
    v2.multiplyScalar(q2.w);
    return this.isNearZero(q1.w - q2.w) &&
           (this.isNearZero(q1.w - 1) || this.isNearZero(q1.w + 1) || this.dirNearEqual(v1, v2));
  
  }
};