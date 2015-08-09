(function(exports) {
  exports.SkeletalJoint = function(type, parentSegment, childSegment,
                                   snapPointParent, snapPointChild) {
    this.type = type;
    this.parentSegment = parentSegment;
    this.childSegment = childSegment;
    this.snapPointParent = snapPointParent;
    this.snapPointChild = snapPointChild;
  },

  exports.SkeletalJoint.types = {
    BALLSOCKET: 0,
    HINGE: 1
  },

  exports.SkeletalSegment = function(body, parent) {
    this.body = body;
    this.parent = parent;
    this.childrenJoints = [];
  };

  exports.SkeletalSegment.prototype.snap = function(jointType, childSegment,
                                                    snapPointParent, snapPointChild) {
    childSegment.parent = this;
    var newJoint = new exports.SkeletalJoint(jointType, this, childSegment,
                                             snapPointParent, snapPointChild);
    this.childrenJoints.push(newJoint);
    return this;
  };

  exports.SkeletalSegment.fromJSON = function(json) {
    var scaleVec3 = function(v, s) {
      return [v[0] * s[0], v[1] * s[1], v[2] * s[2]]
    }

    var body = new json.class(json.mass,
                              new THREE.Vector3(json.size[0],
                                                json.size[1],
                                                json.size[2]));
    var segment = new exports.SkeletalSegment(body);
    if(json.children !== undefined) {
      for (var i = 0; i < json.children.length; i++) {
        segment.snap(json.children[i].jointType,
                     exports.SkeletalSegment.fromJSON(json.children[i].child),
                     scaleVec3(json.children[i].snapPointParent, json.size),
                     scaleVec3(json.children[i].snapPointChild, json.children[i].child.size));
      };
    }
    return segment;
  } 
})(typeof exports === 'undefined' ? this['SkeletalFigure'] = {} : exports);