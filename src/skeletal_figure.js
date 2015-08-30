Function.prototype.construct = function(argArray) {
  var constr = this;
  var inst = Object.create(constr.prototype);
  constr.apply(inst, argArray);
  return inst;
};

var SkeletalJoint = function(model, modelParams, parentSegment, childSegment,
                             snapPointParent, snapPointChild) {
  this.model = model.construct([parentSegment.body, childSegment.body,
                              (new THREE.Vector3()).fromArray(snapPointParent),
                              (new THREE.Vector3()).fromArray(snapPointChild)].concat(modelParams));
  this.parentSegment = parentSegment;
  this.childSegment = childSegment;
  this.snapPointParent = snapPointParent;
  this.snapPointChild = snapPointChild;
};

var SkeletalSegment = function(body, parent) {
  this.body = body;
  this.parent = parent;
  this.childrenJoints = [];
};

SkeletalSegment.prototype.snap = function(jointModel, jointParams, childSegment,
                                          snapPointParent, snapPointChild) {
  childSegment.parent = this;
  var newJoint = new SkeletalJoint(jointModel, jointParams, this, childSegment,
                                   snapPointParent, snapPointChild);
  this.childrenJoints.push(newJoint);
  return this;
};

SkeletalSegment.prototype.buildAndInsert = function(scene)
{
  this.body.buildAndInsert(scene);
  for (var i = 0; i < this.childrenJoints.length; i++) {
    var skJoint = this.childrenJoints[i];
    skJoint.childSegment.body.snapTo((new THREE.Vector3()).fromArray(skJoint.snapPointChild),
                                     skJoint.parentSegment.body,
                                     (new THREE.Vector3()).fromArray(skJoint.snapPointParent));
    skJoint.childSegment.buildAndInsert(scene);
    skJoint.model.buildAndInsert(scene);
  };
};

SkeletalSegment.fromJSON = function(json) {
  var scaleVec3 = function(v, s) {
    return [v[0] * s[0], v[1] * s[1], v[2] * s[2]]
  }

  var body = json.model.construct([json.mass,
                                  (new THREE.Vector3()).fromArray(json.size)].concat(json.modelParams));
  var segment = new SkeletalSegment(body);
  if(json.children !== undefined) {
    for (var i = 0; i < json.children.length; i++) {
      segment.snap(json.children[i].model,
                   json.children[i].modelParams,
                   SkeletalSegment.fromJSON(json.children[i].child),
                   scaleVec3(json.children[i].snapPointParent, json.size),
                   scaleVec3(json.children[i].snapPointChild, json.children[i].child.size));
    };
  }
  return segment;
};

module.exports.SkeletalJoint = SkeletalJoint;
module.exports.SkeletalSegment = SkeletalSegment;