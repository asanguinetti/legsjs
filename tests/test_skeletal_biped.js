var bodies = require('../js/bodies.js');
var SkeletalFigure = require('../js/skeletal_figure.js')

exports.SkeletalBipedLeg = {
  class: bodies.Bone, mass: 5, size: [0.2, 0.2, 1.3],
  children: [
    {
      jointType: SkeletalFigure.SkeletalJoint.types.HINGE,
      snapPointParent: [0, 0, -1],
      snapPointChild: [0, 0, 1],
      child: {
        class: bodies.Bone, mass: 4, size: [0.2, 0.2, 1.3],
        children: [
          {
            jointType: SkeletalFigure.SkeletalJoint.types.HINGE,
            snapPointParent: [0, 0, -1],
            snapPointChild: [0, 0, 1],
            child: {
              class: bodies.Bone, mass: 1, size: [0.3, 0.1, 0.6]
            }
          }
        ]
      }
    }
  ]
};

exports.SkeletalBiped = {
  class: bodies.Bone, mass: 70, size: [0.45, 0.5, 2],
  children: [
    {
      jointType: SkeletalFigure.SkeletalJoint.types.BALLSOCKET,
      snapPointParent: [1, 0, -1],
      snapPointChild: [0, 0, 1],
      child: exports.SkeletalBipedLeg
    },
    {
      jointType: SkeletalFigure.SkeletalJoint.types.BALLSOCKET,
      snapPointParent: [-1, 0, -1],
      snapPointChild: [0, 0, 1],
      child: exports.SkeletalBipedLeg
    }
  ]
};