var model = require('../src/model/model.js');
var SkeletalFigure = require('../src/skeletal_figure.js')

exports.SkeletalBipedLeg = {
  model: model.Bone, mass: 5, size: [0.2, 0.2, 1.3],
  children: [
    {
      model: model.HingeJoint,
      modelParams: [1, 0],
      snapPointParent: [0, 0, -1],
      snapPointChild: [0, 0, 1],
      child: {
        model: model.Bone, mass: 4, size: [0.2, 0.2, 1.3],
        children: [
          {
            model: model.BallSocketJoint,
            modelParams: [[-Math.PI, -0.1, -Math.PI/4], [0, 0.1, Math.PI/4]],
            snapPointParent: [0, 0, -1],
            snapPointChild: [0, 0, 1],
            child: {
              model: model.Bone, mass: 1, size: [0.3, 0.1, 0.6]
            }
          }
        ]
      }
    }
  ]
};

exports.SkeletalBiped = {
  model: model.Bone, mass: 70, size: [0.45, 0.5, 2],
  children: [
    {
      model: model.BallSocketJoint,
      modelParams: [[-Math.PI/2, -Math.PI/4, -Math.PI/4], [Math.PI/2, Math.PI/4, Math.PI/4]],
      snapPointParent: [1, 0, -1],
      snapPointChild: [0, 0, 1],
      child: exports.SkeletalBipedLeg
    },
    {
      model: model.BallSocketJoint,
      modelParams: [[-Math.PI/2, -Math.PI/4, -Math.PI/4], [Math.PI/2, Math.PI/4, Math.PI/4]],
      snapPointParent: [-1, 0, -1],
      snapPointChild: [0, 0, 1],
      child: exports.SkeletalBipedLeg
    }
  ]
};