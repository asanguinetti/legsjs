var Legs = require('legs');

var SkeletalBipedLeg = {
  model: Legs.Model.Bone, mass: 5, size: [0.2, 0.2, 1.3],
  children: [
    {
      model: Legs.Model.HingeJoint,
      modelParams: [1, 0],
      snapPointParent: [0, 0, -1],
      snapPointChild: [0, 0, 1],
      child: {
        model: Legs.Model.Bone, mass: 4, size: [0.2, 0.2, 1.3],
        children: [
          {
            model: Legs.Model.BallSocketJoint,
            modelParams: [[-Math.PI, -0.1, -Math.PI/4], [0, 0.1, Math.PI/4]],
            snapPointParent: [0, 0, -1],
            snapPointChild: [0, 0, 1],
            child: {
              model: Legs.Model.Bone, mass: 1, size: [0.3, 0.1, 0.6]
            }
          }
        ]
      }
    }
  ]
};

var SkeletalBiped = {
  model: Legs.Model.Bone, mass: 70, size: [0.45, 0.5, 2],
  children: [
    {
      model: Legs.Model.BallSocketJoint,
      modelParams: [[-Math.PI/2, -Math.PI/4, -Math.PI/4], [Math.PI/2, Math.PI/4, Math.PI/4]],
      snapPointParent: [1, 0, -1],
      snapPointChild: [0, 0, 1],
      child: SkeletalBipedLeg
    },
    {
      model: Legs.Model.BallSocketJoint,
      modelParams: [[-Math.PI/2, -Math.PI/4, -Math.PI/4], [Math.PI/2, Math.PI/4, Math.PI/4]],
      snapPointParent: [-1, 0, -1],
      snapPointChild: [0, 0, 1],
      child: SkeletalBipedLeg
    }
  ]
};