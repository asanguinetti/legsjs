var Legs = require('legs');

var SkeletalQuadrupedRearLeg = {
  model: Legs.Model.Bone, mass: 5, size: [0.2, 0.2, 0.8],
  children: [
    {
      model: Legs.Model.HingeJoint,
      modelParams: [1, 0],
      snapPointParent: [0, 0, -1],
      snapPointChild: [0, 0, 1],
      child: {
        model: Legs.Model.Bone, mass: 4, size: [0.2, 0.2, 0.6],
        children: [
          {
            model: Legs.Model.HingeJoint,
            modelParams: [1, 0],
            snapPointParent: [0, 0, -1],
            snapPointChild: [0, 0, 1],
            child: {
              model: Legs.Model.Bone, mass: 4, size: [0.2, 0.2, 0.6],
              children: [
                {
                  model: Legs.Model.BallSocketJoint,
                  modelParams: [[-Math.PI, -0.1, -Math.PI/4], [0, 0.1, Math.PI/4]],
                  snapPointParent: [0, 0, -1],
                  snapPointChild: [0, 0, 0.5],
                  child: {
                    model: Legs.Model.Bone, mass: 1, size: [0.3, 0.1, 0.4]
                  }
                }
              ]
            }
          }
        ]
      }
    }
  ]
};

var SkeletalQuadrupedFrontLeg = {
  model: Legs.Model.Bone, mass: 5, size: [0.2, 0.2, 0.8],
  children: [
    {
      model: Legs.Model.HingeJoint,
      modelParams: [1, 0],
      snapPointParent: [0, 0, -1],
      snapPointChild: [0, 0, 1],
      child: {
        model: Legs.Model.Bone, mass: 4, size: [0.2, 0.2, 0.6],
        children: [
          {
            model: Legs.Model.HingeJoint,
            modelParams: [1, 0],
            snapPointParent: [0, 0, -1],
            snapPointChild: [0, 0, 1],
            child: {
              model: Legs.Model.Bone, mass: 4, size: [0.2, 0.2, 0.6],
              children: [
                {
                  model: Legs.Model.BallSocketJoint,
                  modelParams: [[-Math.PI, -0.1, -Math.PI/4], [0, 0.1, Math.PI/4]],
                  snapPointParent: [0, 0, -1],
                  snapPointChild: [0, 0, 0.5],
                  child: {
                    model: Legs.Model.Bone, mass: 1, size: [0.3, 0.1, 0.4]
                  }
                }
              ]
            }
          }
        ]
      }
    }
  ]
};

var SkeletalQuadrupedRearLegFrame = {
  model: Legs.Model.Bone, mass: 20, size: [0.45, 0.5, 1],
  children: [
    {
      model: Legs.Model.BallSocketJoint,
      modelParams: [[-Math.PI/2, -Math.PI/4, -0.2], [Math.PI/2, Math.PI/4, 0.2]],
      snapPointParent: [1, 0, 0],
      snapPointChild: [0, 0, 1],
      child: SkeletalQuadrupedRearLeg
    },
    {
      model: Legs.Model.BallSocketJoint,
      modelParams: [[-Math.PI/2, -Math.PI/4, -0.2], [Math.PI/2, Math.PI/4, 0.2]],
      snapPointParent: [-1, 0, 0],
      snapPointChild: [0, 0, 1],
      child: SkeletalQuadrupedRearLeg
    }
  ]
};

var SkeletalQuadrupedFrontLegFrame = {
  model: Legs.Model.Bone, mass: 20, size: [0.45, 0.5, 1],
  children: [
    {
      model: Legs.Model.BallSocketJoint,
      modelParams: [[-Math.PI/2, -Math.PI/4, -0.2], [Math.PI/2, Math.PI/4, 0.2]],
      snapPointParent: [1, 0, 0],
      snapPointChild: [0, 0, 1],
      child: SkeletalQuadrupedFrontLeg
    },
    {
      model: Legs.Model.BallSocketJoint,
      modelParams: [[-Math.PI/2, -Math.PI/4, -0.2], [Math.PI/2, Math.PI/4, 0.2]],
      snapPointParent: [-1, 0, 0],
      snapPointChild: [0, 0, 1],
      child: SkeletalQuadrupedFrontLeg
    }
  ]
};

var SkeletalQuadruped = {
  model: Legs.Model.Bone, mass: 30, size: [0.45, 2, 1],
  children: [
    {
      model: Legs.Model.BallSocketJoint,
      modelParams: [[-0.1, -0.1, -0.1], [0.1, 0.1, 0.1]],
      snapPointParent: [0, 1, 0],
      snapPointChild: [0, -1, 0],
      child: SkeletalQuadrupedFrontLegFrame
    },
    {
      model: Legs.Model.BallSocketJoint,
      modelParams: [[-0.1, -0.1, -0.1], [0.1, 0.1, 0.1]],
      snapPointParent: [0, -1, 0],
      snapPointChild: [0, 1, 0],
      child: SkeletalQuadrupedRearLegFrame
    }
  ]
};