var SkeletalQuadrupedRearLeg = {
  class: Bone, mass: 5, size: [0.2, 0.2, 0.8],
  children: [
    {
      jointType: SkeletalFigure.SkeletalJoint.types.HINGE,
      snapPointParent: [0, 0, -1],
      snapPointChild: [0, 0, 1],
      child: {
        class: Bone, mass: 4, size: [0.2, 0.2, 0.6],
        children: [
          {
            jointType: SkeletalFigure.SkeletalJoint.types.HINGE,
            snapPointParent: [0, 0, -1],
            snapPointChild: [0, 0, 1],
            child: {
              class: Bone, mass: 4, size: [0.2, 0.2, 0.6],
              children: [
                {
                  jointType: SkeletalFigure.SkeletalJoint.types.HINGE,
                  snapPointParent: [0, 0, -1],
                  snapPointChild: [0, 0, 1],
                  child: {
                    class: Bone, mass: 1, size: [0.3, 0.1, 0.3]
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
  class: Bone, mass: 5, size: [0.2, 0.2, 0.8],
  children: [
    {
      jointType: SkeletalFigure.SkeletalJoint.types.HINGE,
      snapPointParent: [0, 0, -1],
      snapPointChild: [0, 0, 1],
      child: {
        class: Bone, mass: 4, size: [0.2, 0.2, 0.6],
        children: [
          {
            jointType: SkeletalFigure.SkeletalJoint.types.HINGE,
            snapPointParent: [0, 0, -1],
            snapPointChild: [0, 0, 1],
            child: {
              class: Bone, mass: 4, size: [0.2, 0.2, 0.6],
              children: [
                {
                  jointType: SkeletalFigure.SkeletalJoint.types.HINGE,
                  snapPointParent: [0, 0, -1],
                  snapPointChild: [0, 0, 1],
                  child: {
                    class: Bone, mass: 1, size: [0.3, 0.1, 0.3]
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
  class: Bone, mass: 20, size: [0.45, 0.5, 1],
  children: [
    {
      jointType: SkeletalFigure.SkeletalJoint.types.BALLSOCKET,
      snapPointParent: [1, 0, 0],
      snapPointChild: [0, 0, 1],
      child: SkeletalQuadrupedRearLeg
    },
    {
      jointType: SkeletalFigure.SkeletalJoint.types.BALLSOCKET,
      snapPointParent: [-1, 0, 0],
      snapPointChild: [0, 0, 1],
      child: SkeletalQuadrupedRearLeg
    }
  ]
};

var SkeletalQuadrupedFrontLegFrame = {
  class: Bone, mass: 20, size: [0.45, 0.5, 1],
  children: [
    {
      jointType: SkeletalFigure.SkeletalJoint.types.BALLSOCKET,
      snapPointParent: [1, 0, 0],
      snapPointChild: [0, 0, 1],
      child: SkeletalQuadrupedFrontLeg
    },
    {
      jointType: SkeletalFigure.SkeletalJoint.types.BALLSOCKET,
      snapPointParent: [-1, 0, 0],
      snapPointChild: [0, 0, 1],
      child: SkeletalQuadrupedFrontLeg
    }
  ]
};

var SkeletalQuadruped = {
  class: Bone, mass: 30, size: [0.45, 2, 1],
  children: [
    {
      jointType: SkeletalFigure.SkeletalJoint.types.BALLSOCKET,
      snapPointParent: [0, 1, 0],
      snapPointChild: [0, -1, 0],
      child: SkeletalQuadrupedFrontLegFrame
    },
    {
      jointType: SkeletalFigure.SkeletalJoint.types.BALLSOCKET,
      snapPointParent: [0, -1, 0],
      snapPointChild: [0, 1, 0],
      child: SkeletalQuadrupedRearLegFrame
    }
  ]
};