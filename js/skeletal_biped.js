var SkeletalBipedLeg = {
  class: Bone, mass: 5, size: [0.2, 0.2, 1.3],
  children: [
    {
      jointType: SkeletalFigure.SkeletalJoint.types.HINGE,
      snapPointParent: [0, 0, -1],
      snapPointChild: [0, 0, 1],
      child: {
        class: Bone, mass: 4, size: [0.2, 0.2, 1.3],
        children: [
          {
            jointType: SkeletalFigure.SkeletalJoint.types.HINGE,
            snapPointParent: [0, 0, -1],
            snapPointChild: [0, 0, 1],
            child: {
              class: Bone, mass: 1, size: [0.3, 0.1, 0.6]
            }
          }
        ]
      }
    }
  ]
};

var SkeletalBiped = {
  class: Bone, mass: 70, size: [0.45, 0.5, 2],
  children: [
    {
      jointType: SkeletalFigure.SkeletalJoint.types.BALLSOCKET,
      snapPointParent: [1, 0, -1],
      snapPointChild: [0, 0, 1],
      child: SkeletalBipedLeg
    },
    {
      jointType: SkeletalFigure.SkeletalJoint.types.BALLSOCKET,
      snapPointParent: [-1, 0, -1],
      snapPointChild: [0, 0, 1],
      child: SkeletalBipedLeg
    }
  ]
};