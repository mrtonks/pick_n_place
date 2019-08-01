import move_arm


limb = 'right'
goal = {
    'x': 0.38459808171 ,
    'y': -0.542247604291,
    'z': -0.20678082182
}

move_arm.initplannode(goal, limb)