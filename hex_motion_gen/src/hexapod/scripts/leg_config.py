import json

ROBOTHEIGHT = 0.05

robot_leg_configs = {

    'leg0':{
        'EE':'footc_rf',
        'EE_id': 5,
        'joints':['j_c1_rf','j_thigh_rf','j_tibia_rf'],
        'joint_ids':[2,3,4],
        'leg_pose':[0.10,  0.155, -ROBOTHEIGHT]
    },
    'leg1':{
        'EE':'footc_rm',
        'EE_id':10,
        'joints':['j_c1_rm','j_thigh_rm','j_tibia_rm'],
        'joint_ids':[7,8,9],
        'leg_pose':[0.163,  0.0, -ROBOTHEIGHT]
    },
    'leg2':{
        'EE':'footc_rr',
        'EE_id':15,
        'joints':['j_c1_rr','j_thigh_rr','j_tibia_rr'],
        'joint_ids':[12,13,14],
        'leg_pose':[0.10 , -0.155, -ROBOTHEIGHT]
    },
    'leg3':{
        'EE':'footc_lf',
        'EE_id': 20,
        'joints':['j_c1_lf','j_thigh_lf','j_tibia_lf'],
        'joint_ids':[17,18,19],
        'leg_pose':[-0.10,  0.155, -ROBOTHEIGHT]
    },
    'leg4':{
        'EE':'footc_lm',
        'EE_id':25,       
        'joints':['j_c1_lm','j_thigh_lm','j_tibia_lm'],
        'joint_ids':[22,23,24],
        'leg_pose':[-0.163,  0.0, -ROBOTHEIGHT]
    },
    'leg5':{
        'EE':'footc_lr',
        'EE_id':30,
        'joints':['j_c1_lr','j_thigh_lr','j_tibia_lr'],
        'joint_ids':[27,28,29],
        'leg_pose':[-0.10, -0.155, -ROBOTHEIGHT]
    },
    'full_body':{
        'DoF_list':[2,3,4,
                    7,8,9,
                    12,13,14,
                    17,18,19,
                    22,23,24,
                    27,28,29]
    }

}

with open("/home/jichen/paper11_ws/src/hexapod/scripts/leg_config.json", "w") as outfile:
    json.dump(robot_leg_configs, outfile)