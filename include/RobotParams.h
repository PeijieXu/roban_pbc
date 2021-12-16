#pragma once

#define JOINT_NUM 22

#define L_bh 0.0591
#define L_bw 0.0612

#define L_th 0.1028
#define L_sh 0.1293
#define L_lh 0.2321  // L_th+L_sh
#define L_ah 0.0826

#define SING_L_SPT 0  // single support (left leg)
#define SING_R_SPT 1  // single support (right leg)
#define DOUB_SPT 2    // double support

double servoOffset[JOINT_NUM] = {0};
#define MediMotoAlpha 12.80
#define SmalMotoAlpha 18.61

float AngleAlpha[JOINT_NUM] = {
    MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha,
    MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha,
    MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, SmalMotoAlpha, SmalMotoAlpha,
    MediMotoAlpha, SmalMotoAlpha, SmalMotoAlpha, SmalMotoAlpha, SmalMotoAlpha,
    SmalMotoAlpha, SmalMotoAlpha};