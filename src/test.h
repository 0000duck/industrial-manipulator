/*
 * test.h
 *
 *  Created on: Aug 18, 2017
 *      Author: a1994846931931
 */

#ifndef TEST_H_
#define TEST_H_

# include "ext/BaseRobot.h"

#define alpha1 DH_Q_A  //机器人DH的alpha，扭转角
#define alpha2 DH_Q_B
#define alpha3 DH_Q_C
#define alpha4 DH_Q_D
#define alpha5 DH_Q_E
#define alpha6 DH_Q_F

#define  a1 DH_O_A //机器人DH的a，连杆长度
#define  a2 DH_O_B
#define  a3 DH_O_C
#define  a4 DH_O_D
#define  a5 DH_O_E
#define  a6 DH_O_F

#define  d1 DH_P_A  //机器人DH的d，连杆偏移量
#define  d2 DH_P_B
#define  d3 DH_P_C
#define  d4 DH_P_D
#define  d5 DH_P_E
#define  d6 DH_P_F

#define  theta1 JOINT_A  //定义JOINT_ 为theta值
#define  theta2 JOINT_B
#define  theta3 JOINT_C
#define  theta4 JOINT_D
#define  theta5 JOINT_E
#define  theta6 JOINT_F

#define  lmax1 DH_jointrangeU_A  //机器人关节运动范围上限
#define  lmax2 DH_jointrangeU_B
#define  lmax3 DH_jointrangeU_C
#define  lmax4 DH_jointrangeU_D
#define  lmax5 DH_jointrangeU_E
#define  lmax6 DH_jointrangeU_F

#define  lmin1 DH_jointrangeD_A   //机器人关节运动范围下限
#define  lmin2 DH_jointrangeD_B
#define  lmin3 DH_jointrangeD_C
#define  lmin4 DH_jointrangeD_D
#define  lmin5 DH_jointrangeD_E
#define  lmin6 DH_jointrangeD_F


#endif /* TEST_H_ */
