//
// Created by lzx on 2025/10/24.
//

#include "slovetrajectory.h"
/*
@brief: 弹道解算 适配陈君的rm_vision
@author: CodeAlan  华南师大Vanguard战队
*/
// 近点只考虑水平方向的空气阻力



//TODO 完整弹道模型
//TODO 适配英雄机器人弹道解算


#include <math.h>
#include <stdio.h>
#include "slovetrajectory.h"
#include "user_lib.h"
#define YAW_MOTOR_RES_SPEED 1.6f
#define LARGE_ARMOR_WIDTH 0.23
#define SMALL_ARMOR_WIDTH 0.135


struct SolveTrajectoryParams st;
struct tar_pos tar_position[4]; //最多只有四块装甲板
float t = 0.5f; // 飞行时间
float yaw_ang_ref = 0.0f;
float imu_yaw=0.0f;
uint8_t armor_choose = 1;
uint8_t suggest_fire = 0;

void initSTValue(void)
{
  st.k = 0.092;
  st.bullet_type = BULLET_17;
  st.current_v = 25;
  st.current_pitch = 0;
  st.current_yaw = 0;
  st.xw = 0;
  st.yw = 0;
  st.zw = 0;

  st.vxw = 0;
  st.vyw = 0;
  st.vzw = 0;
  st.v_yaw = 0;
  st.tar_yaw = 0;
  st.r1 = 0.2;
  st.r2 = 0.2;
  st.dz = 0.02;
  st.bias_time = 200;//延迟预判提前量100
  st.s_bias = -0.084;
  st.z_bias = 0.00;
  st.armor_id = ARMOR_INFANTRY3;
  st.armor_num = ARMOR_NUM_NORMAL;
}

/*
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
  float z;
  //t为给定v与angle时的飞行时间
  //t = (float)((exp(st.k * s) - 1) / (st.k * v * cos(angle)));
  t = -log(1 - (st.k * s) / (v * cos(angle))) / st.k;
  //z为给定v与angle时的高度
  z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
  return z;
}


/*
@brief 完整弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
//TODO 完整弹道模型
float completeAirResistanceModel(float s, float v, float angle)
{
  // continue;


}



/*
@brief pitch轴解算
@param s:m 距离
@param z:m 高度
@param v:m/s
@return angle_pitch:rad
*/
float pitchTrajectoryCompensation(float s, float z, float v)
{
  float z_temp, z_actual, dz;
  float angle_pitch;
  int i = 0;
  z_temp = z;
  // iteration
  for (i = 0; i < 20; i++)
  {
    angle_pitch = atan2(z_temp, s); // rad
    z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
    dz = 0.3*(z - z_actual);
    z_temp = z_temp + dz;
    if (fabsf(dz) < 0.00001)
    {
      break;
    }
  }
  return angle_pitch;
}

/*
@brief 根据最优决策得出被击打装甲板 自动解算弹道
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
@param aim_x:传出aim_x  打击目标的x
@param aim_y:传出aim_y  打击目标的y
@param aim_z:传出aim_z  打击目标的z
*/
void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z)
{

  // 线性预测
  float timeDelay = st.bias_time/1000.0 + t;
  st.tar_yaw += st.v_yaw * timeDelay;

  //计算四块装甲板的位置
  //装甲板id顺序，以四块装甲板为例，逆时针编号
  //      2
  //   3     1
  //      0
  int use_1 = 1;
  int i = 0;
  int idx = 0; // 选择的装甲板
  //armor_num = ARMOR_NUM_BALANCE 为平衡步兵
  if (st.armor_num == ARMOR_NUM_BALANCE) {
    for (i = 0; i<2; i++) {
      float tmp_yaw = st.tar_yaw + i * PI;
      float r = st.r1;
      tar_position[i].x = st.xw - r*cos(tmp_yaw);
      tar_position[i].y = st.yw - r*sin(tmp_yaw);
      tar_position[i].z = st.zw;
      tar_position[i].yaw = tmp_yaw;
    }

    float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);

    //因为是平衡步兵 只需判断两块装甲板即可
    float temp_yaw_diff = fabsf(*yaw - tar_position[1].yaw);
    if (temp_yaw_diff < yaw_diff_min)
    {
      yaw_diff_min = temp_yaw_diff;
      idx = 1;
    }


  } else if (st.armor_num == ARMOR_NUM_OUTPOST) {  //前哨站
    for (i = 0; i<3; i++) {
      float tmp_yaw = st.tar_yaw + i * 2.0 * PI/3.0;  // 2/3PI
      float r =  (st.r1 + st.r2)/2;   //理论上r1=r2 这里取个平均值
      tar_position[i].x = st.xw - r*cos(tmp_yaw);
      tar_position[i].y = st.yw - r*sin(tmp_yaw);
      tar_position[i].z = st.zw;
      tar_position[i].yaw = tmp_yaw;
    }

    //TODO 选择最优装甲板 选板逻辑你们自己写，这个一般给英雄用


  } else {

    for (i = 0; i<4; i++) {
      float tmp_yaw = st.tar_yaw + i * PI/2.0;
      float r = use_1 ? st.r1 : st.r2;
      tar_position[i].x = st.xw - r*cos(tmp_yaw);
      tar_position[i].y = st.yw - r*sin(tmp_yaw);
      tar_position[i].z = use_1 ? st.zw : st.zw + st.dz;
      tar_position[i].yaw = tmp_yaw;
      use_1 = !use_1;
    }

    //2种常见决策方案：
    //1.计算枪管到目标装甲板yaw最小的那个装甲板
    //2.计算距离最近的装甲板

    //计算距离最近的装甲板
    float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
    int idx = 0;
    for (i = 1; i<4; i++)
    {
      float temp_dis_diff = sqrt(tar_position[i].x * tar_position[0].x + tar_position[i].y * tar_position[0].y);
      if (temp_dis_diff < dis_diff_min)
      {
        dis_diff_min = temp_dis_diff;
        idx = i;
      }
    }


    //计算枪管到目标装甲板yaw最小的那个装甲板
    // float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
    // for (i = 1; i<4; i++) {
    //     float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
    //     if (temp_yaw_diff < yaw_diff_min)
    //     {
    //         yaw_diff_min = temp_yaw_diff;
    //         idx = i;
    //     }
    // }

  }



  *aim_z = tar_position[idx].z + st.vzw * timeDelay;
  *aim_y = tar_position[idx].y + st.vyw * timeDelay;
  *aim_x = tar_position[idx].x + st.vxw * timeDelay;

  // 这里符号给错了
  //    *pitch = -pitchTrajectoryCompensation(sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - st.s_bias,
  //                                         st.z_bias + *aim_z, st.current_v);
  *pitch = -pitchTrajectoryCompensation(sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - st.s_bias,
                                        *aim_z - st.z_bias , st.current_v);
  *yaw = (float)(atan2(*aim_y, *aim_x)+0.05);
}
// 从坐标轴正向看向原点，逆时针方向为正
//


//迭代新火控+选板
void autoSolveTrajectory2(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z)
{
  float armor_x = 0.f, armor_y = 0.f, armor_z = 0.f;

  // 线性预测
  float timeDelay = st.bias_time/1000.0 + t;
  st.tar_yaw += st.v_yaw * timeDelay;

  int use_1 = 1;
  int i = 0;
  int idx = 0; // 选择的装甲板
  float allow_fire_ang_max = 0.f, allow_fire_ang_min = 0.f;

  //armor_num = ARMOR_NUM_BALANCE 为平衡步兵
  if (st.armor_num == ARMOR_NUM_BALANCE) {
    for (i = 0; i<2; i++) {
      float tmp_yaw = st.tar_yaw + i * PI;
      float r = st.r1;
      tar_position[i].x = st.xw - r*cos(tmp_yaw);
      tar_position[i].y = st.yw - r*sin(tmp_yaw);
      tar_position[i].z = st.zw;
      tar_position[i].yaw = tmp_yaw;
    }

    float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
    idx = 0;

    //因为是平衡步兵 只需判断两块装甲板即可
    float temp_yaw_diff = fabsf(*yaw - tar_position[1].yaw);
    if (temp_yaw_diff < yaw_diff_min)
    {
      yaw_diff_min = temp_yaw_diff;
      idx = 1;
    }

    // *** 修复：为平衡步兵模式赋值 aim ***
    *aim_x = tar_position[idx].x;
    *aim_y = tar_position[idx].y;
    *aim_z = tar_position[idx].z;

  } else if (st.armor_num == ARMOR_NUM_OUTPOST) {  //前哨站
    for (i = 0; i<3; i++) {
      float tmp_yaw = st.tar_yaw + i * 2.0 * PI/3.0;  // 2/3PI
      float r =  (st.r1 + st.r2)/2;   //理论上r1=r2 这里取个平均值
      tar_position[i].x = st.xw - r*cos(tmp_yaw);
      tar_position[i].y = st.yw - r*sin(tmp_yaw);
      tar_position[i].z = st.zw;
      tar_position[i].yaw = tmp_yaw;
    }

    // *** 修复：为前哨站模式赋值 aim（这里简单选择第一个装甲板，你可以自己优化选板逻辑）***
    idx = 0;
    float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
    for (i = 1; i < 3; i++) {
      float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
      if (temp_yaw_diff < yaw_diff_min) {
        yaw_diff_min = temp_yaw_diff;
        idx = i;
      }
    }

    *aim_x = tar_position[idx].x;
    *aim_y = tar_position[idx].y;
    *aim_z = tar_position[idx].z;

  } else {
    // 普通模式（4装甲板等）
    float predict_yaw =  st.tar_yaw + timeDelay * st.v_yaw;
    float center_theta = atan2(*aim_y, *aim_x);
    float diff_angle = 2 * PI / st.armor_num;
    use_1 = 1;

    // *** 修复：添加默认值，防止循环没有匹配时返回0 ***
    int found = 0;

    for (i = 0; i<4; i++) {
      float tmp_yaw = st.tar_yaw + i * diff_angle;
      float yaw_diff = get_delta_ang_pi(tmp_yaw, center_theta);

      if (fabsf(yaw_diff) < diff_angle / 2) {
        found = 1;
        armor_choose = 1;

        float r = st.r1;
        armor_z = st.zw;

        if (st.armor_num == 4) {
          r = use_1 ? st.r1 : st.r2;
          armor_z = use_1 ? st.zw : (st.zw + st.dz);
        }

        armor_x = st.xw - r * cos(tmp_yaw);
        armor_y = st.yw - r * sin(tmp_yaw);

        float armor_z_next = st.zw;
        if (st.armor_num == 4) {
          r = !use_1 ? st.r1 : st.r2;
          armor_z_next = !use_1 ? st.zw : (st.zw + st.dz);
        }

        float next_armor_yaw = tmp_yaw - sign(st.v_yaw) * diff_angle;
        float armor_x_next = st.xw - r * cos(next_armor_yaw);
        float armor_y_next = st.yw - r * sin(next_armor_yaw);

        float yaw_motor_delta = get_delta_ang_pi(atan2(armor_y, armor_x),
                                                 atan2(armor_y_next, armor_x_next));
        float angle_of_advance = fabsf(yaw_motor_delta) / YAW_MOTOR_RES_SPEED * fabsf(st.v_yaw) / 2;

        float est_yaw;
        if (sign(st.v_yaw) * yaw_diff < diff_angle / 2 - angle_of_advance || angle_of_advance > diff_angle / 4) {
          *aim_x = armor_x;
          *aim_y = armor_y;
          *aim_z = armor_z;
          est_yaw = tmp_yaw;
        } else {
          *aim_x  = armor_x_next;
          *aim_y= armor_y_next;
          *aim_z = armor_z_next;
          est_yaw = next_armor_yaw;
        }

        float armor_w;
        if ( st.armor_id == 1)
          armor_w = LARGE_ARMOR_WIDTH;
        else
          armor_w = SMALL_ARMOR_WIDTH;

        float ax = *aim_x - 0.5f * armor_w * sin(est_yaw);
        float ay = *aim_y + 0.5f * armor_w * cos(est_yaw);
        float bx = *aim_x + 0.5f * armor_w * sin(est_yaw);
        float by = *aim_y - 0.5f * armor_w * cos(est_yaw);

        float angle_a = atan2(ay, ax);
        float angle_b = atan2(by, bx);
        float angle_c = atan2(*aim_y, *aim_x );

        allow_fire_ang_max = angle_c - angle_b;
        allow_fire_ang_min = angle_c - angle_a;

        break;
      } else {
        armor_choose = 0;
      }
      use_1 = !use_1;
    }

    // *** 修复：如果没有找到合适的装甲板，使用目标中心位置 ***
    if (!found) {
      *aim_x = st.xw;
      *aim_y = st.yw;
      *aim_z = st.zw;
    }
  }

  *pitch = -pitchTrajectoryCompensation(sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - st.s_bias,
                                        *aim_z - st.z_bias , st.current_v);
  *yaw = (float)(atan2(*aim_y, *aim_x)+0.02);
  yaw_ang_ref=*yaw;

  fire_ctrl(allow_fire_ang_max, allow_fire_ang_min);
}





void fire_ctrl(float allow_fire_ang_max, float allow_fire_ang_min) {
  if (armor_choose == 0) {
    suggest_fire = 0;
  }
  else {
    // yaw_ang_ref

    float control_delta_angle =get_delta_ang_pi(yaw_ang_ref / 4096, imu_yaw / 4096);
    suggest_fire = (control_delta_angle < allow_fire_ang_max &&control_delta_angle > allow_fire_ang_min);
  }
  st.ref_yaw=imu_yaw;
  st.set_yaw=yaw_ang_ref;
  st.shoot_flag=suggest_fire;
  st.alo_max=allow_fire_ang_max;
  st.alo_min=allow_fire_ang_min;
}

float get_delta_ang_pi(float ang1, float ang2) {
  float diff = ang1 - ang2;
  while (diff > PI) diff -= 2 * PI;
  while (diff < -PI) diff += 2 * PI;
  return diff;
}



float *solveVision(const VisionRecvPacket *recv_packet)
{
  static float ret[3];  // 注意：static 保存在静态区，不会被销毁

  if (recv_packet == NULL)
    return ret;

  st.xw = recv_packet->x;
  st.yw = recv_packet->y;
  st.zw = recv_packet->z;
  st.vxw = recv_packet->vx;
  st.vyw = recv_packet->vy;
  st.vzw = recv_packet->vz;
  st.tar_yaw = recv_packet->yaw;
  st.v_yaw   = recv_packet->v_yaw;
  st.r1 = recv_packet->r1;
  st.r2 = recv_packet->r2;
  st.dz = recv_packet->dz;
  st.armor_id  = recv_packet->id;
  st.armor_num = recv_packet->armors_num;

  float pitch_rad = 0.0f, yaw_rad = 0.0f;
  float aimx = 0.0f, aimy = 0.0f, aimz = 0.0f;
  autoSolveTrajectory2(&pitch_rad, &yaw_rad, &aimx, &aimy, &aimz);

  hhSerial_Printf("Solve result: aim_x=%.3f aim_y=%.3f aim_z=%.3f\n",
                  aimx, aimy, aimz);

  ret[0] = aimx;
  ret[1] = aimy;
  ret[2] = aimz;
  return ret;
}
