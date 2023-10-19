/*
    Copyright 2021 codenocold codenocold@qq.com
    Address : https://github.com/codenocold/dgm
    This file is part of the dgm firmware.
    The dgm firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    The dgm firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "trapTraj.h"
#include "pwm_curr.h"
#include <math.h>
#include "util.h"

tTraj Traj;

// A sign function where input 0 has positive sign (not 0)//输入0的正负号   
static inline float sign_hard(float val) {
    return (signbit(val)) ? -1.0f : 1.0f;
}
//梯形速度规划
//目标位置 开始位置 开始速度 最大速度 最大加速度 最大减速度
void TRAJ_plan(float position, float start_position, float start_velocity, float Vmax, float Amax, float Dmax)
{
    float distance = position - start_position;             // Distance to travel//需要行驶的距离
    float stop_dist = SQ(start_velocity) / (2.0f * Dmax);   // Minimum stopping distance//最小停止距离
    float dXstop = copysign(stop_dist, start_velocity);     // Minimum stopping displacement//最小停止位移
    float s = sign_hard(distance - dXstop);                 // Sign of coast velocity (if any)//停止速度符号
    Traj.acc =  s * Amax;   // Maximum Acceleration (signed)//最大加速度
    Traj.dec = -s * Dmax;   // Maximum Deceleration (signed)//最大减速度
    Traj.vel =  s * Vmax;   // Maximum Velocity (signed)//最大速度

    // If we start with a speed faster than cruising, then we need to decel instead of accel aka "double deceleration move" in the paper//如果起始速度比齿槽速度大，那么需要减速
    if ((s * start_velocity) > (s * Traj.vel)) {
        Traj.acc = -s * Amax;
    }

    // Time to accel/decel to/from Vr (cruise speed)//从起始速度到齿槽速度的时间
    Traj.t_acc = (Traj.vel - start_velocity) / Traj.acc;
    Traj.t_dec = -Traj.vel / Traj.dec;
         
    // Integral of velocity ramps over the full accel and decel times to get//获得加速和减速的积分
    // minimum displacement required to reach cuising speed//最小位移
    float dXmin = 0.5f * Traj.t_acc * (Traj.vel + start_velocity) + 0.5f * Traj.t_dec * Traj.vel;//最小距离

    // Are we displacing enough to reach cruising speed?//是否能够到达齿槽速度
    if (s * distance < s * dXmin) {//距离不够 
        // Short move (triangle profile)//三角形齿槽
        Traj.vel = s * sqrtf(fmax((Traj.dec * SQ(start_velocity) + 2.0f * Traj.acc * Traj.dec * distance) / (Traj.dec - Traj.acc), 0.0f));
        Traj.t_acc = fmax(0.0f, (Traj.vel - start_velocity) / Traj.acc);//加速时间
        Traj.t_dec = fmax(0.0f, -Traj.vel / Traj.dec);//减速时间
        Traj.t_vel = 0.0f;//匀速时间
    } else {
        // Long move (trapezoidal profile)//梯形齿槽
        Traj.t_vel = (distance - dXmin) / Traj.vel;
    }

    // Fill in the rest of the values used at evaluation-time//在评估时填充剩余值
    Traj.t_total = Traj.t_acc + Traj.t_vel + Traj.t_dec;
    Traj.start_position = start_position;
    Traj.start_velocity = start_velocity;
    Traj.end_position = position;

    //ax=x0+v0t+0.5at^2
    Traj.acc_distance = start_position + start_velocity * Traj.t_acc + 0.5f * Traj.acc * SQ(Traj.t_acc); // pos at end of accel phase//结束加速阶段的位置

    Traj.tick = 0;
    Traj.profile_done = false;
}

void TRAJ_eval(void)
{
    if(Traj.profile_done){
        return;
    }
    
    Traj.tick ++;
    float t = Traj.tick * CURRENT_MEASURE_PERIOD;//从tick转换到物理时间
    
    if (t < 0.0f) {                                     // Initial Condition//初始条件
        Traj.Y   = Traj.start_position;//p
        Traj.Yd  = Traj.start_velocity;//v
        Traj.Ydd = 0.0f;//a
    } else if (t < Traj.t_acc) {                // Accelerating
        Traj.Y   = Traj.start_position + Traj.start_velocity * t + 0.5f * Traj.acc * SQ(t);
        Traj.Yd  = Traj.start_velocity + Traj.acc * t;
        Traj.Ydd = Traj.acc;
    } else if (t < Traj.t_acc + Traj.t_vel) {   // Coasting//齿槽速度
        Traj.Y   = Traj.acc_distance + Traj.vel * (t - Traj.t_acc);
        Traj.Yd  = Traj.vel;
        Traj.Ydd = 0.0f;
    } else if (t < Traj.t_total) {              // Deceleration//减速 
        float td = t - Traj.t_total;
        Traj.Y   = Traj.end_position + 0.5f * Traj.dec * SQ(td);
        Traj.Yd  = Traj.dec * td;
        Traj.Ydd = Traj.dec;
    } else if (t >= Traj.t_total) {             // Final Condition//最终条件
        Traj.Y   = Traj.end_position;
        Traj.Yd  = 0.0f;
        Traj.Ydd = 0.0f;
        Traj.profile_done = true;
    }
}
