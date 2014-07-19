/**
 * =====================================================================================
 *        COPYRIGHT NOTICE
 *        Copyright (c) 2012  HUST-Renesas Lab
 *        ALL rights reserved.
 *
 *        @file     upstand_signal.h
 *
 *        @brief    upstand_signal.c header file
 *
 *        @version  0.1
 *        @date     2012/5/22 11:27:35
 *
 *        @author:  Liu Bojie , 313195046@qq.com
 * =====================================================================================
 *  @0.1    Liu Bojie   2012/5/22   create original file
 * =====================================================================================
 */

#ifndef   UPSTAND_SIGNAL_H
#define   UPSTAND_SIGNAL_H

#include "common.h"

/**
 * 函数kalman_filter的参数定义
 */
#define TIME_CONSTANT       (2 / 1000.0) /**< 采样间隔时间常数（单位：ms） */
//#define TIME_CONSTANT       0.6 /**< 采样间隔时间常数（单位：ms） */
#define  AD_INTERVAL        0.004
#define ACCY_CONVARIANCE    300  /**< 加速度计Y轴协方差 */
#define GYRO_CONVARIANCE    7    /**< 陀螺仪协方差 */
#define ACCY_AD2DEG_RATIO   0.46   /**< 加速度计角度转换比例 */
#define ANGLE_ZERO          156           /**< 加速度计静态零点设置值（粗调） */
/**
  *    @brief  卡尔曼滤波器函数，处理角度信息融合过程
  *
  *    @param  TIME_CONSTANT       采样间隔时间常数（单位：ms）
  *    @param  ACCY_CONVARIANCE    加速度计Y轴协方差
  *    @param  GYRO_CONVARIANCE    陀螺仪协方差
  *    @param  ACCY_AD2DEG_RATIO   加速度计角度转换比例
  *    @param  GYRO_AD2DEG_RATIO   陀螺仪角度转换比例
  *    @param  ANGLE_ZERO          加速度计静态零点设置值（粗调）
  *    @param  ANGLE_ZERO_FIXTURE  加速度计静态零点设置值（细调）
  *
  *    @note   参数体现在宏定义里，只针对不同的车体模型做微调
  */
void kalman_filter(void);
void BalanceFilter(void);

/**
 *    @brief  陀螺仪零点修正函数
 */
void gyrodrift_autofix(void);

extern float printvars[3];
extern int32_t gl_angle;

#endif
