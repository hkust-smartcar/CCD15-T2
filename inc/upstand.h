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

#pragma once

#include <libsc/mpu6050.h>
#include <libbase/kl26/adc.h>
using namespace libsc;

/**
 * 函数kalman_filter的参数定义
 */
#define TIME_CONSTANT       (2 / 1000.0f) /**< 采样间隔时间常数（单位：ms） */
//#define TIME_CONSTANT       0.6 /**< 采样间隔时间常数（单位：ms） */
#define  AD_INTERVAL        0.004f
#define ACCY_CONVARIANCE    1200  /**< 加速度计Y轴协方差 */
#define GYRO_CONVARIANCE    1    /**< 陀螺仪协方差 */
//#define ACCY_AD2DEG_RATIO   0.46f   /**< 加速度计角度转换比例 */
#define ANGLE_ZERO          156           /**< 加速度计静态零点设置值（粗调） */

using namespace libbase::kl26;

class Upstand{
public:
	Upstand(libbase::kl26::Adc* acc_adc_, libbase::kl26::Adc* gyro_adc_):
		m_mpu(NULL),
		m_acc_adc(acc_adc_),
		m_gyro_adc(gyro_adc_),
		m_angle(0)
	{
//		m_acc_adc = acc_adc_;
//		m_gyro_adc = gyro_adc_;
	}

	Upstand(Mpu6050* mpu_, Mma8451q* mma_):
		m_mpu(mpu_),
		m_mma(mma_),
		m_acc_adc(NULL),
		m_gyro_adc(NULL),
		m_angle(0)
	{}

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
	void KalmanFilter(void);
	void BalanceFilter(void);
	/**
	 *    @brief  陀螺仪零点修正函数
	 */
	void GyroDriftAutoFix(void);

	float GetAngle(){
		return m_angle;
	}

	float GetAccAngle(){
		return m_angle_acc;
	}

	float GetGyroAngle(){
		return m_angle_gyro;
	}

	void SetR(uint32_t r){
		m_r = r;
	}

private:
	Mpu6050* m_mpu;
	Mma8451q* m_mma;
	libbase::kl26::Adc* m_acc_adc;
	libbase::kl26::Adc* m_gyro_adc;
	/**
	 *  全局变量定义
	 */
	float m_angle;
	int32_t  m_gyro_ad         = 0;
	int32_t  m_acc_ad          = 0;
	int32_t  m_acc             = 0;    /* 用于显示在液晶上的加速度计的实时值 */
	int32_t  m_gyr             = 0;
	float  m_angle_acc 	  	   = 0;
	float  m_angle_gyro        = 0;
	int32_t  m_gyro_zero       = 327;
	int32_t  m_angle_real      = 0;
	int32_t  m_angle_gyro_real = 0;
	int32_t  m_acc_fix         = 0;
	uint32_t  m_r			   = 0;
};
