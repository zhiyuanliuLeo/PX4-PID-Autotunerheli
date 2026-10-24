/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ActuatorEffectivenessHelicopter.hpp"
#include <lib/mathlib/mathlib.h>

using namespace matrix;
using namespace time_literals;

// 直升机的有效性类，输入参数是尾部控制器的类型，可以是电机也可以是舵机
ActuatorEffectivenessHelicopter::ActuatorEffectivenessHelicopter(ModuleParams *parent, ActuatorType tail_actuator_type)
	: ModuleParams(parent), _tail_actuator_type(tail_actuator_type)
{
	// 循环获取斜盘参数
	for (int i = 0; i < NUM_SWASH_PLATE_SERVOS_MAX; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SP0_ANG%u", i);
		_param_handles.swash_plate_servos[i].angle = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SP0_ARM_L%u", i);
		_param_handles.swash_plate_servos[i].arm_length = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TRIM", i);
		_param_handles.swash_plate_servos[i].trim = param_find(buffer);
	}

	_param_handles.num_swash_plate_servos = param_find("CA_SP0_COUNT");

	// 循环获取油门和总距参数
	for (int i = 0; i < NUM_CURVE_POINTS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_HELI_THR_C%u", i);
		_param_handles.throttle_curve[i] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_HELI_PITCH_C%u", i);
		_param_handles.pitch_curve[i] = param_find(buffer);
	}

	// 获取尾部参数
	_param_handles.yaw_collective_pitch_scale = param_find("CA_HELI_YAW_CP_S");
	_param_handles.yaw_collective_pitch_offset = param_find("CA_HELI_YAW_CP_O");
	_param_handles.yaw_throttle_scale = param_find("CA_HELI_YAW_TH_S");
	_param_handles.yaw_ccw = param_find("CA_HELI_YAW_CCW");
	_param_handles.spoolup_time = param_find("COM_SPOOLUP_TIME");

	updateParams();
}

// 集中更新参数
void ActuatorEffectivenessHelicopter::updateParams()
{
	ModuleParams::updateParams();

	int32_t count = 0;

	if (param_get(_param_handles.num_swash_plate_servos, &count) != 0) {
		PX4_ERR("param_get failed");
		return;
	}

	_geometry.num_swash_plate_servos = math::constrain((int)count, 3, NUM_SWASH_PLATE_SERVOS_MAX);

	for (int i = 0; i < _geometry.num_swash_plate_servos; ++i) {
		float angle_deg{};
		param_get(_param_handles.swash_plate_servos[i].angle, &angle_deg);
		_geometry.swash_plate_servos[i].angle = math::radians(angle_deg);
		param_get(_param_handles.swash_plate_servos[i].arm_length, &_geometry.swash_plate_servos[i].arm_length);
		param_get(_param_handles.swash_plate_servos[i].trim, &_geometry.swash_plate_servos[i].trim);
	}

	for (int i = 0; i < NUM_CURVE_POINTS; ++i) {
		param_get(_param_handles.throttle_curve[i], &_geometry.throttle_curve[i]);
		param_get(_param_handles.pitch_curve[i], &_geometry.pitch_curve[i]);
	}

	param_get(_param_handles.yaw_collective_pitch_scale, &_geometry.yaw_collective_pitch_scale);
	param_get(_param_handles.yaw_collective_pitch_offset, &_geometry.yaw_collective_pitch_offset);
	param_get(_param_handles.yaw_throttle_scale, &_geometry.yaw_throttle_scale);
	param_get(_param_handles.spoolup_time, &_geometry.spoolup_time);
	int32_t yaw_ccw = 0;
	param_get(_param_handles.yaw_ccw, &yaw_ccw);
	// 尾部旋翼旋转方向
	_geometry.yaw_sign = (yaw_ccw == 1) ? -1.f : 1.f;
}

// 获取有效性矩阵
bool ActuatorEffectivenessHelicopter::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// 由于分配是非线性的，我们使用updateSetpoint（）而不是矩阵
	// As the allocation is non-linear, we use updateSetpoint() instead of the matrix
	// 增加一个电机执行器
	// 这里只是增加执行器编号，并没有增加力矩和力系数
	configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{});

	// Tail (yaw) (either ESC or Servo)
	// 增加一个舵机执行器
	// 这里只是增加执行器编号，并没有增加力矩和力系数
	configuration.addActuator(_tail_actuator_type, Vector3f{}, Vector3f{});

	// N swash plate servos
	_first_swash_plate_servo_index = configuration.num_actuators_matrix[0];

	// 循环增加多个斜盘舵机执行器
	// 这里只是增加执行器编号，并没有增加力矩和力系数
	for (int i = 0; i < _geometry.num_swash_plate_servos; ++i) {
		configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
		configuration.trim[configuration.selected_matrix](i) = _geometry.swash_plate_servos[i].trim;
	}

	return true;
}

// 向主程序更新本程序的控制分配器计算得到的执行器指令
void ActuatorEffectivenessHelicopter::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	/**
	 * @brief 任务说明
	 * 1.解锁时，电机立刻启动，但是保持最低转速并锁定在这里。电机需要一个怠速，也作为飞行时的最低转速。也作为指示已经开始旋转的状态。
	 * 2.要把总距指令与油门指令融合，下面的姿态控制与总距指令有关。
	 *
	 */
	_saturation_flags = {};

	// throttle/collective pitch curve
	// 用Z轴油门生成总油门和总距曲线，带有油门缓启动功能。
	const float throttle = math::constrain(-control_sp(ControlAxis::THRUST_Z), 0.15f,1.0f) * throttleSpoolupProgress();
	// 用Z轴油门生成总油门和总距曲线
	// const float collective_pitch = math::interpolateN(-control_sp(ControlAxis::THRUST_Z), _geometry.pitch_curve);

	// actuator mapping
	// 执行器映射

	// 主旋翼信号
	// 主电机PWM信号范围是0~1
	actuator_sp(0) = mainMotorEnaged() ? throttle : NAN;
	// 油门直接对应总距，不定速，油门直接控制转速，起到总距的作用。
	const float throttle_collective = mainMotorEnaged() ? throttle : 0.0f;

	// 尾旋翼信号
	// 这里使用了油门指令，用于计算主旋翼的反扭矩，要改成油门信号。
	// 尾电机PWM信号范围是0~1
	actuator_sp(1) = control_sp(ControlAxis::YAW) * _geometry.yaw_sign		// 偏航位置映射
			 + fabsf(throttle_collective - _geometry.yaw_collective_pitch_offset) * _geometry.yaw_collective_pitch_scale;	// 油门对偏航的预偏差和偏航系数。

	// Saturation check for yaw
	// 饱和检查
	if (actuator_sp(1) < actuator_min(1)) {
		setSaturationFlag(_geometry.yaw_sign, _saturation_flags.yaw_neg, _saturation_flags.yaw_pos);

	} else if (actuator_sp(1) > actuator_max(1)) {
		setSaturationFlag(_geometry.yaw_sign, _saturation_flags.yaw_pos, _saturation_flags.yaw_neg);
	}

	for (int i = 0; i < _geometry.num_swash_plate_servos; i++) {
		// 获取倾斜盘舵机与倾斜盘连接点在x和y方向的投影
		float roll_coeff = sinf(_geometry.swash_plate_servos[i].angle) * _geometry.swash_plate_servos[i].arm_length;
		float pitch_coeff = cosf(_geometry.swash_plate_servos[i].angle) * _geometry.swash_plate_servos[i].arm_length;
		// 循环执行倾斜盘舵机输出信号的混控
		// 双电机直升机不需要在舵机中混控总距油门
		actuator_sp(_first_swash_plate_servo_index + i) =
				+ control_sp(ControlAxis::PITCH) * pitch_coeff
				- control_sp(ControlAxis::ROLL) * roll_coeff
				+ _geometry.swash_plate_servos[i].trim;

		// Saturation check for roll & pitch
		// 饱和检查
		if (actuator_sp(_first_swash_plate_servo_index + i) < actuator_min(_first_swash_plate_servo_index + i)) {
			setSaturationFlag(roll_coeff, _saturation_flags.roll_pos, _saturation_flags.roll_neg);
			setSaturationFlag(pitch_coeff, _saturation_flags.pitch_neg, _saturation_flags.pitch_pos);

		} else if (actuator_sp(_first_swash_plate_servo_index + i) > actuator_max(_first_swash_plate_servo_index + i)) {
			setSaturationFlag(roll_coeff, _saturation_flags.roll_neg, _saturation_flags.roll_pos);
			setSaturationFlag(pitch_coeff, _saturation_flags.pitch_pos, _saturation_flags.pitch_neg);
		}
	}
}

// 根据遥控器开关判断是否启用主电机
bool ActuatorEffectivenessHelicopter::mainMotorEnaged()
{
	manual_control_switches_s manual_control_switches;

	if (_manual_control_switches_sub.update(&manual_control_switches)) {
		_main_motor_engaged = manual_control_switches.engage_main_motor_switch == manual_control_switches_s::SWITCH_POS_NONE
				      || manual_control_switches.engage_main_motor_switch == manual_control_switches_s::SWITCH_POS_ON;
	}

	return _main_motor_engaged;
}

// 油门缓启动程序
float ActuatorEffectivenessHelicopter::throttleSpoolupProgress()
{
	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
		_armed_time = vehicle_status.armed_time;
	}

	// 解锁之后立刻启动？
	const float time_since_arming = (hrt_absolute_time() - _armed_time) / 1e6f;
	const float spoolup_progress = time_since_arming / _geometry.spoolup_time;

	if (_armed && spoolup_progress < 1.f) {
		return spoolup_progress;
	}

	return 1.f;
}


// 设置控制饱和标志位
void ActuatorEffectivenessHelicopter::setSaturationFlag(float coeff, bool &positive_flag, bool &negative_flag)
{
	if (coeff > 0.f) {
		// A positive change in given axis will increase saturation
		positive_flag = true;

	} else if (coeff < 0.f) {
		// A negative change in given axis will increase saturation
		negative_flag = true;
	}
}

// 获取无分配控制？TODO：搞清楚什么作用
void ActuatorEffectivenessHelicopter::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	// Note: the values '-1', '1' and '0' are just to indicate a negative,
	// positive or no saturation to the rate controller. The actual magnitude is not used.
	if (_saturation_flags.roll_pos) {
		status.unallocated_torque[0] = 1.f;

	} else if (_saturation_flags.roll_neg) {
		status.unallocated_torque[0] = -1.f;

	} else {
		status.unallocated_torque[0] = 0.f;
	}

	if (_saturation_flags.pitch_pos) {
		status.unallocated_torque[1] = 1.f;

	} else if (_saturation_flags.pitch_neg) {
		status.unallocated_torque[1] = -1.f;

	} else {
		status.unallocated_torque[1] = 0.f;
	}

	if (_saturation_flags.yaw_pos) {
		status.unallocated_torque[2] = 1.f;

	} else if (_saturation_flags.yaw_neg) {
		status.unallocated_torque[2] = -1.f;

	} else {
		status.unallocated_torque[2] = 0.f;
	}

	if (_saturation_flags.thrust_pos) {
		status.unallocated_thrust[2] = 1.f;

	} else if (_saturation_flags.thrust_neg) {
		status.unallocated_thrust[2] = -1.f;

	} else {
		status.unallocated_thrust[2] = 0.f;
	}
}







// /****************************************************************************
//  *
//  *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions
//  * are met:
//  *
//  * 1. Redistributions of source code must retain the above copyright
//  *    notice, this list of conditions and the following disclaimer.
//  * 2. Redistributions in binary form must reproduce the above copyright
//  *    notice, this list of conditions and the following disclaimer in
//  *    the documentation and/or other materials provided with the
//  *    distribution.
//  * 3. Neither the name PX4 nor the names of its contributors may be
//  *    used to endorse or promote products derived from this software
//  *    without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
//  * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
//  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  * POSSIBILITY OF SUCH DAMAGE.
//  *
//  ****************************************************************************/

// #include "ActuatorEffectivenessHelicopter.hpp"
// #include <lib/mathlib/mathlib.h>

// using namespace matrix;
// using namespace time_literals;

// ActuatorEffectivenessHelicopter::ActuatorEffectivenessHelicopter(ModuleParams *parent, ActuatorType tail_actuator_type)
// 	: ModuleParams(parent), _tail_actuator_type(tail_actuator_type)
// {
// 	for (int i = 0; i < NUM_SWASH_PLATE_SERVOS_MAX; ++i) {
// 		char buffer[17];
// 		snprintf(buffer, sizeof(buffer), "CA_SP0_ANG%u", i);
// 		_param_handles.swash_plate_servos[i].angle = param_find(buffer);
// 		snprintf(buffer, sizeof(buffer), "CA_SP0_ARM_L%u", i);
// 		_param_handles.swash_plate_servos[i].arm_length = param_find(buffer);
// 		snprintf(buffer, sizeof(buffer), "CA_SV_CS%u_TRIM", i);
// 		_param_handles.swash_plate_servos[i].trim = param_find(buffer);
// 	}

// 	_param_handles.num_swash_plate_servos = param_find("CA_SP0_COUNT");

// 	for (int i = 0; i < NUM_CURVE_POINTS; ++i) {
// 		char buffer[17];
// 		snprintf(buffer, sizeof(buffer), "CA_HELI_THR_C%u", i);
// 		_param_handles.throttle_curve[i] = param_find(buffer);
// 		snprintf(buffer, sizeof(buffer), "CA_HELI_PITCH_C%u", i);
// 		_param_handles.pitch_curve[i] = param_find(buffer);
// 	}

// 	_param_handles.yaw_collective_pitch_scale = param_find("CA_HELI_YAW_CP_S");
// 	_param_handles.yaw_collective_pitch_offset = param_find("CA_HELI_YAW_CP_O");
// 	_param_handles.yaw_throttle_scale = param_find("CA_HELI_YAW_TH_S");
// 	_param_handles.yaw_ccw = param_find("CA_HELI_YAW_CCW");
// 	_param_handles.spoolup_time = param_find("COM_SPOOLUP_TIME");

// 	updateParams();
// }

// void ActuatorEffectivenessHelicopter::updateParams()
// {
// 	ModuleParams::updateParams();

// 	int32_t count = 0;

// 	if (param_get(_param_handles.num_swash_plate_servos, &count) != 0) {
// 		PX4_ERR("param_get failed");
// 		return;
// 	}

// 	_geometry.num_swash_plate_servos = math::constrain((int)count, 3, NUM_SWASH_PLATE_SERVOS_MAX);

// 	for (int i = 0; i < _geometry.num_swash_plate_servos; ++i) {
// 		float angle_deg{};
// 		param_get(_param_handles.swash_plate_servos[i].angle, &angle_deg);
// 		_geometry.swash_plate_servos[i].angle = math::radians(angle_deg);
// 		param_get(_param_handles.swash_plate_servos[i].arm_length, &_geometry.swash_plate_servos[i].arm_length);
// 		param_get(_param_handles.swash_plate_servos[i].trim, &_geometry.swash_plate_servos[i].trim);
// 	}

// 	for (int i = 0; i < NUM_CURVE_POINTS; ++i) {
// 		param_get(_param_handles.throttle_curve[i], &_geometry.throttle_curve[i]);
// 		param_get(_param_handles.pitch_curve[i], &_geometry.pitch_curve[i]);
// 	}

// 	param_get(_param_handles.yaw_collective_pitch_scale, &_geometry.yaw_collective_pitch_scale);
// 	param_get(_param_handles.yaw_collective_pitch_offset, &_geometry.yaw_collective_pitch_offset);
// 	param_get(_param_handles.yaw_throttle_scale, &_geometry.yaw_throttle_scale);
// 	param_get(_param_handles.spoolup_time, &_geometry.spoolup_time);
// 	int32_t yaw_ccw = 0;
// 	param_get(_param_handles.yaw_ccw, &yaw_ccw);
// 	_geometry.yaw_sign = (yaw_ccw == 1) ? -1.f : 1.f;
// }

// bool ActuatorEffectivenessHelicopter::getEffectivenessMatrix(Configuration &configuration,
// 		EffectivenessUpdateReason external_update)
// {
// 	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
// 		return false;
// 	}

// 	// As the allocation is non-linear, we use updateSetpoint() instead of the matrix
// 	configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{});

// 	// Tail (yaw) (either ESC or Servo)
// 	configuration.addActuator(_tail_actuator_type, Vector3f{}, Vector3f{});

// 	// N swash plate servos
// 	_first_swash_plate_servo_index = configuration.num_actuators_matrix[0];

// 	for (int i = 0; i < _geometry.num_swash_plate_servos; ++i) {
// 		configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
// 		configuration.trim[configuration.selected_matrix](i) = _geometry.swash_plate_servos[i].trim;
// 	}

// 	return true;
// }

// void ActuatorEffectivenessHelicopter::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
// 		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
// 		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
// {
// 	_saturation_flags = {};

// 	// throttle/collective pitch curve
// 	const float throttle = math::interpolateN(-control_sp(ControlAxis::THRUST_Z),
// 			       _geometry.throttle_curve) * throttleSpoolupProgress();
// 	const float collective_pitch = math::interpolateN(-control_sp(ControlAxis::THRUST_Z), _geometry.pitch_curve)* throttleSpoolupProgress();

// 	// actuator mapping
// 	actuator_sp(0) = mainMotorEnaged() ? throttle : NAN;

// 	actuator_sp(1) = control_sp(ControlAxis::YAW) * _geometry.yaw_sign
// 			 + fabsf(collective_pitch - _geometry.yaw_collective_pitch_offset) * _geometry.yaw_collective_pitch_scale
// 			 + throttle * _geometry.yaw_throttle_scale;

// 	// Saturation check for yaw
// 	if (actuator_sp(1) < actuator_min(1)) {
// 		setSaturationFlag(_geometry.yaw_sign, _saturation_flags.yaw_neg, _saturation_flags.yaw_pos);

// 	} else if (actuator_sp(1) > actuator_max(1)) {
// 		setSaturationFlag(_geometry.yaw_sign, _saturation_flags.yaw_pos, _saturation_flags.yaw_neg);
// 	}

// 	for (int i = 0; i < _geometry.num_swash_plate_servos; i++) {
// 		float roll_coeff = sinf(_geometry.swash_plate_servos[i].angle) * _geometry.swash_plate_servos[i].arm_length;
// 		float pitch_coeff = cosf(_geometry.swash_plate_servos[i].angle) * _geometry.swash_plate_servos[i].arm_length;
// 		actuator_sp(_first_swash_plate_servo_index + i) = collective_pitch
// 				+ control_sp(ControlAxis::PITCH) * pitch_coeff
// 				- control_sp(ControlAxis::ROLL) * roll_coeff
// 				+ _geometry.swash_plate_servos[i].trim;

// 		// Saturation check for roll & pitch
// 		if (actuator_sp(_first_swash_plate_servo_index + i) < actuator_min(_first_swash_plate_servo_index + i)) {
// 			setSaturationFlag(roll_coeff, _saturation_flags.roll_pos, _saturation_flags.roll_neg);
// 			setSaturationFlag(pitch_coeff, _saturation_flags.pitch_neg, _saturation_flags.pitch_pos);

// 		} else if (actuator_sp(_first_swash_plate_servo_index + i) > actuator_max(_first_swash_plate_servo_index + i)) {
// 			setSaturationFlag(roll_coeff, _saturation_flags.roll_neg, _saturation_flags.roll_pos);
// 			setSaturationFlag(pitch_coeff, _saturation_flags.pitch_pos, _saturation_flags.pitch_neg);
// 		}
// 	}
// }

// bool ActuatorEffectivenessHelicopter::mainMotorEnaged()
// {
// 	manual_control_switches_s manual_control_switches;

// 	if (_manual_control_switches_sub.update(&manual_control_switches)) {
// 		_main_motor_engaged = manual_control_switches.engage_main_motor_switch == manual_control_switches_s::SWITCH_POS_NONE
// 				      || manual_control_switches.engage_main_motor_switch == manual_control_switches_s::SWITCH_POS_ON;
// 	}

// 	return _main_motor_engaged;
// }

// float ActuatorEffectivenessHelicopter::throttleSpoolupProgress()
// {
// 	vehicle_status_s vehicle_status;

// 	if (_vehicle_status_sub.update(&vehicle_status)) {
// 		_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
// 		_armed_time = vehicle_status.armed_time;
// 	}

// 	const float time_since_arming = (hrt_absolute_time() - _armed_time) / 1e6f;
// 	const float spoolup_progress = time_since_arming / _geometry.spoolup_time;

// 	if (_armed && spoolup_progress < 1.f) {
// 		return spoolup_progress;
// 	}

// 	return 1.f;
// }


// void ActuatorEffectivenessHelicopter::setSaturationFlag(float coeff, bool &positive_flag, bool &negative_flag)
// {
// 	if (coeff > 0.f) {
// 		// A positive change in given axis will increase saturation
// 		positive_flag = true;

// 	} else if (coeff < 0.f) {
// 		// A negative change in given axis will increase saturation
// 		negative_flag = true;
// 	}
// }

// void ActuatorEffectivenessHelicopter::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
// {
// 	// Note: the values '-1', '1' and '0' are just to indicate a negative,
// 	// positive or no saturation to the rate controller. The actual magnitude is not used.
// 	if (_saturation_flags.roll_pos) {
// 		status.unallocated_torque[0] = 1.f;

// 	} else if (_saturation_flags.roll_neg) {
// 		status.unallocated_torque[0] = -1.f;

// 	} else {
// 		status.unallocated_torque[0] = 0.f;
// 	}

// 	if (_saturation_flags.pitch_pos) {
// 		status.unallocated_torque[1] = 1.f;

// 	} else if (_saturation_flags.pitch_neg) {
// 		status.unallocated_torque[1] = -1.f;

// 	} else {
// 		status.unallocated_torque[1] = 0.f;
// 	}

// 	if (_saturation_flags.yaw_pos) {
// 		status.unallocated_torque[2] = 1.f;

// 	} else if (_saturation_flags.yaw_neg) {
// 		status.unallocated_torque[2] = -1.f;

// 	} else {
// 		status.unallocated_torque[2] = 0.f;
// 	}

// 	if (_saturation_flags.thrust_pos) {
// 		status.unallocated_thrust[2] = 1.f;

// 	} else if (_saturation_flags.thrust_neg) {
// 		status.unallocated_thrust[2] = -1.f;

// 	} else {
// 		status.unallocated_thrust[2] = 0.f;
// 	}
// }
