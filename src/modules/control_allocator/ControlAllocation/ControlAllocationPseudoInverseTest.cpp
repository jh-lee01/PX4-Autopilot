/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

/**
 * @file ControlAllocationTest.cpp
 *
 * Tests for Control Allocation Algorithms
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */
#define MODULE_NAME "control_allocation_test"
#include <gtest/gtest.h>
#include <ControlAllocationPseudoInverse.hpp>
#include <matrix/matrix/math.hpp>
#include <iostream>
#include <iomanip>

using namespace matrix;

TEST(ControlAllocationPseudoInverseTest, SingleStep)
{
    ControlAllocationPseudoInverse method;

	matrix::Vector<float, 6> control_sp;
	matrix::Vector<float, 6> control_allocated;
	matrix::Vector<float, 6> control_allocated_expected;
	matrix::Matrix<float, 6, 16> effectiveness;
	matrix::Matrix<float, 16, 6> mix;
	matrix::Vector<float, 16> actuator_sp;
	matrix::Vector<float, 16> actuator_trim;
	matrix::Vector<float, 16> linearization_point;
	matrix::Vector<float, 16> actuator_sp_expected;


	
	// 0도
	// effectiveness 행렬 각 요소에 내가 원하는 값 반복문 없이 설정
	// effectiveness(0, 0) = -1.0073246f; effectiveness(0, 1) =  1.8164136f; effectiveness(0, 2) =  0.53718674f; effectiveness(0, 3) = -2.5382986f; effectiveness(0, 5) = -5.4366393f;
	// effectiveness(1, 0) =  1.0073246f; effectiveness(1, 1) = -1.8164136f; effectiveness(1, 2) =  0.53718674f; effectiveness(1, 3) =  2.5382986f; effectiveness(1, 5) = -5.4366393f;
	// effectiveness(2, 0) =  1.0073246f; effectiveness(2, 1) =  1.8164136f; effectiveness(2, 2) = -0.53718674f; effectiveness(2, 3) = -2.5382986f; effectiveness(2, 5) = -5.4366393f;
	// effectiveness(3, 0) = -1.0073246f; effectiveness(3, 1) = -1.8164136f; effectiveness(3, 2) = -0.53718674f; effectiveness(3, 3) =  2.5382986f; effectiveness(3, 5) = -5.4366393f;

	// 30도
	// effectiveness(0, 0) = -1.0073246f; effectiveness(0, 1) = 1.5191985f; effectiveness(0, 2) =  0.53718674f; effectiveness(0, 3) = -2.5382986f; effectiveness(0, 5) = -5.4366393f;
	// effectiveness(1, 0) =  1.0073246f; effectiveness(1, 1) = -2.117229f; effectiveness(1, 2) =  0.53718674f; effectiveness(1, 3) =  2.5382986f; effectiveness(1, 5) = -5.4366393f;
	// effectiveness(2, 0) =  1.0073246f; effectiveness(2, 1) = 1.5191985f; effectiveness(2, 2) = -0.53718674f; effectiveness(2, 3) = -2.5382986f; effectiveness(2, 5) = -5.4366393f;
	// effectiveness(3, 0) = -1.0073246f; effectiveness(3, 1) = -2.117229f; effectiveness(3, 2) = -0.53718674f; effectiveness(3, 3) =  2.5382986f; effectiveness(3, 5) = -5.4366393f;
	
	// 0도
	// effectiveness 행렬 각 요소에 내가 원하는 값 반복문 없이 설정
	effectiveness(0, 0) = -1.0073246f;  effectiveness(0, 1) =  1.0073246f;  effectiveness(0, 2) =  1.0073246f;  effectiveness(0, 3) = -1.0073246f;
	effectiveness(1, 0) =  1.8164136f;  effectiveness(1, 1) = -1.8164136f;  effectiveness(1, 2) =  1.8164136f;  effectiveness(1, 3) = -1.8164136f;
	effectiveness(2, 0) =  0.53718674f; effectiveness(2, 1) =  0.53718674f; effectiveness(2, 2) = -0.53718674f; effectiveness(2, 3) = -0.53718674f;
	effectiveness(3, 0) = -2.5382986f;  effectiveness(3, 1) =  2.5382986f;  effectiveness(3, 2) = -2.5382986f;  effectiveness(3, 3) =  2.5382986f;
	effectiveness(5, 0) = -5.4366393f;  effectiveness(5, 1) = -5.4366393f;  effectiveness(5, 2) = -5.4366393f;  effectiveness(5, 3) = -5.4366393f;
	
	// 30도
	// effectiveness(0, 0) = -1.0073246f;  effectiveness(0, 1) =  1.0073246f;  effectiveness(0, 2) =  1.0073246f;  effectiveness(0, 3) = -1.0073246f;
	// effectiveness(1, 0) =  1.591985f;   effectiveness(1, 1) = -2.117229f;   effectiveness(1, 2) =  1.591985f;   effectiveness(1, 3) = -2.117229f;
	// effectiveness(2, 0) =  0.53718674f; effectiveness(2, 1) =  0.53718674f; effectiveness(2, 2) = -0.53718674f; effectiveness(2, 3) = -0.53718674f;
	// effectiveness(3, 0) = -2.5382986f;  effectiveness(3, 1) =  2.5382986f;  effectiveness(3, 2) = -2.5382986f;  effectiveness(3, 3) =  2.5382986f;
	// effectiveness(5, 0) = -5.4366393f;  effectiveness(5, 1) = -5.4366393f;  effectiveness(5, 2) = -5.4366393f;  effectiveness(5, 3) = -5.4366393f;	



	matrix::geninv(effectiveness, mix);

	// pseudo inverse 전치되도록 출력

	printf("transpose of pseudo inverse:\n");
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 16; j++) {
			printf("%f ", double(mix(j, i)));
		}
		printf("\n");
	}


	printf("\neffectiveness:\n");
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 16; j++) {
			printf("%f ", double(effectiveness(i, j)));
		}
		printf("\n");
	}


	method.setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, 16, false);

	control_sp(1) = 1.f;
	control_sp(5) = -0.3f;

	method.setControlSetpoint(control_sp);
	method.allocate();
	method.clipActuatorSetpoint();
	actuator_sp = method.getActuatorSetpoint();
	control_allocated_expected = method.getAllocatedControl();

	printf("\nactuator_sp:\n");
	for (int i = 0; i < 16; i++) {
		printf("%f ", double(actuator_sp(i)));
	}
	printf("\n");

	// EXPECT_EQ(actuator_sp, actuator_sp_expected);
	// EXPECT_EQ(control_allocated, control_allocated_expected);
}
