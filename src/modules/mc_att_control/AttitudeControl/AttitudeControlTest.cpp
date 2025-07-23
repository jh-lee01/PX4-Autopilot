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

#include <gtest/gtest.h>
#include <AttitudeControl.hpp>
#include <mathlib/math/Functions.hpp>
#include <fstream>
#include <sstream>

using namespace matrix;

TEST(AttitudeControlTest, AllZeroCase)
{
	AttitudeControl attitude_control;
	Vector3f rate_setpoint = attitude_control.update(Quatf());
	EXPECT_EQ(rate_setpoint, Vector3f());
}

class AttitudeControlConvergenceTest : public ::testing::Test
{
public:
	AttitudeControlConvergenceTest()
	{
		_attitude_control.setProportionalGain(Vector3f(.5f, .6f, .3f), .4f);
		_attitude_control.setRateLimit(Vector3f(100.f, 100.f, 100.f));
	}

	void checkConvergence()
	{
		int i; // need function scope to check how many steps
		Vector3f rate_setpoint(1000.f, 1000.f, 1000.f);

		_attitude_control.setAttitudeSetpoint(_quat_goal, 0.f);

		for (i = 100; i > 0; i--) {
			// run attitude control to get rate setpoints
			const Vector3f rate_setpoint_new = _attitude_control.update(_quat_state);
			// rotate the simulated state quaternion according to the rate setpoint
			_quat_state = _quat_state * Quatf(AxisAnglef(rate_setpoint_new));
			_quat_state = -_quat_state; // produce intermittent antipodal quaternion states to test against unwinding problem

			// expect the error and hence also the output to get smaller with each iteration
			if (rate_setpoint_new.norm() >= rate_setpoint.norm()) {
				break;
			}

			rate_setpoint = rate_setpoint_new;
		}

		EXPECT_EQ(_quat_state.canonical(), _quat_goal.canonical());
		// it shouldn't have taken longer than an iteration timeout to converge
		EXPECT_GT(i, 0);
	}

	AttitudeControl _attitude_control;
	Quatf _quat_state;
	Quatf _quat_goal;
};

TEST_F(AttitudeControlConvergenceTest, AttitudeControlConvergence)
{
	const int inputs = 8;

	const Quatf QArray[inputs] = {
		Quatf(),
		Quatf(0, 1, 0, 0),
		Quatf(0, 0, 1, 0),
		Quatf(0, 0, 0, 1),
		Quatf(0.698f, 0.024f, -0.681f, -0.220f),
		Quatf(-0.820f, -0.313f, 0.225f, -0.423f),
		Quatf(0.599f, -0.172f, 0.755f, -0.204f),
		Quatf(0.216f, -0.662f, 0.290f, -0.656f)
	};

	for (int i = 0; i < inputs; i++) {
		for (int j = 0; j < inputs; j++) {
			printf("--- Input combination: %d %d\n", i, j);
			_quat_state = QArray[i];
			_quat_goal = QArray[j];
			_quat_state.normalize();
			_quat_goal.normalize();
			checkConvergence();
		}
	}
}

TEST(AttitudeControlTest, YawWeightScaling)
{
	// GIVEN: default tuning and pure yaw turn command
	AttitudeControl attitude_control;
	const float yaw_gain = 2.8f;
	const float yaw_sp = .1f;
	Quatf pure_yaw_attitude(cosf(yaw_sp / 2.f), 0, 0, sinf(yaw_sp / 2.f));
	attitude_control.setProportionalGain(Vector3f(6.5f, 6.5f, yaw_gain), .4f);
	attitude_control.setRateLimit(Vector3f(1000.f, 1000.f, 1000.f));
	attitude_control.setAttitudeSetpoint(pure_yaw_attitude, 0.f);

	// WHEN: we run one iteration of the controller
	Vector3f rate_setpoint = attitude_control.update(Quatf());

	// THEN: no actuation in roll, pitch
	EXPECT_EQ(Vector2f(rate_setpoint), Vector2f());
	// THEN: actuation error * gain in yaw
	EXPECT_NEAR(rate_setpoint(2), yaw_sp * yaw_gain, 1e-4f);

	// GIVEN: additional corner case of zero yaw weight
	attitude_control.setProportionalGain(Vector3f(6.5f, 6.5f, yaw_gain), 0.f);
	// WHEN: we run one iteration of the controller
	rate_setpoint = attitude_control.update(Quatf());
	// THEN: no actuation (also no NAN)
	EXPECT_EQ(rate_setpoint, Vector3f());
}

TEST(AttitudeControlTest, ReplayFromLog)
{
    AttitudeControl attitude_control;
    attitude_control.setProportionalGain(Vector3f(9.f, 5.f, 4.5f), 0.4f);
    attitude_control.setRateLimit(Vector3f(220.f, 220.f, 200.f));

    std::ifstream att_file("/home/carroll/PX4-Autopilot/build/px4_sitl_default/csv_output/log_vehicle_attitude_0.csv");
    std::ifstream att_sp_file("/home/carroll/PX4-Autopilot/build/px4_sitl_default/csv_output/log_vehicle_attitude_setpoint_0.csv");

    if (!att_file.is_open() || !att_sp_file.is_open()) {
        FAIL() << "Failed to open log files for reading.";
        return;
    }

    // 첫 줄(헤더) 스킵
    std::string line_att, line_sp;
    getline(att_file, line_att);
    getline(att_sp_file, line_sp);

    while (getline(att_file, line_att) && getline(att_sp_file, line_sp)) {
        std::stringstream ss_att(line_att);
        std::stringstream ss_sp(line_sp);

        std::vector<std::string> tokens_att;
        std::vector<std::string> tokens_sp;
        std::string token;

        // attitude CSV 파싱
        while (std::getline(ss_att, token, ',')) {
            tokens_att.push_back(token);
        }
        // setpoint CSV 파싱
        while (std::getline(ss_sp, token, ',')) {
            tokens_sp.push_back(token);
        }

        if (tokens_att.size() < 6 || tokens_sp.size() < 6) {
            std::cerr << "Skipping invalid line.\n";
            continue;
        }

        double ts_att = std::stod(tokens_att[0]);
        float q0  = std::stof(tokens_att[2]);
        float q1  = std::stof(tokens_att[3]);
        float q2  = std::stof(tokens_att[4]);
        float q3  = std::stof(tokens_att[5]);

        double ts_sp = std::stod(tokens_sp[0]);
        float qd0 = std::stof(tokens_sp[2]);
        float qd1 = std::stof(tokens_sp[3]);
        float qd2 = std::stof(tokens_sp[4]);
        float qd3 = std::stof(tokens_sp[5]);

        Quatf quat_state(q0, q1, q2, q3);
        Quatf quat_setpoint(qd0, qd1, qd2, qd3);

        // NaN 체크 및 정규화
        if (!PX4_ISFINITE(quat_state.norm()) || quat_state.norm() < 1e-6f) {
            printf("[WARN] Invalid quat_state at t=%.3f\n", ts_att * 1e-6);
            continue;
        }
        if (!PX4_ISFINITE(quat_setpoint.norm()) || quat_setpoint.norm() < 1e-6f) {
            printf("[WARN] Invalid quat_setpoint at t=%.3f\n", ts_sp * 1e-6);
            continue;
        }
        quat_state.normalize();
        quat_setpoint.normalize();

        attitude_control.setAttitudeSetpoint(quat_setpoint, 0.f);
        Vector3f rate_setpoint = attitude_control.update(quat_state);

        printf("[t=%.3f] rate_sp: [%.3f, %.3f, %.3f]\n",
               ts_att * 1e-6,
               (double)rate_setpoint(0),
               (double)rate_setpoint(1),
               (double)rate_setpoint(2));
    }

    EXPECT_TRUE(false);
}


// TEST(AttitudeControlTest, PitchResponse)
// {
// 	AttitudeControl attitude_control;
// 	attitude_control.setProportionalGain(Vector3f(9.f, 5.f, 4.5f), .4f);
// 	attitude_control.setRateLimit(Vector3f(220.f, 220.f, 200.f));

// 	Quatf quat_state;
// 	quat_state.setIdentity();

// 	float roll_deg = 0.0f;
// 	float pitch_deg = -1.1576f;
// 	float yaw_deg = 0.0f;

// 	float roll_rad = math::radians(roll_deg);
// 	float pitch_rad = math::radians(pitch_deg);
// 	float yaw_rad = math::radians(yaw_deg);

// 	Quatf quat_setpoint(Eulerf(roll_rad, pitch_rad, yaw_rad));
// 	attitude_control.setAttitudeSetpoint(quat_setpoint, 0.f);

// 	Vector3f rate_setpoint = attitude_control.update(quat_state);

// 	printf("[Fixed Pitch] RollSP=%.4f deg, PitchSP=%.4f deg, YawSP=%.4f deg -> rate_setpoint: [%.4f, %.4f, %.4f]\n",
// 		(double)roll_deg, (double)pitch_deg, (double)yaw_deg,
// 		(double)rate_setpoint(0),
// 		(double)rate_setpoint(1),
// 		(double)rate_setpoint(2));

// 	EXPECT_TRUE(false);
// }
