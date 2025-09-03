#define MODULE_NAME "mc_rate_control"
#include <gtest/gtest.h>
#include "MulticopterRateControl.hpp"
#include <matrix/matrix/math.hpp>
#include <mathlib/math/Functions.hpp>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>

using namespace matrix;

TEST(MulticopterRateControlTest, ReplayFromLog)
{
    RateControl rate_control;

    // PID gain 설정 (로그에서 사용된 값과 동일하게 설정)
    Vector3f P_gain(0.33f, 0.85f, 0.55f);
    Vector3f I_gain(0.05f, 0.1f, 0.15f);
    Vector3f D_gain(0.003f, 0.003f, 0.0f);

    rate_control.setGains(P_gain, I_gain, D_gain);
    rate_control.setIntegratorLimit(Vector3f(0.3f, 0.6f, 0.3f));

    // 로그 파일 읽기
    std::ifstream rates_file("/home/carroll/PX4-Autopilot/build/px4_sitl_default/csv_output/log_vehicle_angular_velocity_0.csv");
    std::ifstream sp_file("/home/carroll/PX4-Autopilot/build/px4_sitl_default/csv_output/log_vehicle_rates_setpoint_0.csv");

    ASSERT_TRUE(rates_file.is_open() && sp_file.is_open());

    std::string line_rates, line_sp;
    getline(rates_file, line_rates); // 헤더 스킵
    getline(sp_file, line_sp);

    double prev_time = 0.0;

    // 결과를 CSV로 저장
    std::ofstream output_file("/home/carroll/PX4-Autopilot/build/px4_sitl_default/rate_control_output.csv");
    ASSERT_TRUE(output_file.is_open());
    output_file << "time_s,dt,rate_x,rate_y,rate_z,rate_sp_x,rate_sp_y,rate_sp_z,torque_x,torque_y,torque_z\n";

    // 콘솔 디버깅용 헤더 출력
    printf("[DEBUG] Starting ReplayFromLog Test\n");
    printf("[DEBUG] Gains: P=[%.3f, %.3f, %.3f], I=[%.3f, %.3f, %.3f], D=[%.3f, %.3f, %.3f]\n",
           (double)P_gain(0), (double)P_gain(1), (double)P_gain(2),
           (double)I_gain(0), (double)I_gain(1), (double)I_gain(2),
           (double)D_gain(0), (double)D_gain(1), (double)D_gain(2));

    while (getline(rates_file, line_rates) && getline(sp_file, line_sp)) {
        std::stringstream ss_rates(line_rates), ss_sp(line_sp);

        double ts_rates, ts_sample;
        float p, q, r, wx_dot, wy_dot, wz_dot;
        char comma;

        // vehicle_angular_velocity 로그 파싱
        ss_rates >> ts_rates >> comma >> ts_sample >> comma
                 >> p >> comma >> q >> comma >> r >> comma
                 >> wx_dot >> comma >> wy_dot >> comma >> wz_dot;

       float roll_sp, pitch_sp, yaw_sp;
       float thrust_x, thrust_y, thrust_z, reset_integral;

        // vehicle_rates_setpoint 로그 파싱
        ss_sp >> ts_rates >> comma 
              >> roll_sp >> comma >> pitch_sp >> comma >> yaw_sp >> comma 
              >> thrust_x >> comma >> thrust_y >> comma >> thrust_z >> comma
              >> reset_integral;

        float dt = (prev_time > 0.0) ? static_cast<float>((ts_rates - prev_time) * 1e-6) : 0.01f;
        prev_time = ts_rates;

        Vector3f rates(p, q, r);
        Vector3f rate_sp(roll_sp, pitch_sp, yaw_sp);
        Vector3f angular_accel(wx_dot, wy_dot, wz_dot);

        // RateControl 실행
        Vector3f torque_out = rate_control.update(rates, rate_sp, angular_accel, dt, false);

        // 디버그 출력
        printf("[t=%.6f s] dt=%.6f rates=[%.4f, %.4f, %.4f] sp=[%.4f, %.4f, %.4f] torque=[%.4f, %.4f, %.4f]\n",
               ts_rates * 1e-6,
               (double)dt,
               (double)p, (double)q, (double)r,
               (double)rate_sp(0), (double)rate_sp(1), (double)rate_sp(2),
               (double)torque_out(0), (double)torque_out(1), (double)torque_out(2));

        // CSV로 출력
        output_file << std::fixed << std::setprecision(6)
                    << (ts_rates * 1e-6) << ","
                    << dt << ","
                    << p << "," << q << "," << r << ","
                    << rate_sp(0) << "," << rate_sp(1) << "," << rate_sp(2) << ","
                    << torque_out(0) << "," << torque_out(1) << "," << torque_out(2) << "\n";
    }

    output_file.close();
    rates_file.close();
    sp_file.close();

    EXPECT_TRUE(true);
}