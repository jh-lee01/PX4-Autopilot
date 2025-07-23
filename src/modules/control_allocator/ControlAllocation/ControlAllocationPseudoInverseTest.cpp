#define MODULE_NAME "ControlAllocationPseudoInverseTest"
#include <gtest/gtest.h>
#include <ControlAllocationPseudoInverse.hpp>
#include <matrix/matrix/math.hpp>
#include <iostream>
#include <iomanip>

using namespace matrix;

TEST(ControlAllocationPseudoInverseTest, SingleStep)
{
    ControlAllocationPseudoInverse method;

    // --- Effectiveness Matrix 설정 (간단 예시: 4개 모터) ---
    Matrix<float, 6, 16> effectiveness{};
    effectiveness(0,0) = -1.00758f; effectiveness(0,1) =  1.05834f;
    effectiveness(0,2) =  1.05834f; effectiveness(0,3) = -1.00758f;
    effectiveness(1,0) =  1.51920f; effectiveness(1,1) = -2.11723f;
    effectiveness(1,2) =  1.51920f; effectiveness(1,3) = -2.11723f;
    effectiveness(2,0) =  0.53664f; effectiveness(2,1) =  0.42791f;
    effectiveness(2,2) = -0.42791f; effectiveness(2,3) = -0.53664f;
    effectiveness(3,0) = -2.53830f; effectiveness(3,1) =  2.53830f;
    effectiveness(3,2) = -2.53830f; effectiveness(3,3) =  2.53830f;
    effectiveness(5,0) = -5.43664f; effectiveness(5,1) = -5.43664f;
    effectiveness(5,2) = -5.43664f; effectiveness(5,3) = -5.43664f;

    Vector<float, 16> actuator_trim{};
    Vector<float, 16> linearization_point{};
    method.setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, 16, false);

    // --- 테스트용 단일 입력 (rate_control_output.csv에서 가져옴) ---
    float torque_x = -0.098654f;
    float torque_y = 0.714495f;
    float torque_z = -0.003042f;
    float thrust_z = -0.25015f;  // 예시로 thrust_setpoint_0.csv에서 가져온 값 사용
	// time_s,dt,rate_x,rate_y,rate_z,rate_sp_x,rate_sp_y,rate_sp_z,torque_x,torque_y,torque_z
	// 4594.530245,0.014777,0.144274,-0.093653,0.038884,0.009617,0.451762,0.029816,-0.098654,0.714495,-0.003042

    // ControlAllocation 입력 구성
    Vector<float, 6> control_sp;
    control_sp.zero();
    control_sp(2) = thrust_z;    // Fz
    control_sp(3) = torque_x;
    control_sp(4) = torque_y;
    control_sp(5) = torque_z;

    // Control Allocation 실행
    method.setControlSetpoint(control_sp);
    method.allocate();
    // method.clipActuatorSetpoint();
    Vector<float, 16> actuator_sp = method.getActuatorSetpoint();

    // 결과 출력
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "[RAW] actuator_sp: ";
    for (int i = 0; i < 4; i++) {
        std::cout << actuator_sp(i) << (i < 3 ? ", " : "\n");
    }

    EXPECT_TRUE(false);
}