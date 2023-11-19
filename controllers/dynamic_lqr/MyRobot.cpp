

#include "MyRobot.hpp"
#define NeedAngle 90

MyRobot::MyRobot() : balance_angle{-0.0064}, jumpState{JUMP_IDLE}, isJumpInTheAir(false)
{
    time_step = getBasicTimeStep();
    mkeyboard = getKeyboard(), mkeyboard->enable(time_step);
    camera = getCamera("camera"), camera->enable(time_step);
    gyro = getGyro("gyro"), gyro->enable(time_step);
    imu = getInertialUnit("imu"), imu->enable(time_step);
    gps = getGPS("gps"), gps->enable(time_step);
    encoder_wheelL = getPositionSensor("encoder_wheelL"), encoder_wheelL->enable(time_step);
    encoder_wheelR = getPositionSensor("encoder_wheelR"), encoder_wheelR->enable(time_step);
    encoder_BL = getPositionSensor("encoder_BL"), encoder_BL->enable(time_step);
    encoder_BR = getPositionSensor("encoder_BR"), encoder_BR->enable(time_step);
    encoder_FL = getPositionSensor("encoder_FL"), encoder_FL->enable(time_step);
    encoder_FR = getPositionSensor("encoder_FR"), encoder_FR->enable(time_step);
    L_Wheelmotor = getMotor("L_Motor"), BL_legmotor = getMotor("BL_Motor"), FL_legmotor = getMotor("FL_Motor");
    R_Wheelmotor = getMotor("R_Motor"), BR_legmotor = getMotor("BR_Motor"), FR_legmotor = getMotor("FR_Motor");
    L_Wheelmotor->setVelocity(0), R_Wheelmotor->setVelocity(0);
    L_Wheelmotor->setPosition(INFINITY), R_Wheelmotor->setPosition(INFINITY);
    BL_legmotor->setPosition(0), BR_legmotor->setPosition(0), FL_legmotor->setPosition(0), FR_legmotor->setPosition(0);

    BL_legmotor->enableTorqueFeedback(time_step), BR_legmotor->enableTorqueFeedback(time_step), FL_legmotor->enableTorqueFeedback(time_step), FR_legmotor->enableTorqueFeedback(time_step);
    L_Wheelmotor->enableTorqueFeedback(time_step), R_Wheelmotor->enableTorqueFeedback(time_step);
    // 参数初始化
    roll.set = 0, yaw.set = 0, yaw.set_dot = 0,
    velocity.set = 0, velocity.now = 0;
    acc_up_max = 1.0, acc_down_max = 1.0;
    // 调参
    turn_pid.update(3.0, 0, 0.03, 3); // 针对角速度进行PD控制
    split_pid.update(100.0, 0.0, 10, HipTorque_MaxLimit);
    roll_pid.update(1000, 0.0, 10, 25);
    invPendulumInAir_pid.update(10000.0, 10.0, 10, HipTorque_MaxLimit);
    wheelBrakeInAir_pid.update(10000.0, 10.0, 1, DriveTorque_MaxLimit);

    K_coeff << -228.312861923588, 294.032841402069, -161.859980835154, -27.9124607725149,
        -311.553508160858, 488.738545499858, -288.194998275156, 69.8532732024956,
        -25.0754494709052, 30.5969720475325, -27.3011613956695, -7.41107450495795,
        -93.0949035697954, 139.631825126946, -79.3257068867355, 19.0350639940176,
        -33.1309413829780, 40.3900983299054, -17.2457434417403, -7.36223473018761,
        -106.980642246657, 153.627662686586, -82.3368141566720, 17.7880546399025,
        -18.1356341071563, 20.8763808035375, -9.34978742039602, -10.0834547221280,
        -145.995128360192, 202.696600747869, -104.633505889388, 21.7346468775176,
        -378.233687962070, 543.155810337850, -291.104598164008, 62.8902703006772,
        468.542266417460, -571.202248474433, 243.891642697369, 104.117722046717,
        -13.4938391590358, 19.1993789061047, -10.4541636935974, 2.47975499342254,
        13.6341049836551, -16.7207310260419, 7.21326317048111, 3.11249772591918;
}
MyRobot::~MyRobot()
{
}

void MyRobot::MyStep()
{
    if (step(time_step) == -1)
        exit(EXIT_SUCCESS);
}
/**
 * @brief: 毫秒级延时
 * @author: Dandelion
 * @Date: 2023-03-27 16:35:24
 * @param {int} ms
 * @return {*}
 */
void MyRobot::Wait(int ms)
{
    float start_time = getTime();
    float s = ms / 1000.0;
    while (s + start_time >= getTime())
        MyStep();
}

/**
 * @brief Motor command with immediate protection
 * @author: Tim
 */
void MyRobot::command_motor(void)
{
    // Emergency Brake
    if ((pitch.now >= PI / 3.0) || (pitch.now <= -PI / 3.0))
    {
        // Angle is too dangerous, stop control
        HipTorque_MaxLimit = 0;
        DriveTorque_MaxLimit = 0;
    }
    leg_L.TL_set = Limit(leg_L.TL_set, HipTorque_MaxLimit, -HipTorque_MaxLimit);
    leg_L.TR_set = Limit(leg_L.TR_set, HipTorque_MaxLimit, -HipTorque_MaxLimit);
    leg_R.TL_set = Limit(leg_R.TL_set, HipTorque_MaxLimit, -HipTorque_MaxLimit);
    leg_R.TR_set = Limit(leg_R.TR_set, HipTorque_MaxLimit, -HipTorque_MaxLimit);
    leg_L.TWheel_set = Limit(leg_L.TWheel_set, DriveTorque_MaxLimit, -DriveTorque_MaxLimit);
    leg_R.TWheel_set = Limit(leg_R.TWheel_set, DriveTorque_MaxLimit, -DriveTorque_MaxLimit);

    BL_legmotor->setTorque(leg_L.TL_set);
    FL_legmotor->setTorque(leg_L.TR_set);
    BR_legmotor->setTorque(leg_R.TL_set);
    FR_legmotor->setTorque(leg_R.TR_set);

    L_Wheelmotor->setTorque(leg_L.TWheel_set);
    R_Wheelmotor->setTorque(leg_R.TWheel_set);
}
/**
 * @brief: 状态更新
 * @author: Dandelion
 * @Date: 2023-04-01 00:10:38
 * @param {LegClass} *leg_sim
 * @param {LegClass} *leg_L
 * @param {LegClass} *leg_R
 * @param {float} pitch
 * @param {float} pitch_dot
 * @param {float} dt
 * @param {float} v_set
 * @return {*}
 */
void MyRobot::status_update(LegClass *leg_sim, LegClass *leg_L, LegClass *leg_R,
                            const DataStructure pitch, const DataStructure roll, const DataStructure yaw,
                            const float dt, float v_set)
{
    // 获取当前机器人状态信息
    leg_L->dis.now = encoder_wheelL->getValue() * 0.05;
    leg_R->dis.now = encoder_wheelR->getValue() * 0.05;
    leg_sim->dis.now = (leg_L->dis.now + leg_R->dis.now) / 2.0;

    leg_L->dis.dot = (leg_L->dis.now - leg_L->dis.last) * 1000.f / time_step;
    leg_R->dis.dot = (leg_R->dis.now - leg_R->dis.last) * 1000.f / time_step;
    leg_sim->dis.dot = (leg_sim->dis.now - leg_sim->dis.last) * 1000.f / time_step;

    static float leg_L_dis_last_dot = 0;
    static float leg_R_dis_last_dot = 0;
    leg_L->dis.ddot = (leg_L->dis.dot - leg_L_dis_last_dot) * 1000.f / time_step;
    leg_R->dis.ddot = (leg_R->dis.dot - leg_R_dis_last_dot) * 1000.f / time_step;
    leg_L_dis_last_dot = leg_L->dis.dot;
    leg_R_dis_last_dot = leg_R->dis.dot;

    leg_L->dis.last = leg_L->dis.now;
    leg_R->dis.last = leg_R->dis.now;
    leg_sim->dis.last = leg_sim->dis.now;

    leg_L->TL_now = BL_legmotor->getTorqueFeedback();
    leg_L->TR_now = FL_legmotor->getTorqueFeedback();
    leg_R->TL_now = BR_legmotor->getTorqueFeedback();
    leg_R->TR_now = FR_legmotor->getTorqueFeedback();
    leg_L->TWheel_now = L_Wheelmotor->getTorqueFeedback();
    leg_R->TWheel_now = L_Wheelmotor->getTorqueFeedback();

    // 角度更新，统一从右视图看吧
    leg_L->angle1 = 2.0 / 3.0 * PI - encoder_FL->getValue();
    leg_L->angle4 = 1.0 / 3.0 * PI + encoder_BL->getValue();
    leg_L->ForwardKinematics(leg_L->angle1, leg_L->angle4, pitch.now);
    leg_L->angle0.dot = (leg_L->angle0.now - leg_L->angle0.last) / dt;

    leg_R->angle1 = 2.0 / 3.0 * PI - encoder_FR->getValue();
    leg_R->angle4 = 1.0 / 3.0 * PI + encoder_BR->getValue();
    leg_R->ForwardKinematics(leg_R->angle1, leg_R->angle4, pitch.now);
    leg_R->angle0.dot = (leg_R->angle0.now - leg_R->angle0.last) / dt;

    leg_sim->angle1 = (leg_L->angle1 + leg_R->angle1) / 2;
    leg_sim->angle4 = (leg_L->angle4 + leg_R->angle4) / 2;
    leg_sim->ForwardKinematics(leg_sim->angle1, leg_sim->angle4, pitch.now);
    leg_sim->angle0.dot = (leg_sim->angle0.now - leg_sim->angle0.last) / dt;

    // pitch_dot = Deadzone(pitch_dot, 0.01);
    // leg_sim->dis.dot = Deadzone(leg_sim->dis.dot, 0.01);
    // leg_sim->angle0_dot = Deadzone(leg_sim->angle0_dot, 0.1);

    leg_L->L0.dot = (leg_L->L0.now - leg_L->L0.last) / dt;
    leg_L->L0.last = leg_L->L0.now;
    leg_R->L0.dot = (leg_R->L0.now - leg_R->L0.last) / dt;
    leg_R->L0.last = leg_R->L0.now;
}

void MyRobot::inv_pendulum_ctrl(LegClass *leg_sim, LegClass *leg_L, LegClass *leg_R,
                                const DataStructure pitch,
                                const float dt, float v_set)
{
    if (isJumpInTheAir)
    {
        leg_L->TWheel_set = wheelBrakeInAir_pid.compute(0, 0, leg_L->dis.dot, leg_L->dis.ddot, dt);
        leg_R->TWheel_set = wheelBrakeInAir_pid.compute(0, 0, leg_R->dis.dot, leg_R->dis.ddot, dt);

        // @TODO: predict best simLeg angle before landing based on initial velocity
        float out_invPendulumInAir = invPendulumInAir_pid.compute(0, 0, leg_sim->angle0.now, leg_sim->angle0.dot, dt);
        leg_L->Tp_set = -out_invPendulumInAir; // 这里的正负号没研究过，完全是根据仿真工程上得来的（其实这样也更快）
        leg_R->Tp_set = -out_invPendulumInAir;
    }
    else
    {
        // 计算K矩阵数据，根据L0.now拟合得到
        Matrix<float, 2, 1> Matrix_u;

        // 根据当前杆长计算K矩阵的各项值
        for (size_t col = 0; col < 6; col++)
        {
            /* code */
            for (size_t row = 0; row < 2; row++)
            {
                /* code */
                int num = col * 2 + row;
                leg_sim->K(row, col) = K_coeff(num, 0) * pow(leg_sim->L0.now, 3) +
                                       K_coeff(num, 1) * pow(leg_sim->L0.now, 2) +
                                       K_coeff(num, 2) * leg_sim->L0.now +
                                       K_coeff(num, 3);
            }
        }

        // 状态更新
        leg_sim->dis.set += v_set * dt;
        leg_sim->Xd << 0, 0, leg_sim->dis.set, 0, 0, 0;
        leg_sim->X << leg_sim->angle0.now, leg_sim->angle0.dot, leg_sim->dis.now, leg_sim->dis.dot, pitch.now, pitch.dot;
        Matrix_u = leg_sim->K * (leg_sim->Xd - leg_sim->X); // u = K(Xd-X);
        leg_sim->TWheel_set = Matrix_u(0, 0);
        leg_sim->Tp_set = Matrix_u(1, 0);

        // output
        leg_L->TWheel_set = leg_sim->TWheel_set / 2.0;
        leg_R->TWheel_set = leg_sim->TWheel_set / 2.0;
        leg_L->Tp_set = -leg_sim->Tp_set / 2.0;
        leg_R->Tp_set = -leg_sim->Tp_set / 2.0;
    }
}

void MyRobot::torque_ctrl(LegClass *leg_sim, LegClass *leg_L, LegClass *leg_R,
                          const DataStructure pitch, const DataStructure roll, const DataStructure yaw,
                          const float dt, float v_set)
{
    if (jumpState == JUMP_LAUNCH)
    {
        // Force is directly set in JUMP_LAUNCH state
    }
    else if (isJumpInTheAir)
    {
        // flying
        leg_L->F_set = 0;
        leg_R->F_set = 0;
    }
    else
    {
        leg_L->F_set = -UNLOADED_ROBOT_HALF_WEIGHT;
        leg_R->F_set = -UNLOADED_ROBOT_HALF_WEIGHT;
    }

    /****************** Hip joint torque adjustment by PID ******************/
    float out_spilt = split_pid.compute(0, 0, leg_L->angle0.now - leg_R->angle0.now, leg_L->angle0.dot - leg_R->angle0.dot, dt);
    leg_L->Tp_set -= out_spilt; // 这里的正负号没研究过，完全是根据仿真工程上得来的（其实这样也更快）
    leg_R->Tp_set += out_spilt;

    if (isJumpInTheAir == false)
    {
        float out_roll = roll_pid.compute(roll.set, 0, roll.now, roll.dot, dt);
        leg_L->F_set -= out_roll;
        leg_R->F_set += out_roll;
    }

    // leg length adjustment
    if (jumpState == JUMP_LAUNCH)
    {
        // No leg length PID control in JUMP_LAUNCH state
    }
    else
    {
        float out_L;
        float out_R;
        if (isJumpInTheAir)
        {
            out_L = leg_L->supportFInAir_pid.compute(leg_L->L0.set, 0, leg_L->L0.now, leg_L->L0.dot, dt);
            out_R = leg_R->supportFInAir_pid.compute(leg_R->L0.set, 0, leg_R->L0.now, leg_R->L0.dot, dt);
        }
        else
        {
            out_L = leg_L->supportF_pid.compute(leg_L->L0.set, 0, leg_L->L0.now, leg_L->L0.dot, dt);
            out_R = leg_R->supportF_pid.compute(leg_R->L0.set, 0, leg_R->L0.now, leg_R->L0.dot, dt);
        }
        leg_L->F_set -= out_L;
        leg_R->F_set -= out_R;
    }

    // Virtual Model Control (VMC)
    Matrix<float, 2, 1> Torque_L, Torque_R;
    Torque_L = leg_L->VMC(leg_L->F_set, leg_L->Tp_set);
    leg_L->TL_set = -Torque_L(0, 0);
    leg_L->TR_set = Torque_L(1, 0);

    Torque_R = leg_R->VMC(leg_R->F_set, leg_R->Tp_set);
    leg_R->TL_set = -Torque_R(0, 0);
    leg_R->TR_set = Torque_R(1, 0);

    // maintain torque scales after saturation
    float TL_max = max(max(max(abs(leg_L->TL_set), abs(leg_L->TR_set)), abs(leg_R->TL_set)), abs(leg_R->TR_set));
    if (TL_max > HipTorque_MaxLimit)
    {
        float TL_normalizer = HipTorque_MaxLimit / TL_max;
        leg_L->TL_set *= TL_normalizer;
        leg_L->TR_set *= TL_normalizer;
        leg_R->TL_set *= TL_normalizer;
        leg_R->TR_set *= TL_normalizer;
    }

    /****************** Drive motor torque adjustment by PID ******************/
    if (isJumpInTheAir == false)
    {
        float out_turn = turn_pid.compute(yaw.set_dot, 0, yaw.dot, yaw.ddot, dt);
        leg_L->TWheel_set -= out_turn;
        leg_R->TWheel_set += out_turn;
    }
}

void MyRobot::run()
{
    time = getTime();
    // Define   as the current time (you may need to adjust this based on your specific requirements).
    // float t_clk = getTime(); // Get the current time, adjust this as needed.

    static int last_key;

    velocity.now = gps->getSpeed();
    pitch.now = imu->getRollPitchYaw()[1];
    pitch.dot = gyro->getValues()[2];
    roll.now = imu->getRollPitchYaw()[0];
    roll.dot = gyro->getValues()[0];
    yaw_get = imu->getRollPitchYaw()[2];

    static float yaw_dot_last = 0;
    yaw.dot = gyro->getValues()[1];
    yaw.ddot = (yaw.dot - yaw_dot_last) / (time_step * 0.001);
    yaw_dot_last = yaw.dot;
    // float robot_x = gps->getValues()[0];
    pitch.now -= balance_angle; // 得到相对于平衡pitch的角度

    if (yaw_get - yaw.last > 1.5 * PI)
        yaw.now += yaw_get - yaw.last - 2 * PI;
    else if (yaw.now - yaw.last < -1.5 * PI)
        yaw.now += yaw_get - yaw.last + 2 * PI;
    else
        yaw.now += yaw_get - yaw.last;

    yaw.last = yaw.now;
    if (time == 0)
        yaw.set = yaw.now;

    // 时序更新
    int key = mkeyboard->getKey();
    while (key > 0)
    {
        Keyboard kboard;
        switch (key)
        {
        case kboard.UP:
            velocity.set += 0.02f;
            break;
        case kboard.DOWN:
            velocity.set -= 0.02f;
            break;
        case kboard.RIGHT:
            if (ABS(velocity.set) > 0.1)
                yaw.set_dot -= 0.01f;
            else
                yaw.set_dot -= 0.02f;
            break;
        case kboard.LEFT:
            if (ABS(velocity.set) > 0.1)
                yaw.set_dot += 0.01f;
            else
                yaw.set_dot += 0.02f;
            break;
        case kboard.END:
            velocity.set = 0;
            yaw.set_dot = 0;
            break;
        case 'W':
            leg_L.L0.set += 0.0002f;
            leg_R.L0.set += 0.0002f;
            break;
        case 'S':
            leg_L.L0.set -= 0.0002f;
            leg_R.L0.set -= 0.0002f;
            break;
        case 'A':
            roll.set -= 0.002f;
            break;
        case 'D':
            roll.set += 0.002f;
            break;
        case 'O':
            sampling_flag = 1;
            sampling_time = time;
            break;
        case 'J': // 'J' key to trigger the jump
            if ((jumpState == JUMP_IDLE) && (isJumpInTheAir == false))
            {
                jumpState = JUMP_INIT;
            }
            break;
        case ' ':
            if (last_key != key)
                stop_flag = True;
            break;
        }
        last_key = key;
        key = mkeyboard->getKey();
    }

    /*测试用的，追踪一个持续4s的速度期望*/
    // if (sampling_flag == 1)
    // {
    //     if (time - sampling_time < 1)
    //     {
    //         velocity.set = 0;
    //     }
    //     else if (time - sampling_time < 5)
    //     {
    //         // velocity.set += acc_up_max * time_step * 0.001;
    //         velocity.set = 1.0;
    //     }
    //     // else if (time < 6)
    //     // {
    //     //     velocity.set = 3;
    //     // }
    //     // else if (time < 9)
    //     // {
    //     //     velocity.set -= acc_down_max * time_step * 0.001;
    //     // }
    //     else
    //     {
    //         velocity.set = 0;
    //         sampling_flag = 0;
    //     }
    // }

    jumpManager();

    leg_L.L0.set = Limit(leg_L.L0.set, LegL0_Max, LegL0_Min);
    leg_R.L0.set = Limit(leg_R.L0.set, LegL0_Max, LegL0_Min);

    velocity.set = Limit(velocity.set, 2.5, -2.5);
    yaw.set_dot = Limit(yaw.set_dot, 2.0, -2.0);
    yaw.set += yaw.set_dot * time_step * 0.001;

    status_update(&leg_simplified, &leg_L, &leg_R, this->pitch, this->roll, this->yaw, time_step * 0.001, velocity.set);

    velocity.set = limitVelocity(velocity.set, leg_simplified.L0.now);
    inv_pendulum_ctrl(&leg_simplified, &leg_L, &leg_R, this->pitch, time_step * 0.001, velocity.set);
    torque_ctrl(&leg_simplified, &leg_L, &leg_R, this->pitch, this->roll, this->yaw, time_step * 0.001, velocity.set);

    command_motor();

    // ofstream outfile;
    // outfile.open("data2.dat", ios::trunc);
    // outfile << time << ' ' << pitch << ' ' << disL_dot << ' ' << robot_x << ' ' << L_Torque << endl;
    // outfile.close();
}
/**
 * @brief: 限制速度
 * @author: Dandelion
 * @Date: 2023-04-18 19:45:57
 * @param {float} L0
 * @return {*}
 */

void MyRobot::jumpManager(void)
{
    switch (jumpState)
    {
    case JUMP_INIT:
    {
        isJumpInTheAir = false;

        // Raise the legs to initiate the jump
        leg_L.L0.set = LegL0_Min;
        leg_R.L0.set = LegL0_Min;

        yaw.set_dot = 0;
        // @TODO: jump with non-zero roll
        roll.set = 0;

        jumpState = JUMP_CHARGE;
        std::cout << "Jump state is: " << jumpState << std::endl;
        break;
    }
    case JUMP_CHARGE:
    {
        if ((abs(roll.now) < 0.04f) &&
            (min(leg_L.L0.now, leg_R.L0.now) <= LegL0_Min_Threshold))
        {
            isJumpInTheAir = false;
            jumpState = JUMP_LAUNCH;
            std::cout << "Jump state is: " << jumpState << std::endl;
        }
        break;
    }
    case JUMP_LAUNCH:
    {
        if (max(leg_L.L0.now, leg_R.L0.now) >= LegL0_Max_Threshold)
        // if ((getTime() - starttime > duration) // ((leg_L.angle2 >= 1.3*0.8) || (leg_R.angle2 >= 1.3*0.8))
        {
            isJumpInTheAir = true;

            // Back to PID control
            leg_L.L0.set = LegL0_Min_Threshold;
            leg_R.L0.set = LegL0_Min_Threshold;
            // Avoid integral windup
            leg_L.supportFInAir_pid.clear();
            leg_R.supportFInAir_pid.clear();
            leg_L.supportF_pid.clear();
            leg_R.supportF_pid.clear();

            jumpState = JUMP_SHRINK;
            std::cout << "Jump state is: " << jumpState << std::endl;
        }
        else
        {
            // @TODO: Direct decaying torque control
            float initSupportF = -300; // arbitrary large value
            float decaySupportF = initSupportF * (1 - exp((max(leg_L.L0.now, leg_R.L0.now) - LegL0_Max_Threshold) / JumpTorqueDecayTau));
            leg_L.F_set = decaySupportF;
            leg_R.F_set = decaySupportF;
        }
        break;
    }
    case JUMP_SHRINK:
    {
        // Shrink until landing
        if (max(abs(leg_L.F_set), abs(leg_R.F_set)) >= UNLOADED_ROBOT_HALF_WEIGHT)
        {
            isJumpInTheAir = false;
            jumpState = JUMP_IDLE;
            std::cout << "Jump state is: " << jumpState << std::endl;
        }
        break;
    }
    case JUMP_IDLE:
    {
        // safety guard
        isJumpInTheAir = false;
        break;
    }
    default:
    {
        // should not reach here
        isJumpInTheAir = false;
        break;
    }
    }
}

float MyRobot::limitVelocity(float speed_set, float L0)
{
    float speed_max = -30 * L0 + 10.7;
    return (speed_set > speed_max) ? speed_max : speed_set;
}
