#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <Eigen/Dense>

using namespace UNITREE_LEGGED_SDK;

class Instruction {
protected:
    Instruction* next;
    virtual HighCmd update(HighState state);
};

class Forward : Instruction {
    float d;
    
    Forward(float meters) :d{meters} {}
    
    std::pair<HighCmd, float> update(HighState state)
    {
	HighCmd cmd {0};
	cmd.mode = 2;
	float v_0 = state.forwardSpeed;
        float t = d/v_0;
	return cmd. 
	// float a_0 = state.imu.accelerometer;
	
    }
};

class Rotate : Instruction {
    HighCmd update(HighState state)
    {
	// 
    }
};


class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::A1), udp(level){
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
};


void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotControl() 
{
    udp.GetRecv(state);
    

    cmd.forwardSpeed = 0.0f;
    cmd.sideSpeed = 0.0f;
    cmd.rotateSpeed = 0.0f;
    cmd.bodyHeight = 0.0f;

    cmd.mode = 0;      // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.roll  = 0;
    cmd.pitch = 0;
    cmd.yaw = 0;

    if(motiontime>1000 && motiontime<1500){
        cmd.mode = 1;
        cmd.roll = 0.5f;
    }

    if(motiontime>1500 && motiontime<2000){
        cmd.mode = 1;
        cmd.pitch = 0.3f;
    }

    if(motiontime>2000 && motiontime<2500){
        cmd.mode = 1;
        cmd.yaw = 0.3f;
    }

    if(motiontime>2500 && motiontime<3000){
        cmd.mode = 1;
        cmd.bodyHeight = -0.3f;
    }

    if(motiontime>3000 && motiontime<3500){
        cmd.mode = 1;
        cmd.bodyHeight = 0.3f;
    }

    if(motiontime>3500 && motiontime<4000){
        cmd.mode = 1;
        cmd.bodyHeight = 0.0f;
    }

    if(motiontime>4000 && motiontime<5000){
        cmd.mode = 2;
    }

    if(motiontime>5000 && motiontime<8500){
        cmd.mode = 2;
        cmd.forwardSpeed = 0.1f; // -1  ~ +1
    }

    if(motiontime>8500 && motiontime<12000){
        cmd.mode = 2;
        cmd.forwardSpeed = -0.2f; // -1  ~ +1
    }

    if(motiontime>12000 && motiontime<16000){
        cmd.mode = 2;
        cmd.rotateSpeed = 0.3f;   // turn
    }

    if(motiontime>16000 && motiontime<20000){
        cmd.mode = 2;
        cmd.rotateSpeed = -0.3f;   // turn
    }

    if(motiontime>20000 ){
        cmd.mode = 1;
    }

    udp.SetSend(cmd);
}
