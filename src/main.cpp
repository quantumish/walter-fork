#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <Eigen/Dense>

using namespace UNITREE_LEGGED_SDK;

class Instruction {
public:
    virtual std::pair<HighCmd, float> update(HighState state) {return {{0}, 0};};
};

class Forward : public Instruction {
public:
    float d;
    
    Forward(float meters) :d{meters} {}
    
    std::pair<HighCmd, float> update(HighState state)
    {
	HighCmd cmd {0};
	cmd.mode = 2;
	float v_0 = state.forwardSpeed;
        float t = d/v_0;
	return {cmd, t};
    }
    
};

// class Rotate : Instruction {
//     std::HighCmd update(HighState state)
//     {
// 	// 
//     }
// };

class Bot
{
private:    
    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
    std::vector<Instruction*> instructions;
    
    void UDPRecv();
    void UDPSend();
    void RobotControl();    
public:
    Bot(uint8_t level): safe(LeggedType::A1), udp(level){
        udp.InitCmdData(cmd);
    }

    void forward(float distance);
    void rotate(float theta);
};


void Bot::UDPRecv()
{
    udp.Recv();
}

void Bot::UDPSend()
{  
    udp.Send();
}

void Bot::RobotControl() 
{
    motiontime += 2;
    udp.GetRecv(state);
    int prev_end = 0;
    for (Instruction* i : instructions) {
	auto out = i->update(state);
	if (motiontime > prev_end && motiontime < prev_end + out.second) {
	    cmd = out.first;
	}
    }
    udp.SetSend(cmd);
}

void Bot::forward(float distance) { 
    instructions.push_back(new Forward(distance)); // Super hacky.
}

int main() {    
    Bot b(HIGHLEVEL);
    b.forward(5);
}
