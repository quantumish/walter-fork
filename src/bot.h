#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <functional>
using namespace UNITREE_LEGGED_SDK;

/** 
 * Lambda that takes in current state and outputs a command 
 * and the amount of time it should be issued for. 
 * @see Bot::execute()
 */
using Instruction = std::function<std::pair<HighCmd, float>(HighState)>;

/**
 * Class representing the Unitree A1.
 */
class Bot
{
    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
    std::vector<Instruction> instructions;
    
    void UDPRecv();
    void UDPSend();
    void RobotControl();    
public:
    Bot(): safe(LeggedType::A1), udp(HIGHLEVEL){
        udp.InitCmdData(cmd);
    }

    /**
     * Begins issuing instructions to the robot.
     * Initializes the UDP threads and the control loop, then forces main thread to wait.
     * Only call when all instructions have been specified.
     */
    void execute();
    
    /** 
     * Adds an instruction to walk forward.
     * @param d Distance (in meters) to walk forward.
     */
    void forward(float distance);
    void rotate(float theta);
};
