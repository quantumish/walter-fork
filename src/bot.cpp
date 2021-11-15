#include "bot.h"

void Bot::UDPRecv()
{
    udp.Recv();
}

void Bot::UDPSend()
{  
    udp.Send();
}

void Bot::execute() {
    LoopFunc loop_control("control_loop", dt, boost::bind(&Bot::RobotControl, this));
    LoopFunc loop_udpSend("udp_send", dt, 3, boost::bind(&Bot::UDPSend, this));
    LoopFunc loop_udpRecv("udp_recv", dt, 3, boost::bind(&Bot::UDPRecv, this));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while (true) {
	sleep(1);
    }
}

void Bot::RobotControl() 
{
    motiontime += 2;
    udp.GetRecv(state);   
    InstructionData out;
    if (!executing) {
	out = instructions[index](state, state);
	initial_state = state;
	executing = true;
    } else {
        out = instructions[index](initial_state, state);
    }    
    if (!out.done) {
	cmd = out.cmd;
    } else {
	executing = false;
	index++;
    }
    udp.SetSend(cmd);
}

void Bot::forward(float d, float v) {
    instructions.push_back(
        [d](HighState initial_state, HighState state) {
	    HighCmd cmd {0};
	    cmd.mode = 2;
	    float v_0 = state.forwardSpeed;
	    float t = d/v_0;
	    return InstructionData{cmd, true};
	}
    );
}

void Bot::rotate(float theta) {
}
