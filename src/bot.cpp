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
    int prev_end = 0;
    for (Instruction i : instructions) {
	auto out = i(state);
	if (motiontime > prev_end && motiontime < prev_end + out.second) {
	    cmd = out.first;
	}
    }
    udp.SetSend(cmd);
}

void Bot::forward(float d) {
    instructions.push_back(
        [d](HighState state) {
	    HighCmd cmd {0};
	    cmd.mode = 2;
	    float v_0 = state.forwardSpeed;
	    float t = d/v_0;
	    return std::pair<HighCmd, double>(cmd, t);
	}
    );
}
