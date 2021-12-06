# Unitree A1 SDK

<p align="center">
<img src="https://i.imgur.com/MATyvmI.jpeg" style="margin-left: auto; margin-right: auto; display: block; width: 256px;">
</p>
<h1 align="center">Using the Unitree A1 SDK</h1>

Although `walter` should cover most usecases, this is a guide on how to directly use and understand the Unitree SDK yourself since its documentation is somewhat sparse.

## General Architecture
Your code itself does not actually make the robot do anything, it'll just send commands to a UDP server located on the robot and *that'll* make it do something. Your interaction with the bot (at least concerning movement) is done entirely through this UDP server (it relays state to you too).

In order to create your own `Bot` class it'll need to have a `Safety` (which specifies operation mode?), a `UDP` (which contains send and recv buffers that are relayed to the bot's UDP server), some tick length `dt`, and (optionally) a counter `t` that is updated so you can have time-dependent behavior in your code.

At runtime, the class will need to instantiate three `LoopFunc`s (essentially just threads that repeatedly run a function). Two of them, `UDPSend` and `UDPRecv` respectively, will repeatedly call wrappers around some of the methods of the `UDP` struct in the `Bot` class (which has the effect of constantly recieving the current state into a buffer inside the `UDP` struct and constantly sending the command located in another buffer in the `UDP` struct). Finally, you'll have a `Control`

<p align="center">
<img src="https://i.imgur.com/J1DmMYb.png" style="margin-left: auto; margin-right: auto; display: block; width: 50%;">
<div style="text-align: center">
	<i>Illustrated diagram of interaction between Bot class and threads.</i>
</div>

To actually instantiate these `LoopFunc`s you can call the following constructor:
```cpp
LoopFunc(std::string name, float dt, boost::function<void()> f)
```
The name is essentially useless and is just used for logs. There's also another constructor that lets you set CPU core affinity, but that may not be entirely useful:
```cpp
LoopFunc(std::string name, float dt, int cpu, boost::function<void()> f)
```
Disclaimer: if you actually hunt through the headers you'll realize I'm simplifying here: the `boost::function` is typedef'd, `dt` is called `period`, etc. This is mostly meant to be an *intuitive* guide, not a rigorous one.

## HighState and HighCmd
You'll likely be primarily operating in high-level mode, so here's a slightly edited version of the struct definitions in the header files for your reference:
```cpp
struct HighState {
	uint8_t levelFlag; // Operation mode? 
	uint16_t commVersion;
	uint16_t robotID;
	uint32_t SN; // ???
	uint8_t bandWidth;
	uint8_t mode; // Also mode?
	struct {
		float quaternion[4];  // quaternion, normalized, (w,x,y,z)
		float gyroscope[3];  // angular velocity （unit: rad/s)
		float accelerometer[3];  // m/(s2)
		float rpy[3];  // euler angle（unit: rad)
		int8_t temperature;
	} imu;
	float forwardSpeed;
	float sideSpeed;
	float rotateSpeed;
	float bodyHeight;
	float updownSpeed;  // speed of stand up or squat down
	float forwardPosition;  // front or rear displacement, an integrated number form kinematics function, usually drift
	float sidePosition;  // left or right displacement, an integrated number form kinematics function, usually drift
	Cartesian footPosition2Body[4];  // foot position relative to body
	Cartesian footSpeed2Body[4]; // foot speed relative to body
	int16_t footForce[4];
	int16_t footForceEst[4];
	uint32_t tick;  // reference real-time from motion controller (unit: us)
	uint8_t wirelessRemote[40];
	uint32_t reserve; // Reserved.
	uint32_t crc; // Potentially for Hamming-code-style error detection? Don't touch.
}
```

```cpp
struct HighCmd {
	uint8_t levelFlag;
	uint16_t commVersion;
	uint16_t robotID;
	uint32_t SN;
	uint8_t bandWidth;
	uint8_t mode;                      // 0:idle, default stand      1:forced stand     2:walk continuously
	float forwardSpeed;                // speed of move forward or backward, scale: -1~1
	float sideSpeed;                   // speed of move left or right, scale: -1~1
	float rotateSpeed;	               // speed of spin left or right, scale: -1~1
	float bodyHeight;                  // body height, scale: -1~1
	float footRaiseHeight;             // foot up height while walking (unavailable now)
	float yaw;                         // unit: radian, scale: -1~1
	float pitch;                       // unit: radian, scale: -1~1
	float roll;                        // unit: radian, scale: -1~1
	LED led[4];                        // Controls the LEDs on the legs. Don't touch.
	uint8_t wirelessRemote[40];
	uint8_t AppRemote[40];
	uint32_t reserve;
	uint32_t crc;
};
```

```cpp
struct Cartesian {
	float x;
	float y;
	float z;
}
```

```cpp
struct LED {
	uint8_t r;
	uint8_t g;
	uint8_t b;
}
```

## LowState and LowCmd
```cpp
struct LowState {
	uint8_t levelFlag;                 // flag to distinguish high level or low level
	uint16_t commVersion;
	uint16_t robotID;
	uint32_t SN; 
	uint8_t bandWidth;
	IMU imu;
	MotorState motorState[20];
	int16_t footForce[4];              // force sensors
	int16_t footForceEst[4];           // force sensors
	uint32_t tick;                     // reference real-time from motion controller (unit: us)
	uint8_t wirelessRemote[40];        // wireless commands
	uint32_t reserve;
	uint32_t crc;
}
```

```cpp
struct LowCmd {
	uint8_t levelFlag;
	uint16_t commVersion;
	uint16_t robotID;
	uint32_t SN;
	uint8_t bandWidth;
	MotorCmd motorCmd[20];
	LED led[4];
	uint8_t wirelessRemote[40];
	uint32_t reserve;
	uint32_t crc;
}
```

```cpp
struct MotorState {
	uint8_t mode;                      // motor working mode 
	float q;                           // current angle (unit: radian)
	float dq;                          // current velocity (unit: radian/second)
	float ddq;                         // current acc (unit: radian/second*second)
	float tauEst;                      // current estimated output torque (unit: N.m)
	float q_raw;                       // current angle (unit: radian)
	float dq_raw;                      // current velocity (unit: radian/second)
	float ddq_raw;
	int8_t temperature;                // current temperature (temperature conduction is slow that leads to lag)
	uint32_t reserve[2];
};
```

```cpp
struct MotorCmd {
	uint8_t mode;                      // desired working mode
	float q;                           // desired angle (unit: radian) 
	float dq;                          // desired velocity (unit: radian/second)
	float tau;                         // desired output torque (unit: N.m)
	float Kp;                          // desired position stiffness (unit: N.m/rad )
	float Kd;                          // desired velocity stiffness (unit: N.m/(rad/s) )
	uint32_t reserve[3];
}
```
