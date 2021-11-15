
# walter 

## About
A high level API to controlling a Unitree A1 robot.

## Usage
After initializing the `Bot` class, specify a series of commands through its members. These commands are *not* run in real time, but run sequentially as soon as `Bot::execute()` is called.

A sample usage would look like:
```cpp
#include "bot.h"

int main() {
	Bot bot{};
	bot.forward(5);
	bot.rotate(30);
	bot.forward(2);
	bot.execute();	
}
```
