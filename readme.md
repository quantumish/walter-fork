
# walter 

## About
A high level API to controlling a Unitree A1 robot.

## Building
This project depends on `lcm` and the Conan package manager. It will only compile on Linux machines as the Unitree SDK is solely in the form of a specifc static object file and cannot be linked properly on other operating systems do to its format. 

```
mkdir build && cd build
conan install .. --build
cmake ..
make -C ..
```

The binary is then located at `bin/walter` and documentation at `docs/html/index.html`.

## Usage
After initializing the `Bot` class, specify a series of commands through its members. These commands are *not* run in real time, but run sequentially as soon as `Bot::execute()` is called.

A sample usage would look like:
```cpp
#include "bot.h"

int main() {
	Bot bot{};
	bot.move_y(5, 0.2);
    bot.smooth_move({4,3}, 3);
	bot.execute();	
}
```
