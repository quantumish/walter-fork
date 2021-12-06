<p align="center">
<img src="https://i.imgur.com/G4j0Xuj.jpeg" style="margin-left: auto; margin-right: auto; display: block;">
</p>
<h1 align="center">walter</h1>
<div align="center" style="text-align:center;">
	A high level API to controlling a Unitree A1 robot.<br>
	<img src="https://img.shields.io/github/workflow/status/RoboticsTeam4904/walter/ubuntu-latest"> <img src="https://img.shields.io/maintenance/yes/2021">
</div>

<h2>Installing</h2>
Clone the repository and its submodules:

```
git clone https://github.com/RoboticsTeam4904/walter.git
git submodule init
git submodule update
```

<h2>Building</h2>
This project depends on the Conan package manager. It will only compile on Linux machines as the Unitree SDK is solely in the form of a specific static object file and cannot be linked properly on other operating systems do to its format.

```
mkdir build && cd build
conan create ../lcm-recipe/ -o lcm:shared=True
conan install .. --build
cmake ..
make -C ..
```

The binary is then located at `bin/walter` and documentation at `docs/html/index.html`.

<h2>Usage</h2>
After initializing the `Bot` class, specify a series of commands through its members. These commands are *not* run in real time, but run sequentially as soon as `Bot::execute()` is called.

A sample usage would look like:
```cpp
#include "bot.h"

int main() {
	Bot bot{};
	bot.move_y(5, 0.2);
	bot.rotate(pi/10, pi/3);
	bot.execute();
}
```
