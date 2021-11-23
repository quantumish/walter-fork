#include <numbers>
#include <iostream>
#include "bot.h"
using namespace std::numbers;

int main() {
    Bot b{};
    b.rotate(pi/2, pi/3);
    b.execute();
}
