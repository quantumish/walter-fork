#include <numbers>
#include <iostream>
#include "bot.h"
using namespace std::numbers;

int main() {
    Bot b{};
    b.move_y(2, 0.5);
    b.smooth_move({4,3}, 3);
    b.execute();
}
