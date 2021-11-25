#include <numbers>
#include <iostream>
#include "bot.h"
using std::numbers::pi;

int main() {
    Bot b{};
    b.move_y(2, 0.5);
    b.smooth_move({4,3}, 3);
    b.execute();
}
