#include <iostream>
#include "bot.h"

int main() {
    Bot b{};
    b.move_y(2, 0.5);
    b.smooth_move({4,3}, 0.1);
    b.execute();
}
