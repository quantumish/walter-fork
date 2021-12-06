#include <iostream>
#include "bot.h"

int main() {
    Bot b{};
    b.move_y(2, 0.5);
    b.rotate(0.1*pi, pi/3);
    b.execute();    
}
