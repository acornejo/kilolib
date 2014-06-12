#include "kilolib.h"

void setup() {
    // put your setup code here, will be run once at the beginning
}

void loop() {
    // put your main code here, will be run repeatedly
    set_color(RGB(1,0,0));
    delay(100);
    set_color(RGB(0,1,0));
    delay(100);
}

int main() {
    kilo_init();
    kilo_start(setup, loop);

    return 0;
}
