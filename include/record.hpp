#pragma once

#include <cstdint>

// records driving + subsystems, prints it back as lemlib code.
// RIGHT+X starts/stops it, RIGHT+Y prints the last one. stops after a min.
// the recording sits in ram til the program restarts, so you can drive
// unplugged, then read it off the brain screen with RIGHT+Y.

namespace record {

const uint32_t RECORD_MAX_MS = 60000;

void toggle();
void print_last(); // dump whatever is in memory to the terminal
bool active();
int count();
uint32_t elapsed();

// call every opcontrol loop with what you just told the motors to do
void update(int lift_cmd, int intake_cmd, int spin_cmd);

// the generated code as text, so the brain screen can show it with no cable
const int LINE_LEN = 80;
int line_count();
const char *line(int i);

}
