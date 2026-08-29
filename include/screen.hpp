#pragma once

// The brain screen: pick a routine on the left, watch it drive on the right.
//
// Plain LVGL. Every widget here has been verified to render on this robot,
// which is why it is hand-written rather than a library.

namespace screen {

// Waits for the display, builds the UI, and starts the redraw task. Call once,
// first thing in initialize(). If the display never comes up it gives up
// quietly and the robot still drives.
void init();

// Runs whichever routine is selected. Call from autonomous().
void run_selected();

// Starts the selected routine on its own task, for the RUN button and the
// controller chord. Refuses under competition control -- at an event the only
// thing that starts autonomous is the field. Returns false if it refused.
bool request_run();

// Stops a run started by request_run().
void request_stop();

bool running();

// The current selection, for the terminal.
const char* selected_name();

} // namespace screen
