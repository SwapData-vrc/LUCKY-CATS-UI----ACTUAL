# toggle-y coder notes

Implemented per user answer: new `toggle_motor` on port 18 (green, guess),
plain Y (RIGHT not held) flips `toggle_on`; on -> move(127) every loop,
off -> brake() (coast brake mode). Also fixed the lift log line to "ports 20,19".

Files: src/subsystems.cpp (65-66), include/subsystems.hpp (35),
src/main.cpp (98, 118, 179-181, 280-289). Not committed.

## Build

`pros` and `arm-none-eabi-g++` are not on PATH. Plain `make` failed with
`arm-none-eabi-g++: command not found` (Error 127).

Re-ran with the PROS VS Code extension toolchain on PATH:

    I="$HOME/Library/Application Support/Kiro/User/globalStorage/sigbots.pros/install"
    PATH="$I/pros-toolchain-macos/bin:$I/pros-cli-macos:$PATH" \
    PROS_TOOLCHAIN="$I/pros-toolchain-macos" pros make

Result: exit 0. All 5 sources compiled [OK], linked, bin/monolith.bin
created. 0 warnings, 0 errors.

No physical robot test possible.
