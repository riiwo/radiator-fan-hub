#pragma once

// Test console on USB for the hub test jig, in the hub-test program only:
// "flow <C>" fixes the flow input, "flow probe" goes back to the probe; "resend on|off";
// "mode <name>" while the XIAO owns the modes; "nodes <1-50>" shortens the frame.

// Installs the USB serial driver so the console can read.
void hub_console_start();

// Reads waiting characters and runs each complete line.
void hub_console_poll();
