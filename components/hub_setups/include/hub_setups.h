#pragma once

// The ways a hub can be fitted. Each runs its loop on hub_board's tick.

// The XIAO alone owns bus, LED and button. Returns when an ATtiny powers up late and answers.
void run_xiao_alone();

// The XIAO with the ATtiny, which owns the button; never returns.
void run_xiao_with_attiny();
