// The largest single correction a move cap may allow, in arcminutes: the page,
// the host's $M= and the controller all take it from here, and $M? reports it
// so a host reads the limit instead of assuming it.
#pragma once

constexpr double kMaxMoveCapArcmin = 180.0;
