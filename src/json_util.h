// Shared JSON helper for the pure formatters.
#pragma once
#include <cstddef>

// JSON-escapes in into out; false when it does not fit.
bool jsonEscape(const char *in, char *out, size_t len);
