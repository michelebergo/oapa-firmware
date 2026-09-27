#include "json_util.h"

#include <cstdio>

bool jsonEscape(const char *in, char *out, size_t len) {
  size_t pos = 0;
  for (const char *c = in; *c; ++c) {
    char chunk[7];
    unsigned char ch = static_cast<unsigned char>(*c);
    if (ch == '"' || ch == '\\') {
      chunk[0] = '\\';
      chunk[1] = static_cast<char>(ch);
      chunk[2] = 0;
    } else if (ch < 0x20) {
      std::snprintf(chunk, sizeof chunk, "\\u%04x", ch);
    } else {
      chunk[0] = static_cast<char>(ch);
      chunk[1] = 0;
    }
    for (const char *k = chunk; *k; ++k) {
      if (pos + 1 >= len) return false;
      out[pos++] = *k;
    }
  }
  if (pos >= len) return false;
  out[pos] = 0;
  return true;
}
