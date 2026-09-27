#include "asiair_parser.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace asiair {
namespace {

// Start of the value of "key" at or after from, or nullptr. Matching the opening
// quote keeps "state" from matching inside "pa_state" and "code" inside "state_code".
const char *valueOf(const char *from, const char *key) {
  char pattern[40];
  std::snprintf(pattern, sizeof pattern, "\"%s\":", key);
  const char *at = std::strstr(from, pattern);
  return at ? at + std::strlen(pattern) : nullptr;
}

bool stringValue(const char *from, const char *key, char *out, size_t len) {
  const char *v = valueOf(from, key);
  if (!v || *v != '"') return false;
  v++;
  const char *end = std::strchr(v, '"');
  if (!end) return false;
  size_t n = static_cast<size_t>(end - v);
  if (n >= len) n = len - 1;
  std::memcpy(out, v, n);
  out[n] = 0;
  return true;
}

bool numberValue(const char *from, const char *key, double &out) {
  const char *v = valueOf(from, key);
  if (!v) return false;
  char *end = nullptr;
  out = std::strtod(v, &end);
  return end != v;
}

int intValue(const char *from, const char *key, int fallback) {
  double d = 0;
  return numberValue(from, key, d) ? static_cast<int>(d) : fallback;
}

}  // namespace

bool parseLine(const char *line, AsiairEvent &out) {
  out = AsiairEvent();
  if (!line) return false;
  while (*line == ' ' || *line == '\t') line++;
  size_t len = std::strlen(line);
  while (len && (line[len - 1] == '\r' || line[len - 1] == '\n' || line[len - 1] == ' ')) len--;
  if (len < 2 || line[0] != '{' || line[len - 1] != '}') return false;
  char name[24];
  if (!stringValue(line, "Event", name, sizeof name)) return false;

  char page[16] = "";
  stringValue(line, "page", page, sizeof page);
  bool pa = std::strcmp(page, "pa") == 0;
  if (std::strcmp(name, "3PPA") == 0) out.kind = EventKind::Pa3ppa;
  else if (pa && std::strcmp(name, "Exposure") == 0) out.kind = EventKind::PaExposure;
  else if (pa && std::strcmp(name, "PlateSolve") == 0) out.kind = EventKind::PaSolve;
  else return true;

  stringValue(line, "state", out.state, sizeof out.state);
  out.stateCode = intValue(line, "state_code", -1);
  out.retryCnt = intValue(line, "retry_cnt", 0);
  out.code = intValue(line, "code", 0);
  if (const char *error = valueOf(line, "pa_error")) {
    out.hasError = *error == '{' && numberValue(error, "x", out.xDeg) && numberValue(error, "y", out.yDeg);
  }
  return true;
}

}  // namespace asiair
