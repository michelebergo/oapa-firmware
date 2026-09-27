// Host-side stand-in for the Arduino core: just enough for the verbatim 1.2.2
// protocol region to compile and behave the same on the PC.
#pragma once
#include <ctype.h>
#include <math.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>

#define HIGH 1
#define LOW 0
#define INPUT 1
#define OUTPUT 3
#define SERIAL_8N1 0

template <typename T, typename L, typename H>
T constrain(T v, L lo, H hi) { return v < lo ? (T)lo : (v > hi ? (T)hi : v); }

class String {
 public:
  String(const char *s = "") : s_(s ? s : "") {}
  explicit String(const std::string &s) : s_(s) {}
  unsigned int length() const { return (unsigned int)s_.size(); }
  char charAt(unsigned int i) const { return i < s_.size() ? s_[i] : 0; }
  int indexOf(char c) const { return pos(s_.find(c)); }
  int indexOf(const char *t) const { return pos(s_.find(t)); }
  String substring(unsigned int from) const {
    return from >= s_.size() ? String("") : String(s_.substr(from));
  }
  String substring(unsigned int from, unsigned int to) const {
    if (from > to) std::swap(from, to);
    if (from >= s_.size()) return String("");
    return String(s_.substr(from, to - from));
  }
  bool startsWith(const char *p) const { return s_.rfind(p, 0) == 0; }
  long toInt() const { return std::atol(s_.c_str()); }
  float toFloat() const { return (float)std::atof(s_.c_str()); }
  void trim() {
    size_t b = s_.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) { s_.clear(); return; }
    size_t e = s_.find_last_not_of(" \t\r\n");
    s_ = s_.substr(b, e - b + 1);
  }
  String &operator+=(char c) { s_ += c; return *this; }
  const char *c_str() const { return s_.c_str(); }

 private:
  static int pos(size_t p) { return p == std::string::npos ? -1 : (int)p; }
  std::string s_;
};

class Stream {};

class HardwareSerial : public Stream {
 public:
  std::string out;  // everything written; println appends "\r\n" like Arduino
  std::string in;
  void begin(unsigned long, int = 0, int = -1, int = -1) {}
  int available() { return (int)in.size(); }
  int read() {
    if (in.empty()) return -1;
    char c = in[0];
    in.erase(0, 1);
    return c;
  }
  void print(const char *s) { out += s; }
  void print(const String &s) { out += s.c_str(); }
  void print(float v, int digits) {
    char b[32];
    std::snprintf(b, sizeof b, "%.*f", digits, v);
    out += b;
  }
  void println(const char *s = "") { out += s; out += "\r\n"; }
  void println(const String &s) { out += s.c_str(); out += "\r\n"; }
};

// The real Arduino.h declares the sketch entry points; without them a name
// clash with a global `loop` (for example a namespace) compiles only on the PC.
void setup();
void loop();

inline HardwareSerial Serial;
inline HardwareSerial Serial1;
inline unsigned long g_hostMillis = 0;
inline unsigned long millis() { return g_hostMillis; }
inline void delay(unsigned long) {}
inline int digitalRead(int) { return LOW; }
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
