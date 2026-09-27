// Minimal header-only test harness for the host build (no external deps).
#pragma once
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

struct TtCase { const char *name; std::function<void()> fn; };
inline std::vector<TtCase> &ttRegistry() { static std::vector<TtCase> r; return r; }
inline int &ttFailures() { static int f = 0; return f; }
struct TtRegister {
  TtRegister(const char *n, std::function<void()> f) { ttRegistry().push_back({n, f}); }
};

#define TT_CAT2(a, b) a##b
#define TT_CAT(a, b) TT_CAT2(a, b)
#define TEST(name) \
  static void name(); \
  static TtRegister TT_CAT(ttreg_, name)(#name, name); \
  static void name()
#define CHECK(cond) \
  do { if (!(cond)) { std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); ++ttFailures(); } } while (0)
#define CHECK_EQ_STR(actual, expected) \
  do { std::string a_(actual), e_(expected); \
    if (a_ != e_) { std::printf("  FAIL %s:%d: got \"%s\" want \"%s\"\n", __FILE__, __LINE__, a_.c_str(), e_.c_str()); ++ttFailures(); } } while (0)
#define CHECK_NEAR(a, b, tol) CHECK(std::fabs((double)(a) - (double)(b)) <= (tol))

inline int ttRunAll() {
  for (auto &c : ttRegistry()) {
    int before = ttFailures();
    c.fn();
    std::printf("%s %s\n", ttFailures() == before ? "PASS" : "FAIL", c.name);
  }
  std::printf("\n%zu tests, %d failed checks\n", ttRegistry().size(), ttFailures());
  return ttFailures() ? 1 : 0;
}
