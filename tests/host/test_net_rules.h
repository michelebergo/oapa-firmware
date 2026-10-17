// Network mode decisions: never locked out, never stuck in setup.
#pragma once
using netrules::Action;
using netrules::Inputs;
using netrules::Mode;

static Inputs in(bool creds, bool connected, uint32_t now, uint32_t since, uint32_t lastRetry = 0, int clients = 0) {
  return Inputs{creds, connected, now, since, lastRetry, clients};
}

TEST(Net_NoCredentials_GoesToSetup_AndStays) {
  CHECK(netrules::decide(Mode::Connecting, in(false, false, 0, 0)) == Action::EnterSetup);
  CHECK(netrules::decide(Mode::Setup, in(false, false, 500000, 0)) == Action::None);
}

TEST(Net_Connecting_BecomesStationOnLink) {
  CHECK(netrules::decide(Mode::Connecting, in(true, true, 1200, 0)) == Action::EnterStation);
}

TEST(Net_Connecting_FallsBackToSetupAfter30s) {
  CHECK(netrules::decide(Mode::Connecting, in(true, false, 29999, 0)) == Action::None);
  CHECK(netrules::decide(Mode::Connecting, in(true, false, 30000, 0)) == Action::EnterSetup);
}

TEST(Net_Station_LinkLost_StartsTheTimeoutAgain) {
  CHECK(netrules::decide(Mode::Station, in(true, true, 90000, 1000)) == Action::None);
  CHECK(netrules::decide(Mode::Station, in(true, false, 90000, 1000)) == Action::BackToConnecting);
}

TEST(Net_Setup_RetriesEvery60s_OnlyWithoutPhoneAttached) {
  CHECK(netrules::decide(Mode::Setup, in(true, false, 59999, 0, 0, 0)) == Action::None);
  CHECK(netrules::decide(Mode::Setup, in(true, false, 60000, 0, 0, 0)) == Action::RetryStation);
  CHECK(netrules::decide(Mode::Setup, in(true, false, 60000, 0, 0, 1)) == Action::None);
}

TEST(Net_Setup_LeavesWhenRetrySucceeds) {
  CHECK(netrules::decide(Mode::Setup, in(true, true, 61000, 0, 60000, 0)) == Action::EnterStation);
}

TEST(Net_TimeoutsSurviveMillisWrap) {
  CHECK(netrules::decide(Mode::Connecting, in(true, false, 0x00001000u, 0xFFFFF000u)) == Action::None);
  CHECK(netrules::decide(Mode::Connecting, in(true, false, 0x00008000u, 0xFFFFF000u)) == Action::EnterSetup);
}

TEST(Net_HostnameFromMac_IsLowercaseHexOfTheLastThreeBytes) {
  const uint8_t mac[6] = {0x24, 0x6f, 0x28, 0xa1, 0xb2, 0xc3};
  char name[12] = {0};
  CHECK(netrules::hostnameFromMac(mac, name, sizeof name));
  CHECK_EQ_STR(name, "oapa-a1b2c3");
}

TEST(Net_HostnameFromMac_PadsZeroesAndSeparatesTwoBoards) {
  const uint8_t first[6] = {0x24, 0x6f, 0x28, 0x00, 0x0f, 0x05};
  const uint8_t second[6] = {0x24, 0x6f, 0x28, 0x00, 0x0f, 0x06};
  char a[12] = {0}, b[12] = {0};
  CHECK(netrules::hostnameFromMac(first, a, sizeof a));
  CHECK(netrules::hostnameFromMac(second, b, sizeof b));
  CHECK_EQ_STR(a, "oapa-000f05");
  CHECK(std::strcmp(a, b) != 0);
}

TEST(Net_HostnameFromMac_RefusesASmallBuffer) {
  const uint8_t mac[6] = {0x24, 0x6f, 0x28, 0xa1, 0xb2, 0xc3};
  char name[11] = "untouched";
  CHECK(!netrules::hostnameFromMac(mac, name, sizeof name));
  CHECK_EQ_STR(name, "untouched");
  CHECK(!netrules::hostnameFromMac(nullptr, name, sizeof name));
}

TEST(Net_CredentialRules_FollowWpa2) {
  CHECK(netrules::validCredentials("ASIAIR_1234", "12345678"));
  CHECK(netrules::validCredentials("OpenNet", ""));
  CHECK(!netrules::validCredentials("", "12345678"));
  CHECK(!netrules::validCredentials(std::string(33, 'a').c_str(), "12345678"));
  CHECK(!netrules::validCredentials("ASIAIR", "1234567"));
  CHECK(!netrules::validCredentials("ASIAIR", std::string(64, 'p').c_str()));
  CHECK(!netrules::validCredentials(nullptr, "12345678"));
}
