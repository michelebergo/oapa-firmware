// Two uploads at once (a double tap on Upload, 14/09/2026): the first one keeps the
// update, the second is refused instead of aborting it and mixing their chunks.
#pragma once

TEST(OTA_Gate_ASecondUploadCannotTakeOverAnActiveOne) {
  ota::UploadGate gate;
  CHECK(gate.claim(1, 0));
  CHECK(!gate.claim(2, 1400));  // the second tap, 1.4 s later
  CHECK(gate.owns(1, 1500));    // the first upload's next chunk is still its own
  CHECK(!gate.owns(2, 1500));
}

TEST(OTA_Gate_ChunksKeepTheUploadAlive_AnAbandonedOneExpires) {
  ota::UploadGate gate;
  CHECK(gate.claim(1, 0));
  for (uint32_t t = 1000; t <= 30000; t += 1000) CHECK(gate.owns(1, t));  // 30 s of steady chunks
  CHECK(!gate.claim(2, 44999));  // 14.999 s after the last chunk
  CHECK(gate.claim(2, 45000));   // 15 s of silence: the first upload is abandoned
  CHECK(!gate.owns(1, 45001));
  CHECK(gate.owns(2, 45001));
}

TEST(OTA_Gate_ReleaseFreesIt_OnlyForTheOwner) {
  ota::UploadGate gate;
  CHECK(gate.claim(1, 0));
  gate.release(2);
  CHECK(!gate.claim(3, 10));
  gate.release(1);
  CHECK(gate.claim(3, 20));
}

TEST(OTA_Gate_SurvivesMillisWrap) {
  ota::UploadGate gate;
  CHECK(gate.claim(1, 0xFFFFF000u));
  CHECK(!gate.claim(2, 0x00001000u));  // 8 s later, across the wrap
  CHECK(gate.claim(2, 0xFFFFF000u + 15000u));
}
