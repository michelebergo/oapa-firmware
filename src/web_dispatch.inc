// Web-originated commands enter the 1.2.2 dispatcher exactly like serial ones.
// The reply is dropped on purpose: the USB line belongs to N.I.N.A.
bool runWebCommand(const char *cmd) {
  if (!web::isAllowedCommand(cmd)) return false;
  dispatchCommand(String(cmd));
  return true;
}

// Driver configuration from the page, through the same dispatcher. Silent on Serial.
bool runWebDriverCommand(const char *cmd) {
  if (!web::isDriverCommand(cmd)) return false;
  dispatchCommand(String(cmd));
  return true;
}
