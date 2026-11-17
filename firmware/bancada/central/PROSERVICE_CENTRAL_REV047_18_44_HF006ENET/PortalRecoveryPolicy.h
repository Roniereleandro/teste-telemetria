#pragma once
namespace PortalRecoveryPolicy {
struct State {
  bool bootReady, idle, outputsOff, savedReceiptAvailable, orderMatches;
  bool noPending, noAutoRestart, remoteMode, persistenceHealthy, serverHasNoItems;
};
constexpr bool canReportCancellation(const State& s) {
  return s.bootReady && s.idle && s.outputsOff && s.savedReceiptAvailable && s.orderMatches &&
      s.noPending && s.noAutoRestart && s.remoteMode && s.persistenceHealthy && s.serverHasNoItems;
}
constexpr bool legacyReceiptAccepted(bool authorized, bool cancelledReceipt) {
  return authorized || cancelledReceipt;
}
}
