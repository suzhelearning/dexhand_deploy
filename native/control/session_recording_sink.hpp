#pragma once
#include "session_cycle_hdf5.hpp"
#include "session_output_bridge.hpp"
#include "manus_hdf5.hpp"

namespace tianji_control {
// Single output-thread owner; feed from a bounded output lane, not the control
// thread. Only request_stop() is concurrent. The schema/file is created by the
// launcher. Destruction never commits; finish(true) is called ONLY after all
// output lanes have drained and shutdown/Home has completed successfully.
class SessionRecordingSink {
  using Json=nlohmann::json;
 public:
  SessionRecordingSink(OwnedDescriptor descriptor,const Json& manifest,
                       std::int64_t origin_ns,int timeout_ms)
      :cycles_(manifest),run_(manifest.at("run_id")),
       receiver_(manifest.at("source_authority").at("instance")),origin_(origin_ns),
       disk_(std::move(descriptor),timeout_ms),batch_enabled_(manifest.contains("hand_runtime")) {
    if(origin_<=0) throw std::invalid_argument("recording origin must be positive");
    if(manifest.contains("manus_source_authority")) {
      const auto& a=manifest.at("manus_source_authority");
      Authority authority{a.at("logical"),a.at("instance"),a.at("router")};
      if(!authority.bounded() || authority.router!=manifest.at("source_authority").at("router"))
        throw std::invalid_argument("invalid recording Manus authority");
      manus_receiver_=authority.instance;
    }
    audit("lifecycle",{{"stage","opening"}},origin_);
  }
  void write(const ManusIngressRecord& raw) {
    check();
    try {
      if(manus_receiver_.empty()) throw std::invalid_argument("Manus recording source not configured");
      append(encode_manus_ingress_columns(raw,origin_,manus_receiver_,run_));
    }catch(...) {poison();throw;}
  }
  void write(const SessionOutputItem& item) {
    check();
    try {
      if(const auto* cycle=std::get_if<SessionCycleSnapshot>(&item)) append(cycles_.encode(*cycle,origin_));
      else if(const auto* raw=std::get_if<SessionRawSnapshot>(&item))
        append(encode_raw_tjvr_columns(*raw,origin_,receiver_,run_));
      else if(const auto* manus=std::get_if<ManusIngressRecord>(&item)) write(*manus);
      else if(const auto* reply=std::get_if<SessionReply>(&item)) {
        if(reply->action.empty() || reply->action.size()>64 || reply->action.find('\0')!=std::string::npos)
          throw std::invalid_argument("recording operator action missing or invalid");
        audit("operator_result",{{"kind","operator_result"},{"action",reply->action},
          {"accepted",reply->outcome.accepted},{"reason",reply->outcome.reason}},reply->timestamp_ns);
      }
      // Standalone command receipts are also present in the cycle audit; the
      // Python consumer likewise does not append duplicate receipt rows.
    }catch(...) {poison();throw;}
  }
  void finish(bool complete,std::int64_t timestamp_ns,const std::string& reason) {
    check();
    try {
      audit("lifecycle",{{"stage","native_recording_close"},{"complete",complete},{"reason",reason}},timestamp_ns);
      flush_batch();disk_.close(complete);closed_=true;
    }catch(...) {poison();throw;}
  }
  void request_stop() noexcept {disk_.request_stop();}
 private:
  void flush_batch() {
    if(!batch_count_)return;
    disk_.append(batch_);batch_=Hdf5Block{};batch_count_=0;
    batch_started_=std::chrono::steady_clock::now();
  }
  void append(const Hdf5Block& block) {
    // Joint mode only: amortize dataset extension and IPC over bounded batches.
    // Single-arm recording retains its established immediate-write behavior.
    if(!batch_enabled_) {disk_.append(block);return;}
    if(batch_count_ && batch_.size()+block.size()>4U*1024U*1024U)flush_batch();
    batch_.merge(block);++batch_count_;
    if(batch_count_>=64 || std::chrono::steady_clock::now()-batch_started_>=std::chrono::milliseconds(20))
      flush_batch();
  }
  void check() const {if(closed_ || poisoned_) throw std::logic_error("recording sink is closed or failed");}
  void poison() noexcept {poisoned_=true;disk_.request_stop();}
  void audit(const std::string& kind,Json payload,std::int64_t timestamp_ns) {
    if(timestamp_ns<=0) throw std::invalid_argument("invalid audit timestamp");
    payload["run_id"]=run_;
    const auto encoded=payload.dump();
    if(encoded.size()>1048576) throw std::invalid_argument("audit exceeds 1 MiB");
    Hdf5Block block;
    block.integers("/meta/dual_audit/time_ns",1,{timestamp_ns-origin_});
    block.integers("/meta/dual_audit/received_timestamp_ns",1,{timestamp_ns});
    block.strings("/meta/dual_audit/kind",{kind});
    block.strings("/meta/dual_audit/payload_json",{encoded});
    append(block);
  }
  SessionCycleRecording cycles_;
  std::string run_,receiver_,manus_receiver_;
  std::int64_t origin_;
  Hdf5StreamClient disk_;
  bool batch_enabled_=false;
  Hdf5Block batch_;
  std::size_t batch_count_=0;
  std::chrono::steady_clock::time_point batch_started_=std::chrono::steady_clock::now();
  bool closed_=false,poisoned_=false;
};
} // namespace tianji_control
