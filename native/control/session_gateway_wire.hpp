#pragma once

#include "session_runtime.hpp"
#include "datagram_receiver.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

namespace tianji_control {

// Fixed command/control wire.  It is intentionally independent of Python's
// object protocol: Python may send these records at the cold control boundary,
// while the C++ scheduler owns every periodic decision.
enum class GatewayAction : std::uint8_t {
  start=1, return_home=2, shutdown=3, rearm=4, calibrate=5
};

struct GatewayCommand {
  GatewayAction action=GatewayAction::start;
  std::uint64_t id=0;
  std::int64_t next_epoch=0;
};

inline constexpr std::size_t kGatewayCommandWireSize=24;
inline constexpr std::size_t kGatewayFrameHeaderWireSize=28;
inline constexpr std::size_t kGatewayMaxReason=4096;
inline constexpr std::size_t kGatewayMaxResultWire=1206;
inline constexpr std::size_t kGatewayMaxPacket=656;

enum class GatewayFrameKind : std::uint8_t {
  reply=1, receipt=2, cycle=3, complete=4, failure=5, raw=6, summary=7
};

class GatewayWireCodec {
 public:
  using Bytes=std::vector<std::uint8_t>;

  static GatewayCommand decode_command(const std::array<std::uint8_t,kGatewayCommandWireSize>& bytes) {
    if(std::memcmp(bytes.data(),"TJAC",4)!=0 || bytes[4]!=1 || bytes[6]!=0 || bytes[7]!=0)
      throw std::runtime_error("invalid native gateway command header");
    const auto action=bytes[5];
    if(action<static_cast<std::uint8_t>(GatewayAction::start) ||
       action>static_cast<std::uint8_t>(GatewayAction::calibrate))
      throw std::runtime_error("invalid native gateway action");
    const auto id=read_unsigned(bytes.data()+8,8);
    const auto epoch=read_signed(bytes.data()+16,8);
    if(id==0 || epoch<0) throw std::runtime_error("invalid native gateway command identity");
    return {static_cast<GatewayAction>(action),id,epoch};
  }

  static Bytes encode_reply(const SessionReply& reply) {
    Bytes payload;
    payload.reserve(8+kGatewayMaxReason);
    append_unsigned(payload,reply.id,8);
    payload.push_back(reply.outcome.accepted?1:0);
    payload.insert(payload.end(),3,0);
    append_text(payload,reply.outcome.reason,kGatewayMaxReason);
    return frame(GatewayFrameKind::reply,reply.id,0,payload);
  }

  static Bytes encode_receipt(const SessionCommandResult& result) {
    if(!result.receipt) throw std::invalid_argument("native gateway receipt missing");
    const auto& receipt=*result.receipt;
    Bytes payload;
    payload.reserve(8+8+8+4+14*8+3*4+kGatewayMaxReason);
    append_unsigned(payload,static_cast<std::uint64_t>(receipt.tick),8);
    append_signed(payload,receipt.epoch,8);
    append_signed(payload,receipt.timestamp,8);
    payload.push_back(receipt.accepted?1:0);
    payload.insert(payload.end(),3,0);
    for(const auto& side:result.positions) for(double q:side) append_real(payload,q);
    append_text(payload,receipt.run,kGatewayMaxReason);
    append_text(payload,receipt.router,kGatewayMaxReason);
    append_text(payload,receipt.reason,kGatewayMaxReason);
    return frame(GatewayFrameKind::receipt,static_cast<std::uint64_t>(receipt.tick),receipt.timestamp,payload);
  }

  static Bytes encode_cycle(const SessionCycleSnapshot& cycle) {
    const auto& snapshot=cycle.snapshot;
    Bytes payload;
    payload.reserve(128+28*8+kGatewayMaxPacket+kGatewayMaxResultWire+kGatewayMaxReason);
    append_signed(payload,cycle.timestamp_ns,8);
    append_signed(payload,cycle.source_received_ns,8);
    append_unsigned(payload,cycle.source_revision,8);
    append_unsigned(payload,cycle.source.epoch,8);
    append_unsigned(payload,cycle.source.sequence,8);
    append_unsigned(payload,cycle.source.generation,8);
    append_signed(payload,snapshot.state.epoch,8);
    append_unsigned(payload,snapshot.ticks,8);
    append_unsigned(payload,snapshot.late_ticks,8);
    payload.push_back(state_code(snapshot.state.phase));
    std::uint8_t flags=0;
    if(snapshot.command) flags|=1U<<0U;
    if(snapshot.feedback) flags|=1U<<1U;
    if(cycle.request) flags|=1U<<2U;
    if(cycle.result) flags|=1U<<3U;
    if(cycle.ik_adopted) flags|=1U<<4U;
    if(cycle.source.accepted) flags|=1U<<5U;
    if(cycle.source.skeleton_valid) flags|=1U<<6U;
    if(cycle.source.rotations_valid) flags|=1U<<7U;
    payload.push_back(flags);
    payload.push_back(snapshot.reset_pending?1:0);
    payload.push_back(snapshot.cycle_capture_failed?1:0);
    for(int side=0;side<2;++side) for(int joint=0;joint<7;++joint)
      append_real(payload,snapshot.command?(*snapshot.command).positions[side][joint]:0.0);
    for(int side=0;side<2;++side) for(int joint=0;joint<7;++joint)
      append_real(payload,snapshot.feedback?(*snapshot.feedback)[side][joint]:0.0);

    if(cycle.request) {
      const auto& request=*cycle.request;
      if(request.packet.size()>kGatewayMaxPacket) throw std::invalid_argument("oversized gateway request packet");
      append_unsigned(payload,request.id,8);
      append_signed(payload,request.now_ns,8);
      append_signed(payload,request.received_ns,8);
      append_unsigned(payload,request.generation,8);
      append_unsigned(payload,request.source_sequence,8);
      payload.push_back(request.discontinuity?1:0);
      payload.insert(payload.end(),7,0);
      append_unsigned(payload,request.packet.size(),4);
      payload.insert(payload.end(),request.packet.begin(),request.packet.end());
      payload.insert(payload.end(),kGatewayMaxPacket-request.packet.size(),0);
    } else {
      payload.insert(payload.end(),8+8+8+8+8+1+7+4+kGatewayMaxPacket,0);
    }
    if(cycle.result) {
      if(cycle.result->wire.size()>kGatewayMaxResultWire)
        throw std::invalid_argument("oversized gateway result wire");
      append_unsigned(payload,cycle.result->tick,8);
      append_unsigned(payload,cycle.result->timestamp,8);
      append_unsigned(payload,cycle.result->applied_epoch,8);
      append_unsigned(payload,cycle.result->applied_sequence,8);
      append_unsigned(payload,cycle.result->wire.size(),4);
      payload.insert(payload.end(),cycle.result->wire.begin(),cycle.result->wire.end());
      payload.insert(payload.end(),kGatewayMaxResultWire-cycle.result->wire.size(),0);
    } else {
      payload.insert(payload.end(),8*4+4+kGatewayMaxResultWire,0);
    }
    append_text(payload,snapshot.state.reason,kGatewayMaxReason);
    return frame(GatewayFrameKind::cycle,snapshot.ticks,cycle.timestamp_ns,payload);
  }

  static Bytes encode_raw(const SessionRawSnapshot& raw) {
    if(raw.sequence==0 || raw.received_ns<=0 || raw.bytes.empty() ||
       raw.bytes.size()>kGatewayMaxPacket)
      throw std::invalid_argument("invalid native raw snapshot");
    Bytes payload;
    payload.reserve(8+8+4+4+kGatewayMaxPacket);
    append_unsigned(payload,raw.sequence,8);
    append_signed(payload,raw.received_ns,8);
    payload.push_back(raw.accepted?1:0);
    payload.insert(payload.end(),3,0);
    append_unsigned(payload,raw.bytes.size(),4);
    payload.insert(payload.end(),raw.bytes.begin(),raw.bytes.end());
    payload.insert(payload.end(),kGatewayMaxPacket-raw.bytes.size(),0);
    return frame(GatewayFrameKind::raw,raw.sequence,raw.received_ns,payload);
  }

  static Bytes encode_complete(bool complete,const SessionSnapshot& snapshot) {
    Bytes payload;
    payload.reserve(16+kGatewayMaxReason);
    payload.push_back(complete?1:0);
    payload.push_back(state_code(snapshot.state.phase));
    payload.insert(payload.end(),2,0);
    append_signed(payload,snapshot.state.epoch,8);
    append_text(payload,snapshot.state.reason,kGatewayMaxReason);
    return frame(GatewayFrameKind::complete,snapshot.ticks,0,payload);
  }

  static Bytes encode_summary(const SessionSnapshot& snapshot,std::int64_t stamp,std::uint64_t cycles) {
    Bytes payload;
    append_signed(payload,snapshot.state.epoch,8);
    append_unsigned(payload,snapshot.ticks,8);
    append_unsigned(payload,snapshot.late_ticks,8);
    append_unsigned(payload,cycles,8);
    payload.push_back(state_code(snapshot.state.phase));
    payload.push_back(snapshot.reset_pending?1:0);
    payload.push_back(snapshot.cycle_capture_failed?1:0);
    payload.push_back(0);
    append_text(payload,snapshot.state.reason,kGatewayMaxReason);
    return frame(GatewayFrameKind::summary,snapshot.ticks,stamp,payload);
  }

  static Bytes encode_failure(const std::string& reason) {
    Bytes payload;
    append_text(payload,reason,kGatewayMaxReason);
    return frame(GatewayFrameKind::failure,0,0,payload);
  }

 private:
  static std::uint64_t read_unsigned(const std::uint8_t* bytes,unsigned size) {
    std::uint64_t result=0;
    if(size>8) throw std::invalid_argument("invalid gateway integer size");
    for(unsigned i=0;i<size;++i) result|=std::uint64_t(bytes[i])<<(8U*i);
    return result;
  }
  static std::int64_t read_signed(const std::uint8_t* bytes,unsigned size) {
    const auto value=read_unsigned(bytes,size);
    if(size!=8) throw std::invalid_argument("invalid gateway signed integer size");
    if((value&(std::uint64_t(1)<<63U))==0) return static_cast<std::int64_t>(value);
    if(value==(std::uint64_t(1)<<63U)) return std::numeric_limits<std::int64_t>::min();
    return -static_cast<std::int64_t>((~value)+1U);
  }
  static void append_unsigned(Bytes& bytes,std::uint64_t value,unsigned size) {
    if(size>8) throw std::invalid_argument("invalid gateway integer size");
    for(unsigned i=0;i<size;++i) bytes.push_back(static_cast<std::uint8_t>(value>>(8U*i)));
  }
  static void append_signed(Bytes& bytes,std::int64_t value,unsigned size) {
    append_unsigned(bytes,static_cast<std::uint64_t>(value),size);
  }
  static void append_real(Bytes& bytes,double value) {
    if(!std::isfinite(value)) throw std::invalid_argument("nonfinite native gateway value");
    std::uint64_t bits=0; std::memcpy(&bits,&value,sizeof(bits)); append_unsigned(bytes,bits,8);
  }
  static void append_text(Bytes& bytes,const std::string& value,std::size_t maximum) {
    if(value.size()>maximum || value.find('\0')!=std::string::npos)
      throw std::invalid_argument("oversized or NUL native gateway text");
    append_unsigned(bytes,value.size(),4); bytes.insert(bytes.end(),value.begin(),value.end());
  }
  static std::uint8_t state_code(const std::string& phase) {
    if(phase=="idle") return 1;
    if(phase=="teleop") return 2;
    if(phase=="returning") return 3;
    if(phase=="fault") return 4;
    throw std::invalid_argument("unknown native gateway session state");
  }
  static Bytes frame(GatewayFrameKind kind,std::uint64_t sequence,std::int64_t timestamp,const Bytes& payload) {
    if(payload.size()>std::numeric_limits<std::uint32_t>::max())
      throw std::invalid_argument("oversized native gateway frame");
    Bytes bytes; bytes.reserve(kGatewayFrameHeaderWireSize+payload.size());
    bytes.insert(bytes.end(),{'T','J','S','O'}); bytes.push_back(1);
    bytes.push_back(static_cast<std::uint8_t>(kind)); bytes.insert(bytes.end(),2,0);
    append_unsigned(bytes,payload.size(),4); append_unsigned(bytes,sequence,8);
    append_signed(bytes,timestamp,8); bytes.insert(bytes.end(),payload.begin(),payload.end());
    return bytes;
  }
};

// The output side is the only owner that writes to the gateway control stream.
// It uses bounded nonblocking I/O so a dead viewer/adapter faults the native
// session instead of blocking the fixed-rate scheduler indefinitely.
class GatewayWireWriter {
 public:
  using Bytes=GatewayWireCodec::Bytes;
  explicit GatewayWireWriter(OwnedDescriptor fd,int timeout_ms=1000,bool summaries=false)
      :fd_(std::move(fd)),timeout_ms_(timeout_ms),summaries_(summaries) {
    if(timeout_ms<=0 || fd_.get()<0) throw std::invalid_argument("valid gateway output required");
    int type=0; socklen_t size=sizeof(type);
    if(getsockopt(fd_.get(),SOL_SOCKET,SO_TYPE,&type,&size)<0 || type!=SOCK_STREAM)
      throw std::invalid_argument("gateway output must be a connected stream");
    const int flags=fcntl(fd_.get(),F_GETFD);
    if(flags<0 || fcntl(fd_.get(),F_SETFD,flags|FD_CLOEXEC)<0)
      throw std::runtime_error("gateway output close-on-exec failed");
    if(fcntl(fd_.get(),F_SETFL,fcntl(fd_.get(),F_GETFL)|O_NONBLOCK)<0)
      throw std::runtime_error("gateway output nonblocking setup failed");
  }
  GatewayWireWriter(const GatewayWireWriter&)=delete;
  GatewayWireWriter& operator=(const GatewayWireWriter&)=delete;
  void write(const Bytes& bytes) { write_all(bytes); }
  void write(const SessionReply& reply) { write(GatewayWireCodec::encode_reply(reply)); }
  void write(const SessionCommandResult& result) { if(!summaries_) write(GatewayWireCodec::encode_receipt(result)); }
  void write(const SessionCycleSnapshot& cycle) {
    if(!summaries_) {write(GatewayWireCodec::encode_cycle(cycle));return;}
    const bool changed=cycles_==0 || last_.state.phase!=cycle.snapshot.state.phase ||
      last_.state.epoch!=cycle.snapshot.state.epoch || last_.reset_pending!=cycle.snapshot.reset_pending ||
      last_.cycle_capture_failed!=cycle.snapshot.cycle_capture_failed;
    last_=cycle.snapshot;stamp_=cycle.timestamp_ns;++cycles_;pending_=true;
    if(changed || stamp_<sent_stamp_ || stamp_-sent_stamp_>=100000000) flush_summary();
  }
  void write(const SessionRawSnapshot& raw) { if(!summaries_) write(GatewayWireCodec::encode_raw(raw)); }
  void write(const ManusIngressRecord&) {
    // Raw Manus is consumed by the native recording lane. The compatibility
    // Python wire protocol has no matching frame: refuse rather than drop it.
    if(!summaries_) throw std::runtime_error("Manus output requires native summary gateway mode");
  }
  void complete(bool success,const SessionSnapshot& snapshot) {
    if(summaries_) flush_summary();
    write(GatewayWireCodec::encode_complete(success,snapshot));
  }
  void failure(const std::string& reason) { write(GatewayWireCodec::encode_failure(reason)); }
  void request_stop() noexcept { stopped_.store(true); }
 private:
  using Clock=std::chrono::steady_clock;
  void flush_summary() {
    if(!pending_) return;
    write(GatewayWireCodec::encode_summary(last_,stamp_,cycles_));
    pending_=false;sent_stamp_=stamp_;
  }
  void write_all(const Bytes& bytes) {
    if(bytes.empty() || bytes.size()>64U*1024U*1024U) throw std::invalid_argument("invalid gateway output frame");
    const auto end=Clock::now()+std::chrono::milliseconds(timeout_ms_);
    std::size_t offset=0;
    while(offset<bytes.size()) {
      if(stopped_.load()) throw std::runtime_error("native gateway output cancelled");
      const auto remaining=std::chrono::duration_cast<std::chrono::milliseconds>(end-Clock::now()).count();
      if(remaining<0) throw std::runtime_error("native gateway output timeout");
      pollfd descriptor{fd_.get(),POLLOUT,0};
      const auto ready=poll(&descriptor,1,static_cast<int>(std::min<std::int64_t>(remaining+1,10)));
      if(ready<0 && errno==EINTR) continue;
      if(ready<0) throw std::runtime_error("native gateway output poll failed");
      if(ready==0) continue;
      if(descriptor.revents&(POLLERR|POLLHUP|POLLNVAL)) throw std::runtime_error("native gateway output closed");
      const auto count=send(fd_.get(),bytes.data()+offset,bytes.size()-offset,MSG_NOSIGNAL|MSG_DONTWAIT);
      if(count>0) offset+=static_cast<std::size_t>(count);
      else if(count<0 && (errno==EINTR || errno==EAGAIN || errno==EWOULDBLOCK)) continue;
      else throw std::runtime_error("native gateway output write failed");
    }
  }
  OwnedDescriptor fd_;
  int timeout_ms_;
  bool summaries_=false,pending_=false;
  SessionSnapshot last_;
  std::uint64_t cycles_=0;
  std::int64_t stamp_=0,sent_stamp_=0;
  std::atomic<bool> stopped_{false};
};

} // namespace tianji_control
