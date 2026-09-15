#pragma once
#include "session_hand_domain.hpp"
#include "../hand/manus_input.h"
#include <functional>
#include <memory>

namespace tianji_control {
// Value-owned audit record: driver line, SDK metadata and optional normalized
// right-then-left callback. Local receive time is not an SDK timestamp.
struct ManusIngressRecord {
  std::uint64_t line_sequence=0;
  std::int64_t received_ns=0;
  std::string raw_line;
  std::array<std::int64_t,2> source_sequences{},source_timestamps{};
  std::optional<HandSampleEnvelope> sample;
};
// Single receive-thread owner. The sink must be bounded/nonblocking and copy or
// move records before returning. It must preserve raw records, not just samples.
// This decoder never publishes commands or grants motion authority.
class ManusIngress {
 public:
  using Sink=std::function<bool(const ManusIngressRecord&)>;
  ManusIngress(Authority source,std::uint64_t generation,const std::string& right,
               const std::string& left,Sink sink)
      :source_(std::move(source)),generation_(generation),sink_(std::move(sink)),
       parser_(tianji_manus_create(3,right.c_str(),left.c_str()),tianji_manus_destroy) {
    if(!source_.bounded() || !generation || generation>=limit_ || !sink_ || !parser_)
      throw std::invalid_argument("invalid native Manus ingress configuration");
    pending_.reserve(65536);
  }
  void feed(const char* bytes,std::size_t count,std::int64_t received_ns) {
    if(finished_ || failed_) throw std::logic_error("Manus ingress closed/failed");
    try {
      if((!bytes && count) || received_ns<=0 || received_ns<last_time_)
        throw std::invalid_argument("invalid Manus receive timestamp/buffer");
      last_time_=received_ns;
      for(std::size_t i=0;i<count;++i) {
        if(bytes[i]=='\0') throw std::runtime_error("NUL in rawviz stream");
        if(bytes[i]=='\n') {emit(received_ns);pending_.clear();}
        else {
          if(pending_.size()==65536) throw std::runtime_error("rawviz line exceeds bound");
          pending_.push_back(bytes[i]);
        }
      }
    }catch(...) {failed_=true;throw;}
  }
  void finish() {
    if(failed_) throw std::runtime_error("Manus ingress failed");
    if(!pending_.empty()) {failed_=true;throw std::runtime_error("truncated rawviz line at EOF");}
    finished_=true;
  }
 private:
  void emit(std::int64_t timestamp) {
    if(line_sequence_+1>=limit_) throw std::overflow_error("Manus line sequence exhausted");
    ManusIngressRecord record;record.line_sequence=++line_sequence_;
    record.received_ns=timestamp;record.raw_line=pending_;
    std::array<float,126> points{};std::array<char,1024> error{};
    const int count=tianji_manus_line(parser_.get(),pending_.c_str(),points.data(),
      record.source_sequences.data(),record.source_timestamps.data(),error.data(),error.size());
    if(count<0) throw std::runtime_error(error.data());
    if(count) {
      if(count!=126 || sequence_+1>=limit_) throw std::runtime_error("invalid Manus bilateral callback");
      HandSampleEnvelope envelope;envelope.source=source_;
      auto& value=envelope.value;value.sequence=++sequence_;value.timestamp_ns=timestamp;
      value.generation=generation_;value.flags=3;
      std::copy(points.begin(),points.end(),value.points.begin());
      record.sample=std::move(envelope);
    }
    if(!sink_(record)) throw std::runtime_error("native Manus ingress sink rejected record");
  }
  static constexpr std::uint64_t limit_=std::uint64_t(1)<<63;
  Authority source_;
  std::uint64_t generation_,sequence_=0,line_sequence_=0;
  Sink sink_;
  std::unique_ptr<void,decltype(&tianji_manus_destroy)> parser_;
  std::string pending_;
  std::int64_t last_time_=0;
  bool finished_=false,failed_=false;
};
}
