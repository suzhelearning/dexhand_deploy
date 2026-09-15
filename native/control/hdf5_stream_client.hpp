#pragma once
#include "datagram_receiver.hpp"
#include <cstring>
#include <limits>
#include <set>
#include <map>

namespace tianji_control {
// Encoder for the existing hdf5_recorder B protocol. Dataset schemas are
// initialized by the launcher; this layer neither creates files nor owns a PID.
class Hdf5Block {
 public:
  using Bytes=std::vector<std::uint8_t>;
  static constexpr std::size_t max_bytes=64U*1024U*1024U;
  void integers(const std::string& path,std::uint32_t rows,const std::vector<std::int64_t>& values) {
    numeric(path,rows,values,[](Bytes& b,std::int64_t v){word(b,static_cast<std::uint64_t>(v),8);});
  }
  void reals(const std::string& path,std::uint32_t rows,const std::vector<double>& values) {
    static_assert(sizeof(double)==8 && std::numeric_limits<double>::is_iec559);
    numeric(path,rows,values,[](Bytes& b,double v){std::uint64_t bits;std::memcpy(&bits,&v,8);word(b,bits,8);});
  }
  void octets(const std::string& path,std::uint32_t rows,const Bytes& values) {
    numeric(path,rows,values,[](Bytes& b,std::uint8_t v){word(b,v,1);});
  }
  void strings(const std::string& path,const std::vector<std::string>& values) {
    auto b=column(path,values.size());
    for(const auto& v:values) text(b,v);
    commit(path,b);
  }
  void bytes(const std::string& path,const std::vector<Bytes>& values) {
    auto b=column(path,values.size());
    for(const auto& v:values) {reserve(b,4+v.size());word(b,v.size(),4);b.insert(b.end(),v.begin(),v.end());}
    commit(path,b);
  }
  void attribute(const std::string& path,const std::string& name,const std::string& value) {
    check_path(path);
    if(name!="names" && name!="joint_names" && name!="logical_id") invalid();
    if(attributes_count_>=2048 || attribute_keys_.count({path,name})) invalid();
    Bytes b;text(b,path);text(b,name);text(b,value);check_total(b.size());
    attributes_.insert(attributes_.end(),b.begin(),b.end());
    attribute_keys_.insert({path,name});++attributes_count_;
    attribute_values_[{path,name}]=value;
  }
  Bytes payload() const {
    Bytes b;b.reserve(9+columns_size_+attributes_.size());b.push_back('B');
    word(b,paths_.size(),4);
    for(const auto& [path,column]:columns_) b.insert(b.end(),column.begin(),column.end());
    word(b,attributes_count_,4);b.insert(b.end(),attributes_.begin(),attributes_.end());return b;
  }
  std::size_t size() const {return 9+columns_size_+attributes_.size();}
  void merge(const Hdf5Block& other) {
    // Used only for blocks produced by the same schema encoders. Retain FIFO
    // rows within each dataset; B has no cross-dataset ordering contract.
    check_total(other.columns_size_+other.attributes_.size());
    for(const auto& [key,value]:other.attribute_values_) {
      auto found=attribute_values_.find(key);
      if(found==attribute_values_.end()) attribute(key.first,key.second,value);
      else if(found->second!=value) invalid();
    }
    for(const auto& [path,column]:other.columns_) {
      auto found=columns_.find(path);
      if(found==columns_.end()) {commit(path,column);continue;}
      auto& target=found->second;
      const auto offset=4+path.size();
      auto rows=[&](const Bytes& b){std::uint32_t n=0;for(unsigned i=0;i<4;++i)n|=std::uint32_t(b[offset+i])<<(8*i);return n;};
      const auto total=rows(target)+rows(column);
      if(total>4096) invalid();
      for(unsigned i=0;i<4;++i)target[offset+i]=static_cast<std::uint8_t>(total>>(8*i));
      target.insert(target.end(),column.begin()+offset+4,column.end());
      columns_size_+=column.size()-offset-4;
    }
  }
 private:
  [[noreturn]] static void invalid(){throw std::invalid_argument("invalid HDF5 block");}
  static void reserve(const Bytes& b,std::size_t n){if(n>max_bytes || b.size()>max_bytes-n) invalid();}
  static void word(Bytes& b,std::uint64_t v,unsigned n){reserve(b,n);for(unsigned i=0;i<n;++i)b.push_back(static_cast<std::uint8_t>(v>>(i*8)));}
  static void text(Bytes& b,const std::string& s){
    if(s.find('\0')!=std::string::npos || s.size()>max_bytes-4) invalid();
    reserve(b,4+s.size());word(b,s.size(),4);b.insert(b.end(),s.begin(),s.end());
  }
  static void check_path(const std::string& p){if(p.empty() || p[0]!='/' || p.find('\0')!=std::string::npos)invalid();}
  Bytes column(const std::string& path,std::size_t rows) const {
    check_path(path);
    if(!rows || rows>4096 || paths_.size()>=2048 || paths_.count(path))invalid();
    Bytes b;text(b,path);word(b,rows,4);return b;
  }
  void check_total(std::size_t n) const {if(n>max_bytes-9-columns_size_-attributes_.size())invalid();}
  void commit(const std::string& path,const Bytes& b){check_total(b.size());if(paths_.size()>=2048)invalid();columns_.emplace(path,b);columns_size_+=b.size();paths_.insert(path);}
  template<class T,class Encode> void numeric(const std::string& path,std::uint32_t rows,const std::vector<T>& values,Encode encode){
    auto b=column(path,rows);
    if(values.empty() || values.size()%rows || values.size()>max_bytes/sizeof(T))invalid();
    reserve(b,values.size()*sizeof(T));
    for(const auto v:values)encode(b,v);
    commit(path,b);
  }
  std::map<std::string,Bytes> columns_;
  std::size_t columns_size_=0;
  Bytes attributes_;
  std::set<std::string> paths_;
  std::set<std::pair<std::string,std::string>> attribute_keys_;
  std::map<std::pair<std::string,std::string>,std::string> attribute_values_;
  std::uint32_t attributes_count_=0;
};

// Single I/O owner; only request_stop may be called concurrently. The launcher
// transfers an exclusive connected UNIX stream and retains child-process cleanup.
// Destructor never sends C: unacknowledged/abandoned recordings stay incomplete.
class Hdf5StreamClient {
 public:
  explicit Hdf5StreamClient(OwnedDescriptor fd,int timeout_ms):fd_(std::move(fd)),timeout_ms_(timeout_ms){
    int type=0;socklen_t n=sizeof(type);sockaddr_storage address{};socklen_t length=sizeof(address);
    if(timeout_ms<=0 || getsockopt(fd_.get(),SOL_SOCKET,SO_TYPE,&type,&n)<0 || type!=SOCK_STREAM ||
       getpeername(fd_.get(),reinterpret_cast<sockaddr*>(&address),&length)<0 || address.ss_family!=AF_UNIX)
      throw std::invalid_argument("connected exclusive UNIX stream required");
    const int flags=fcntl(fd_.get(),F_GETFD);
    if(flags<0 || fcntl(fd_.get(),F_SETFD,flags|FD_CLOEXEC)<0)throw std::runtime_error("recorder close-on-exec failed");
    try{ack("READY 1\n",deadline());}catch(...){fail();throw;}
  }
  void append(const Hdf5Block& block){exchange(block.payload());}
  void flush(){exchange({'F'});}
  void close(bool complete){exchange({static_cast<std::uint8_t>(complete?'C':'A')});closed_=true;fd_.reset();}
  void request_stop() noexcept {stopped_=true;}
  bool failed() const noexcept{return failed_;}
 private:
  using Clock=std::chrono::steady_clock;
  Clock::time_point deadline() const{return Clock::now()+std::chrono::milliseconds(timeout_ms_);}
  void fail() noexcept {failed_=true;fd_.reset();}
  void check(Clock::time_point end){if(stopped_ || Clock::now()>=end)throw std::runtime_error("recorder cancelled or timed out");}
  void wait(short events,Clock::time_point end){
    check(end);pollfd p{fd_.get(),events,0};int r=poll(&p,1,10);
    if(r<0 && errno!=EINTR)throw std::runtime_error("recorder poll failed");
  }
  void ack(const std::string& expected,Clock::time_point end){
    std::string line;
    while(true){
      check(end);char b[257];auto n=recv(fd_.get(),b,sizeof(b),MSG_DONTWAIT);
      if(n==0)throw std::runtime_error("recorder EOF");
      if(n<0){if(errno==EINTR)continue;if(errno==EAGAIN || errno==EWOULDBLOCK){wait(POLLIN,end);continue;}throw std::runtime_error("recorder read failed");}
      line.append(b,static_cast<std::size_t>(n));
      if(line.size()>256)throw std::runtime_error("oversized recorder acknowledgement");
      if(line.find('\n')!=std::string::npos){if(line!=expected)throw std::runtime_error("invalid recorder acknowledgement");return;}
    }
  }
  void exchange(const Hdf5Block::Bytes& payload){
    if(failed_ || closed_)throw std::logic_error("recorder is closed or failed");
    try{
      auto end=deadline();check(end);
      if(payload.empty() || payload.size()>Hdf5Block::max_bytes)throw std::invalid_argument("recorder payload size");
      std::uint8_t header[4];for(unsigned i=0;i<4;++i)header[i]=static_cast<std::uint8_t>(payload.size()>>(8*i));
      send_all(header,4,end);send_all(payload.data(),payload.size(),end);ack("OK\n",end);
    }catch(...){fail();throw;}
  }
  void send_all(const std::uint8_t* bytes,std::size_t size,Clock::time_point end){
    while(size){check(end);auto n=send(fd_.get(),bytes,size,MSG_NOSIGNAL|MSG_DONTWAIT);
      if(n<0){if(errno==EINTR)continue;if(errno==EAGAIN || errno==EWOULDBLOCK){wait(POLLOUT,end);continue;}throw std::runtime_error("recorder send failed");}
      if(!n)throw std::runtime_error("recorder send made no progress");
      bytes+=n;size-=n;
    }
  }
  OwnedDescriptor fd_;
  int timeout_ms_;
  std::atomic<bool> stopped_{false};
  bool failed_=false,closed_=false;
};
} // namespace tianji_control
