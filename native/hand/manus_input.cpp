// Post-driver rawviz parser. No SDK, device, retargeting or command authority.
#include "manus_input.h"
#include <array>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace {
std::string lower(std::string text) {
  for(auto& c:text)c=static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return text;
}
std::int64_t integer(const std::string& raw,bool metadata_only=false) {
  std::string text;
  const std::size_t sign=(raw[0]=='+' || raw[0]=='-')?1:0;
  const bool prefixed=raw.size()>sign+2 && raw[sign]=='0' &&
    std::string("xXbBoO").find(raw[sign+1])!=std::string::npos;
  for(std::size_t i=0;i<raw.size();++i) {
    if(raw[i]!='_')text.push_back(raw[i]);
    else if(i==0 || i+1==raw.size() ||
            (!(prefixed && i==sign+2) && !std::isxdigit(static_cast<unsigned char>(raw[i-1]))) ||
            !std::isxdigit(static_cast<unsigned char>(raw[i+1])))
      throw std::invalid_argument("integer underscore");
  }
  // Same base-0 spelling as Python's integer protocol, including binary.
  std::size_t start=(text[0]=='+' || text[0]=='-')?1:0;
  auto digits=text.substr(start);int base=10;std::size_t prefix=0;
  if(digits.size()>1 && digits[0]=='0') {
    const char code=static_cast<char>(std::tolower(digits[1]));
    if(code=='x'){base=16;prefix=2;}
    else if(code=='b'){base=2;prefix=2;}
    else if(code=='o'){base=8;prefix=2;}
    else if(digits.find_first_not_of('0')!=std::string::npos) throw std::invalid_argument("integer spelling");
  }
  const auto normalized=text.substr(0,start)+digits.substr(prefix);
  if(metadata_only) {
    const auto magnitude=digits.substr(prefix);
    if(magnitude.empty())throw std::invalid_argument("empty integer");
    for(char c:lower(magnitude)) {
      const int value=c>='0' && c<='9'?c-'0':c>='a' && c<='f'?c-'a'+10:-1;
      if(value<0 || value>=base)throw std::invalid_argument("integer digit");
    }
    return 0; // SDK publish time / IDs are preserved verbatim in raw recording.
  }
  std::size_t consumed=0;auto result=std::stoll(normalized,&consumed,base);
  if(consumed!=normalized.size())throw std::invalid_argument("integer suffix");
  return result;
}
struct Node {std::int64_t index,chain,joint;};
double floating(const std::string& token) {
  const auto normalized=lower(token);
  const auto body=normalized.substr((normalized[0]=='+' || normalized[0]=='-')?1:0);
  std::string text;
  if(body=="nan" || body=="inf" || body=="infinity")text=normalized;
  else for(std::size_t i=0;i<token.size();++i) {
    const char c=token[i];
    if(c=='_') {
      if(i==0 || i+1==token.size() || token[i-1]<'0' || token[i-1]>'9' ||
         token[i+1]<'0' || token[i+1]>'9')throw std::invalid_argument("float underscore");
    } else {
      if(!(c>='0' && c<='9') && c!='.' && c!='e' && c!='E' && c!='+' && c!='-')
        throw std::invalid_argument("float spelling");
      text.push_back(c);
    }
  }
  char* end=nullptr;const double value=std::strtod(text.c_str(),&end);
  if(end==text.c_str() || end!=text.c_str()+text.size())throw std::invalid_argument("float suffix");
  return value;
}
struct Stream {int side;std::int64_t count;std::vector<Node> nodes;};
struct Sample {bool valid=false;std::int64_t sequence=0,time=0;std::array<float,63> points{};};
class Parser {
 public:
  Parser(unsigned flags,std::string right,std::string left):flags_(flags) {
    if(flags==0 || flags>3 || (!right.empty() && right==left))throw std::invalid_argument("invalid Manus sides/binding");
    if(!right.empty())bindings_[right]=0;
    if(!left.empty())bindings_[left]=1;
  }
  int line(const std::string& text,float* points,std::int64_t* sequences,std::int64_t* times) {
    if(text.size()>65536)throw std::runtime_error("rawviz line exceeds 65536 bytes");
    std::istringstream input(text);std::vector<std::string> f;std::string token;
    while(input>>token)f.push_back(std::move(token));
    if(f.empty())return 0;
    const auto kind=lower(f[0]);
    try {
      if(kind=="hand") {
        if(f.size()!=4)return 0;
        const auto binding=bindings_.find(f[1]);const auto side=lower(f[2]);
        const int which=binding!=bindings_.end()?binding->second:side=="right"?0:side=="left"?1:-1;
        if(which<0)return 0;
        const auto count=integer(f[3]);
        if(count>512)throw std::runtime_error("Manus node count exceeds bound");
        if(streams_.size()>=64 && !streams_.count(f[1]))throw std::runtime_error("Manus stream count exceeds bound");
        streams_[f[1]]={which,count,{}};return 0;
      }
      if(kind=="node") {
        if(f.size()!=8)return 0;
        std::array<std::int64_t,6> v{};
        for(int i=0;i<6;++i)v[i]=integer(f[i+2],i==1 || i==2 || i==4);
        auto it=streams_.find(f[1]);if(it==streams_.end())return 0;
        if(it->second.nodes.size()>=512)throw std::runtime_error("Manus metadata exceeds bound");
        it->second.nodes.push_back({v[0],v[3],v[5]});return 0;
      }
      if(kind!="pose" || f.size()<5 || (f.size()-5)%7)return 0;
      const auto sequence=integer(f[2]),time=integer(f[3]);integer(f[4],true);
      std::vector<float> values;values.reserve(f.size()-5);
      for(std::size_t i=5;i<f.size();++i) {
        values.push_back(static_cast<float>(floating(f[i])));
      }
      auto it=streams_.find(f[1]);if(it==streams_.end())return 0;
      const auto& stream=it->second;auto& sample=latest_[stream.side];
      auto invalidate=[&]{sample.valid=false;dirty_=false;return 0;};
      if(stream.count<0 || values.size()/7!=static_cast<std::size_t>(stream.count))return invalidate();
      for(std::size_t i=0;i<values.size();i+=7)
        for(int j=0;j<3;++j)if(!std::isfinite(values[i+j]))return invalidate();
      std::array<std::int64_t,21> indices;indices.fill(-1);
      for(const auto& n:stream.nodes) {
        int key=-1;
        if(n.chain==13)key=0;
        else if(n.chain==5) {
          if(n.joint==1)key=1;else if(n.joint==2)key=2;
          else if(n.joint==4)key=3;else if(n.joint==5)key=4;
        } else if(n.chain>=6 && n.chain<=9 && n.joint>=2 && n.joint<=5)
          key=5+static_cast<int>((n.chain-6)*4+n.joint-2);
        if(key<0)continue;
        if(n.index<0 || n.index>=stream.count || indices[key]>=0)return invalidate();
        indices[key]=n.index;
      }
      if(std::find(indices.begin(),indices.end(),-1)!=indices.end())return invalidate();
      if((flags_&(1U<<stream.side)) && (!sample.valid || sequence>sample.sequence)) {
        for(int i=0;i<21;++i)for(int j=0;j<3;++j)
          sample.points[i*3+j]=values[indices[i]*7+j]*(j==1?-1.f:1.f);
        sample.valid=true;sample.sequence=sequence;sample.time=time;dirty_=true;
      }
      if(!dirty_)return 0;
      for(int side=0;side<2;++side)if((flags_&(1U<<side)) && !latest_[side].valid)return 0;
      int count=0;
      for(int side=0;side<2;++side)if(flags_&(1U<<side)) {
        std::copy(latest_[side].points.begin(),latest_[side].points.end(),points+count);
        sequences[side]=latest_[side].sequence;times[side]=latest_[side].time;count+=63;
      }
      dirty_=false;return count;
    }catch(const std::invalid_argument&){return 0;}
     catch(const std::out_of_range&){return 0;}
  }
 private:
  unsigned flags_;bool dirty_=false;
  std::array<Sample,2> latest_{};
  std::unordered_map<std::string,int> bindings_;
  std::unordered_map<std::string,Stream> streams_;
};
}
extern "C" {
void* tianji_manus_create(unsigned flags,const char* right,const char* left) noexcept {
  try{return new Parser(flags,right?right:"",left?left:"");}catch(...){return nullptr;}
}
void tianji_manus_destroy(void* parser) noexcept {delete static_cast<Parser*>(parser);}
int tianji_manus_line(void* parser,const char* line,float* points,std::int64_t* sequences,
                     std::int64_t* times,char* error,std::size_t capacity) noexcept {
  try {
    if(!parser || !line || !points || !sequences || !times)throw std::invalid_argument("invalid parser arguments");
    return static_cast<Parser*>(parser)->line(line,points,sequences,times);
  }catch(const std::exception& e){
    if(error && capacity){std::strncpy(error,e.what(),capacity-1);error[capacity-1]=0;}return -1;
  }catch(...){return -1;}
}
}
