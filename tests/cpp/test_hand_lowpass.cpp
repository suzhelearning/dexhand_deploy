#include "../../native/hand/lowpass.cpp"
#include <cassert>
#include <limits>
int main() {
  assert(!tianji_hand_filter_create(0));
  assert(!tianji_hand_filter_create(std::numeric_limits<double>::infinity()));
  auto* handle=tianji_hand_filter_create(.2);assert(handle);
  std::array<double,20> input{},output{};input.fill(1);
  assert(tianji_hand_filter_next(handle,input.data(),20,output.data(),1)==0);
  for(auto x:output) assert(x==1);
  input[19]=std::numeric_limits<double>::quiet_NaN();output.fill(42);
  assert(tianji_hand_filter_next(handle,input.data(),20,output.data(),1)==-1);
  for(auto x:output) assert(x==42);
  input.fill(0);assert(tianji_hand_filter_next(handle,input.data(),20,output.data(),1)==0);
  for(auto x:output) assert(x==static_cast<double>(.8f));
  assert(tianji_hand_filter_reset(handle)==0);
  assert(tianji_hand_filter_next(handle,input.data(),19,output.data(),0)==-1);
  assert(tianji_hand_filter_next(handle,input.data(),20,output.data(),0)==0);
  for(auto x:output) assert(x==0);
  tianji_hand_filter_destroy(handle);
  assert(tianji_hand_filter_reset(nullptr)==-1);
}
