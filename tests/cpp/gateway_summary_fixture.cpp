#include "../../native/control/session_gateway_wire.hpp"
#include <iostream>
using namespace tianji_control;
int main() {
  int sockets[2];
  if(socketpair(AF_UNIX,SOCK_STREAM,0,sockets)) return 2;
  OwnedDescriptor peer(sockets[1]);
  GatewayWireWriter writer(OwnedDescriptor(sockets[0]),1000,true);
  SessionCycleSnapshot c;
  c.snapshot.state.epoch=1;c.snapshot.state.phase="idle";
  for(int i=1;i<=26;++i) {
    c.timestamp_ns=i*5000000;c.snapshot.ticks=i;
    if(i==5)c.snapshot.state.phase="teleop";
    writer.write(c);
    // No Python raw/receipt consumers in negotiated summary mode.
    writer.write(SessionRawSnapshot{});
    writer.write(SessionCommandResult{});
  }
  writer.complete(true,c.snapshot);
  shutdown(sockets[0],SHUT_WR);
  char buffer[4096];ssize_t n;
  while((n=read(peer.get(),buffer,sizeof(buffer)))>0) std::cout.write(buffer,n);
}
