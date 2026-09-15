#include "../../native/hand/geometry.cpp"
#include <array>
#include <cassert>

int main() {
    assert(tianji_hand_geometry_abi()==1);
    std::array<double,63> input{}, output;
    std::array<double,18> config{1,1,1, 1,0,0, 0,1,0, 0,0,1, 0,0,0, 0,0,0};
    input[15]=.02; input[16]=.03; input[28]=.05;
    assert(tianji_hand_geometry_prepare(input.data(),63,config.data(),18,0,output.data())==0);
    for (double x:output) assert(std::isfinite(x));
    output.fill(42);
    input[62]=std::numeric_limits<double>::quiet_NaN();
    assert(tianji_hand_geometry_prepare(input.data(),63,config.data(),18,0,output.data())==-1);
    for (double x:output) assert(x==42);
    assert(tianji_hand_geometry_prepare(nullptr,63,config.data(),18,0,output.data())==-1);
    assert(tianji_hand_geometry_prepare(input.data(),62,config.data(),18,0,output.data())==-1);
    input[62]=0; config[3]=2;
    assert(tianji_hand_geometry_prepare(input.data(),63,config.data(),18,0,output.data())==-1);
}
