// glibc >= 2.38 的 <stdlib.h> 在 _GNU_SOURCE 下把 strtol 宏重定义为
// __isoc23_strtol（GLIBC_2.38 版本化符号）。本项目的便携 runtime
// 层基于 glibc 2.35（runtime/abi），没有该符号；GCC 13 会把
// std::stoi（basic_string.h 内联）里的 strtol 展开成 __isoc23_strtol，
// 导致二进制在便携层下无法加载。
//
// 这里通过链接器 --wrap=__isoc23_strtol 把该引用改接到本 shim，
// 再转发到无版本化要求的 libc strtol。C23 与 C99 strtol 的行为差异
// 仅在于 C23 额外接受 "0b"/"0B" 二进制前缀；本项目用 stoi 解析的
// 参数均为十进制整数，语义等价。
//
// 不能直接写 std::strtol / ::strtol：glibc 2.38+ 的 <cstdlib>/
// <stdlib.h> 包装会用宏或 using 声明把 strtol 重定向到
// __isoc23_strtol，--wrap 会把它改回本函数形成无限递归。
// 这里用 asm label 在汇编层强制引用符号 "strtol"（libc 的
// 无版本化定义），不受任何头文件名字替换影响。
extern "C" long libc_strtol_forward(
    const char* nptr, char** endptr, int base) __asm__("strtol");

extern "C" long __wrap___isoc23_strtol(
    const char* nptr, char** endptr, int base) {
  return libc_strtol_forward(nptr, endptr, base);
}
