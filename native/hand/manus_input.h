#pragma once
#include <cstddef>
#include <cstdint>
extern "C" {
void* tianji_manus_create(unsigned,const char*,const char*) noexcept;
void tianji_manus_destroy(void*) noexcept;
int tianji_manus_line(void*,const char*,float*,std::int64_t*,std::int64_t*,char*,std::size_t) noexcept;
}
