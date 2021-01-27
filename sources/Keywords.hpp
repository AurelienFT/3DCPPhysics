#pragma once

#pragma warning(push)
#pragma warning(disable : 4068)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wkeyword-macro"
#pragma clang diagnostic ignored "-Wunused-macros"
#define use(...) if (__VA_ARGS__; true)
#define var      decltype(auto)
#pragma clang diagnostic pop
#pragma warning(pop)
