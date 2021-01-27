#pragma once

#include <stdexcept>

template <std::size_t w, std::size_t h>
class mat final {
public:
  float data[h][w];

public:
};
