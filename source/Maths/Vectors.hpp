#pragma once

#include <stdexcept>
#include <vector>
#include <array>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <iostream>

template <class T, std::size_t width, std::size_t height>
class Matrix;

template <class T, uint32_t size>
class Vector {
protected:
  std::array<T, size> m_array;

public:
  explicit Vector();
  explicit Vector(const std::array<T, size> &array);
  explicit Vector(const std::vector<T> &array);
  Vector(const Vector<T, size> &v);

  ~Vector()                  = default;
  Vector<T, size> &  operator=(const Vector<T, size> &v);
  [[nodiscard]] bool operator!=(const Vector<T, size> &v) const;
  [[nodiscard]] bool operator==(const Vector<T, size> &v) const;
  [[nodiscard]] bool operator==(const float &v) const;
  [[nodiscard]] bool operator>(const Vector<T, size> &v) const;
  [[nodiscard]] bool operator<(const Vector<T, size> &v) const;
  // the 2 functions above return true only if both members agree with the condition
  // Examples: this > v is true if this = {1, 1} and v = {0, 0} but if this = {1, 0} and v = {0, 1} this will be false
  // this < v will be true if this = {0, 0} and v = {1, 1} but if this = {1, 0} and v = {0, 1} this will be false
  [[nodiscard]] bool            operator>=(const Vector<T, size> &v) const;
  [[nodiscard]] bool            operator<=(const Vector<T, size> &v) const;
  [[nodiscard]] T &             operator[](uint32_t);
  [[nodiscard]] const T &       operator[](uint32_t) const;
  [[nodiscard]] T &             operator()(uint32_t);
  [[nodiscard]] Vector<T, size> operator+(const Vector<T, size> &v) const;
  [[nodiscard]] Vector<T, size> operator-(const Vector<T, size> &v) const;
  [[nodiscard]] Vector<T, size> operator*(const Vector<T, size> &v) const;
  [[nodiscard]] Vector<T, size> operator*(const int &v) const;
  [[nodiscard]] Vector<T, size> operator*(const Matrix<T, size, size> &v) const;
  [[nodiscard]] Vector<T, size> operator*(const float &v) const;
  [[nodiscard]] Vector<T, size> operator/(const Vector<T, size> &v) const;
  [[nodiscard]] Vector<T, size> operator/(const float &v) const;
  [[nodiscard]] Vector<T, size> operator^(const Vector<T, size> &v) const;
  Vector<T, size> &             operator+=(const Vector<T, size> &v);  // This function return a reference to itself to be able to chain itself or with other but the return value may not be used.
  Vector<T, size> &             operator-=(const Vector<T, size> &v);  // This function return a reference to itself to be able to chain itself or with other but the return value may not be used.
  Vector<T, size> &             operator*=(const int &v);
  Vector<T, size> &             operator*=(const float &v);
  Vector<T, size> &             operator*=(const Vector<T, size> &v);  // This function return a reference to itself to be able to chain itself or with other but the return value may not be used.
  Vector<T, size> &             operator/=(const Vector<T, size> &v);  // This function return a reference to itself to be able to chain itself or with other but the return value may not be used.
  Vector<T, size> &             operator/=(const float &v);            // This function return a reference to itself to be able to chain itself or with other but the return value may not be used.
  [[nodiscard]] Vector<T, size> lerp(const Vector<T, size> &_end, float percent) const;
  [[nodiscard]] T               length() const;
  [[nodiscard]] T               dot(const Vector<T, size> &b) const;
  [[nodiscard]] size_t          hash() const;
  void                          normalize();
  [[nodiscard]] Vector<T, size> clamp(const Vector<T, size> &min, const Vector<T, size> &max) const;
  [[nodiscard]] Vector<T, size> rotate(T const &angle, const Vector<T, size> &normal);
};

template <class T, uint32_t size>
Vector<T, size> &Vector<T, size>::operator*=(const int &v) {
  T *this_data = m_array.data();
  for (auto &data : m_array)
    data *= v;
  return *this;
}

template <class T, uint32_t size>
Vector<T, size> &Vector<T, size>::operator*=(const float &v) {
  T *this_data = m_array.data();
  for (auto &data : m_array)
    data *= v;
  return *this;
}

template <class T, uint32_t size>
inline Vector<T, size>::Vector() : m_array() {}  // fix for aurelien

template <class T, uint32_t size>
inline Vector<T, size>::Vector(const std::array<T, size> &array) : m_array{array} {}

template <class T, uint32_t size>
inline Vector<T, size>::Vector(const std::vector<T> &array) : m_array{} {
  std::copy_n(array.begin(), size, m_array.begin());
}

template <class T, uint32_t size>
inline Vector<T, size> &Vector<T, size>::operator=(const Vector<T, size> &v) {
  std::copy_n(v.m_array.begin(), size, m_array.begin());
  return *this;
}

template <class T, uint32_t size>
inline bool Vector<T, size>::operator!=(const Vector<T, size> &v) const {
  return m_array != v.m_array;  // will probably replace by SIMD
}

template <class T, uint32_t size>
inline bool Vector<T, size>::operator==(const Vector<T, size> &v) const {
  return m_array == v.m_array;  // will probably replace by SIMD
}

template <class T, uint32_t size>
inline bool Vector<T, size>::operator>(const Vector<T, size> &v) const {
  T *this_data  = m_array.data();
  T *other_data = v.m_array.data();
  for (int i = 0; i < size; ++i, ++this_data, ++other_data) {
    if (*this_data <= *other_data)
      return false;
  }
  return true;
}

template <class T, uint32_t size>
inline bool Vector<T, size>::operator<(const Vector<T, size> &v) const {
  T *this_data  = m_array.data();
  T *other_data = v.m_array.data();
  for (int i = 0; i < size; ++i, ++this_data, ++other_data) {
    if (*this_data >= *other_data)
      return false;
  }
  return true;
}

template <class T, uint32_t size>
inline bool Vector<T, size>::operator>=(const Vector<T, size> &v) const {
  T *this_data  = m_array.data();
  T *other_data = v.m_array.data();
  for (int i = 0; i < size; ++i, ++this_data, ++other_data) {
    if (*this_data < *other_data)
      return false;
  }
  return true;
}

template <class T, uint32_t size>
inline bool Vector<T, size>::operator<=(const Vector<T, size> &v) const {
  T *this_data  = m_array.data();
  T *other_data = v.m_array;
  for (int i = 0; i < size; ++i, ++this_data, ++other_data) {
    if (*this_data > *other_data)
      return false;
  }
  return true;
}

template <class T, uint32_t size>
inline T &Vector<T, size>::operator[](uint32_t i) {
  return m_array[i];
}

template <class T, uint32_t size>
inline T &Vector<T, size>::operator()(uint32_t i) {
  return m_array[i];
}

template <class T, uint32_t size>
inline Vector<T, size> Vector<T, size>::operator+(const Vector<T, size> &v) const {
  Vector<T, size> ret{m_array};
  const T *       other_data = v.m_array.data();
  T *             this_data  = ret.m_array.data();
  for (int i = 0; i < size; ++i, ++this_data, ++other_data)
    (*this_data) += (*other_data);
  return ret;
}

template <class T, uint32_t size>
inline Vector<T, size> Vector<T, size>::operator-(const Vector<T, size> &v) const {
  Vector<T, size> ret{m_array};
  const T *       other_data = v.m_array.data();
  T *             this_data  = ret.m_array.data();
  for (int i = 0; i < size; ++i, ++this_data, ++other_data)
    *this_data -= *other_data;
  return ret;
}

template <class T, uint32_t size>
inline Vector<T, size> Vector<T, size>::operator*(const Vector<T, size> &v) const {
  Vector<T, size> ret{m_array};
  const T *       other_data = v.m_array.data();
  T *             this_data  = ret.m_array.data();
  for (int i = 0; i < size; ++i, ++this_data, ++other_data)
    *this_data *= *other_data;
  return ret;
}

template <class T, uint32_t size>
inline Vector<T, size> Vector<T, size>::operator/(const Vector<T, size> &v) const {
  Vector<T, size> ret{m_array};
  const T *       other_data = v.m_array.data();
  T *             this_data  = ret.m_array.data();
  for (int i = 0; i < size; ++i, ++this_data, ++other_data) {
    if ((*other_data) == 0.0f)
      throw std::runtime_error{"try to divide by 0"};
    *this_data /= *other_data;
  }
  return ret;
}

template <class T, uint32_t size>
inline Vector<T, size> Vector<T, size>::operator^(const Vector<T, size> &v) const {
  Vector<T, size> ret{};
  T *             this_data  = m_array.data();
  T *             other_data = v.m_array.data();
  T *             ret_data   = ret.m_array.data();
  for (int i = 0; i < size; ++i, ++this_data, ++other_data, ++ret_data)
    *ret_data = *this_data ^ *other_data;
  return ret;
}

template <class T, uint32_t size>
inline Vector<T, size> &Vector<T, size>::operator+=(const Vector<T, size> &v) {
  T *      this_data  = m_array.data();
  const T *other_data = v.m_array.data();
  for (int i = 0; i < size; ++i, ++this_data, ++other_data)
    *this_data += *other_data;
  return *this;
}

template <class T, uint32_t size>
inline Vector<T, size> &Vector<T, size>::operator-=(const Vector<T, size> &v) {
  T *      this_data  = m_array.data();
  const T *other_data = v.m_array.data();
  for (int i = 0; i < size; ++i, ++this_data, ++other_data)
    *this_data -= *other_data;
  return *this;
}

template <class T, uint32_t size>
inline Vector<T, size> &Vector<T, size>::operator*=(const Vector<T, size> &v) {
  T *      this_data  = m_array.data();
  const T *other_data = v.m_array.data();
  for (int i = 0; i < size; ++i, ++this_data, ++other_data)
    *this_data *= *other_data;
  return *this;
}

template <class T, uint32_t size>
inline Vector<T, size> &Vector<T, size>::operator/=(const Vector<T, size> &v) {
  T *      this_data  = m_array.data();
  const T *other_data = v.m_array.data();
  for (int i = 0; i < size; ++i, ++this_data, ++other_data) {
    if (*other_data == 0.0f)
      throw std::runtime_error{"try to divide by 0"};
    *this_data /= *other_data;
  }
  return *this;
}

template <class T, uint32_t size>
inline Vector<T, size> &Vector<T, size>::operator/=(const float &v) {
  T *this_data = m_array.data();
  for (int i = 0; i < size; ++i, ++this_data) {
    if (v == 0.0f)
      throw std::runtime_error{"try to divide by 0"};
    *this_data /= v;
  }
  return *this;
}

template <class T, uint32_t size>
bool Vector<T, size>::operator==(const float &v) const {
  return false;
}

template <class T, uint32_t size>
Vector<T, size> Vector<T, size>::operator*(const int &v) const {
  Vector<T, size> ret{m_array};
  for (auto &data : ret.m_array)
    data *= v;
  return ret;
}

template <class T, uint32_t size>
Vector<T, size> Vector<T, size>::operator*(const float &v) const {
  Vector<T, size> ret{m_array};
  for (auto &data : ret.m_array)
    data *= v;
  return ret;
}

template <class T, uint32_t size>
Vector<T, size> Vector<T, size>::operator/(const float &v) const {
  Vector<T, size> ret{m_array};
  for (auto &data : ret.m_array)
    data /= v;
  return ret;
}

template <class T, uint32_t size>
const T &Vector<T, size>::operator[](uint32_t i) const {
  return m_array[i];
}

template <class T, uint32_t size>
Vector<T, size> Vector<T, size>::lerp(const Vector<T, size> &_end, float percent) const {
  return *this + (_end - *this) * percent;
}

template <class T, uint32_t size>
T Vector<T, size>::length() const {
  return sqrt(std::accumulate(m_array.begin(), m_array.end(), 0.0, [](T prev, T _new) {
    return prev + _new * _new;
  }));
}

template <class T, uint32_t size>
void Vector<T, size>::normalize() {
  T l = length();
  if (l != 0.0f) {
    for (auto &tmp : m_array)
     tmp /= l;
  }
}

template <class T, uint32_t size>
T Vector<T, size>::dot(const Vector<T, size> &b) const {
  T p = 0.0f;
  for (int i{0}; i < size; ++i)
    p += m_array[i] * b[i];
  return p;
}

template <class T, uint32_t size>
Vector<T, size> Vector<T, size>::operator*(const Matrix<T, size, size> &v) const {
  Vector<T, size> ret{};
  for (int i = 0; i < size; ++i) {
    ret[i] = 0.0f;
    for (int j = 0; j < size; ++j)
      ret[i] += v[i][j] * ret[j];
  }
  return ret;
}

template <class T, uint32_t size>
Vector<T, size> Vector<T, size>::rotate(const T &angle, const Vector<T, size> &normal) {
  return *this * cosf(normal) + (normal * *this) * sinf(angle) + normal * (normal.dot(*this)) * (1 - cosf(angle));
}

template <class T, uint32_t size>
Vector<T, size>::Vector(const Vector<T, size> &v) : m_array{v.m_array} {}

template <class T, uint32_t size>
Vector<T, size> Vector<T, size>::clamp(const Vector<T, size> &min, const Vector<T, size> &max) const {
  Vector<T, size> ret{m_array};
  T *             values     = ret.m_array.data();
  const T *       min_values = min.m_array.data();
  const T *       max_values = max.m_array.data();
  for (int i = 0; i < size; ++i) {
    values[i] = std::clamp(values[i], min_values[i], max_values[i]);
  }
  return ret;
}

template <class T, uint32_t size>
size_t Vector<T, size>::hash() const {
  size_t seed = 0;
  for (int i = 0; i < size; ++i)
    seed ^= std::hash<T>()(m_array[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  return seed;
}

template <class T>
class Vector3 : public Vector<T, 3> {
public:
  explicit Vector3(T a, T b, T c);
  Vector3(const Vector3<T> &v);
  Vector3(const Vector<T, 3> &v);  // need to not explicit because used for operations

  explicit Vector3(const std::array<T, 3> &array);
  explicit Vector3(const std::vector<T> &array);
  Vector3<T> &             operator=(const Vector3<T> &v);
  T &                      x;
  T &                      y;
  T &                      z;
  [[nodiscard]] T          getMaxElement() const;
  [[nodiscard]] Vector3<T> cross(const Vector3<T> &b) const;
};

template <class T>
Vector3<T>::Vector3(T a, T b, T c) : Vector<T, 3>{std::array<T, 3>{a, b, c}},
                                     x{this->m_array[0]},
                                     y{this->m_array[1]},
                                     z{this->m_array[2]} {}

template <class T>
Vector3<T> Vector3<T>::cross(const Vector3<T> &b) const {
  return Vector3<T>{y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x};
}

template <class T>
T Vector3<T>::getMaxElement() const {
  T v = x;
  if (y > v) {
    v = y;
  }
  if (z > v) {
    v = z;
  }
  return v;
}

template <class T>
Vector3<T>::Vector3(const std::array<T, 3> &array) : Vector<T, 3>{array},
                                                     x{this->m_array[0]},
                                                     y{this->m_array[1]},
                                                     z{this->m_array[2]} {}

template <class T>
Vector3<T>::Vector3(const std::vector<T> &array) : Vector<T, 3>{array},
                                                   x{this->m_array[0]},
                                                   y{this->m_array[1]},
                                                   z{this->m_array[2]} {}

template <class T>
Vector3<T> &Vector3<T>::operator=(const Vector3<T> &v) {
  x = v.x;
  y = v.y;
  z = v.z;
  return *this;
}

template <class T>
Vector3<T>::Vector3(const Vector3<T> &v) : Vector<T, 3>{v.m_array},
                                           x{this->m_array[0]},
                                           y(this->m_array[1]),
                                           z{this->m_array[2]} {}
template <class T>
Vector3<T>::Vector3(const Vector<T, 3> &v) : Vector<T, 3>{v},
                                             x{this->m_array[0]},
                                             y(this->m_array[1]),
                                             z{this->m_array[2]} {}

using Vector3i = Vector3<int>;
using Vector3f = Vector3<float>;

template <class T>
class Vector2 : public Vector<T, 2> {
public:
  explicit Vector2(T a, T b);
  explicit Vector2(const std::array<T, 2> &array);
  explicit Vector2(const std::vector<T> &array);
  Vector2(const Vector2<T> &v);
  Vector2<T> &             operator=(const Vector2<T> &v);
  T &                      x;
  T &                      y;
  [[nodiscard]] Vector2<T> rotate(T const &angle);
};

template <class T>
Vector2<T>::Vector2(T a, T b) : Vector<T, 2>{std::array<T, 2>{a, b}},
                                x{this->m_array[0]},
                                y{this->m_array[1]} {}

template <class T>
Vector2<T>::Vector2(const std::array<T, 2> &array) : Vector<T, 2>{array},
                                                     x{this->m_array[0]},
                                                     y{this->m_array[1]} {}

template <class T>
Vector2<T>::Vector2(const std::vector<T> &array) : Vector<T, 2>{array},
                                                   x{this->m_array[0]},
                                                   y{this->m_array[1]} {}

template <class T>
Vector2<T> &Vector2<T>::operator=(const Vector2<T> &v) {
  x = v.x;
  y = v.y;
  return *this;
}

template <class T>
Vector2<T> Vector2<T>::rotate(T const &angle) {
  return Vector2<T>(T(x * cosf(angle) - y * sinf(angle)), T(x * sinf(angle) + y * cosf(angle)));
}

template <class T>
Vector2<T>::Vector2(const Vector2<T> &v) : Vector<T, 2>{v.m_array},
                                           x(this->m_array[0]),
                                           y{this->m_array[1]} {}

using Vector2i = Vector2<int>;
using Vector2f = Vector2<float>;
