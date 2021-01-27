#include "Quaternion.hpp"

Quaternion::Quaternion(float _x, float _y, float _z, float _w) : x(_x), y(_y), z(_z), w(_w) {}

void Quaternion::normalize() {
  float mag = sqrt(w * w + x * x + y * y + z * z);
  if (mag > 0.0f) {
    float t = 1.0f / mag;
    w *= t;
    x *= t;
    y *= t;
    z *= t;
  }
}

Matrix<float, 4, 4> Quaternion::toRotationMatrix() const {
  const float         xy       = x * y;
  const float         xz       = x * z;
  const float         xw       = x * w;
  const float         yz       = y * z;
  const float         yw       = y * w;
  const float         zw       = z * w;
  const float         xSquared = x * x;
  const float         ySquared = y * y;
  const float         zSquared = z * z;
  Matrix<float, 4, 4> ret{{{1 - 2 * (ySquared + zSquared), 2 * (xy - zw), 2 * (xz + yw), 0}, {2 * (xy + zw), 1 - 2 * (xSquared + zSquared), 2 * (yz - xw), 0}, {2 * (xz - yw), 2 * (yz + xw), 1 - 2 * (xSquared + ySquared), 0}, {0, 0, 0, 1}}};
  return ret;
}

Quaternion Quaternion::fromMatrix(Matrix<float, 4, 4> mat) {
  float w, x, y, z;
  float diagonal = mat[0][0] + mat[1][1] + mat[2][2];
  if (diagonal > 0) {
    float w4 = sqrt(diagonal + 1.0f) * 2.0f;
    w        = w4 / 4.0f;
    x        = (mat[2][1] - mat[1][2]) / w4;
    y        = (mat[0][2] - mat[2][0]) / w4;
    z        = (mat[1][0] - mat[0][1]) / w4;
  } else if ((mat[0][0] > mat[1][1]) && (mat[0][0] > mat[2][2])) {
    float x4 = sqrt(1.0f + mat[0][0] - mat[1][1] - mat[2][2]) * 2.0f;
    w        = (mat[2][1] - mat[1][2]) / x4;
    x        = x4 / 4.0f;
    y        = (mat[0][1] + mat[1][0]) / x4;
    z        = (mat[0][2] + mat[2][0]) / x4;
  } else if (mat[1][1] > mat[2][2]) {
    float y4 = sqrt(1.0f + mat[1][1] - mat[0][0] - mat[2][2]) * 2.0f;
    w        = (mat[0][2] - mat[2][0]) / y4;
    x        = (mat[0][1] + mat[1][0]) / y4;
    y        = y4 / 4.0f;
    z        = (mat[1][2] + mat[2][1]) / y4;
  } else {
    float z4 = sqrt(1.0f + mat[2][2] - mat[0][0] - mat[1][1]) * 2.0f;
    w        = (mat[1][0] - mat[0][1]) / z4;
    x        = (mat[0][2] + mat[2][0]) / z4;
    y        = (mat[1][2] + mat[2][1]) / z4;
    z        = z4 / 4.0f;
  }
  return Quaternion(x, y, z, w);
}

Quaternion Quaternion::fromMatrix(Matrix<float, 3, 3> mat) {
  float w = sqrt(std::max(0.000000001f, (1.0f + mat[0][0] + mat[1][1] + mat[2][2]))) * 0.5f;

  float qrFour      = 4.0f * w;
  float qrFourRecip = 1.0f / qrFour;

  float x = (mat[1][2] - mat[2][1]) * qrFourRecip;
  float y = (mat[2][0] - mat[0][2]) * qrFourRecip;
  float z = (mat[0][1] - mat[1][0]) * qrFourRecip;
  return Quaternion(x, y, z, w);
}

Quaternion Quaternion::nlerp(Quaternion a, Quaternion b, float blend) {
  Quaternion result{0, 0, 0, 1};
  float      dot    = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
  float      blendI = 1.0f - blend;
  if (dot < 0) {
    result.w = blendI * a.w + blend * -b.w;
    result.x = blendI * a.x + blend * -b.x;
    result.y = blendI * a.y + blend * -b.y;
    result.z = blendI * a.z + blend * -b.z;
  } else {
    result.w = blendI * a.w + blend * b.w;
    result.x = blendI * a.x + blend * b.x;
    result.y = blendI * a.y + blend * b.y;
    result.z = blendI * a.z + blend * b.z;
  }
  result.normalize();
  return result;
}

Quaternion Quaternion::slerp(Quaternion a, Quaternion b, float blend) {
  float dotproduct = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
  // algorithm adapted from Shoemake's paper
  blend       = blend / 2.0f;
  float theta = acosf(dotproduct);
  if (theta < 0.0f)
    theta = -theta;

  const float st     = sinf(theta);
  const float sut    = sinf(blend * theta);
  const float sout   = sinf((1 - blend) * theta);
  const float coeff1 = sout / st;
  const float coeff2 = sut / st;

  return Quaternion{coeff1 * a.x + coeff2 * b.x, coeff1 * a.y + coeff2 * b.y, coeff1 * a.z + coeff2 * b.z, coeff1 * a.w + coeff2 * b.w};
}

Quaternion Quaternion::operator*(const Quaternion &b) const {
  return Quaternion((x * b.w) + (w * b.x) + (y * b.z) - (z * b.y), (y * b.w) + (w * b.y) + (z * b.x) - (x * b.z), (z * b.w) + (w * b.z) + (x * b.y) - (y * b.x), (w * b.w) - (x * b.x) - (y * b.y) - (z * b.z));
}

Quaternion Quaternion::operator+(const Quaternion &a) const {
  return Quaternion(x + a.x, y + a.y, z + a.z, w + a.w);
}

Quaternion Quaternion::conjugate() const {
  return Quaternion(-x, -y, -z, w);
}

Matrix<float, 3, 3> Quaternion::toMatrix3() const {
  Matrix<float, 3, 3> mat;

  float yy = y * y;
  float zz = z * z;
  float xy = x * y;
  float zw = z * w;
  float xz = x * z;
  float yw = y * w;
  float xx = x * x;
  float yz = y * z;
  float xw = x * w;

  mat[0][0] = 1 - 2 * yy - 2 * zz;
  mat[0][1] = 2 * xy + 2 * zw;
  mat[0][2] = 2 * xz - 2 * yw;

  mat[1][0] = 2 * xy - 2 * zw;
  mat[1][1] = 1 - 2 * xx - 2 * zz;
  mat[1][2] = 2 * yz + 2 * xw;

  mat[2][0] = 2 * xz + 2 * yw;
  mat[2][1] = 2 * yz - 2 * xw;
  mat[2][2] = 1 - 2 * xx - 2 * yy;
  return mat;
}
