#pragma once

#include <etl/algorithm.h>
#include <etl/array.h>
#include <math.h>
#include <stddef.h>

namespace bb {

using vec2 = etl::array<float, 2>;
using vec3 = etl::array<float, 3>;
using vec4 = etl::array<float, 4>;
using vec5 = etl::array<float, 5>;
using vec10 = etl::array<float, 10>;
using mat33 = etl::array<etl::array<float, 3>, 3>;
using mat55 = etl::array<etl::array<float, 5>, 5>;

constexpr float PI = 3.14159265358979F;

template <size_t N>
inline etl::array<float, N> operator+(const etl::array<float, N>& a,
                                      const etl::array<float, N>& b) {
  etl::array<float, N> r{};
  for (size_t i = 0; i < N; ++i) r[i] = a[i] + b[i];
  return r;
}

template <size_t N>
inline etl::array<float, N> operator-(const etl::array<float, N>& a,
                                      const etl::array<float, N>& b) {
  etl::array<float, N> r{};
  for (size_t i = 0; i < N; ++i) r[i] = a[i] - b[i];
  return r;
}

template <size_t N>
inline etl::array<float, N> operator*(const etl::array<float, N>& a, float s) {
  etl::array<float, N> r{};
  for (size_t i = 0; i < N; ++i) r[i] = a[i] * s;
  return r;
}

template <size_t N>
inline etl::array<float, N> operator*(float s, const etl::array<float, N>& a) {
  return a * s;
}

template <size_t N>
inline float dot(const etl::array<float, N>& a, const etl::array<float, N>& b) {
  float s = 0.0F;
  for (size_t i = 0; i < N; ++i) s += a[i] * b[i];
  return s;
}

template <size_t N>
inline float norm(const etl::array<float, N>& a) {
  return sqrtf(dot(a, a));
}

inline float max_abs(const vec3& a) {
  return etl::max(fabsf(a[0]), etl::max(fabsf(a[1]), fabsf(a[2])));
}

inline vec3 cross(const vec3& a, const vec3& b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}

inline vec3 mul(const mat33& m, const vec3& v) {
  vec3 r{};
  for (int i = 0; i < 3; ++i)
    r[i] = m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2];
  return r;
}

inline mat33 transpose(const mat33& m) {
  mat33 r{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) r[i][j] = m[j][i];
  return r;
}

inline mat33 skew(const vec3& v) {
  return mat33{{{0.0F, -v[2], v[1]}, {v[2], 0.0F, -v[0]}, {-v[1], v[0], 0.0F}}};
}

inline mat33 mat_mul(const mat33& a, const mat33& b) {
  mat33 r{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      r[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
  return r;
}

inline mat33 inverse3(const mat33& m) {
  const float c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
  const float c01 = m[0][2] * m[2][1] - m[0][1] * m[2][2];
  const float c02 = m[0][1] * m[1][2] - m[0][2] * m[1][1];
  const float det = m[0][0] * c00 + m[1][0] * c01 + m[2][0] * c02;
  const float inv = 1.0F / det;
  mat33 r{};
  r[0][0] = c00 * inv;
  r[0][1] = c01 * inv;
  r[0][2] = c02 * inv;
  r[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv;
  r[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv;
  r[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv;
  r[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv;
  r[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv;
  r[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv;
  return r;
}

inline mat33 mat_add(const mat33& a, const mat33& b) {
  mat33 r{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) r[i][j] = a[i][j] + b[i][j];
  return r;
}

inline mat33 mat_scale(const mat33& a, float s) {
  mat33 r{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) r[i][j] = a[i][j] * s;
  return r;
}

inline mat33 mat_identity3() {
  return mat33{{{1.0F, 0.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, {0.0F, 0.0F, 1.0F}}};
}

// --- Quaternion helpers (w-first, matching ballbotEstimatorStep) ---

inline vec4 quaternion_multiply(const vec4& q1, const vec4& q2) {
  return {q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
          q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
          q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1],
          q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]};
}

inline vec4 integrate_quaternion(const vec4& q, const vec3& rate, float dt) {
  vec4 r{};
  r[0] = q[0] + 0.5F * dt * (-rate[0] * q[1] - rate[1] * q[2] - rate[2] * q[3]);
  r[1] = q[1] + 0.5F * dt * (rate[0] * q[0] + rate[2] * q[2] - rate[1] * q[3]);
  r[2] = q[2] + 0.5F * dt * (rate[1] * q[0] - rate[2] * q[1] + rate[0] * q[3]);
  r[3] = q[3] + 0.5F * dt * (rate[2] * q[0] + rate[1] * q[1] - rate[0] * q[2]);
  const float n = etl::max(norm(r), 1e-30F);
  for (auto& v : r) v /= n;
  return r;
}

inline mat33 quaternion_to_rotation(const vec4& q_in) {
  vec4 q = q_in;
  const float n = etl::max(norm(q), 1e-30F);
  for (auto& v : q) v /= n;
  const float w = q[0];
  const float x = q[1];
  const float y = q[2];
  const float z = q[3];
  return mat33{{{1.0F - 2.0F * (y * y + z * z), 2.0F * (x * y - z * w),
                 2.0F * (x * z + y * w)},
                {2.0F * (x * y + z * w), 1.0F - 2.0F * (x * x + z * z),
                 2.0F * (y * z - x * w)},
                {2.0F * (x * z - y * w), 2.0F * (y * z + x * w),
                 1.0F - 2.0F * (x * x + y * y)}}};
}

inline vec3 rotation_to_euler(const mat33& r) {
  const float pitch = asinf(etl::clamp(-r[2][0], -1.0F, 1.0F));
  const float roll = atan2f(r[2][1], r[2][2]);
  const float yaw = atan2f(r[1][0], r[0][0]);
  return {roll, pitch, yaw};
}

// --- 5x5 helpers for the EKF ---

inline vec5 mul(const mat55& m, const vec5& v) {
  vec5 r{};
  for (int i = 0; i < 5; ++i) {
    float s = 0.0F;
    for (int j = 0; j < 5; ++j) s += m[i][j] * v[j];
    r[i] = s;
  }
  return r;
}

inline mat55 mul(const mat55& a, const mat55& b) {
  mat55 r{};
  for (int i = 0; i < 5; ++i)
    for (int j = 0; j < 5; ++j) {
      float s = 0.0F;
      for (int k = 0; k < 5; ++k) s += a[i][k] * b[k][j];
      r[i][j] = s;
    }
  return r;
}

inline mat55 transpose(const mat55& m) {
  mat55 r{};
  for (int i = 0; i < 5; ++i)
    for (int j = 0; j < 5; ++j) r[i][j] = m[j][i];
  return r;
}

inline mat55 mat_add(const mat55& a, const mat55& b) {
  mat55 r{};
  for (int i = 0; i < 5; ++i)
    for (int j = 0; j < 5; ++j) r[i][j] = a[i][j] + b[i][j];
  return r;
}

inline mat55 mat_identity5() {
  mat55 r{};
  for (int i = 0; i < 5; ++i)
    for (int j = 0; j < 5; ++j) r[i][j] = (i == j) ? 1.0F : 0.0F;
  return r;
}

}  // namespace bb
