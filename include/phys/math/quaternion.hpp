#pragma once
#include <cmath>
#include "phys/math/vec3.hpp"
#include "phys/math/mat3.hpp"

namespace phys {

struct Quaternion
{
    float x{};
    float y{};
    float z{};
    float w{1.0f};

    static constexpr Quaternion identity() { return {0.0f, 0.0f, 0.0f, 1.0f}; }

    // axis must be unit-length; angleRadians is the total rotation about axis.
    static Quaternion fromAxisAngle(Vec3 axis, float angleRadians) {
        float half = angleRadians * 0.5f;
        float s = std::sin(half);
        return {axis.x * s, axis.y * s, axis.z * s, std::cos(half)};
    }

    Quaternion operator*(const Quaternion& other) const {
        return {
            w * other.x + x * other.w + y * other.z - z * other.y,
            w * other.y - x * other.z + y * other.w + z * other.x,
            w * other.z + x * other.y - y * other.x + z * other.w,
            w * other.w - x * other.x - y * other.y - z * other.z
        };
    }

    Quaternion conjugate() const {
        return {-x, -y, -z, w};
    }

    float length() const {
        return std::sqrt(x * x + y * y + z * z + w * w);
    }

    Quaternion normalized() const {
        float len = length();
        if (len == 0.0f) return identity();
        return {x / len, y / len, z / len, w / len};
    }

    Vec3 rotate(const Vec3& v) const {
        Quaternion p{v.x, v.y, v.z, 0.0f};
        Quaternion r = (*this) * p * conjugate();
        return {r.x, r.y, r.z};
    }

    // Prefer this over rotate() when transforming many vectors with the same orientation.
    Mat3 toMat3() const {
        float xx = x * x, yy = y * y, zz = z * z;
        float xy = x * y, xz = x * z, yz = y * z;
        float wx = w * x, wy = w * y, wz = w * z;

        return {
            1.0f - 2.0f * (yy + zz), 2.0f * (xy - wz),        2.0f * (xz + wy),
            2.0f * (xy + wz),        1.0f - 2.0f * (xx + zz), 2.0f * (yz - wx),
            2.0f * (xz - wy),        2.0f * (yz + wx),        1.0f - 2.0f * (xx + yy)
        };
    }
};

}

