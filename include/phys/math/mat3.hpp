#pragma once

#include "phys/math/vec3.hpp"

namespace phys {

struct Mat3 {
    float m00{}, m01{}, m02{};
    float m10{}, m11{}, m12{};
    float m20{}, m21{}, m22{};

    static Mat3 identity() {
        return {
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f
        };
    }

    Vec3 operator*(const Vec3& value) const {
        return {
            m00 * value.x + m01 * value.y + m02 * value.z,
            m10 * value.x + m11 * value.y + m12 * value.z,
            m20 * value.x + m21 * value.y + m22 * value.z
        };
    }

    Mat3& operator+=(const Mat3& other) {
        m00 += other.m00; m01 += other.m01; m02 += other.m02;
        m10 += other.m10; m11 += other.m11; m12 += other.m12;
        m20 += other.m20; m21 += other.m21; m22 += other.m22;
        return *this;
    }
};

inline Mat3 operator+(Mat3 left, const Mat3& right) {
    left += right;
    return left;
}

inline Mat3 transpose(const Mat3& value) {
    return {
        value.m00, value.m10, value.m20,
        value.m01, value.m11, value.m21,
        value.m02, value.m12, value.m22
    };
}

inline Mat3 operator*(const Mat3& left, const Mat3& right) {
    return {
        left.m00 * right.m00 + left.m01 * right.m10 + left.m02 * right.m20,
        left.m00 * right.m01 + left.m01 * right.m11 + left.m02 * right.m21,
        left.m00 * right.m02 + left.m01 * right.m12 + left.m02 * right.m22,

        left.m10 * right.m00 + left.m11 * right.m10 + left.m12 * right.m20,
        left.m10 * right.m01 + left.m11 * right.m11 + left.m12 * right.m21,
        left.m10 * right.m02 + left.m11 * right.m12 + left.m12 * right.m22,

        left.m20 * right.m00 + left.m21 * right.m10 + left.m22 * right.m20,
        left.m20 * right.m01 + left.m21 * right.m11 + left.m22 * right.m21,
        left.m20 * right.m02 + left.m21 * right.m12 + left.m22 * right.m22
    };
}

}
