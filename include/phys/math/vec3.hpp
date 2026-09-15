#pragma once

namespace phys {

struct Vec3 
{
    float x{};
    float y{};
    float z{};

    Vec3& operator+=(const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;

        return *this;
    }

    Vec3 operator-() const {
        return {-x, -y, -z};
    }

    static constexpr Vec3 zero() { return {0.0f, 0.0f, 0.0f}; }

};

inline Vec3 operator+(Vec3 left, const Vec3& right) {
    left += right;
    return left;
}

inline Vec3 operator-(Vec3 left, const Vec3& right) {
    left.x -= right.x;
    left.y -= right.y;
    left.z -= right.z;
    return left;
}

inline Vec3 operator*(Vec3 value, float scalar) {
    return {value.x * scalar, value.y * scalar, value.z * scalar};
}

inline Vec3 operator*(float scalar, Vec3 value) {
    return value * scalar;
}

inline Vec3 operator/(Vec3 value, float scalar) {
    return {value.x / scalar, value.y / scalar, value.z / scalar};
}

}
