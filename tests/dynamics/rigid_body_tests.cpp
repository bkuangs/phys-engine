#include <phys/core/constants.hpp>
#include <phys/dynamics/rigid_body.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {

constexpr float tolerance = 1e-5f;

bool near(float actual, float expected)
{
    return std::abs(actual - expected) <= tolerance;
}

bool testSphereUsesVolumeForMass()
{
    phys::RigidBody body;
    std::string error;

    if (!phys::RigidBody::createSphere(
            1.0f, {}, 1.0f, false, 0.0f, body, error)) {
        std::cerr << "sphere creation failed: " << error << '\n';
        return false;
    }

    const float expectedMass = (4.0f / 3.0f) * phys::MathConstants::pi;
    if (!near(body.mass, expectedMass)) {
        std::cerr << "expected sphere mass " << expectedMass
            << ", got " << body.mass << '\n';
        return false;
    }
    return true;
}

bool testDensityValidation()
{
    phys::RigidBody body;
    std::string error;

    if (phys::RigidBody::createBox(1.0f, 1.0f, 1.0f, {},
            phys::BodyLimits::minDensity - 0.01f, false, 0.0f, body, error)
        || error != "Density is out of range") {
        std::cerr << "density below minimum was accepted\n";
        return false;
    }
    if (phys::RigidBody::createBox(1.0f, 1.0f, 1.0f, {},
            phys::BodyLimits::maxDensity + 0.01f, false, 0.0f, body, error)
        || error != "Density is out of range") {
        std::cerr << "density above maximum was accepted\n";
        return false;
    }
    if (phys::RigidBody::createBox(1.0f, 1.0f, 1.0f, {},
            std::numeric_limits<float>::quiet_NaN(), false, 0.0f, body, error)
        || error != "Density is out of range") {
        std::cerr << "NaN density was accepted\n";
        return false;
    }

    return phys::RigidBody::createBox(1.0f, 1.0f, 1.0f, {},
            phys::BodyLimits::minDensity, false, 0.0f, body, error)
        && phys::RigidBody::createBox(1.0f, 1.0f, 1.0f, {},
            phys::BodyLimits::maxDensity, false, 0.0f, body, error);
}

}

int main()
{
    return testSphereUsesVolumeForMass() && testDensityValidation() ? 0 : 1;
}