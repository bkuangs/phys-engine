#pragma once

namespace phys {

struct BodyLimits
{
    static constexpr float minSize = 0.01f * 0.01f;
    static constexpr float maxSize = 64.0f * 64.0f;
    static constexpr float minDensity = 0.2f;   // g/cm^3
    static constexpr float maxDensity = 21.4f;  // platinum
};

}
