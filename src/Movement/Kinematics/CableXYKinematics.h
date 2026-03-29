#ifndef CABLEXYKINEMATICS_H
#define CABLEXYKINEMATICS_H

#include "Kinematics.h"

// 2D Cable Robot Kinematics for RepRapFirmware
// 4 motors, one in each corner of a rectangle
// Configure with: M669 K100 A-399:-253 B399:-253 C399:253 D-399:253

class CableXYKinematics : public Kinematics
{
public:
    CableXYKinematics() noexcept;

    const char *_ecv_array GetName(bool forStatusReport = false) const noexcept override;

    bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply,
                   bool& error) THROWS(GCodeException) override;

    MovementError CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[],
                                         size_t numVisibleAxes, size_t numTotalAxes,
                                         int32_t motorPos[], bool isCoordinated) const noexcept override;

    void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[],
                                size_t numVisibleAxes, size_t numTotalAxes,
                                float machinePos[]) const noexcept override;

    bool IsReachable(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept override;

    LimitPositionResult LimitPosition(float finalCoords[], const float *_ecv_array _ecv_null initialCoords,
                                       size_t numVisibleAxes, AxesBitmap axesToLimit,
                                       bool isCoordinated, bool applyM208Limits) const noexcept override;

    void GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept override;

    HomingMode GetHomingMode() const noexcept override { return HomingMode::homeCartesianAxes; }

    AxesBitmap AxesToHomeBeforeProbing() const noexcept override { return AxesBitmap(); }

    bool SupportsAutoCalibration() const noexcept override { return false; }

private:
    float anchorX[4];
    float anchorY[4];

    float CableLength(int motor, float x, float y) const noexcept;

    static constexpr float DefaultAnchorX[4] = { -399.0, 399.0,  399.0, -399.0 };
    static constexpr float DefaultAnchorY[4] = { -253.0, -253.0, 253.0,  253.0 };
};

#endif // CABLEXYKINEMATICS_H
