#ifndef CABLEXYKINEMATICS_H
#define CABLEXYKINEMATICS_H

#include "Kinematics.h"
#include <Movement/MovementError.h>

// 2D Cable Robot Kinematics for RepRapFirmware
// 4 motors, one in each corner of a rectangle
// Motors A(X), B(Y), C(Z), D(U) pull cables to a central effector
//
// Coordinate system (viewed from above):
//   D----C
//   |    |
//   A----B
//
// Configure anchors with M669:
//   M669 K100 Ax:y Bx:y Cx:y Dx:y
//
// Example for 800x565mm frame centered at origin:
//   M669 K100 A-399:-253 B399:-253 C399:253 D-399:253

class CableXYKinematics : public Kinematics
{
public:
    CableXYKinematics() noexcept;

    // Overridden base class functions
    const char *GetName(bool forStatusReport) const noexcept override;

    bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply,
                   bool& error) noexcept override;

    MovementError CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[],
                                         size_t numVisibleAxes, size_t numTotalAxes,
                                         int32_t motorPos[], bool isCoordinated) const noexcept override;

    void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[],
                                size_t numVisibleAxes, size_t numTotalAxes,
                                float machinePos[]) const noexcept override;

    bool IsReachable(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept override;

    LimitPositionResult LimitPosition(float finalCoords[], const float * null initialCoords,
                                       size_t numAxes, AxesBitmap axesToLimit,
                                       bool isCoordinated, bool applyM208Limits) const noexcept override;

    void GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept override;

    HomingMode GetHomingMode() const noexcept override { return HomingMode::homeCartesianAxes; }

    AxesBitmap AxesToHomeBeforeProbing() const noexcept override { return AxesBitmap(); }

    AxesBitmap MustHomeAxesFirst(const AxesBitmap axesHomed) const noexcept override
        { return AxesBitmap(); }

    bool QueryTerminateHomingMove(unsigned int axis) const noexcept override { return false; }

    void OnHomingSwitchTriggered(unsigned int axis, bool highEnd,
                                  const float stepsPerMm[], DDA& dda) const noexcept override {}

    bool SupportsAutoCalibration() const noexcept override { return false; }

private:
    // Anchor positions in mm from origin (center of workspace)
    // A = bottom-left, B = bottom-right, C = top-right, D = top-left
    float anchorX[4];   // X coordinates of A, B, C, D
    float anchorY[4];   // Y coordinates of A, B, C, D

    // Compute cable length from anchor[i] to effector at (x, y)
    float CableLength(int motor, float x, float y) const noexcept;

    // Default anchor positions (800x565mm frame)
    static constexpr float DefaultAnchorX[4] = { -399.0, 399.0,  399.0, -399.0 };
    static constexpr float DefaultAnchorY[4] = { -253.0, -253.0, 253.0,  253.0 };
};

#endif // CABLEXYKINEMATICS_H
