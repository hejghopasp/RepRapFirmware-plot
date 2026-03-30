#ifndef CABLEXYKINEMATICS_H
#define CABLEXYKINEMATICS_H

#include "Kinematics.h"

// 2D Cable Robot Kinematics for RepRapFirmware
// 4 motors, one in each corner of a rectangle
//
// Configure anchors (motor positions):
//   M669 K100 A-399:-253 B399:-253 C399:253 D-399:253
//
// Configure effector attachment points (where cable meets plate):
//   M669 EA-15.5:-15.5 EB15.5:-15.5 EC15.5:15.5 ED-15.5:15.5
//
// EA = where cable A attaches on effector plate, relative to plate centre
// EB = where cable B attaches on effector plate, relative to plate centre
// EC = where cable C attaches on effector plate, relative to plate centre
// ED = where cable D attaches on effector plate, relative to plate centre

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
    // Anchor positions = motor/pulley positions in the frame
    float anchorX[4];
    float anchorY[4];

    // Effector attachment offsets = where each cable connects on the effector plate
    // relative to the effector centre. Default 0 = all cables meet at centre.
    float effectorOffsetX[4];
    float effectorOffsetY[4];

    float CableLength(int motor, float x, float y) const noexcept;
};

#endif // CABLEXYKINEMATICS_H
