#ifndef CABLEXYKINEMATICS_H
#define CABLEXYKINEMATICS_H

#include "Kinematics.h"

// 2D Cable Robot Kinematics with tension distribution
// 4 motors in corners, cable tension kept above minimum at all times
//
// M669 K100 Ax:y Bx:y Cx:y Dx:y    anchor positions
// M669 Eax:ay:bx:by:cx:cy:dx:dy     effector attachment offsets
// M669 Wkg                           effector weight in kg
// M669 Tnn                           target minimum tension in Newton (default 20)
// M669 Xnn                           max tension in Newton (default 70)
// M669 Snn                           line spring constant mm/N (default 0.01)

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
    float effectorOffsetX[4];
    float effectorOffsetY[4];

    float moverWeight_kg   = 0.0f;
    float minTension       = 20.0f;   // N
    float maxTension       = 70.0f;   // N
    float springConstant   = 0.01f;   // mm per Newton – how much cable stretches per N of tension

    float CableLength(int motor, float x, float y) const noexcept;
    void  ComputeTensions(float x, float y, float tensions[4]) const noexcept;

    static constexpr float DefaultAnchorX[4]        = { -395.0f,  394.0f,  479.0f, -322.0f };
    static constexpr float DefaultAnchorY[4]        = { -270.0f, -270.0f,  212.0f,  249.0f };
    static constexpr float DefaultEffectorOffsetX[4] = { -15.5f,  15.5f,  15.5f, -15.5f };
    static constexpr float DefaultEffectorOffsetY[4] = { -15.5f, -15.5f,  15.5f,  15.5f };
};

#endif
