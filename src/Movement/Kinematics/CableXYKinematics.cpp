#include "CableXYKinematics.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"
#include <cmath>

constexpr float CableXYKinematics::DefaultAnchorX[4];
constexpr float CableXYKinematics::DefaultAnchorY[4];

CableXYKinematics::CableXYKinematics() noexcept
    : Kinematics(KinematicsType::cableXY, SegmentationType(true, true, true))
{
    for (int i = 0; i < 4; i++)
    {
        anchorX[i] = DefaultAnchorX[i];
        anchorY[i] = DefaultAnchorY[i];
    }
}

const char *_ecv_array CableXYKinematics::GetName(bool forStatusReport) const noexcept
{
    return "CableXY";
}

float CableXYKinematics::CableLength(int motor, float x, float y) const noexcept
{
    const float dx = x - anchorX[motor];
    const float dy = y - anchorY[motor];
    return sqrtf(dx * dx + dy * dy);
}

bool CableXYKinematics::Configure(unsigned int mCode, GCodeBuffer& gb,
                                   const StringRef& reply, bool& error) THROWS(GCodeException)
{
    if (mCode == 669)
    {
        bool seen = false;

        if (gb.Seen('A'))
        {
            float coords[2];
            size_t numCoords = 2;
            gb.GetFloatArray(coords, numCoords, false);
            if (numCoords >= 2) { anchorX[0] = coords[0]; anchorY[0] = coords[1]; seen = true; }
        }
        if (gb.Seen('B'))
        {
            float coords[2];
            size_t numCoords = 2;
            gb.GetFloatArray(coords, numCoords, false);
            if (numCoords >= 2) { anchorX[1] = coords[0]; anchorY[1] = coords[1]; seen = true; }
        }
        if (gb.Seen('C'))
        {
            float coords[2];
            size_t numCoords = 2;
            gb.GetFloatArray(coords, numCoords, false);
            if (numCoords >= 2) { anchorX[2] = coords[0]; anchorY[2] = coords[1]; seen = true; }
        }
        if (gb.Seen('D'))
        {
            float coords[2];
            size_t numCoords = 2;
            gb.GetFloatArray(coords, numCoords, false);
            if (numCoords >= 2) { anchorX[3] = coords[0]; anchorY[3] = coords[1]; seen = true; }
        }

        if (!seen)
        {
            reply.printf("CableXY anchors: A(%.1f,%.1f) B(%.1f,%.1f) C(%.1f,%.1f) D(%.1f,%.1f)",
                         (double)anchorX[0], (double)anchorY[0],
                         (double)anchorX[1], (double)anchorY[1],
                         (double)anchorX[2], (double)anchorY[2],
                         (double)anchorX[3], (double)anchorY[3]);
        }
        return seen;
    }
    return Kinematics::Configure(mCode, gb, reply, error);
}

MovementError CableXYKinematics::CartesianToMotorSteps(const float machinePos[],
                                                        const float stepsPerMm[],
                                                        size_t numVisibleAxes,
                                                        size_t numTotalAxes,
                                                        int32_t motorPos[],
                                                        bool isCoordinated) const noexcept
{
    const float x = machinePos[0];
    const float y = machinePos[1];
    MovementError err = MovementError::ok;

    for (int i = 0; i < 4; i++)
    {
        const float length = CableLength(i, x, y);
        RoundToInt32(err, length * stepsPerMm[i], motorPos[i]);
    }

    return err;
}

void CableXYKinematics::MotorStepsToCartesian(const int32_t motorPos[],
                                               const float stepsPerMm[],
                                               size_t numVisibleAxes,
                                               size_t numTotalAxes,
                                               float machinePos[]) const noexcept
{
    float L[4];
    for (int i = 0; i < 4; i++)
    {
        L[i] = (float)motorPos[i] / stepsPerMm[i];
    }

    float x = 0.0f;
    float y = 0.0f;

    const int maxIter = 100;
    const float tolerance = 0.001f;

    for (int iter = 0; iter < maxIter; iter++)
    {
        float JtJxx = 0.0f, JtJxy = 0.0f, JtJyy = 0.0f;
        float JtFx  = 0.0f, JtFy  = 0.0f;

        for (int i = 0; i < 4; i++)
        {
            const float dx = x - anchorX[i];
            const float dy = y - anchorY[i];
            const float len = sqrtf(dx * dx + dy * dy);
            if (len < 0.001f) continue;

            const float residual = len - L[i];
            const float jx = dx / len;
            const float jy = dy / len;

            JtJxx += jx * jx;
            JtJxy += jx * jy;
            JtJyy += jy * jy;
            JtFx  += jx * residual;
            JtFy  += jy * residual;
        }

        const float det = JtJxx * JtJyy - JtJxy * JtJxy;
        if (fabsf(det) < 1e-10f) break;

        const float dx = (JtJyy * JtFx - JtJxy * JtFy) / det;
        const float dy = (JtJxx * JtFy - JtJxy * JtFx) / det;

        x -= dx;
        y -= dy;

        if (fabsf(dx) < tolerance && fabsf(dy) < tolerance) break;
    }

    machinePos[0] = x;
    machinePos[1] = y;
    for (size_t i = 2; i < numVisibleAxes; i++) { machinePos[i] = 0.0f; }
}

bool CableXYKinematics::IsReachable(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept
{
    float minX = anchorX[0], maxX = anchorX[0];
    float minY = anchorY[0], maxY = anchorY[0];
    for (int i = 1; i < 4; i++)
    {
        if (anchorX[i] < minX) minX = anchorX[i];
        if (anchorX[i] > maxX) maxX = anchorX[i];
        if (anchorY[i] < minY) minY = anchorY[i];
        if (anchorY[i] > maxY) maxY = anchorY[i];
    }
    const float margin = 20.0f;
    return axesCoords[0] > (minX + margin) && axesCoords[0] < (maxX - margin)
        && axesCoords[1] > (minY + margin) && axesCoords[1] < (maxY - margin);
}

LimitPositionResult CableXYKinematics::LimitPosition(float finalCoords[],
                                                       const float *_ecv_array _ecv_null initialCoords,
                                                       size_t numVisibleAxes,
                                                       AxesBitmap axesToLimit,
                                                       bool isCoordinated,
                                                       bool applyM208Limits) const noexcept
{
    return (applyM208Limits && LimitPositionFromAxis(finalCoords, 0, numVisibleAxes, axesToLimit))
        ? LimitPositionResult::adjusted : LimitPositionResult::ok;
}

void CableXYKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept
{
    for (size_t i = 0; i < numAxes; i++) { positions[i] = 0.0f; }
}
