#include "CableXYKinematics.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"
#include <cmath>

constexpr float CableXYKinematics::DefaultAnchorX[4];
constexpr float CableXYKinematics::DefaultAnchorY[4];
constexpr float CableXYKinematics::DefaultEffectorOffsetX[4];
constexpr float CableXYKinematics::DefaultEffectorOffsetY[4];

CableXYKinematics::CableXYKinematics() noexcept
    : Kinematics(KinematicsType::cableXY, SegmentationType(true, true, true))
{
    for (int i = 0; i < 4; i++)
    {
        anchorX[i]         = DefaultAnchorX[i];
        anchorY[i]         = DefaultAnchorY[i];
        effectorOffsetX[i] = DefaultEffectorOffsetX[i];
        effectorOffsetY[i] = DefaultEffectorOffsetY[i];
    }
}

const char *_ecv_array CableXYKinematics::GetName(bool forStatusReport) const noexcept
{
    return "CableXY";
}

float CableXYKinematics::CableLength(int motor, float x, float y) const noexcept
{
    const float attachX = x + effectorOffsetX[motor];
    const float attachY = y + effectorOffsetY[motor];
    const float dx = anchorX[motor] - attachX;
    const float dy = anchorY[motor] - attachY;
    return sqrtf(dx * dx + dy * dy);
}

// Compute cable tensions using minimum-norm pseudoinverse tension distribution.
// Finds tensions t[i] >= minTension such that sum(J_i * t[i]) = gravity_force
// where J_i is the unit vector from attachment point to anchor.
void CableXYKinematics::ComputeTensions(float x, float y, float tensions[4]) const noexcept
{
    // Unit vectors from attachment points toward anchors
    float jx[4], jy[4];
    for (int i = 0; i < 4; i++)
    {
        const float ax = anchorX[i] - (x + effectorOffsetX[i]);
        const float ay = anchorY[i] - (y + effectorOffsetY[i]);
        const float len = sqrtf(ax * ax + ay * ay);
        if (len < 0.001f) { jx[i] = 0.0f; jy[i] = 0.0f; continue; }
        jx[i] = ax / len;
        jy[i] = ay / len;
    }

    // External force to counteract (gravity pulls down in -Y direction)
    const float fg = moverWeight_kg * 9.81f;
    const float fx_ext = 0.0f;
    const float fy_ext = fg;   // cables must pull UP to counter gravity

    // Start with minimum tension on all cables
    for (int i = 0; i < 4; i++) tensions[i] = minTension;

    // Current net force with minimum tensions
    float fx = fx_ext;
    float fy = fy_ext;
    for (int i = 0; i < 4; i++)
    {
        fx -= jx[i] * tensions[i];
        fy -= jy[i] * tensions[i];
    }

    // Use pseudoinverse J^+ = J^T * (J*J^T)^-1 to find minimum-norm correction
    // J is 2x4, J^T is 4x2, J*J^T is 2x2
    float JJT00 = 0.0f, JJT01 = 0.0f, JJT11 = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        JJT00 += jx[i] * jx[i];
        JJT01 += jx[i] * jy[i];
        JJT11 += jy[i] * jy[i];
    }

    const float det = JJT00 * JJT11 - JJT01 * JJT01;
    if (fabsf(det) > 1e-6f)
    {
        // (J*J^T)^-1 * [fx, fy]
        const float rx = ( fx * JJT11 - fy * JJT01) / det;
        const float ry = (-fx * JJT01 + fy * JJT00) / det;

        // delta_t = J^T * [rx, ry]
        for (int i = 0; i < 4; i++)
        {
            tensions[i] += jx[i] * rx + jy[i] * ry;
        }
    }

    // Ensure all tensions >= minTension by shifting via null space
    // Null space of J for 4 cables is 2D – simplest shift is uniform
    float minT = tensions[0];
    for (int i = 1; i < 4; i++) if (tensions[i] < minT) minT = tensions[i];
    if (minT < minTension)
    {
        const float shift = minTension - minT;
        for (int i = 0; i < 4; i++) tensions[i] += shift;
    }

    // Clamp to max tension
    for (int i = 0; i < 4; i++)
    {
        if (tensions[i] > maxTension) tensions[i] = maxTension;
    }
}

bool CableXYKinematics::Configure(unsigned int mCode, GCodeBuffer& gb,
                                   const StringRef& reply, bool& error) THROWS(GCodeException)
{
    if (mCode == 669)
    {
        bool seen = false;

        if (gb.Seen('A'))
        {
            float c[2]; size_t n = 2;
            gb.GetFloatArray(c, n, false);
            if (n >= 2) { anchorX[0] = c[0]; anchorY[0] = c[1]; seen = true; }
        }
        if (gb.Seen('B'))
        {
            float c[2]; size_t n = 2;
            gb.GetFloatArray(c, n, false);
            if (n >= 2) { anchorX[1] = c[0]; anchorY[1] = c[1]; seen = true; }
        }
        if (gb.Seen('C'))
        {
            float c[2]; size_t n = 2;
            gb.GetFloatArray(c, n, false);
            if (n >= 2) { anchorX[2] = c[0]; anchorY[2] = c[1]; seen = true; }
        }
        if (gb.Seen('D'))
        {
            float c[2]; size_t n = 2;
            gb.GetFloatArray(c, n, false);
            if (n >= 2) { anchorX[3] = c[0]; anchorY[3] = c[1]; seen = true; }
        }
        if (gb.Seen('E'))
        {
            float o[8]; size_t n = 8;
            gb.GetFloatArray(o, n, false);
            if (n >= 8)
            {
                effectorOffsetX[0]=o[0]; effectorOffsetY[0]=o[1];
                effectorOffsetX[1]=o[2]; effectorOffsetY[1]=o[3];
                effectorOffsetX[2]=o[4]; effectorOffsetY[2]=o[5];
                effectorOffsetX[3]=o[6]; effectorOffsetY[3]=o[7];
                seen = true;
            }
        }
        gb.TryGetFValue('W', moverWeight_kg, seen);
        gb.TryGetFValue('T', minTension,     seen);
        gb.TryGetFValue('X', maxTension,     seen);
        gb.TryGetFValue('S', springConstant, seen);

        if (!seen)
        {
            reply.printf("CableXY: A(%.1f,%.1f) B(%.1f,%.1f) C(%.1f,%.1f) D(%.1f,%.1f) "
                         "W%.2f T%.1f X%.1f S%.4f",
                         (double)anchorX[0], (double)anchorY[0],
                         (double)anchorX[1], (double)anchorY[1],
                         (double)anchorX[2], (double)anchorY[2],
                         (double)anchorX[3], (double)anchorY[3],
                         (double)moverWeight_kg, (double)minTension,
                         (double)maxTension, (double)springConstant);
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

    // Compute tensions for this position
    float tensions[4];
    ComputeTensions(x, y, tensions);

    for (int i = 0; i < 4; i++)
    {
        // Geometric cable length
        float length = CableLength(i, x, y);

        // Subtract spring stretch: higher tension = shorter cable needed
        // delta_L = -tension * springConstant (springConstant in mm/N)
        length -= tensions[i] * springConstant;
        if (length < 1.0f) length = 1.0f;

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
        L[i] = (float)motorPos[i] / stepsPerMm[i];

    float x = 0.0f, y = 0.0f;
    const int maxIter = 100;
    const float tolerance = 0.001f;

    for (int iter = 0; iter < maxIter; iter++)
    {
        float tensions[4];
        ComputeTensions(x, y, tensions);

        float JtJxx=0, JtJxy=0, JtJyy=0, JtFx=0, JtFy=0;

        for (int i = 0; i < 4; i++)
        {
            const float attachX = x + effectorOffsetX[i];
            const float attachY = y + effectorOffsetY[i];
            const float dx = anchorX[i] - attachX;
            const float dy = anchorY[i] - attachY;
            const float len = sqrtf(dx*dx + dy*dy);
            if (len < 0.001f) continue;

            const float adjLen = len - tensions[i] * springConstant;
            const float residual = adjLen - L[i];
            const float jx = dx / len;
            const float jy = dy / len;

            JtJxx += jx*jx; JtJxy += jx*jy; JtJyy += jy*jy;
            JtFx  += jx*residual; JtFy += jy*residual;
        }

        const float det = JtJxx*JtJyy - JtJxy*JtJxy;
        if (fabsf(det) < 1e-10f) break;

        const float dx = (JtJyy*JtFx - JtJxy*JtFy) / det;
        const float dy = (JtJxx*JtFy - JtJxy*JtFx) / det;

        x -= dx; y -= dy;
        if (fabsf(dx) < tolerance && fabsf(dy) < tolerance) break;
    }

    machinePos[0] = x;
    machinePos[1] = y;
    for (size_t i = 2; i < numVisibleAxes; i++) machinePos[i] = 0.0f;
}

bool CableXYKinematics::IsReachable(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept
{
    float minX=anchorX[0], maxX=anchorX[0], minY=anchorY[0], maxY=anchorY[0];
    for (int i=1; i<4; i++)
    {
        if (anchorX[i]<minX) minX=anchorX[i]; if (anchorX[i]>maxX) maxX=anchorX[i];
        if (anchorY[i]<minY) minY=anchorY[i]; if (anchorY[i]>maxY) maxY=anchorY[i];
    }
    const float margin = 20.0f;
    return axesCoords[0]>(minX+margin) && axesCoords[0]<(maxX-margin)
        && axesCoords[1]>(minY+margin) && axesCoords[1]<(maxY-margin);
}

LimitPositionResult CableXYKinematics::LimitPosition(float finalCoords[],
                                                       const float *_ecv_array _ecv_null initialCoords,
                                                       size_t numVisibleAxes, AxesBitmap axesToLimit,
                                                       bool isCoordinated, bool applyM208Limits) const noexcept
{
    return (applyM208Limits && LimitPositionFromAxis(finalCoords, 0, numVisibleAxes, axesToLimit))
        ? LimitPositionResult::adjusted : LimitPositionResult::ok;
}

void CableXYKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept
{
    for (size_t i = 0; i < numAxes; i++) positions[i] = 0.0f;
}
