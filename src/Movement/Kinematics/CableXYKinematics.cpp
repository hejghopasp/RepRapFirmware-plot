#include "CableXYKinematics.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"
#include "Movement/DDA.h"
#include <cmath>

// Static default anchor positions
constexpr float CableXYKinematics::DefaultAnchorX[4];
constexpr float CableXYKinematics::DefaultAnchorY[4];

CableXYKinematics::CableXYKinematics() noexcept
    : Kinematics(KinematicsType::cableXY, SegmentationType(true, true, true))
{
    // Set default anchor positions
    for (int i = 0; i < 4; i++)
    {
        anchorX[i] = DefaultAnchorX[i];
        anchorY[i] = DefaultAnchorY[i];
    }
}

const char *CableXYKinematics::GetName(bool forStatusReport) const noexcept
{
    return "CableXY";
}

// Compute cable length from anchor i to effector at (x, y)
float CableXYKinematics::CableLength(int motor, float x, float y) const noexcept
{
    const float dx = x - anchorX[motor];
    const float dy = y - anchorY[motor];
    return sqrtf(dx * dx + dy * dy);
}

// Configure via M669
// Example: M669 K100 A-399:-253 B399:-253 C399:253 D-399:253
bool CableXYKinematics::Configure(unsigned int mCode, GCodeBuffer& gb,
                                   const StringRef& reply, bool& error) noexcept
{
    if (mCode == 669)
    {
        bool seen = false;

        // Parse anchor A (motor 0 = X axis)
        if (gb.Seen('A'))
        {
            float coords[2];
            size_t numCoords = 2;
            gb.GetFloatArray(coords, numCoords, false);
            if (numCoords >= 2)
            {
                anchorX[0] = coords[0];
                anchorY[0] = coords[1];
                seen = true;
            }
        }

        // Parse anchor B (motor 1 = Y axis)
        if (gb.Seen('B'))
        {
            float coords[2];
            size_t numCoords = 2;
            gb.GetFloatArray(coords, numCoords, false);
            if (numCoords >= 2)
            {
                anchorX[1] = coords[0];
                anchorY[1] = coords[1];
                seen = true;
            }
        }

        // Parse anchor C (motor 2 = Z axis)
        if (gb.Seen('C'))
        {
            float coords[2];
            size_t numCoords = 2;
            gb.GetFloatArray(coords, numCoords, false);
            if (numCoords >= 2)
            {
                anchorX[2] = coords[0];
                anchorY[2] = coords[1];
                seen = true;
            }
        }

        // Parse anchor D (motor 3 = U axis)
        if (gb.Seen('D'))
        {
            float coords[2];
            size_t numCoords = 2;
            gb.GetFloatArray(coords, numCoords, false);
            if (numCoords >= 2)
            {
                anchorX[3] = coords[0];
                anchorY[3] = coords[1];
                seen = true;
            }
        }

        if (!seen)
        {
            // Report current configuration
            reply.printf("CableXY kinematics:\n"
                         "  A anchor: (%.2f, %.2f)\n"
                         "  B anchor: (%.2f, %.2f)\n"
                         "  C anchor: (%.2f, %.2f)\n"
                         "  D anchor: (%.2f, %.2f)",
                         anchorX[0], anchorY[0],
                         anchorX[1], anchorY[1],
                         anchorX[2], anchorY[2],
                         anchorX[3], anchorY[3]);
        }
        return true;
    }
    return Kinematics::Configure(mCode, gb, reply, error);
}

// Inverse kinematics: Cartesian position -> motor steps
// This is the core function. For each motor, compute the required cable length
// and convert to steps.
MovementError CableXYKinematics::CartesianToMotorSteps(const float machinePos[],
                                               const float stepsPerMm[],
                                               size_t numVisibleAxes,
                                               size_t numTotalAxes,
                                               int32_t motorPos[],
                                               bool isCoordinated) const noexcept
{
    const float x = machinePos[0];
    const float y = machinePos[1];

    for (int i = 0; i < 4; i++)
    {
        const float length = CableLength(i, x, y);
        RoundToInt32(MovementError::ok, length * stepsPerMm[i], motorPos[i]);
    }

    return MovementError::ok;
}

// Forward kinematics: motor steps -> Cartesian position
// With 4 cables and 2 DOF this is overdetermined.
// We use a least-squares iterative approach (Newton-Raphson).
void CableXYKinematics::MotorStepsToCartesian(const int32_t motorPos[],
                                               const float stepsPerMm[],
                                               size_t numVisibleAxes,
                                               size_t numTotalAxes,
                                               float machinePos[]) const noexcept
{
    // Convert motor steps to cable lengths
    float L[4];
    for (int i = 0; i < 4; i++)
    {
        L[i] = (float)motorPos[i] / stepsPerMm[i];
    }

    // Iterative Newton-Raphson solver
    // Start from center (0, 0) as initial guess
    float x = 0.0f;
    float y = 0.0f;

    const int maxIter = 100;
    const float tolerance = 0.001f; // 0.001mm

    for (int iter = 0; iter < maxIter; iter++)
    {
        // Residuals: difference between actual and target cable lengths
        float fx = 0.0f;
        float fy = 0.0f;

        // Jacobian accumulators (least squares: J^T * r)
        float JtFx = 0.0f;
        float JtFy = 0.0f;
        float JtJxx = 0.0f;
        float JtJxy = 0.0f;
        float JtJyy = 0.0f;

        for (int i = 0; i < 4; i++)
        {
            const float dx = x - anchorX[i];
            const float dy = y - anchorY[i];
            const float len = sqrtf(dx * dx + dy * dy);

            if (len < 0.001f) continue;

            const float residual = len - L[i];

            // Jacobian of cable length w.r.t. x, y
            const float jx = dx / len;
            const float jy = dy / len;

            // Accumulate J^T * J and J^T * residual
            JtJxx += jx * jx;
            JtJxy += jx * jy;
            JtJyy += jy * jy;
            JtFx  += jx * residual;
            JtFy  += jy * residual;
        }

        // Solve 2x2 system: [JtJxx JtJxy; JtJxy JtJyy] * [dx; dy] = [JtFx; JtFy]
        const float det = JtJxx * JtJyy - JtJxy * JtJxy;
        if (fabsf(det) < 1e-10f) break;

        const float dx = (JtJyy * JtFx - JtJxy * JtFy) / det;
        const float dy = (JtJxx * JtFy - JtJxy * JtFx) / det;

        x -= dx;
        y -= dy;

        // Check convergence
        if (fabsf(dx) < tolerance && fabsf(dy) < tolerance) break;
    }

    machinePos[0] = x;
    machinePos[1] = y;

    // Z and higher axes are not used, set to zero
    for (size_t i = 2; i < numVisibleAxes; i++)
    {
        machinePos[i] = 0.0f;
    }
}

bool CableXYKinematics::IsReachable(float axesCoords[MaxAxes],
                                     AxesBitmap axes) const noexcept
{
    // Check that the requested position is within the convex hull
    // of the anchor points (with some margin)
    const float x = axesCoords[0];
    const float y = axesCoords[1];

    // Simple bounding box check using anchor extents
    float minX = anchorX[0], maxX = anchorX[0];
    float minY = anchorY[0], maxY = anchorY[0];
    for (int i = 1; i < 4; i++)
    {
        if (anchorX[i] < minX) minX = anchorX[i];
        if (anchorX[i] > maxX) maxX = anchorX[i];
        if (anchorY[i] < minY) minY = anchorY[i];
        if (anchorY[i] > maxY) maxY = anchorY[i];
    }

    // Stay 20mm inside the anchor boundary for cable tension
    const float margin = 20.0f;
    return x > (minX + margin) && x < (maxX - margin) &&
           y > (minY + margin) && y < (maxY - margin);
}

LimitPositionResult CableXYKinematics::LimitPosition(float finalCoords[],
                                                       const float * null initialCoords,
                                                       size_t numAxes,
                                                       AxesBitmap axesToLimit,
                                                       bool isCoordinated,
                                                       bool applyM208Limits) const noexcept
{
    return LimitPositionResult::ok;
}

void CableXYKinematics::GetAssumedInitialPosition(size_t numAxes,
                                                    float positions[]) const noexcept
{
    // Assume starting at center (0, 0)
    for (size_t i = 0; i < numAxes; i++)
    {
        positions[i] = 0.0f;
    }
}
