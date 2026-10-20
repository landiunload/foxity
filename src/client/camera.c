#include "client/camera.h"

#include <math.h>
#include <stddef.h>

#define MIN_PITCH 0.12
#define MAX_PITCH 1.50
/* The render origin snaps to this grid, so it changes rarely and stays within
 * half a kilometre of the focus. */
#define ORIGIN_SHIFT 20u

static void Normalize(double vector[3])
{
    const double length = sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
    if (length > 0.0)
    {
        vector[0] /= length;
        vector[1] /= length;
        vector[2] /= length;
    }
}

static void Cross(const double a[3], const double b[3], double out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static void UpAt(FoxVec point, double out[3])
{
    out[0] = (double)point.x;
    out[1] = (double)point.y;
    out[2] = (double)point.z;
    const double length = sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
    if (length == 0.0)
    {
        out[0] = 0.0;
        out[1] = 0.0;
        out[2] = 1.0;
        return;
    }
    out[0] /= length;
    out[1] /= length;
    out[2] /= length;
}

/* Keeps the heading a unit tangent at the focus. */
static void Reorthogonalize(FoxCamera *camera, const double up[3])
{
    double *heading = camera->heading;
    const double along = heading[0] * up[0] + heading[1] * up[1] + heading[2] * up[2];
    heading[0] -= up[0] * along;
    heading[1] -= up[1] * along;
    heading[2] -= up[2] * along;
    if (heading[0] * heading[0] + heading[1] * heading[1] + heading[2] * heading[2] < 1e-12)
    {
        /* Any tangent will do. */
        const double axis[3] = {fabs(up[2]) < 0.9 ? 0.0 : 1.0, 0.0, fabs(up[2]) < 0.9 ? 1.0 : 0.0};
        Cross(axis, up, heading);
    }
    Normalize(heading);
}

static void Frame(FoxCamera *camera, double up[3], double forward[3], double right[3])
{
    UpAt(camera->focus, up);
    Reorthogonalize(camera, up);
    double left[3];
    Cross(up, camera->heading, left);
    for (uint32_t axis = 0u; axis < 3u; ++axis)
    {
        forward[axis] = cos(camera->yaw) * camera->heading[axis] + sin(camera->yaw) * left[axis];
    }
    Cross(forward, up, right);
}

void FoxCameraInit(FoxCamera *camera, FoxVec focus)
{
    *camera = (FoxCamera){
        .focus = focus,
        .heading = {0.0, 1.0, 0.0},
        .distance = 260.0,
        .yaw = 0.0,
        .pitch = 0.85,
    };
    double up[3];
    UpAt(focus, up);
    Reorthogonalize(camera, up);
}

void FoxCameraZoom(FoxCamera *camera, float steps)
{
    camera->distance *= pow(FOX_CAMERA_ZOOM_FACTOR, (double)steps);
    camera->distance = fmin(fmax(camera->distance, FOX_CAMERA_MIN_DISTANCE), FOX_CAMERA_MAX_DISTANCE);
}

void FoxCameraRotate(FoxCamera *camera, int32_t deltaX, int32_t deltaY)
{
    camera->yaw = fmod(camera->yaw + (double)deltaX * FOX_CAMERA_RADIANS_PER_STEP, 6.283185307179586);
    camera->pitch = fmin(fmax(camera->pitch + (double)deltaY * FOX_CAMERA_RADIANS_PER_STEP, MIN_PITCH),
                         MAX_PITCH);
}

void FoxCameraPan(FoxCamera *camera, const FoxPlanet *planet, float forward, float right,
                  double seconds)
{
    double up[3];
    double ahead[3];
    double side[3];
    Frame(camera, up, ahead, side);
    /* Far out the planet itself is the limit of a sensible step. */
    const double metres = fmin(camera->distance, 600.0) * seconds;
    const FoxVec moved = {
        camera->focus.x + (int64_t)llround((ahead[0] * forward + side[0] * right) * metres *
                                           FOX_UNITS_PER_METRE),
        camera->focus.y + (int64_t)llround((ahead[1] * forward + side[1] * right) * metres *
                                           FOX_UNITS_PER_METRE),
        camera->focus.z + (int64_t)llround((ahead[2] * forward + side[2] * right) * metres *
                                           FOX_UNITS_PER_METRE),
    };
    FoxVec ground;
    camera->focus = FoxPlanetGround(planet, moved, &ground) ? ground : moved;
    UpAt(camera->focus, up);
    Reorthogonalize(camera, up);
}

void FoxCameraRelative(const FoxCameraView *view, FoxVec point, float outMetres[3])
{
    outMetres[0] = (float)((double)(point.x - view->renderOrigin.x) / FOX_UNITS_PER_METRE);
    outMetres[1] = (float)((double)(point.y - view->renderOrigin.y) / FOX_UNITS_PER_METRE);
    outMetres[2] = (float)((double)(point.z - view->renderOrigin.z) / FOX_UNITS_PER_METRE);
}

bool FoxCameraBuildView(FoxCamera *camera, const LaiueSceneMathServiceV1 *math, float aspectRatio,
                        FoxCameraView *outView)
{
    if (camera == NULL || math == NULL || outView == NULL || !(aspectRatio > 0.0f))
    {
        return false;
    }
    Frame(camera, outView->up, outView->forward, outView->right);
    const int64_t mask = ~((INT64_C(1) << ORIGIN_SHIFT) - 1);
    outView->renderOrigin =
        (FoxVec){camera->focus.x & mask, camera->focus.y & mask, camera->focus.z & mask};
    float focus[3];
    FoxCameraRelative(outView, camera->focus, focus);
    const double back = cos(camera->pitch) * camera->distance;
    const double rise = sin(camera->pitch) * camera->distance;
    float up[3];
    for (uint32_t axis = 0u; axis < 3u; ++axis)
    {
        outView->eye[axis] = focus[axis] - (float)(outView->forward[axis] * back) +
                             (float)(outView->up[axis] * rise);
        up[axis] = (float)outView->up[axis];
    }
    outView->nearPlane = (float)fmax(0.1, camera->distance * 0.004);
    outView->farPlane = (float)(camera->distance + 4000.0);
    float view[16];
    float projection[16];
    if (!math->matrix4LookAt(outView->eye, focus, up, view) ||
        !math->matrix4Perspective(FOX_CAMERA_FIELD_OF_VIEW, aspectRatio, outView->nearPlane,
                                  outView->farPlane, projection))
    {
        return false;
    }
    math->matrix4Multiply(view, projection, outView->viewProjection);
    math->matrix4ExtractFrustumPlanes(outView->viewProjection, outView->planes);
    return true;
}
