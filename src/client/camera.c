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
    /* Moving the mouse right turns the view right, as when looking around. */
    camera->yaw = fmod(camera->yaw - (double)deltaX * FOX_CAMERA_RADIANS_PER_STEP, 6.283185307179586);
    camera->pitch = fmin(fmax(camera->pitch + (double)deltaY * FOX_CAMERA_RADIANS_PER_STEP, MIN_PITCH),
                         MAX_PITCH);
}

void FoxCameraTurn(FoxCamera *camera, float radians)
{
    camera->yaw = fmod(camera->yaw + (double)radians, 6.283185307179586);
}

/* Adds offset units to the focus: the whole units to its position, the rest
 * to its fraction. */
static void Shift(FoxCamera *camera, const double offset[3])
{
    int64_t whole[3];
    for (uint32_t axis = 0u; axis < 3u; ++axis)
    {
        const double exact = camera->focusFraction[axis] + offset[axis];
        whole[axis] = llround(exact);
        camera->focusFraction[axis] = exact - (double)whole[axis];
    }
    camera->focus = (FoxVec){camera->focus.x + whole[0], camera->focus.y + whole[1],
                             camera->focus.z + whole[2]};
}

/* Moves the focus along the ground by metres ahead and to the right. The
 * ground sets only the focus's height: the point it returns lies on a grid of
 * directions about 1.5 cm apart on the default planet, which would swallow
 * steps smaller than that and bend the others. */
static void Move(FoxCamera *camera, const FoxPlanet *planet, double forward, double right)
{
    double up[3];
    double ahead[3];
    double side[3];
    Frame(camera, up, ahead, side);
    double offset[3];
    for (uint32_t axis = 0u; axis < 3u; ++axis)
    {
        offset[axis] = (ahead[axis] * forward + side[axis] * right) * FOX_UNITS_PER_METRE;
    }
    Shift(camera, offset);
    FoxVec ground;
    if (FoxPlanetGround(planet, camera->focus, &ground))
    {
        /* Along the focus's own direction from the planet's centre to the
         * ground's distance from it. */
        const double point[3] = {
            (double)camera->focus.x + camera->focusFraction[0],
            (double)camera->focus.y + camera->focusFraction[1],
            (double)camera->focus.z + camera->focusFraction[2],
        };
        const double length = sqrt(point[0] * point[0] + point[1] * point[1] + point[2] * point[2]);
        const double height = sqrt((double)ground.x * (double)ground.x +
                                   (double)ground.y * (double)ground.y +
                                   (double)ground.z * (double)ground.z);
        if (length > 0.0)
        {
            const double lift = height / length - 1.0;
            for (uint32_t axis = 0u; axis < 3u; ++axis)
            {
                offset[axis] = point[axis] * lift;
            }
            Shift(camera, offset);
        }
    }
    UpAt(camera->focus, up);
    Reorthogonalize(camera, up);
}

void FoxCameraPan(FoxCamera *camera, const FoxPlanet *planet, float forward, float right,
                  double seconds)
{
    /* Far out the planet itself is the limit of a sensible step. */
    const double metres = fmin(camera->distance, 600.0) * seconds;
    Move(camera, planet, (double)forward * metres, (double)right * metres);
}

void FoxCameraDrag(FoxCamera *camera, const FoxPlanet *planet, float deltaX, float deltaY,
                   uint32_t viewHeight)
{
    if (viewHeight == 0u)
    {
        return;
    }
    /* Metres per pixel at the focus. Ahead the ground is foreshortened by the
     * pitch, which a low view would exaggerate past the finger. */
    const double perPixel =
        2.0 * camera->distance * tan(FOX_CAMERA_FIELD_OF_VIEW / 2.0) / (double)viewHeight;
    /* Dragging down brings the ground ahead closer: the focus moves ahead. */
    Move(camera, planet, (double)deltaY * perPixel / fmax(sin(camera->pitch), 0.25),
         -(double)deltaX * perPixel);
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
    /* The eye is never closer to the ground than its height over the focus,
     * so the near plane can sit far out, and the depth buffer keeps paint
     * apart from the asphalt under it even far away. */
    for (uint32_t axis = 0u; axis < 3u; ++axis)
    {
        outView->look[axis] = outView->forward[axis] * cos(camera->pitch) -
                              outView->up[axis] * sin(camera->pitch);
        outView->screenRight[axis] = outView->right[axis];
    }
    Cross(outView->screenRight, outView->look, outView->screenUp);
    Normalize(outView->screenUp);
    outView->aspectRatio = aspectRatio;
    outView->nearPlane = (float)fmax(0.3, camera->distance * 0.03);
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
