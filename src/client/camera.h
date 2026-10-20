#ifndef FOX_CLIENT_CAMERA_H
#define FOX_CLIENT_CAMERA_H

#include "scene/math_service.h"
#include "sim/planet.h"
#include "sim/space.h"

#include <stdbool.h>
#include <stdint.h>

/* The mayor's view: an orbit around a point on the ground, from a street
 * corner up to the whole planet hanging in the sky. "Up" is away from the
 * planet's centre wherever the focus is, and the heading is carried along
 * the surface as the focus moves, so the view never flips at a pole. The
 * focus is an integer world position; floats appear only relative to the
 * render origin near it. */

#define FOX_CAMERA_MIN_DISTANCE 4.0
#define FOX_CAMERA_MAX_DISTANCE 9000.0
/* Vertical field of view: 50 degrees. */
#define FOX_CAMERA_FIELD_OF_VIEW 0.8726646f
#define FOX_CAMERA_RADIANS_PER_STEP 0.005
#define FOX_CAMERA_ZOOM_FACTOR 0.85

typedef struct FoxCamera
{
    FoxVec focus;
    /* A unit tangent at the focus that the yaw is measured from. */
    double heading[3];
    /* Metres from the focus to the eye. */
    double distance;
    double yaw;
    double pitch;
} FoxCamera;

typedef struct FoxCameraView
{
    FoxVec renderOrigin;
    float viewProjection[16];
    float planes[6][4];
    /* The eye relative to the render origin, in metres. */
    float eye[3];
    float nearPlane;
    float farPlane;
    /* Unit vectors at the focus: up, the ground direction the camera looks
     * in, and its right. */
    double up[3];
    double forward[3];
    double right[3];
    /* Where the eye looks and the screen's up and right, unit vectors. */
    double look[3];
    double screenUp[3];
    double screenRight[3];
    float aspectRatio;
} FoxCameraView;

void FoxCameraInit(FoxCamera *camera, FoxVec focus);
/* Positive steps move closer; each step changes the distance by 15 %. */
void FoxCameraZoom(FoxCamera *camera, float steps);
void FoxCameraRotate(FoxCamera *camera, int32_t deltaX, int32_t deltaY);
/* Moves the focus along the ground; a second covers about the visible area at
 * any zoom. */
void FoxCameraPan(FoxCamera *camera, const FoxPlanet *planet, float forward, float right,
                  double seconds);
bool FoxCameraBuildView(FoxCamera *camera, const LaiueSceneMathServiceV1 *math, float aspectRatio,
                        FoxCameraView *outView);

/* A world position relative to the view's render origin, in metres. */
void FoxCameraRelative(const FoxCameraView *view, FoxVec point, float outMetres[3]);

#endif
