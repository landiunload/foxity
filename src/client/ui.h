#ifndef FOX_CLIENT_UI_H
#define FOX_CLIENT_UI_H

#include "client/assist.h"
#include "graphics/graphics_device_v2.h"
#include "sim/network.h"

#include <stdbool.h>
#include <stdint.h>

/* The mayor's tools on screen, without a word of text: a dock of icons
 * along the bottom with a tab for each kind of work, the tools of the open
 * tab in a row over it and, over the chosen tool, a panel with its settings,
 * in the manner of a Mac's dock and popovers. The interface is drawn with
 * the engine's interface layer: translucent rounded panels, and icons drawn
 * here into the layer's atlas. Without that layer the game is steered from
 * the keyboard alone. */

typedef enum FoxTool
{
    /* Look around; clicks do nothing to the city. */
    FOX_TOOL_NONE = 0,
    /* Click a start and an end: one lane of the chosen kind, run that way,
     * or beside a road the whole length of it, that road one lane wider. The
     * next lane continues from the end. */
    FOX_TOOL_ROAD,
    /* Click a start, a point to bend towards and an end. */
    FOX_TOOL_CURVE,
    /* Click a lane: it runs the other way, a lone lane one way, the other,
     * both ways in turn. */
    FOX_TOOL_DIRECTION,
    FOX_TOOL_BULLDOZE,
    /* Click near a road end: the chosen sign for the traffic arriving there. */
    FOX_TOOL_SIGNS,
    /* Click an intersection: traffic lights on or off. */
    FOX_TOOL_SIGNALS,
    /* Click a road: the chosen speed limit. */
    FOX_TOOL_SPEED_LIMIT,
    /* Click a road: a pedestrian crossing there, or the one there away. */
    FOX_TOOL_CROSSING,
    /* Click: the district around the point drives on the other side. */
    FOX_TOOL_DRIVE_ZONE,
    /* Held: dig the ground away or pile it up under the pointer. */
    FOX_TOOL_DIG,
    FOX_TOOL_RAISE,
    FOX_TOOL_COUNT,
} FoxTool;

/* The tabs of the dock, each with its tools. */
typedef enum FoxTab
{
    FOX_TAB_NONE = 0,
    /* Lanes, their directions, the bulldozer. */
    FOX_TAB_ROADS,
    /* Signs, lights, limits, crossings and the drive side. */
    FOX_TAB_RULES,
    FOX_TAB_GROUND,
    /* How many cars and people the city keeps. */
    FOX_TAB_PEOPLE,
    FOX_TAB_COUNT,
} FoxTab;

/* The tab a tool sits under. */
FoxTab FoxToolTab(FoxTool tool);

#define FOX_ANGLE_STEP_COUNT 5u
#define FOX_LENGTH_STEP_COUNT 4u
/* The speed tool's choices: the road's own limit, then 20 to 130 km/h. */
#define FOX_SPEED_CHOICE_COUNT 13u
#define FOX_BRUSH_COUNT 3u

/* Everything the interface shows and changes. The scene owns it and changes
 * it from the keyboard as well. */
typedef struct FoxBuildSettings
{
    FoxTool tool;
    FoxTab tab;
    /* The kind of lane the road tools lay. */
    FoxLaneKind laneKind;
    uint32_t snaps;
    uint32_t angleStep;
    uint32_t lengthStep;
    FoxSign sign;
    uint32_t speedChoice;
    uint32_t brush;
} FoxBuildSettings;

/* The snapping angle in degrees, the length step in metres, the limit a
 * speed choice sets in km/h (zero: the road's own) and how big a ground brush
 * is against the usual one. */
double FoxSnapAngleDegrees(uint32_t step);
double FoxSnapLengthMetres(uint32_t step);
int32_t FoxSpeedChoiceKmh(uint32_t choice);
double FoxBrushScale(uint32_t brush);

typedef struct FoxUiInput
{
    /* The view in pixels and the pointer from its top left. */
    uint32_t width;
    uint32_t height;
    bool pointer;
    float x;
    float y;
    /* The button went down this frame. */
    bool click;
    /* Pixels per interface point: about 1 on a monitor, 3 on a phone; zero
     * counts as 1. */
    float density;
} FoxUiInput;

/* How many speeds the speed button goes round, and the factor of each. */
#define FOX_TIME_SCALE_COUNT 3u
uint32_t FoxTimeScaleFactor(uint32_t scale);

/* The time controls at the end of the dock, and what the people tab shows. */
typedef struct FoxUiStatus
{
    bool paused;
    /* 0, 1 or 2: one, two or four times real time. */
    uint32_t timeScale;
    bool statistics;
    uint32_t cars;
    uint32_t walkers;
} FoxUiStatus;

/* What a click on the dock asked for this frame. */
typedef struct FoxUiRequests
{
    bool togglePause;
    /* The speed button: on to the next speed. */
    bool nextTimeScale;
    bool toggleStatistics;
    /* More or fewer cars and people, in steps: -1, 0 or 1. */
    int32_t carsChange;
    int32_t walkersChange;
} FoxUiRequests;

typedef struct FoxUi FoxUi;

/* NULL when the device has no interface layer. */
FoxUi *FoxUiCreate(LaiueGraphicsDeviceV2 *device);
void FoxUiDestroy(FoxUi *ui);

/* Lays the interface out for the frame and handles the pointer: clicks on it
 * change the settings and fill the requests. True when the pointer is over
 * the interface, which the world then does not see. */
bool FoxUiUpdate(FoxUi *ui, const FoxUiInput *input, const FoxUiStatus *status,
                 FoxBuildSettings *settings, FoxUiRequests *requests);

/* While a road is drawn: its length in metres and its angle in degrees to
 * the way it continues from, beside the pointer. */
void FoxUiSetReadout(FoxUi *ui, bool show, double metres, double degrees);

/* Draws the frame's interface over the scene; call between the frame's
 * begin and end. */
void FoxUiDraw(FoxUi *ui);

#endif
