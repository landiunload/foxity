#include "client/ui.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- Look ------------------------------------------------------------------------------ */

#define RGBA(r, g, b, a)                                                                           \
    ((uint32_t)(r) | ((uint32_t)(g) << 8u) | ((uint32_t)(b) << 16u) | ((uint32_t)(a) << 24u))

#define COLOR_PANEL RGBA(250, 250, 252, 236)
#define COLOR_BORDER RGBA(0, 0, 0, 28)
#define COLOR_SHADOW RGBA(0, 0, 0, 22)
#define COLOR_ICON RGBA(58, 58, 64, 255)
#define COLOR_ICON_DIM RGBA(58, 58, 64, 90)
#define COLOR_ACCENT RGBA(0, 122, 255, 255)
#define COLOR_ACCENT_TINT RGBA(0, 122, 255, 36)
#define COLOR_HOVER RGBA(0, 0, 0, 16)
#define COLOR_DIVIDER RGBA(0, 0, 0, 30)
#define COLOR_TRACK RGBA(118, 118, 128, 30)
#define COLOR_WHITE RGBA(255, 255, 255, 255)
#define COLOR_RED RGBA(255, 59, 48, 255)

/* Sizes in points; a point is a pixel up to a 900-pixel high view and grows
 * with taller ones. */
#define DOCK_BUTTON 44.0f
#define DOCK_ICON 26.0f
#define DOCK_GAP 4.0f
#define DOCK_PADDING 8.0f
#define DOCK_RADIUS 18.0f
#define DOCK_MARGIN 16.0f
#define GROUP_GAP 9.0f
#define PANEL_BUTTON 36.0f
#define PANEL_ICON 22.0f
#define PANEL_GAP 6.0f
#define PANEL_PADDING 10.0f
#define PANEL_RADIUS 14.0f
#define PANEL_ABOVE 14.0f
#define SEGMENT_WIDTH 40.0f
#define SEGMENT_HEIGHT 28.0f
#define DIGIT_HEIGHT 13.0f

/* ---- Shapes ----------------------------------------------------------------------------- */

/* Icons are drawn on a 24-point grid, y down, as unions of shapes some of
 * which cut the ones before them away; edges are antialiased from the
 * distance to each shape. */
typedef enum ShapeKind
{
    SHAPE_SEGMENT = 0,
    SHAPE_CIRCLE,
    SHAPE_ARC,
    SHAPE_BOX,
    SHAPE_TRIANGLE,
    SHAPE_POLYGON,
} ShapeKind;

typedef struct Shape
{
    uint8_t kind;
    bool erase;
    /* With a positive stroke only the outline is drawn, this wide. */
    float stroke;
    /* Segment: x0 y0 x1 y1 width. Circle: x y radius. Arc: x y radius start
     * sweep (degrees, clockwise on screen) width. Box: x0 y0 x1 y1 corner
     * radius. Triangle: three points. Polygon: x y radius sides rotation. */
    float p[7];
} Shape;

#define SEG(x0, y0, x1, y1, w) {SHAPE_SEGMENT, false, 0.0f, {x0, y0, x1, y1, w, 0.0f, 0.0f}}
#define CUT_SEG(x0, y0, x1, y1, w) {SHAPE_SEGMENT, true, 0.0f, {x0, y0, x1, y1, w, 0.0f, 0.0f}}
#define DISC(x, y, r) {SHAPE_CIRCLE, false, 0.0f, {x, y, r, 0.0f, 0.0f, 0.0f, 0.0f}}
#define CUT_DISC(x, y, r) {SHAPE_CIRCLE, true, 0.0f, {x, y, r, 0.0f, 0.0f, 0.0f, 0.0f}}
#define RING(x, y, r, w) {SHAPE_CIRCLE, false, w, {x, y, r, 0.0f, 0.0f, 0.0f, 0.0f}}
#define ARC(x, y, r, start, sweep, w) {SHAPE_ARC, false, 0.0f, {x, y, r, start, sweep, w, 0.0f}}
#define CUT_ARC(x, y, r, start, sweep, w) {SHAPE_ARC, true, 0.0f, {x, y, r, start, sweep, w, 0.0f}}
#define BOX(x0, y0, x1, y1, r) {SHAPE_BOX, false, 0.0f, {x0, y0, x1, y1, r, 0.0f, 0.0f}}
#define CUT_BOX(x0, y0, x1, y1, r) {SHAPE_BOX, true, 0.0f, {x0, y0, x1, y1, r, 0.0f, 0.0f}}
#define FRAME(x0, y0, x1, y1, r, w) {SHAPE_BOX, false, w, {x0, y0, x1, y1, r, 0.0f, 0.0f}}
#define TRI(x0, y0, x1, y1, x2, y2) {SHAPE_TRIANGLE, false, 0.0f, {x0, y0, x1, y1, x2, y2, 0.0f}}
#define TRI_LINE(x0, y0, x1, y1, x2, y2, w)                                                        \
    {SHAPE_TRIANGLE, false, w, {x0, y0, x1, y1, x2, y2, 0.0f}}
#define CUT_TRI(x0, y0, x1, y1, x2, y2) {SHAPE_TRIANGLE, true, 0.0f, {x0, y0, x1, y1, x2, y2, 0.0f}}
#define GON(x, y, r, sides, turn) {SHAPE_POLYGON, false, 0.0f, {x, y, r, sides, turn, 0.0f, 0.0f}}
#define GON_LINE(x, y, r, sides, turn, w)                                                          \
    {SHAPE_POLYGON, false, w, {x, y, r, sides, turn, 0.0f, 0.0f}}

#define PI_F 3.14159265f

static float Clamp01(float value)
{
    return value < 0.0f ? 0.0f : (value > 1.0f ? 1.0f : value);
}

static float SegmentDistance(float x, float y, float x0, float y0, float x1, float y1)
{
    const float dx = x1 - x0;
    const float dy = y1 - y0;
    const float lengthSquared = dx * dx + dy * dy;
    const float t =
        lengthSquared > 0.0f ? Clamp01(((x - x0) * dx + (y - y0) * dy) / lengthSquared) : 0.0f;
    return hypotf(x - (x0 + dx * t), y - (y0 + dy * t));
}

/* Signed distance to a triangle: negative inside. */
static float TriangleDistance(float x, float y, const float *p)
{
    float inside = 1.0f;
    float nearest = 1e9f;
    for (size_t edge = 0u; edge < 3u; ++edge)
    {
        const float ax = p[edge * 2u];
        const float ay = p[edge * 2u + 1u];
        const float bx = p[(edge + 1u) % 3u * 2u];
        const float by = p[(edge + 1u) % 3u * 2u + 1u];
        nearest = fminf(nearest, SegmentDistance(x, y, ax, ay, bx, by));
        /* The same side of every edge as the triangle's other corner. */
        const float cx = p[(edge + 2u) % 3u * 2u];
        const float cy = p[(edge + 2u) % 3u * 2u + 1u];
        const float side = (bx - ax) * (y - ay) - (by - ay) * (x - ax);
        const float other = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
        if (side * other < 0.0f)
        {
            inside = -1.0f;
        }
    }
    return inside > 0.0f ? -nearest : nearest;
}

static float ShapeDistance(const Shape *shape, float x, float y)
{
    const float *p = shape->p;
    float distance = 0.0f;
    switch (shape->kind)
    {
    case SHAPE_SEGMENT:
        distance = SegmentDistance(x, y, p[0], p[1], p[2], p[3]) - p[4] / 2.0f;
        break;
    case SHAPE_CIRCLE:
        distance = hypotf(x - p[0], y - p[1]) - p[2];
        break;
    case SHAPE_ARC:
    {
        /* Angles grow clockwise on screen, where y points down. */
        const float start = p[3] * PI_F / 180.0f;
        const float sweep = p[4] * PI_F / 180.0f;
        float angle = atan2f(y - p[1], x - p[0]) - start;
        while (angle < 0.0f)
        {
            angle += 2.0f * PI_F;
        }
        while (angle >= 2.0f * PI_F)
        {
            angle -= 2.0f * PI_F;
        }
        if (angle <= sweep)
        {
            distance = fabsf(hypotf(x - p[0], y - p[1]) - p[2]) - p[5] / 2.0f;
        }
        else
        {
            const float ends[2] = {start, start + sweep};
            distance = 1e9f;
            for (uint32_t end = 0u; end < 2u; ++end)
            {
                distance = fminf(distance, hypotf(x - (p[0] + p[2] * cosf(ends[end])),
                                                  y - (p[1] + p[2] * sinf(ends[end]))));
            }
            distance -= p[5] / 2.0f;
        }
        break;
    }
    case SHAPE_BOX:
    {
        const float halfX = (p[2] - p[0]) / 2.0f - p[4];
        const float halfY = (p[3] - p[1]) / 2.0f - p[4];
        const float qx = fabsf(x - (p[0] + p[2]) / 2.0f) - halfX;
        const float qy = fabsf(y - (p[1] + p[3]) / 2.0f) - halfY;
        distance = hypotf(fmaxf(qx, 0.0f), fmaxf(qy, 0.0f)) + fminf(fmaxf(qx, qy), 0.0f) - p[4];
        break;
    }
    case SHAPE_TRIANGLE:
        distance = TriangleDistance(x, y, p);
        break;
    case SHAPE_POLYGON:
    {
        /* A regular polygon: the furthest of its edges' lines. */
        const uint32_t sides = (uint32_t)p[3];
        const float apothem = p[2] * cosf(PI_F / (float)sides);
        distance = -1e9f;
        for (uint32_t side = 0u; side < sides; ++side)
        {
            const float normal =
                p[4] * PI_F / 180.0f + (2.0f * (float)side + 1.0f) * PI_F / (float)sides;
            distance =
                fmaxf(distance, (x - p[0]) * cosf(normal) + (y - p[1]) * sinf(normal) - apothem);
        }
        break;
    }
    default:
        break;
    }
    return shape->stroke > 0.0f ? fabsf(distance) - shape->stroke / 2.0f : distance;
}

/* ---- Icons ------------------------------------------------------------------------------ */

typedef enum Icon
{
    ICON_POINTER = 0,
    ICON_ROAD,
    ICON_CURVE,
    ICON_BULLDOZE,
    ICON_DIG,
    ICON_RAISE,
    ICON_ZONE,
    ICON_SIGNALS,
    ICON_SIGNS,
    ICON_SPEED,
    ICON_PLAY,
    ICON_PAUSE,
    ICON_CHART,
    ICON_ASPHALT,
    ICON_CONCRETE,
    ICON_BIKE,
    ICON_FOOT,
    ICON_DIRECTION,
    ICON_CROSSING,
    ICON_PERSON,
    ICON_CAR,
    ICON_PLUS,
    ICON_MINUS,
    ICON_GROUND,
    ICON_MAGNET,
    ICON_ANGLE,
    ICON_RULER,
    ICON_GRID,
    ICON_GUIDES,
    ICON_TANGENT,
    ICON_SIGN_AUTO,
    ICON_SIGN_NONE,
    ICON_SIGN_PRIORITY,
    ICON_SIGN_YIELD,
    ICON_SIGN_STOP,
    ICON_RING,
    ICON_RESET,
    ICON_BRUSH_SMALL,
    ICON_BRUSH_MEDIUM,
    ICON_BRUSH_LARGE,
    ICON_POINTER_DOWN,
    ICON_COUNT,
} Icon;

typedef struct IconDrawing
{
    const Shape *shapes;
    uint32_t count;
} IconDrawing;

static const Shape pointerShapes[] = {
    TRI(7.0f, 3.5f, 7.0f, 19.0f, 18.5f, 13.0f),
    SEG(11.5f, 13.5f, 15.0f, 20.0f, 3.0f),
};
static const Shape roadShapes[] = {
    SEG(5.5f, 3.0f, 5.5f, 21.0f, 2.0f),    SEG(18.5f, 3.0f, 18.5f, 21.0f, 2.0f),
    SEG(12.0f, 3.5f, 12.0f, 6.5f, 2.0f),   SEG(12.0f, 10.5f, 12.0f, 13.5f, 2.0f),
    SEG(12.0f, 17.5f, 12.0f, 20.5f, 2.0f),
};
static const Shape curveShapes[] = {
    ARC(22.0f, 22.0f, 17.5f, 180.0f, 90.0f, 2.0f), ARC(22.0f, 22.0f, 6.5f, 180.0f, 90.0f, 2.0f),
    ARC(22.0f, 22.0f, 12.0f, 186.0f, 10.0f, 2.0f), ARC(22.0f, 22.0f, 12.0f, 220.0f, 10.0f, 2.0f),
    ARC(22.0f, 22.0f, 12.0f, 254.0f, 10.0f, 2.0f),
};
static const Shape bulldozeShapes[] = {
    SEG(4.5f, 6.0f, 19.5f, 6.0f, 2.0f),          SEG(10.0f, 3.5f, 14.0f, 3.5f, 2.0f),
    FRAME(6.5f, 8.5f, 17.5f, 21.0f, 2.0f, 2.0f), SEG(10.0f, 11.5f, 10.0f, 18.0f, 1.6f),
    SEG(14.0f, 11.5f, 14.0f, 18.0f, 1.6f),
};
static const Shape digShapes[] = {
    SEG(3.0f, 20.0f, 21.0f, 20.0f, 2.0f),
    SEG(12.0f, 3.0f, 12.0f, 15.0f, 2.0f),
    SEG(7.5f, 10.5f, 12.0f, 15.0f, 2.0f),
    SEG(16.5f, 10.5f, 12.0f, 15.0f, 2.0f),
};
static const Shape raiseShapes[] = {
    SEG(3.0f, 20.0f, 21.0f, 20.0f, 2.0f),
    SEG(12.0f, 15.5f, 12.0f, 3.5f, 2.0f),
    SEG(7.5f, 8.0f, 12.0f, 3.5f, 2.0f),
    SEG(16.5f, 8.0f, 12.0f, 3.5f, 2.0f),
};
static const Shape zoneShapes[] = {
    SEG(8.0f, 20.0f, 8.0f, 4.5f, 2.0f),    SEG(4.5f, 8.0f, 8.0f, 4.5f, 2.0f),
    SEG(11.5f, 8.0f, 8.0f, 4.5f, 2.0f),    SEG(16.0f, 4.0f, 16.0f, 19.5f, 2.0f),
    SEG(12.5f, 16.0f, 16.0f, 19.5f, 2.0f), SEG(19.5f, 16.0f, 16.0f, 19.5f, 2.0f),
};
static const Shape signalsShapes[] = {
    BOX(7.5f, 2.0f, 16.5f, 22.0f, 3.0f),
    CUT_DISC(12.0f, 6.5f, 2.2f),
    CUT_DISC(12.0f, 12.0f, 2.2f),
    CUT_DISC(12.0f, 17.5f, 2.2f),
};
static const Shape signsShapes[] = {
    TRI_LINE(3.5f, 4.5f, 20.5f, 4.5f, 12.0f, 19.5f, 2.2f),
};
static const Shape speedShapes[] = {
    ARC(12.0f, 14.0f, 9.0f, 150.0f, 240.0f, 2.0f),
    SEG(12.0f, 14.0f, 16.5f, 9.0f, 2.0f),
    DISC(12.0f, 14.0f, 2.0f),
};
static const Shape playShapes[] = {
    TRI(7.5f, 4.5f, 7.5f, 19.5f, 19.5f, 12.0f),
};
static const Shape pauseShapes[] = {
    BOX(6.5f, 5.0f, 10.5f, 19.0f, 1.2f),
    BOX(13.5f, 5.0f, 17.5f, 19.0f, 1.2f),
};
static const Shape chartShapes[] = {
    BOX(4.0f, 13.0f, 8.0f, 20.5f, 1.2f),
    BOX(10.0f, 8.0f, 14.0f, 20.5f, 1.2f),
    BOX(16.0f, 3.5f, 20.0f, 20.5f, 1.2f),
};
/* A lane of asphalt with its dashed line cut out; one of concrete in slabs. */
static const Shape asphaltShapes[] = {
    BOX(5.0f, 2.0f, 19.0f, 22.0f, 2.0f),
    CUT_SEG(12.0f, 4.0f, 12.0f, 8.0f, 2.0f),
    CUT_SEG(12.0f, 11.0f, 12.0f, 14.0f, 2.0f),
    CUT_SEG(12.0f, 17.0f, 12.0f, 20.5f, 2.0f),
};
static const Shape concreteShapes[] = {
    FRAME(5.0f, 2.0f, 19.0f, 22.0f, 1.5f, 2.0f),
    SEG(5.5f, 8.7f, 18.5f, 8.7f, 1.6f),
    SEG(5.5f, 15.3f, 18.5f, 15.3f, 1.6f),
};
static const Shape bikeShapes[] = {
    RING(6.0f, 15.5f, 4.0f, 1.7f),        RING(18.0f, 15.5f, 4.0f, 1.7f),
    SEG(6.0f, 15.5f, 10.0f, 9.0f, 1.6f),  SEG(10.0f, 9.0f, 16.0f, 9.0f, 1.6f),
    SEG(16.0f, 9.0f, 18.0f, 15.5f, 1.6f), SEG(10.0f, 9.0f, 12.5f, 15.5f, 1.6f),
    SEG(12.5f, 15.5f, 16.0f, 9.0f, 1.6f), SEG(15.0f, 5.5f, 17.5f, 5.5f, 1.6f),
    SEG(16.0f, 5.5f, 16.0f, 9.0f, 1.6f),  SEG(8.5f, 6.5f, 11.5f, 6.5f, 1.6f),
};
static const Shape footShapes[] = {
    DISC(13.0f, 4.2f, 2.4f),
    SEG(12.5f, 8.5f, 10.5f, 14.0f, 2.6f),
    SEG(10.5f, 14.0f, 7.5f, 21.0f, 2.2f),
    SEG(10.5f, 14.0f, 14.5f, 21.0f, 2.2f),
    SEG(12.0f, 9.5f, 16.5f, 12.5f, 2.0f),
    SEG(12.0f, 9.5f, 7.5f, 12.5f, 2.0f),
};
static const Shape directionShapes[] = {
    SEG(8.0f, 21.0f, 8.0f, 7.0f, 2.0f),
    TRI(3.5f, 9.0f, 12.5f, 9.0f, 8.0f, 2.5f),
    SEG(16.0f, 3.0f, 16.0f, 17.0f, 2.0f),
    TRI(11.5f, 15.0f, 20.5f, 15.0f, 16.0f, 21.5f),
};
static const Shape crossingShapes[] = {
    BOX(2.5f, 4.0f, 6.0f, 20.0f, 0.8f),
    BOX(8.0f, 4.0f, 11.0f, 20.0f, 0.8f),
    BOX(13.0f, 4.0f, 16.0f, 20.0f, 0.8f),
    BOX(18.0f, 4.0f, 21.5f, 20.0f, 0.8f),
};
static const Shape personShapes[] = {
    DISC(12.0f, 6.5f, 3.8f),
    BOX(5.5f, 12.0f, 18.5f, 21.5f, 4.5f),
};
static const Shape carShapes[] = {
    BOX(2.5f, 10.5f, 21.5f, 17.0f, 2.5f),
    BOX(6.5f, 6.0f, 16.5f, 12.0f, 2.5f),
    DISC(7.0f, 17.5f, 2.6f),
    DISC(17.0f, 17.5f, 2.6f),
    CUT_DISC(7.0f, 17.5f, 1.0f),
    CUT_DISC(17.0f, 17.5f, 1.0f),
};
static const Shape plusShapes[] = {
    SEG(12.0f, 5.0f, 12.0f, 19.0f, 2.4f),
    SEG(5.0f, 12.0f, 19.0f, 12.0f, 2.4f),
};
static const Shape minusShapes[] = {SEG(5.0f, 12.0f, 19.0f, 12.0f, 2.4f)};
static const Shape groundShapes[] = {
    TRI(1.5f, 20.0f, 9.5f, 6.0f, 17.5f, 20.0f),
    TRI(10.5f, 20.0f, 16.0f, 11.0f, 22.5f, 20.0f),
};
static const Shape magnetShapes[] = {
    ARC(12.0f, 12.5f, 6.25f, 0.0f, 180.0f, 4.5f), BOX(3.5f, 3.5f, 8.0f, 12.5f, 0.0f),
    BOX(16.0f, 3.5f, 20.5f, 12.5f, 0.0f),         CUT_SEG(2.0f, 7.0f, 9.5f, 7.0f, 1.2f),
    CUT_SEG(14.5f, 7.0f, 22.0f, 7.0f, 1.2f),
};
static const Shape angleShapes[] = {
    SEG(4.0f, 20.0f, 21.0f, 20.0f, 2.0f),
    SEG(4.0f, 20.0f, 15.0f, 5.0f, 2.0f),
    ARC(4.0f, 20.0f, 9.0f, 306.3f, 53.7f, 1.6f),
};
static const Shape rulerShapes[] = {
    FRAME(2.0f, 7.5f, 22.0f, 16.5f, 1.5f, 1.8f), SEG(6.0f, 8.5f, 6.0f, 12.5f, 1.4f),
    SEG(10.0f, 8.5f, 10.0f, 11.0f, 1.4f),        SEG(14.0f, 8.5f, 14.0f, 12.5f, 1.4f),
    SEG(18.0f, 8.5f, 18.0f, 11.0f, 1.4f),
};
static const Shape gridShapes[] = {
    FRAME(3.0f, 3.0f, 21.0f, 21.0f, 2.5f, 1.8f), SEG(9.0f, 3.5f, 9.0f, 20.5f, 1.4f),
    SEG(15.0f, 3.5f, 15.0f, 20.5f, 1.4f),        SEG(3.5f, 9.0f, 20.5f, 9.0f, 1.4f),
    SEG(3.5f, 15.0f, 20.5f, 15.0f, 1.4f),
};
static const Shape guidesShapes[] = {
    SEG(2.5f, 15.0f, 5.5f, 15.0f, 1.6f),   SEG(15.5f, 15.0f, 18.5f, 15.0f, 1.6f),
    SEG(20.5f, 15.0f, 21.5f, 15.0f, 1.6f), SEG(13.5f, 12.5f, 15.5f, 10.5f, 1.6f),
    SEG(17.5f, 8.5f, 19.5f, 6.5f, 1.6f),   SEG(5.0f, 21.0f, 7.0f, 19.0f, 1.6f),
    RING(11.0f, 15.0f, 3.0f, 1.8f),        DISC(11.0f, 15.0f, 1.0f),
};
static const Shape tangentShapes[] = {
    SEG(3.0f, 21.0f, 11.0f, 13.0f, 2.0f),
    ARC(16.66f, 18.66f, 8.0f, 225.0f, 80.0f, 2.0f),
    DISC(11.0f, 13.0f, 2.2f),
};
static const Shape signAutoShapes[] = {
    SEG(4.5f, 19.5f, 14.5f, 9.5f, 2.4f),   SEG(17.0f, 3.0f, 17.0f, 7.0f, 1.5f),
    SEG(15.0f, 5.0f, 19.0f, 5.0f, 1.5f),   SEG(20.0f, 10.0f, 20.0f, 13.0f, 1.3f),
    SEG(18.5f, 11.5f, 21.5f, 11.5f, 1.3f), SEG(10.0f, 3.0f, 10.0f, 6.0f, 1.3f),
    SEG(8.5f, 4.5f, 11.5f, 4.5f, 1.3f),
};
static const Shape signNoneShapes[] = {
    RING(12.0f, 12.0f, 8.5f, 2.0f),
    SEG(6.0f, 18.0f, 18.0f, 6.0f, 2.0f),
};
static const Shape signPriorityShapes[] = {
    GON_LINE(12.0f, 12.0f, 10.0f, 4.0f, 0.0f, 2.0f),
    GON(12.0f, 12.0f, 5.5f, 4.0f, 0.0f),
};
static const Shape signYieldShapes[] = {
    TRI_LINE(3.0f, 5.0f, 21.0f, 5.0f, 12.0f, 20.5f, 2.2f),
};
static const Shape signStopShapes[] = {
    GON_LINE(12.0f, 12.0f, 10.0f, 8.0f, 22.5f, 2.0f),
    SEG(8.0f, 12.0f, 16.0f, 12.0f, 2.2f),
};
static const Shape ringShapes[] = {
    RING(12.0f, 12.0f, 10.0f, 3.0f),
};
static const Shape resetShapes[] = {
    ARC(12.0f, 12.0f, 7.0f, 300.0f, 280.0f, 2.0f),
    TRI(9.95f, 4.58f, 8.79f, 8.95f, 5.45f, 4.97f),
};
static const Shape brushSmallShapes[] = {DISC(12.0f, 12.0f, 3.0f)};
static const Shape brushMediumShapes[] = {DISC(12.0f, 12.0f, 5.5f)};
static const Shape brushLargeShapes[] = {DISC(12.0f, 12.0f, 8.5f)};
static const Shape pointerDownShapes[] = {TRI(0.0f, 0.0f, 24.0f, 0.0f, 12.0f, 12.0f)};

#define DRAWING(shapes) {(shapes), (uint32_t)(sizeof(shapes) / sizeof((shapes)[0]))}

static const IconDrawing iconDrawings[ICON_COUNT] = {
    [ICON_POINTER] = DRAWING(pointerShapes),
    [ICON_ROAD] = DRAWING(roadShapes),
    [ICON_CURVE] = DRAWING(curveShapes),
    [ICON_BULLDOZE] = DRAWING(bulldozeShapes),
    [ICON_DIG] = DRAWING(digShapes),
    [ICON_RAISE] = DRAWING(raiseShapes),
    [ICON_ZONE] = DRAWING(zoneShapes),
    [ICON_SIGNALS] = DRAWING(signalsShapes),
    [ICON_SIGNS] = DRAWING(signsShapes),
    [ICON_SPEED] = DRAWING(speedShapes),
    [ICON_PLAY] = DRAWING(playShapes),
    [ICON_PAUSE] = DRAWING(pauseShapes),
    [ICON_CHART] = DRAWING(chartShapes),
    [ICON_ASPHALT] = DRAWING(asphaltShapes),
    [ICON_CONCRETE] = DRAWING(concreteShapes),
    [ICON_BIKE] = DRAWING(bikeShapes),
    [ICON_FOOT] = DRAWING(footShapes),
    [ICON_DIRECTION] = DRAWING(directionShapes),
    [ICON_CROSSING] = DRAWING(crossingShapes),
    [ICON_PERSON] = DRAWING(personShapes),
    [ICON_CAR] = DRAWING(carShapes),
    [ICON_PLUS] = DRAWING(plusShapes),
    [ICON_MINUS] = DRAWING(minusShapes),
    [ICON_GROUND] = DRAWING(groundShapes),
    [ICON_MAGNET] = DRAWING(magnetShapes),
    [ICON_ANGLE] = DRAWING(angleShapes),
    [ICON_RULER] = DRAWING(rulerShapes),
    [ICON_GRID] = DRAWING(gridShapes),
    [ICON_GUIDES] = DRAWING(guidesShapes),
    [ICON_TANGENT] = DRAWING(tangentShapes),
    [ICON_SIGN_AUTO] = DRAWING(signAutoShapes),
    [ICON_SIGN_NONE] = DRAWING(signNoneShapes),
    [ICON_SIGN_PRIORITY] = DRAWING(signPriorityShapes),
    [ICON_SIGN_YIELD] = DRAWING(signYieldShapes),
    [ICON_SIGN_STOP] = DRAWING(signStopShapes),
    [ICON_RING] = DRAWING(ringShapes),
    [ICON_RESET] = DRAWING(resetShapes),
    [ICON_BRUSH_SMALL] = DRAWING(brushSmallShapes),
    [ICON_BRUSH_MEDIUM] = DRAWING(brushMediumShapes),
    [ICON_BRUSH_LARGE] = DRAWING(brushLargeShapes),
    [ICON_POINTER_DOWN] = DRAWING(pointerDownShapes),
};

/* ---- Digits ----------------------------------------------------------------------------- */

/* Figures for measurements, drawn on a 10 by 16 grid; the last three glyphs
 * are the decimal point, the degree sign and the times sign. */
#define GLYPH_COUNT 13u
#define GLYPH_POINT 10u
#define GLYPH_DEGREE 11u
#define GLYPH_TIMES 12u

static const Shape zeroShapes[] = {FRAME(1.5f, 1.5f, 8.5f, 14.5f, 3.5f, 1.8f)};
static const Shape oneShapes[] = {
    SEG(5.5f, 1.5f, 5.5f, 14.5f, 1.8f),
    SEG(5.5f, 1.5f, 2.6f, 4.2f, 1.8f),
};
static const Shape twoShapes[] = {
    ARC(5.0f, 5.0f, 3.5f, 180.0f, 210.0f, 1.8f),
    SEG(8.03f, 6.75f, 1.5f, 14.5f, 1.8f),
    SEG(1.5f, 14.5f, 8.7f, 14.5f, 1.8f),
};
static const Shape threeShapes[] = {
    ARC(5.0f, 4.6f, 3.1f, 200.0f, 250.0f, 1.8f),
    ARC(5.0f, 11.1f, 3.4f, 270.0f, 250.0f, 1.8f),
};
static const Shape fourShapes[] = {
    SEG(6.5f, 1.5f, 1.2f, 10.5f, 1.8f),
    SEG(1.2f, 10.5f, 8.8f, 10.5f, 1.8f),
    SEG(6.5f, 1.5f, 6.5f, 14.5f, 1.8f),
};
static const Shape fiveShapes[] = {
    SEG(8.2f, 1.5f, 2.3f, 1.5f, 1.8f),
    SEG(2.3f, 1.5f, 1.9f, 7.6f, 1.8f),
    ARC(4.8f, 10.4f, 3.9f, 222.0f, 290.0f, 1.8f),
};
static const Shape sixShapes[] = {
    RING(5.0f, 10.6f, 3.6f, 1.8f),
    SEG(7.2f, 1.5f, 2.2f, 8.4f, 1.8f),
};
static const Shape sevenShapes[] = {
    SEG(1.5f, 1.5f, 8.6f, 1.5f, 1.8f),
    SEG(8.6f, 1.5f, 3.6f, 14.5f, 1.8f),
};
static const Shape eightShapes[] = {
    RING(5.0f, 4.6f, 3.1f, 1.8f),
    RING(5.0f, 11.1f, 3.5f, 1.8f),
};
static const Shape nineShapes[] = {
    RING(5.0f, 5.4f, 3.6f, 1.8f),
    SEG(7.8f, 7.6f, 2.8f, 14.5f, 1.8f),
};
static const Shape pointShapes[] = {DISC(2.5f, 13.6f, 1.2f)};
static const Shape degreeShapes[] = {RING(3.0f, 3.2f, 2.0f, 1.4f)};
static const Shape timesShapes[] = {
    SEG(2.0f, 7.0f, 7.5f, 13.5f, 1.7f),
    SEG(7.5f, 7.0f, 2.0f, 13.5f, 1.7f),
};

static const IconDrawing glyphDrawings[GLYPH_COUNT] = {
    DRAWING(zeroShapes),  DRAWING(oneShapes),  DRAWING(twoShapes),   DRAWING(threeShapes),
    DRAWING(fourShapes),  DRAWING(fiveShapes), DRAWING(sixShapes),   DRAWING(sevenShapes),
    DRAWING(eightShapes), DRAWING(nineShapes), DRAWING(pointShapes), DRAWING(degreeShapes),
    DRAWING(timesShapes),
};
/* How far each glyph moves the pen, in grid units. */
static const float glyphAdvance[GLYPH_COUNT] = {10.5f, 8.5f,  10.5f, 10.5f, 10.5f, 10.5f, 10.5f,
                                                10.5f, 10.5f, 10.5f, 5.0f,  7.0f,  9.5f};

/* ---- Atlas ------------------------------------------------------------------------------ */

/* Icons are drawn at twice their usual size and glyphs likewise, so the
 * layer's filtering halves them cleanly. */
#define ATLAS_SCALE 2.0f
#define ICON_CELL 48u
#define ICONS_PER_ROW 10u
#define GLYPH_CELL_WIDTH 20u
#define GLYPH_CELL_HEIGHT 32u
#define ATLAS_WIDTH 512u
#define ATLAS_HEIGHT 512u
#define ICON_ROWS 5u
#define GLYPH_TOP (ICON_ROWS * ICON_CELL)

_Static_assert(ICONS_PER_ROW *ICON_CELL <= ATLAS_WIDTH, "icons fit the atlas across");
_Static_assert(ICON_ROWS *ICONS_PER_ROW >= ICON_COUNT, "every icon has a cell");
_Static_assert(GLYPH_TOP + GLYPH_CELL_HEIGHT <= ATLAS_HEIGHT, "glyphs fit below the icons");

static void Paint(uint8_t *atlas, uint32_t left, uint32_t top, uint32_t width, uint32_t height,
                  const IconDrawing *drawing)
{
    for (uint32_t y = 0u; y < height; ++y)
    {
        for (uint32_t x = 0u; x < width; ++x)
        {
            const float ux = ((float)x + 0.5f) / ATLAS_SCALE;
            const float uy = ((float)y + 0.5f) / ATLAS_SCALE;
            float cover = 0.0f;
            for (uint32_t index = 0u; index < drawing->count; ++index)
            {
                const Shape *shape = &drawing->shapes[index];
                const float alpha = Clamp01(0.5f - ShapeDistance(shape, ux, uy) * ATLAS_SCALE);
                cover = shape->erase ? fminf(cover, 1.0f - alpha) : fmaxf(cover, alpha);
            }
            atlas[(size_t)(top + y) * ATLAS_WIDTH + left + x] = (uint8_t)lroundf(cover * 255.0f);
        }
    }
}

static void IconCell(Icon icon, uint32_t *outX, uint32_t *outY)
{
    *outX = (uint32_t)icon % ICONS_PER_ROW * ICON_CELL;
    *outY = (uint32_t)icon / ICONS_PER_ROW * ICON_CELL;
}

/* ---- The interface ---------------------------------------------------------------------- */

#define QUAD_CAPACITY 768u

struct FoxUi
{
    LaiueGraphicsDeviceV2 *device;
    LaiueGraphicsUiQuadV1 quads[QUAD_CAPACITY];
    uint32_t quadCount;
    float scale;
    FoxUiInput input;
    /* Over the interface this frame. */
    bool over;
    /* Clicking the chosen tool again folds its panel away; another tool
     * shows its own. */
    bool folded;
    FoxTool shownTool;
    bool readout;
    double metres;
    double degrees;
};

static void Quad(FoxUi *ui, float x0, float y0, float x1, float y1, uint32_t color, float radius,
                 const float uv[4], uint32_t flags)
{
    if (ui->quadCount >= QUAD_CAPACITY || x1 <= x0 || y1 <= y0)
    {
        return;
    }
    LaiueGraphicsUiQuadV1 *quad = &ui->quads[ui->quadCount++];
    *quad = (LaiueGraphicsUiQuadV1){
        .rect = {x0, y0, x1, y1},
        .colorRGBA = color,
        .cornerRadius = radius,
        .flags = flags,
    };
    if (uv != NULL)
    {
        memcpy(quad->uv, uv, sizeof(quad->uv));
    }
}

static void Box(FoxUi *ui, float x0, float y0, float x1, float y1, uint32_t color, float radius)
{
    Quad(ui, x0, y0, x1, y1, color, fminf(radius, fminf(x1 - x0, y1 - y0) / 2.0f), NULL, 0u);
}

/* A panel as the dock and popovers have: a soft shadow, a hairline edge and
 * the translucent face. */
static void Panel(FoxUi *ui, float x0, float y0, float x1, float y1, float radius)
{
    const float s = ui->scale;
    Box(ui, x0 - 1.0f * s, y0 + 1.0f * s, x1 + 1.0f * s, y1 + 3.0f * s, COLOR_SHADOW,
        radius + 1.0f * s);
    Box(ui, x0 - 0.5f * s, y0 - 0.5f * s, x1 + 0.5f * s, y1 + 0.5f * s, COLOR_BORDER,
        radius + 0.5f * s);
    Box(ui, x0, y0, x1, y1, COLOR_PANEL, radius);
}

static void DrawIcon(FoxUi *ui, Icon icon, float centreX, float centreY, float size, uint32_t color)
{
    uint32_t cellX = 0u;
    uint32_t cellY = 0u;
    IconCell(icon, &cellX, &cellY);
    const float uv[4] = {(float)cellX / ATLAS_WIDTH, (float)cellY / ATLAS_HEIGHT,
                         (float)(cellX + ICON_CELL) / ATLAS_WIDTH,
                         (float)(cellY + ICON_CELL) / ATLAS_HEIGHT};
    Quad(ui, centreX - size / 2.0f, centreY - size / 2.0f, centreX + size / 2.0f,
         centreY + size / 2.0f, color, 0.0f, uv, LAIUE_GRAPHICS_UI_QUAD_TEXT);
}

static uint32_t GlyphOf(char character)
{
    if (character >= '0' && character <= '9')
    {
        return (uint32_t)(character - '0');
    }
    if (character == 'x')
    {
        return GLYPH_TIMES;
    }
    return character == '.' ? GLYPH_POINT : GLYPH_DEGREE;
}

/* How wide a string of figures is at a height. */
static float FiguresWidth(const char *text, float height)
{
    float width = 0.0f;
    for (const char *cursor = text; *cursor != '\0'; ++cursor)
    {
        width += glyphAdvance[GlyphOf(*cursor)] * height / 16.0f;
    }
    return width;
}

/* Figures from the left at a vertical centre: digits, '.', '*' for the
 * degree sign and 'x' for times. */
static void Figures(FoxUi *ui, const char *text, float left, float centreY, float height,
                    uint32_t color)
{
    const float unit = height / 16.0f;
    for (const char *cursor = text; *cursor != '\0'; ++cursor)
    {
        const uint32_t glyph = GlyphOf(*cursor);
        const float x = (float)(glyph * GLYPH_CELL_WIDTH);
        const float uv[4] = {x / ATLAS_WIDTH, (float)GLYPH_TOP / ATLAS_HEIGHT,
                             (x + GLYPH_CELL_WIDTH) / ATLAS_WIDTH,
                             (float)(GLYPH_TOP + GLYPH_CELL_HEIGHT) / ATLAS_HEIGHT};
        Quad(ui, left, centreY - 8.0f * unit, left + 10.0f * unit, centreY + 8.0f * unit, color,
             0.0f, uv, LAIUE_GRAPHICS_UI_QUAD_TEXT);
        left += glyphAdvance[glyph] * unit;
    }
}

static bool Inside(const FoxUi *ui, float x0, float y0, float x1, float y1)
{
    return ui->input.pointer && ui->input.x >= x0 && ui->input.x < x1 && ui->input.y >= y0 &&
           ui->input.y < y1;
}

/* A press on a rectangle this frame; each press is taken once. */
static bool Pressed(FoxUi *ui, float x0, float y0, float x1, float y1)
{
    if (ui->input.click && Inside(ui, x0, y0, x1, y1))
    {
        ui->input.click = false;
        return true;
    }
    return false;
}

typedef enum Look
{
    /* An icon on the panel, a light wash under the pointer. */
    LOOK_PLAIN = 0,
    /* The choice in force: white on the accent. */
    LOOK_CHOSEN,
    /* A setting that is on: accent on a tint of it. */
    LOOK_ON,
    /* Has no effect now. */
    LOOK_DIM,
} Look;

static bool Button(FoxUi *ui, float x, float y, float size, float iconSize, Icon icon, Look look)
{
    const float radius = size * 0.26f;
    const bool hover = Inside(ui, x, y, x + size, y + size);
    uint32_t color = COLOR_ICON;
    if (look == LOOK_CHOSEN)
    {
        Box(ui, x, y, x + size, y + size, COLOR_ACCENT, radius);
        color = COLOR_WHITE;
    }
    else if (look == LOOK_ON)
    {
        Box(ui, x, y, x + size, y + size, COLOR_ACCENT_TINT, radius);
        color = COLOR_ACCENT;
    }
    else
    {
        if (hover)
        {
            Box(ui, x, y, x + size, y + size, COLOR_HOVER, radius);
        }
        color = look == LOOK_DIM ? COLOR_ICON_DIM : COLOR_ICON;
    }
    DrawIcon(ui, icon, x + size / 2.0f, y + size / 2.0f, iconSize, color);
    return Pressed(ui, x, y, x + size, y + size);
}

/* A row of figures to choose one of, as a segmented control. Returns the
 * index pressed, or -1. */
static int32_t Segments(FoxUi *ui, float x, float y, const char *const *labels, uint32_t count,
                        uint32_t chosen, bool dim)
{
    const float s = ui->scale;
    const float width = SEGMENT_WIDTH * s;
    const float height = SEGMENT_HEIGHT * s;
    Box(ui, x, y, x + width * (float)count, y + height, COLOR_TRACK, 8.0f * s);
    int32_t pressed = -1;
    for (uint32_t index = 0u; index < count; ++index)
    {
        const float left = x + width * (float)index;
        if (index == chosen)
        {
            Box(ui, left + 2.0f * s, y + 2.0f * s + 0.5f * s, left + width - 2.0f * s,
                y + height - 2.0f * s + 1.0f * s, COLOR_SHADOW, 6.5f * s);
            Box(ui, left + 2.0f * s, y + 2.0f * s, left + width - 2.0f * s, y + height - 2.0f * s,
                COLOR_WHITE, 6.0f * s);
        }
        const float figures = DIGIT_HEIGHT * s;
        const float textWidth = FiguresWidth(labels[index], figures);
        Figures(ui, labels[index], left + (width - textWidth) / 2.0f, y + height / 2.0f, figures,
                dim ? COLOR_ICON_DIM : COLOR_ICON);
        if (Pressed(ui, left, y, left + width, y + height))
        {
            pressed = (int32_t)index;
        }
    }
    return pressed;
}

/* ---- Settings ---------------------------------------------------------------------------- */

static const double angleSteps[FOX_ANGLE_STEP_COUNT] = {5.0, 15.0, 30.0, 45.0, 90.0};
static const double lengthSteps[FOX_LENGTH_STEP_COUNT] = {1.0, 4.0, 8.0, 16.0};
static const char *const angleLabels[FOX_ANGLE_STEP_COUNT] = {"5*", "15*", "30*", "45*", "90*"};
static const char *const lengthLabels[FOX_LENGTH_STEP_COUNT] = {"1", "4", "8", "16"};
static const double brushScales[FOX_BRUSH_COUNT] = {0.6, 1.0, 1.8};

double FoxSnapAngleDegrees(uint32_t step)
{
    return angleSteps[step < FOX_ANGLE_STEP_COUNT ? step : 1u];
}

double FoxSnapLengthMetres(uint32_t step)
{
    return lengthSteps[step < FOX_LENGTH_STEP_COUNT ? step : 2u];
}

int32_t FoxSpeedChoiceKmh(uint32_t choice)
{
    return choice == 0u || choice >= FOX_SPEED_CHOICE_COUNT ? 0 : 10 + (int32_t)choice * 10;
}

double FoxBrushScale(uint32_t brush)
{
    return brushScales[brush < FOX_BRUSH_COUNT ? brush : 1u];
}

/* ---- Creation ---------------------------------------------------------------------------- */

FoxUi *FoxUiCreate(LaiueGraphicsDeviceV2 *device)
{
    if (device == NULL || device->submitUi == NULL || device->setUiFontAtlas == NULL)
    {
        return NULL;
    }
    uint8_t *atlas = calloc((size_t)ATLAS_WIDTH * ATLAS_HEIGHT, 1u);
    FoxUi *ui = calloc(1u, sizeof(*ui));
    if (atlas == NULL || ui == NULL)
    {
        free(atlas);
        free(ui);
        return NULL;
    }
    for (uint32_t icon = 0u; icon < ICON_COUNT; ++icon)
    {
        uint32_t x = 0u;
        uint32_t y = 0u;
        IconCell((Icon)icon, &x, &y);
        Paint(atlas, x, y, ICON_CELL, ICON_CELL, &iconDrawings[icon]);
    }
    for (uint32_t glyph = 0u; glyph < GLYPH_COUNT; ++glyph)
    {
        Paint(atlas, glyph * GLYPH_CELL_WIDTH, GLYPH_TOP, GLYPH_CELL_WIDTH, GLYPH_CELL_HEIGHT,
              &glyphDrawings[glyph]);
    }
    const bool uploaded = device->setUiFontAtlas(device, atlas, ATLAS_WIDTH, ATLAS_HEIGHT) != 0u;
    free(atlas);
    if (!uploaded)
    {
        free(ui);
        return NULL;
    }
    ui->device = device;
    ui->scale = 1.0f;
    return ui;
}

void FoxUiDestroy(FoxUi *ui)
{
    free(ui);
}

/* ---- Layout ----------------------------------------------------------------------------- */

typedef enum DockKind
{
    DOCK_POINTER = 0,
    DOCK_DIVIDER,
    DOCK_TAB,
    DOCK_PAUSE,
    DOCK_SPEED,
    DOCK_STATISTICS,
} DockKind;

typedef struct DockItem
{
    uint8_t kind;
    uint8_t value;
    uint8_t icon;
} DockItem;

static const DockItem dockItems[] = {
    {DOCK_POINTER, 0u, ICON_POINTER},
    {DOCK_DIVIDER, 0u, 0u},
    {DOCK_TAB, FOX_TAB_ROADS, ICON_ROAD},
    {DOCK_TAB, FOX_TAB_RULES, ICON_SIGNS},
    {DOCK_TAB, FOX_TAB_GROUND, ICON_GROUND},
    {DOCK_TAB, FOX_TAB_PEOPLE, ICON_PERSON},
    {DOCK_DIVIDER, 0u, 0u},
    {DOCK_PAUSE, 0u, ICON_PAUSE},
    {DOCK_SPEED, 0u, ICON_PLAY},
    {DOCK_DIVIDER, 0u, 0u},
    {DOCK_STATISTICS, 0u, ICON_CHART},
};
#define DOCK_ITEM_COUNT ((uint32_t)(sizeof(dockItems) / sizeof(dockItems[0])))

typedef struct ToolButton
{
    FoxTool tool;
    Icon icon;
} ToolButton;

/* The tools under each tab, in the order of their row. */
static const ToolButton roadTools[] = {
    {FOX_TOOL_ROAD, ICON_ROAD},
    {FOX_TOOL_CURVE, ICON_CURVE},
    {FOX_TOOL_DIRECTION, ICON_DIRECTION},
    {FOX_TOOL_BULLDOZE, ICON_BULLDOZE},
};
static const ToolButton ruleTools[] = {
    {FOX_TOOL_SIGNS, ICON_SIGNS},       {FOX_TOOL_SIGNALS, ICON_SIGNALS},
    {FOX_TOOL_SPEED_LIMIT, ICON_SPEED}, {FOX_TOOL_CROSSING, ICON_CROSSING},
    {FOX_TOOL_DRIVE_ZONE, ICON_ZONE},
};
static const ToolButton groundTools[] = {
    {FOX_TOOL_DIG, ICON_DIG},
    {FOX_TOOL_RAISE, ICON_RAISE},
};

typedef struct TabTools
{
    const ToolButton *buttons;
    uint32_t count;
} TabTools;

#define TOOLS(buttons) {(buttons), (uint32_t)(sizeof(buttons) / sizeof((buttons)[0]))}

static const TabTools tabTools[FOX_TAB_COUNT] = {
    [FOX_TAB_ROADS] = TOOLS(roadTools),
    [FOX_TAB_RULES] = TOOLS(ruleTools),
    [FOX_TAB_GROUND] = TOOLS(groundTools),
};

FoxTab FoxToolTab(FoxTool tool)
{
    for (uint32_t tab = 0u; tab < FOX_TAB_COUNT; ++tab)
    {
        for (uint32_t index = 0u; index < tabTools[tab].count; ++index)
        {
            if (tabTools[tab].buttons[index].tool == tool)
            {
                return (FoxTab)tab;
            }
        }
    }
    return FOX_TAB_NONE;
}

uint32_t FoxTimeScaleFactor(uint32_t scale)
{
    static const uint32_t factors[FOX_TIME_SCALE_COUNT] = {1u, 2u, 4u};
    return factors[scale < FOX_TIME_SCALE_COUNT ? scale : 0u];
}

/* The kinds of lane the road tools lay. */
static const Icon laneIcons[FOX_LANE_KIND_COUNT] = {
    [FOX_LANE_ASPHALT] = ICON_ASPHALT,
    [FOX_LANE_CONCRETE] = ICON_CONCRETE,
    [FOX_LANE_BIKE] = ICON_BIKE,
    [FOX_LANE_FOOT] = ICON_FOOT,
};

typedef struct SnapButton
{
    uint32_t snap;
    Icon icon;
} SnapButton;

static const SnapButton snapButtons[] = {
    {FOX_SNAP_NETWORK, ICON_MAGNET}, {FOX_SNAP_ANGLE, ICON_ANGLE},
    {FOX_SNAP_LENGTH, ICON_RULER},   {FOX_SNAP_GRID, ICON_GRID},
    {FOX_SNAP_GUIDES, ICON_GUIDES},  {FOX_SNAP_TANGENT, ICON_TANGENT},
};

static const FoxSign signChoices[] = {FOX_SIGN_AUTO, FOX_SIGN_NONE, FOX_SIGN_PRIORITY,
                                      FOX_SIGN_YIELD, FOX_SIGN_STOP};
static const Icon signIcons[] = {ICON_SIGN_AUTO, ICON_SIGN_NONE, ICON_SIGN_PRIORITY,
                                 ICON_SIGN_YIELD, ICON_SIGN_STOP};
#define SIGN_CHOICES ((uint32_t)(sizeof(signChoices) / sizeof(signChoices[0])))

/* A speed limit sign as a button: a white disc in a red ring with its
 * figures; the road's own limit is the circular arrow. */
static bool SpeedButton(FoxUi *ui, float x, float y, float size, uint32_t choice, bool chosen)
{
    const float s = ui->scale;
    const float centreX = x + size / 2.0f;
    const float centreY = y + size / 2.0f;
    const bool hover = Inside(ui, x, y, x + size, y + size);
    if (chosen)
    {
        Box(ui, x, y, x + size, y + size, COLOR_ACCENT, size / 2.0f);
    }
    else if (hover)
    {
        Box(ui, x, y, x + size, y + size, COLOR_HOVER, size / 2.0f);
    }
    if (choice == 0u)
    {
        DrawIcon(ui, ICON_RESET, centreX, centreY, PANEL_ICON * s,
                 chosen ? COLOR_WHITE : COLOR_ICON);
    }
    else
    {
        const float disc = size - 6.0f * s;
        Box(ui, centreX - disc / 2.0f, centreY - disc / 2.0f, centreX + disc / 2.0f,
            centreY + disc / 2.0f, COLOR_WHITE, disc / 2.0f);
        DrawIcon(ui, ICON_RING, centreX, centreY, disc, COLOR_RED);
        char figures[8];
        (void)snprintf(figures, sizeof(figures), "%d", (int)FoxSpeedChoiceKmh(choice));
        const float height = (FoxSpeedChoiceKmh(choice) >= 100 ? 9.0f : 11.0f) * s;
        Figures(ui, figures, centreX - FiguresWidth(figures, height) / 2.0f, centreY, height,
                COLOR_ICON);
    }
    return Pressed(ui, x, y, x + size, y + size);
}

/* The panel of a tool over its dock button, fitted to the view. */
static void PopoverFrame(FoxUi *ui, float anchorX, float bottom, float width, float height,
                         float *outLeft, float *outTop)
{
    const float s = ui->scale;
    const float margin = 8.0f * s;
    float left = anchorX - width / 2.0f;
    left = fmaxf(margin, fminf(left, (float)ui->input.width - margin - width));
    const float top = bottom - height;
    Panel(ui, left, top, left + width, bottom, PANEL_RADIUS * s);
    /* The little point at the tool. */
    const float point = 18.0f * s;
    DrawIcon(ui, ICON_POINTER_DOWN, anchorX, bottom + point / 2.0f - 0.5f * s, point, COLOR_PANEL);
    ui->over = ui->over || Inside(ui, left, top, left + width, bottom + point / 2.0f);
    *outLeft = left;
    *outTop = top;
}

static float RowWidth(uint32_t buttons, float s)
{
    return (float)buttons * PANEL_BUTTON * s + (float)(buttons - 1u) * PANEL_GAP * s;
}

static void RoadPopover(FoxUi *ui, FoxBuildSettings *settings, float anchorX, float bottom)
{
    const float s = ui->scale;
    const float button = PANEL_BUTTON * s;
    const float gap = PANEL_GAP * s;
    const float padding = PANEL_PADDING * s;
    const uint32_t snapCount = settings->tool == FOX_TOOL_CURVE
                                   ? (uint32_t)(sizeof(snapButtons) / sizeof(snapButtons[0]))
                                   : (uint32_t)(sizeof(snapButtons) / sizeof(snapButtons[0])) - 1u;
    const float labelWidth = PANEL_ICON * s + gap;
    const float angleWidth = labelWidth + SEGMENT_WIDTH * s * (float)FOX_ANGLE_STEP_COUNT;
    const float lengthWidth = labelWidth + SEGMENT_WIDTH * s * (float)FOX_LENGTH_STEP_COUNT;
    const float inner = fmaxf(fmaxf(RowWidth(FOX_LANE_KIND_COUNT, s), RowWidth(snapCount, s)),
                              fmaxf(angleWidth, lengthWidth));
    const float separator = 9.0f * s;
    const float height =
        padding * 2.0f + button * 2.0f + gap + separator * 2.0f + SEGMENT_HEIGHT * s * 2.0f + gap;
    float left = 0.0f;
    float top = 0.0f;
    PopoverFrame(ui, anchorX, bottom, inner + padding * 2.0f, height, &left, &top);
    float y = top + padding;
    float x = left + padding + (inner - RowWidth(FOX_LANE_KIND_COUNT, s)) / 2.0f;
    for (uint32_t kind = 0u; kind < FOX_LANE_KIND_COUNT; ++kind)
    {
        if (Button(ui, x, y, button, PANEL_ICON * s, laneIcons[kind],
                   settings->laneKind == (FoxLaneKind)kind ? LOOK_CHOSEN : LOOK_PLAIN))
        {
            settings->laneKind = (FoxLaneKind)kind;
        }
        x += button + gap;
    }
    y += button + separator / 2.0f;
    Box(ui, left + padding, y, left + padding + inner, y + fmaxf(1.0f, 0.75f * s), COLOR_DIVIDER,
        0.0f);
    y += separator / 2.0f;
    x = left + padding + (inner - RowWidth(snapCount, s)) / 2.0f;
    for (uint32_t index = 0u; index < snapCount; ++index)
    {
        const bool on = (settings->snaps & snapButtons[index].snap) != 0u;
        if (Button(ui, x, y, button, PANEL_ICON * s, snapButtons[index].icon,
                   on ? LOOK_ON : LOOK_PLAIN))
        {
            settings->snaps ^= snapButtons[index].snap;
        }
        x += button + gap;
    }
    y += button + separator / 2.0f;
    Box(ui, left + padding, y, left + padding + inner, y + fmaxf(1.0f, 0.75f * s), COLOR_DIVIDER,
        0.0f);
    y += separator / 2.0f;
    const bool angle = (settings->snaps & FOX_SNAP_ANGLE) != 0u;
    const bool length = (settings->snaps & (FOX_SNAP_LENGTH | FOX_SNAP_GRID)) != 0u;
    x = left + padding + (inner - angleWidth) / 2.0f;
    DrawIcon(ui, ICON_ANGLE, x + PANEL_ICON * s / 2.0f, y + SEGMENT_HEIGHT * s / 2.0f,
             PANEL_ICON * s * 0.85f, angle ? COLOR_ICON : COLOR_ICON_DIM);
    const int32_t angleChoice = Segments(ui, x + labelWidth, y, angleLabels, FOX_ANGLE_STEP_COUNT,
                                         settings->angleStep, !angle);
    if (angleChoice >= 0)
    {
        settings->angleStep = (uint32_t)angleChoice;
        settings->snaps |= FOX_SNAP_ANGLE;
    }
    y += SEGMENT_HEIGHT * s + gap;
    x = left + padding + (inner - angleWidth) / 2.0f;
    DrawIcon(ui, ICON_RULER, x + PANEL_ICON * s / 2.0f, y + SEGMENT_HEIGHT * s / 2.0f,
             PANEL_ICON * s * 0.85f, length ? COLOR_ICON : COLOR_ICON_DIM);
    const int32_t lengthChoice = Segments(ui, x + labelWidth, y, lengthLabels,
                                          FOX_LENGTH_STEP_COUNT, settings->lengthStep, !length);
    if (lengthChoice >= 0)
    {
        settings->lengthStep = (uint32_t)lengthChoice;
        if (!length)
        {
            settings->snaps |= FOX_SNAP_LENGTH;
        }
    }
}

static void SignPopover(FoxUi *ui, FoxBuildSettings *settings, float anchorX, float bottom)
{
    const float s = ui->scale;
    const float padding = PANEL_PADDING * s;
    const float button = PANEL_BUTTON * s;
    float left = 0.0f;
    float top = 0.0f;
    PopoverFrame(ui, anchorX, bottom, RowWidth(SIGN_CHOICES, s) + padding * 2.0f,
                 button + padding * 2.0f, &left, &top);
    for (uint32_t index = 0u; index < SIGN_CHOICES; ++index)
    {
        const float x = left + padding + (float)index * (button + PANEL_GAP * s);
        if (Button(ui, x, top + padding, button, PANEL_ICON * s, signIcons[index],
                   settings->sign == signChoices[index] ? LOOK_CHOSEN : LOOK_PLAIN))
        {
            settings->sign = signChoices[index];
        }
    }
}

static void SpeedPopover(FoxUi *ui, FoxBuildSettings *settings, float anchorX, float bottom)
{
    const float s = ui->scale;
    const float padding = PANEL_PADDING * s;
    const float button = PANEL_BUTTON * s;
    const float gap = PANEL_GAP * s;
    const uint32_t perRow = (FOX_SPEED_CHOICE_COUNT + 1u) / 2u;
    float left = 0.0f;
    float top = 0.0f;
    PopoverFrame(ui, anchorX, bottom, RowWidth(perRow, s) + padding * 2.0f,
                 button * 2.0f + gap + padding * 2.0f, &left, &top);
    for (uint32_t choice = 0u; choice < FOX_SPEED_CHOICE_COUNT; ++choice)
    {
        const uint32_t row = choice / perRow;
        const uint32_t column = choice % perRow;
        const float x = left + padding + (float)column * (button + gap);
        const float y = top + padding + (float)row * (button + gap);
        if (SpeedButton(ui, x, y, button, choice, settings->speedChoice == choice))
        {
            settings->speedChoice = choice;
        }
    }
}

static void BrushPopover(FoxUi *ui, FoxBuildSettings *settings, float anchorX, float bottom)
{
    static const Icon brushIcons[FOX_BRUSH_COUNT] = {ICON_BRUSH_SMALL, ICON_BRUSH_MEDIUM,
                                                     ICON_BRUSH_LARGE};
    const float s = ui->scale;
    const float padding = PANEL_PADDING * s;
    const float button = PANEL_BUTTON * s;
    float left = 0.0f;
    float top = 0.0f;
    PopoverFrame(ui, anchorX, bottom, RowWidth(FOX_BRUSH_COUNT, s) + padding * 2.0f,
                 button + padding * 2.0f, &left, &top);
    for (uint32_t brush = 0u; brush < FOX_BRUSH_COUNT; ++brush)
    {
        const float x = left + padding + (float)brush * (button + PANEL_GAP * s);
        if (Button(ui, x, top + padding, button, PANEL_ICON * s, brushIcons[brush],
                   settings->brush == brush ? LOOK_CHOSEN : LOOK_PLAIN))
        {
            settings->brush = brush;
        }
    }
}

/* The measurement beside the pointer: a ruler and the length, an angle and
 * its degrees. */
static void Readout(FoxUi *ui)
{
    if (!ui->readout || !ui->input.pointer)
    {
        return;
    }
    const float s = ui->scale;
    char length[16];
    char angle[16];
    (void)snprintf(length, sizeof(length), "%.0f", fmin(fmax(ui->metres, 0.0), 99999.0));
    (void)snprintf(angle, sizeof(angle), "%.0f*", fmin(fmax(ui->degrees, 0.0), 360.0));
    const float figures = DIGIT_HEIGHT * s;
    const float icon = 16.0f * s;
    const float gap = 5.0f * s;
    const float padding = 9.0f * s;
    const float height = 26.0f * s;
    const float width = padding * 2.0f + icon * 2.0f + gap * 2.0f + 10.0f * s +
                        FiguresWidth(length, figures) + FiguresWidth(angle, figures);
    float x = ui->input.x + 18.0f * s;
    float y = ui->input.y + 18.0f * s;
    x = fminf(x, (float)ui->input.width - width - 4.0f * s);
    y = fminf(y, (float)ui->input.height - height - 4.0f * s);
    Panel(ui, x, y, x + width, y + height, height / 2.0f);
    const float middle = y + height / 2.0f;
    x += padding;
    DrawIcon(ui, ICON_RULER, x + icon / 2.0f, middle, icon, COLOR_ICON);
    x += icon + gap;
    Figures(ui, length, x, middle, figures, COLOR_ICON);
    x += FiguresWidth(length, figures) + 10.0f * s;
    DrawIcon(ui, ICON_ANGLE, x + icon / 2.0f, middle, icon, COLOR_ICON);
    x += icon + gap;
    Figures(ui, angle, x, middle, figures, COLOR_ICON);
}

/* How many cars or people, with a button for fewer and one for more:
 * -1, 0 or 1 for which was pressed. */
static int32_t CountRow(FoxUi *ui, float x, float y, Icon icon, uint32_t count, float width)
{
    const float s = ui->scale;
    const float button = PANEL_BUTTON * s;
    DrawIcon(ui, icon, x + button / 2.0f, y + button / 2.0f, PANEL_ICON * s, COLOR_ICON);
    int32_t change = 0;
    if (Button(ui, x + button + PANEL_GAP * s, y, button, PANEL_ICON * s, ICON_MINUS, LOOK_PLAIN))
    {
        change = -1;
    }
    if (Button(ui, x + width - button, y, button, PANEL_ICON * s, ICON_PLUS, LOOK_PLAIN))
    {
        change = 1;
    }
    char figures[16];
    (void)snprintf(figures, sizeof(figures), "%u", count);
    const float height = DIGIT_HEIGHT * s;
    const float middle =
        x + button * 2.0f + PANEL_GAP * s + (width - button * 3.0f - PANEL_GAP * s) / 2.0f;
    Figures(ui, figures, middle - FiguresWidth(figures, height) / 2.0f, y + button / 2.0f, height,
            COLOR_ICON);
    return change;
}

/* The people tab: how many cars and walkers the city keeps. */
static void PeoplePopover(FoxUi *ui, const FoxUiStatus *status, FoxUiRequests *requests,
                          float anchorX, float bottom)
{
    const float s = ui->scale;
    const float padding = PANEL_PADDING * s;
    const float button = PANEL_BUTTON * s;
    const float width = button * 3.0f + PANEL_GAP * s + 70.0f * s;
    float left = 0.0f;
    float top = 0.0f;
    PopoverFrame(ui, anchorX, bottom, width + padding * 2.0f,
                 button * 2.0f + PANEL_GAP * s + padding * 2.0f, &left, &top);
    requests->carsChange =
        CountRow(ui, left + padding, top + padding, ICON_CAR, status->cars, width);
    requests->walkersChange = CountRow(ui, left + padding, top + padding + button + PANEL_GAP * s,
                                       ICON_PERSON, status->walkers, width);
}

/* The tools of the open tab in a row over its button; where the chosen one
 * stands, through outAnchor, or -1 when it is not in the row. */
static float ToolRow(FoxUi *ui, FoxBuildSettings *settings, float anchorX, float bottom,
                     float *outTop)
{
    const TabTools *row = &tabTools[settings->tab];
    const float s = ui->scale;
    const float padding = PANEL_PADDING * s;
    const float button = PANEL_BUTTON * s;
    float left = 0.0f;
    float top = 0.0f;
    PopoverFrame(ui, anchorX, bottom, RowWidth(row->count, s) + padding * 2.0f,
                 button + padding * 2.0f, &left, &top);
    *outTop = top;
    float anchor = -1.0f;
    for (uint32_t index = 0u; index < row->count; ++index)
    {
        const ToolButton *tool = &row->buttons[index];
        const float x = left + padding + (float)index * (button + PANEL_GAP * s);
        const bool chosen = settings->tool == tool->tool;
        if (chosen)
        {
            anchor = x + button / 2.0f;
        }
        if (Button(ui, x, top + padding, button, PANEL_ICON * s, tool->icon,
                   chosen ? LOOK_CHOSEN : LOOK_PLAIN))
        {
            /* The chosen tool again folds or unfolds its panel. */
            ui->folded = chosen && !ui->folded;
            settings->tool = tool->tool;
            ui->shownTool = settings->tool;
            anchor = x + button / 2.0f;
        }
    }
    return anchor;
}

/* The speed button: x1, x2 or x4, on to the next one at a press. */
static bool SpeedToggle(FoxUi *ui, float x, float y, float size, const FoxUiStatus *status)
{
    const bool hover = Inside(ui, x, y, x + size, y + size);
    if (hover)
    {
        Box(ui, x, y, x + size, y + size, COLOR_HOVER, size * 0.26f);
    }
    char label[8];
    (void)snprintf(label, sizeof(label), "x%u", FoxTimeScaleFactor(status->timeScale));
    const float height = 17.0f * ui->scale;
    Figures(ui, label, x + (size - FiguresWidth(label, height)) / 2.0f, y + size / 2.0f, height,
            status->paused ? COLOR_ICON_DIM : COLOR_ACCENT);
    return Pressed(ui, x, y, x + size, y + size);
}

bool FoxUiUpdate(FoxUi *ui, const FoxUiInput *input, const FoxUiStatus *status,
                 FoxBuildSettings *settings, FoxUiRequests *requests)
{
    if (requests != NULL)
    {
        *requests = (FoxUiRequests){0};
    }
    if (ui == NULL || input == NULL || status == NULL || settings == NULL || requests == NULL)
    {
        return false;
    }
    ui->quadCount = 0u;
    ui->input = *input;
    ui->over = false;
    if (input->width == 0u || input->height == 0u)
    {
        return false;
    }
    if (settings->tool != ui->shownTool)
    {
        ui->folded = false;
        ui->shownTool = settings->tool;
    }
    /* Sizes are in points. The display's density keeps a button the size of
     * a finger on a phone, a tall window grows them up to twice, and the dock
     * always fits across the view. */
    float points = DOCK_PADDING * 2.0f;
    for (uint32_t index = 0u; index < DOCK_ITEM_COUNT; ++index)
    {
        points += dockItems[index].kind == DOCK_DIVIDER ? GROUP_GAP * 2.0f : DOCK_BUTTON;
        points += index + 1u < DOCK_ITEM_COUNT && dockItems[index].kind != DOCK_DIVIDER &&
                          dockItems[index + 1u].kind != DOCK_DIVIDER
                      ? DOCK_GAP
                      : 0.0f;
    }
    const float density = input->density > 0.0f ? input->density : 1.0f;
    const float s = fminf(fmaxf(density, fminf(2.0f, (float)input->height / 900.0f)),
                          (float)input->width / (points + DOCK_MARGIN * 2.0f));
    ui->scale = s;
    const float button = DOCK_BUTTON * s;
    const float gap = DOCK_GAP * s;
    const float width = points * s;
    const float height = button + DOCK_PADDING * s * 2.0f;
    const float left = ((float)input->width - width) / 2.0f;
    const float top = (float)input->height - DOCK_MARGIN * s - height;
    Panel(ui, left, top, left + width, top + height, DOCK_RADIUS * s);
    ui->over = Inside(ui, left, top, left + width, top + height);

    float x = left + DOCK_PADDING * s;
    const float y = top + DOCK_PADDING * s;
    float tabAnchor = -1.0f;
    for (uint32_t index = 0u; index < DOCK_ITEM_COUNT; ++index)
    {
        const DockItem *item = &dockItems[index];
        if (item->kind == DOCK_DIVIDER)
        {
            x += GROUP_GAP * s;
            Box(ui, x - 0.5f * s, y + button * 0.22f, x + 0.5f * s, y + button * 0.78f,
                COLOR_DIVIDER, 0.0f);
            x += GROUP_GAP * s;
            continue;
        }
        if (item->kind == DOCK_SPEED)
        {
            requests->nextTimeScale = SpeedToggle(ui, x, y, button, status);
            x += button + gap;
            continue;
        }
        Look look = LOOK_PLAIN;
        Icon icon = (Icon)item->icon;
        switch (item->kind)
        {
        case DOCK_POINTER:
            look = settings->tab == FOX_TAB_NONE && settings->tool == FOX_TOOL_NONE ? LOOK_CHOSEN
                                                                                    : LOOK_PLAIN;
            break;
        case DOCK_TAB:
            look = settings->tab == (FoxTab)item->value ? LOOK_CHOSEN : LOOK_PLAIN;
            if (look == LOOK_CHOSEN)
            {
                tabAnchor = x + button / 2.0f;
            }
            break;
        case DOCK_PAUSE:
            icon = status->paused ? ICON_PLAY : ICON_PAUSE;
            look = status->paused ? LOOK_ON : LOOK_PLAIN;
            break;
        default:
            look = status->statistics ? LOOK_ON : LOOK_PLAIN;
            break;
        }
        if (Button(ui, x, y, button, DOCK_ICON * s, icon, look))
        {
            switch (item->kind)
            {
            case DOCK_POINTER:
                settings->tab = FOX_TAB_NONE;
                settings->tool = FOX_TOOL_NONE;
                break;
            case DOCK_TAB:
                if (settings->tab == (FoxTab)item->value)
                {
                    /* The open tab again closes it and puts the tool down. */
                    settings->tab = FOX_TAB_NONE;
                    settings->tool = FOX_TOOL_NONE;
                    tabAnchor = -1.0f;
                }
                else
                {
                    settings->tab = (FoxTab)item->value;
                    if (FoxToolTab(settings->tool) != settings->tab)
                    {
                        const TabTools *row = &tabTools[settings->tab];
                        settings->tool = row->count != 0u ? row->buttons[0].tool : FOX_TOOL_NONE;
                    }
                    tabAnchor = x + button / 2.0f;
                }
                break;
            case DOCK_PAUSE:
                requests->togglePause = true;
                break;
            default:
                requests->toggleStatistics = true;
                break;
            }
        }
        x += button + (index + 1u < DOCK_ITEM_COUNT && dockItems[index + 1u].kind != DOCK_DIVIDER
                           ? gap
                           : 0.0f);
    }

    const float bottom = top - PANEL_ABOVE * s;
    if (tabAnchor >= 0.0f && settings->tab == FOX_TAB_PEOPLE)
    {
        PeoplePopover(ui, status, requests, tabAnchor, bottom);
    }
    else if (tabAnchor >= 0.0f && tabTools[settings->tab].count != 0u)
    {
        float rowTop = 0.0f;
        const float anchor = ToolRow(ui, settings, tabAnchor, bottom, &rowTop);
        const float above = rowTop - PANEL_ABOVE * s;
        if (anchor >= 0.0f && !ui->folded)
        {
            switch (settings->tool)
            {
            case FOX_TOOL_ROAD:
            case FOX_TOOL_CURVE:
                RoadPopover(ui, settings, anchor, above);
                break;
            case FOX_TOOL_SIGNS:
                SignPopover(ui, settings, anchor, above);
                break;
            case FOX_TOOL_SPEED_LIMIT:
                SpeedPopover(ui, settings, anchor, above);
                break;
            case FOX_TOOL_DIG:
            case FOX_TOOL_RAISE:
                BrushPopover(ui, settings, anchor, above);
                break;
            default:
                break;
            }
        }
    }
    if (!ui->over)
    {
        Readout(ui);
    }
    return ui->over;
}

void FoxUiSetReadout(FoxUi *ui, bool show, double metres, double degrees)
{
    if (ui != NULL)
    {
        ui->readout = show;
        ui->metres = metres;
        ui->degrees = degrees;
    }
}

void FoxUiDraw(FoxUi *ui)
{
    if (ui != NULL && ui->quadCount != 0u)
    {
        (void)ui->device->submitUi(ui->device, ui->quads, ui->quadCount);
    }
}
