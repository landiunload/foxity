#include "client/road_mesh.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* Heights above the ground: the asphalt, then everything painted on it. */
#define ASPHALT_LIFT 0.15
#define MARKING_LIFT 0.19
/* What the tools show lies over the markings. */
#define OVERLAY_LIFT 0.3
#define RING_SEGMENTS 32u
/* Paved beyond the outer lanes. */
#define SHOULDER ((double)FOX_ROAD_SHOULDER / FOX_UNITS_PER_METRE)
#define LINE_HALF 0.08
#define EDGE_HALF 0.07
#define DOUBLE_LINE_SPACING 0.14
#define DASH 3.0
#define DASH_GAP 6.0
/* Lane lines turn solid this far before an intersection. */
#define SOLID_NEAR 15.0
#define FILLET_SEGMENTS 8u
#define CAP_SEGMENTS 12u
/* Signs stand this far beyond the paved edge. */
#define POST_CLEARANCE 0.6
/* Roads shorter than this between their trims get no limit sign. */
#define SPEED_SIGN_MIN_LENGTH 30.0
#define PI 3.14159265358979323846

#define COLOR_ASPHALT UINT32_C(0xFF3C3A38)
#define COLOR_LINE UINT32_C(0xFFE6E6E6)
#define COLOR_POLE UINT32_C(0xFF8C8C8C)
#define COLOR_HOUSING UINT32_C(0xFF262626)
#define COLOR_LAMP_OFF UINT32_C(0xFF3A3A3A)
#define COLOR_RED UINT32_C(0xFF2A2AD0)
#define COLOR_YELLOW UINT32_C(0xFF1EC4F2)
#define COLOR_WHITE UINT32_C(0xFFF2F2F2)
#define COLOR_BLACK UINT32_C(0xFF1E1E1E)

/* ---- Vectors ------------------------------------------------------------------ */

typedef struct V3
{
    double x;
    double y;
    double z;
} V3;

static V3 Add(V3 a, V3 b)
{
    return (V3){a.x + b.x, a.y + b.y, a.z + b.z};
}

static V3 Sub(V3 a, V3 b)
{
    return (V3){a.x - b.x, a.y - b.y, a.z - b.z};
}

static V3 Scale(V3 a, double s)
{
    return (V3){a.x * s, a.y * s, a.z * s};
}

static double Dot(V3 a, V3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static V3 Cross(V3 a, V3 b)
{
    return (V3){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

static double Length(V3 a)
{
    return sqrt(Dot(a, a));
}

static V3 Unit(V3 a)
{
    const double length = Length(a);
    return length > 0.0 ? Scale(a, 1.0 / length) : a;
}

/* a + b * s + c * t */
static V3 Along(V3 a, V3 b, double s, V3 c, double t)
{
    return Add(a, Add(Scale(b, s), Scale(c, t)));
}

static V3 Local(FoxVec point, FoxVec origin)
{
    return (V3){(double)(point.x - origin.x) / FOX_UNITS_PER_METRE,
                (double)(point.y - origin.y) / FOX_UNITS_PER_METRE,
                (double)(point.z - origin.z) / FOX_UNITS_PER_METRE};
}

static FoxVec World(V3 local, FoxVec origin)
{
    return (FoxVec){origin.x + llround(local.x * FOX_UNITS_PER_METRE),
                    origin.y + llround(local.y * FOX_UNITS_PER_METRE),
                    origin.z + llround(local.z * FOX_UNITS_PER_METRE)};
}

static V3 UpOf(FoxVec point)
{
    return Unit((V3){(double)point.x, (double)point.y, (double)point.z});
}

static V3 FromDirection(FoxVec direction)
{
    return Unit((V3){(double)direction.x, (double)direction.y, (double)direction.z});
}

/* ---- Meshes -------------------------------------------------------------------- */

void FoxMeshReset(FoxMesh *mesh)
{
    mesh->count = 0u;
    mesh->failed = false;
}

void FoxMeshFree(FoxMesh *mesh)
{
    free(mesh->vertices);
    memset(mesh, 0, sizeof(*mesh));
}

static void Vertex(FoxMesh *mesh, V3 position, uint32_t color)
{
    if (mesh->count >= mesh->capacity)
    {
        if (mesh->count > UINT32_MAX / 2u)
        {
            mesh->failed = true;
            return;
        }
        const uint32_t capacity = mesh->count < 512u ? 1024u : mesh->count * 2u;
        LaiueGraphicsVertexV2 *grown =
            realloc(mesh->vertices, (size_t)capacity * sizeof(*grown));
        if (grown == NULL)
        {
            mesh->failed = true;
            return;
        }
        mesh->vertices = grown;
        mesh->capacity = capacity;
    }
    mesh->vertices[mesh->count++] = (LaiueGraphicsVertexV2){
        .position = {(float)position.x, (float)position.y, (float)position.z},
        .colorRGBA = color,
    };
}

static void Triangle(FoxMesh *mesh, V3 a, V3 b, V3 c, uint32_t color)
{
    Vertex(mesh, a, color);
    Vertex(mesh, b, color);
    Vertex(mesh, c, color);
}

static void Quad(FoxMesh *mesh, V3 a, V3 b, V3 c, V3 d, uint32_t color)
{
    Triangle(mesh, a, b, c, color);
    Triangle(mesh, a, c, d, color);
}

static uint32_t Shade(uint32_t color, double factor)
{
    uint32_t result = color & UINT32_C(0xFF000000);
    for (uint32_t channel = 0u; channel < 3u; ++channel)
    {
        const double value = (double)((color >> (8u * channel)) & 255u) * factor;
        result |= (uint32_t)lround(fmin(value, 255.0)) << (8u * channel);
    }
    return result;
}

/* A box around `centre` with half sizes along three perpendicular axes; the
 * top is lit most, the bottom least. */
static void Box(FoxMesh *mesh, V3 centre, V3 ax, V3 ay, V3 az, double hx, double hy, double hz,
                uint32_t color)
{
    V3 corners[8];
    for (uint32_t corner = 0u; corner < 8u; ++corner)
    {
        const double sx = (corner & 1u) != 0u ? hx : -hx;
        const double sy = (corner & 2u) != 0u ? hy : -hy;
        const double sz = (corner & 4u) != 0u ? hz : -hz;
        corners[corner] = Add(Along(centre, ax, sx, ay, sy), Scale(az, sz));
    }
    static const uint8_t faces[6][4] = {
        {4u, 5u, 7u, 6u}, {0u, 2u, 3u, 1u}, {0u, 1u, 5u, 4u},
        {2u, 6u, 7u, 3u}, {0u, 4u, 6u, 2u}, {1u, 3u, 7u, 5u},
    };
    static const double shades[6] = {1.0, 0.55, 0.8, 0.72, 0.86, 0.66};
    for (uint32_t face = 0u; face < 6u; ++face)
    {
        Quad(mesh, corners[faces[face][0]], corners[faces[face][1]], corners[faces[face][2]],
             corners[faces[face][3]], Shade(color, shades[face]));
    }
}

/* A regular polygon in the plane of `right` and `up`, a fan from its centre. */
static void Polygon(FoxMesh *mesh, V3 centre, V3 right, V3 up, uint32_t sides, double radius,
                    double startAngle, uint32_t color)
{
    for (uint32_t side = 0u; side < sides; ++side)
    {
        const double a = startAngle + 2.0 * PI * (double)side / (double)sides;
        const double b = startAngle + 2.0 * PI * (double)(side + 1u) / (double)sides;
        Triangle(mesh, centre, Along(centre, right, cos(a) * radius, up, sin(a) * radius),
                 Along(centre, right, cos(b) * radius, up, sin(b) * radius), color);
    }
}

/* A rectangle [x0, x1] x [y0, y1] in the plane of `right` and `up`. */
static void Rectangle(FoxMesh *mesh, V3 centre, V3 right, V3 up, double x0, double x1, double y0,
                      double y1, uint32_t color)
{
    Quad(mesh, Along(centre, right, x0, up, y0), Along(centre, right, x1, up, y0),
         Along(centre, right, x1, up, y1), Along(centre, right, x0, up, y1), color);
}

/* ---- Stations along a centre line -------------------------------------------- */

/* A point of a centre line with the frame there: up from the planet, right
 * of the direction from the first point to the last. */
typedef struct Station
{
    V3 point;
    V3 up;
    V3 right;
    double along;
} Station;

static Station MakeStation(V3 point, V3 tangent, V3 up, double along)
{
    return (Station){
        .point = point,
        .up = up,
        .right = Unit(Cross(Unit(tangent), up)),
        .along = along,
    };
}

/* The station `along` metres down a polyline, with the direction of the
 * segment holding it. */
static Station StationAt(const FoxVec *points, uint32_t count, FoxVec origin, double along)
{
    double travelled = 0.0;
    for (uint32_t index = 0u; index + 1u < count; ++index)
    {
        const V3 a = Local(points[index], origin);
        const V3 b = Local(points[index + 1u], origin);
        const double length = Length(Sub(b, a));
        if (travelled + length >= along || index + 2u == count)
        {
            const double t = length > 0.0 ? fmin(fmax((along - travelled) / length, 0.0), 1.0) : 0.0;
            const V3 point = Add(a, Scale(Sub(b, a), t));
            const FoxVec at = FoxVecLerp(points[index], points[index + 1u],
                                         (int64_t)llround(t * (double)FOX_UNIT));
            return MakeStation(point, Sub(b, a), UpOf(at), along);
        }
        travelled += length;
    }
    return MakeStation(Local(points[0], origin), (V3){1.0, 0.0, 0.0}, UpOf(points[0]), along);
}

/* Stations from `from` to `to` metres: the ends and every centre point
 * between them, whose frames follow the mean of the segments they join so
 * neighbouring pieces share their edges. Returns the count written. */
static uint32_t BuildStations(const FoxVec *points, uint32_t count, FoxVec origin, double from,
                              double to, Station *out)
{
    uint32_t written = 0u;
    out[written++] = StationAt(points, count, origin, from);
    double travelled = 0.0;
    for (uint32_t index = 1u; index + 1u < count; ++index)
    {
        const V3 previous = Local(points[index - 1u], origin);
        const V3 point = Local(points[index], origin);
        const V3 next = Local(points[index + 1u], origin);
        travelled += Length(Sub(point, previous));
        if (travelled > from && travelled < to)
        {
            const V3 tangent = Add(Unit(Sub(point, previous)), Unit(Sub(next, point)));
            out[written++] = MakeStation(point, tangent, UpOf(points[index]), travelled);
        }
    }
    out[written++] = StationAt(points, count, origin, to);
    return written;
}

static V3 Offset(const Station *station, double right, double lift)
{
    return Along(station->point, station->right, right, station->up, lift);
}

static Station Blend(const Station *a, const Station *b, double t)
{
    return (Station){
        .point = Add(a->point, Scale(Sub(b->point, a->point), t)),
        .up = Unit(Add(a->up, Scale(Sub(b->up, a->up), t))),
        .right = Unit(Add(a->right, Scale(Sub(b->right, a->right), t))),
        .along = a->along + (b->along - a->along) * t,
    };
}

/* A strip between two offsets from the centre over [from, to] metres of it,
 * dashed when `dash` is non-zero; dashes count from zero along the line. */
static void Strip(FoxMesh *mesh, const Station *stations, uint32_t count, double left,
                  double right, double lift, double from, double to, double dash, double gap,
                  uint32_t color)
{
    for (uint32_t index = 0u; index + 1u < count; ++index)
    {
        const Station *a = &stations[index];
        const Station *b = &stations[index + 1u];
        const double span = b->along - a->along;
        if (span <= 0.0 || b->along <= from || a->along >= to)
        {
            continue;
        }
        double piece = fmax(from, a->along);
        while (piece < fmin(to, b->along))
        {
            double end = fmin(to, b->along);
            if (dash > 0.0)
            {
                const double phase = fmod(piece, dash + gap);
                if (phase >= dash)
                {
                    piece += dash + gap - phase;
                    continue;
                }
                end = fmin(end, piece + dash - phase);
            }
            const Station p0 = Blend(a, b, (piece - a->along) / span);
            const Station p1 = Blend(a, b, (end - a->along) / span);
            Quad(mesh, Offset(&p0, left, lift), Offset(&p0, right, lift), Offset(&p1, right, lift),
                 Offset(&p1, left, lift), color);
            piece = end;
        }
    }
}

/* A painted line at an offset, solid except for a dashed middle stretch. */
static void LaneLine(FoxMesh *mesh, const Station *stations, uint32_t count, double at, double from,
                     double to, double dashFrom, double dashTo, bool dashed)
{
    if (!dashed || dashFrom >= dashTo)
    {
        Strip(mesh, stations, count, at - LINE_HALF, at + LINE_HALF, MARKING_LIFT, from, to, 0.0,
              0.0, COLOR_LINE);
        return;
    }
    Strip(mesh, stations, count, at - LINE_HALF, at + LINE_HALF, MARKING_LIFT, from, dashFrom, 0.0,
          0.0, COLOR_LINE);
    Strip(mesh, stations, count, at - LINE_HALF, at + LINE_HALF, MARKING_LIFT, dashFrom, dashTo,
          DASH, DASH_GAP, COLOR_LINE);
    Strip(mesh, stations, count, at - LINE_HALF, at + LINE_HALF, MARKING_LIFT, dashTo, to, 0.0, 0.0,
          COLOR_LINE);
}

double FoxRoadMeshHalfWidth(FoxRoadType type)
{
    const FoxRoadTypeInfo *info = FoxRoadTypeGet(type);
    return info == NULL ? 0.0
                        : (double)(info->forwardLanes + info->backwardLanes) *
                              (double)FOX_LANE_WIDTH / 2.0 / FOX_UNITS_PER_METRE;
}

static double LaneWidth(void)
{
    return (double)FOX_LANE_WIDTH / FOX_UNITS_PER_METRE;
}

/* The lane markings of a road type over [from, to] of a centre line: edge
 * lines, the centre line and lane lines, dashed in [dashFrom, dashTo]. */
static void Markings(FoxMesh *mesh, const Station *stations, uint32_t count, FoxRoadType type,
                     double from, double to, double dashFrom, double dashTo)
{
    const FoxRoadTypeInfo *info = FoxRoadTypeGet(type);
    const double half = FoxRoadMeshHalfWidth(type);
    const double lane = LaneWidth();
    for (int side = -1; side <= 1; side += 2)
    {
        const double edge = (double)side * (half - EDGE_HALF);
        Strip(mesh, stations, count, edge - EDGE_HALF, edge + EDGE_HALF, MARKING_LIFT, from, to,
              0.0, 0.0, COLOR_LINE);
    }
    if (info->backwardLanes != 0u)
    {
        if (info->forwardLanes == 1u)
        {
            /* A street: one dashed line to overtake across. */
            LaneLine(mesh, stations, count, 0.0, from, to, dashFrom, dashTo, true);
        }
        else
        {
            /* Bigger roads keep their directions apart with a double line. */
            LaneLine(mesh, stations, count, -DOUBLE_LINE_SPACING, from, to, 0.0, 0.0, false);
            LaneLine(mesh, stations, count, DOUBLE_LINE_SPACING, from, to, 0.0, 0.0, false);
        }
        /* Both directions carry as many lanes, so the lines are symmetric. */
        for (uint32_t index = 1u; index < info->forwardLanes; ++index)
        {
            for (int side = -1; side <= 1; side += 2)
            {
                LaneLine(mesh, stations, count, (double)side * lane * (double)index, from, to,
                         dashFrom, dashTo, true);
            }
        }
        return;
    }
    for (uint32_t index = 1u; index < info->forwardLanes; ++index)
    {
        LaneLine(mesh, stations, count, lane * ((double)info->forwardLanes / 2.0 - (double)index),
                 from, to, dashFrom, dashTo, true);
    }
}

/* ---- Painted signs on the lanes ------------------------------------------------- */

/* A lane's frame at a distance along it: the point, where it heads and its
 * right. */
typedef struct LaneFrame
{
    V3 point;
    V3 forward;
    V3 right;
    V3 up;
} LaneFrame;

static bool LaneFrameAt(const FoxNetwork *network, uint32_t lane, int64_t distance, FoxVec origin,
                        LaneFrame *out)
{
    FoxVec point;
    FoxVec direction;
    if (!FoxNetworkPathPoint(network, lane, distance, &point, &direction))
    {
        return false;
    }
    out->up = UpOf(point);
    V3 forward = FromDirection(direction);
    forward = Unit(Sub(forward, Scale(out->up, Dot(forward, out->up))));
    out->point = Local(point, origin);
    out->forward = forward;
    out->right = Unit(Cross(forward, out->up));
    return true;
}

/* A point of a lane frame `ahead` metres forward and `right` metres right. */
static V3 InLane(const LaneFrame *frame, double ahead, double right)
{
    return Add(Along(frame->point, frame->forward, ahead, frame->right, right),
               Scale(frame->up, MARKING_LIFT));
}

static void LaneQuad(FoxMesh *mesh, const LaneFrame *frame, double back, double front, double left,
                     double right)
{
    Quad(mesh, InLane(frame, back, left), InLane(frame, back, right),
         InLane(frame, front, right), InLane(frame, front, left), COLOR_LINE);
}

/* An arrow for every way a lane leads, painted at its frame. */
static void Arrow(FoxMesh *mesh, const LaneFrame *frame, uint32_t turns)
{
    const double stem = 0.08;
    LaneQuad(mesh, frame, -2.2, (turns & FOX_TURN_STRAIGHT) != 0u ? 0.9 : -0.3, -stem, stem);
    if ((turns & FOX_TURN_STRAIGHT) != 0u)
    {
        Triangle(mesh, InLane(frame, 0.9, -0.32), InLane(frame, 0.9, 0.32), InLane(frame, 1.7, 0.0),
                 COLOR_LINE);
    }
    for (int side = -1; side <= 1; side += 2)
    {
        const uint32_t bit = side < 0 ? FOX_TURN_LEFT : FOX_TURN_RIGHT;
        if ((turns & bit) == 0u)
        {
            continue;
        }
        /* A branch leaving the stem at 45 degrees, with its head. */
        const double s = (double)side;
        const double reach = 0.75;
        const double tipAhead = -0.5 + reach;
        const double tipSide = s * reach;
        const double nx = 0.7071 * stem;
        Quad(mesh, InLane(frame, -0.5 - nx, s * nx), InLane(frame, -0.5 + nx, -s * nx),
             InLane(frame, tipAhead + nx, tipSide - s * nx),
             InLane(frame, tipAhead - nx, tipSide + s * nx), COLOR_LINE);
        const double head = 0.3;
        const double length = 0.6;
        Triangle(mesh, InLane(frame, tipAhead - 0.7071 * head, tipSide + s * 0.7071 * head),
                 InLane(frame, tipAhead + 0.7071 * head, tipSide - s * 0.7071 * head),
                 InLane(frame, tipAhead + 0.7071 * length, tipSide + s * 0.7071 * length),
                 COLOR_LINE);
    }
}

/* The line across a lane at its end: a solid bar to stop at, or the teeth
 * of a give-way line pointing at the drivers. */
static void EndLine(FoxMesh *mesh, const LaneFrame *frame, bool stop)
{
    const double half = LaneWidth() / 2.0;
    if (stop)
    {
        LaneQuad(mesh, frame, -0.8, -0.4, -half, half);
        return;
    }
    const uint32_t teeth = 5u;
    const double width = 2.0 * half / (double)teeth;
    for (uint32_t tooth = 0u; tooth < teeth; ++tooth)
    {
        const double left = -half + width * ((double)tooth + 0.15);
        const double right = -half + width * ((double)tooth + 0.85);
        Triangle(mesh, InLane(frame, -0.3, left), InLane(frame, -0.3, right),
                 InLane(frame, -0.9, (left + right) / 2.0), COLOR_LINE);
    }
}

/* ---- Signs and lights ------------------------------------------------------------- */

/* A post standing at `base`, facing the traffic that comes along `forward`. */
typedef struct Post
{
    V3 base;
    V3 forward;
    V3 right;
    V3 up;
} Post;

/* The centre of a plate on a post, `layer` steps towards the traffic. */
static V3 PlateCentre(const Post *post, double height, uint32_t layer)
{
    return Along(post->base, post->up, height, post->forward, -0.06 - 0.012 * (double)layer);
}

static void Pole(FoxMesh *mesh, const Post *post, double height)
{
    Box(mesh, Add(post->base, Scale(post->up, height / 2.0)), post->forward, post->right, post->up,
        0.04, 0.04, height / 2.0, COLOR_POLE);
}

static void Digit(FoxMesh *mesh, V3 centre, V3 right, V3 up, uint32_t digit)
{
    static const uint8_t segments[10] = {0x3Fu, 0x06u, 0x5Bu, 0x4Fu, 0x66u,
                                         0x6Du, 0x7Du, 0x07u, 0x7Fu, 0x6Fu};
    const double w = 0.06;
    const double h = 0.12;
    const double t = 0.03;
    /* a, b, c, d, e, f, g: x0, x1, y0, y1. */
    const double boxes[7][4] = {
        {-w, w, h - t, h}, {w - t, w, 0.0, h},  {w - t, w, -h, 0.0}, {-w, w, -h, -h + t},
        {-w, -w + t, -h, 0.0}, {-w, -w + t, 0.0, h}, {-w, w, -t / 2.0, t / 2.0},
    };
    for (uint32_t segment = 0u; segment < 7u; ++segment)
    {
        if ((segments[digit % 10u] & (1u << segment)) != 0u)
        {
            Rectangle(mesh, centre, right, up, boxes[segment][0], boxes[segment][1],
                      boxes[segment][2], boxes[segment][3], COLOR_BLACK);
        }
    }
}

static void SpeedSign(FoxMesh *mesh, const Post *post, int32_t speed)
{
    const double height = 2.1;
    Pole(mesh, post, height);
    Polygon(mesh, PlateCentre(post, height, 0u), post->right, post->up, 24u, 0.42, 0.0, COLOR_RED);
    Polygon(mesh, PlateCentre(post, height, 1u), post->right, post->up, 24u, 0.33, 0.0,
            COLOR_WHITE);
    const uint32_t kmh = (uint32_t)lround((double)speed * 3.6 / FOX_UNITS_PER_METRE);
    uint32_t digits[3];
    uint32_t count = 0u;
    for (uint32_t value = kmh; count == 0u || (value != 0u && count < 3u); value /= 10u)
    {
        digits[count++] = value % 10u;
    }
    const double spacing = 0.16;
    for (uint32_t index = 0u; index < count; ++index)
    {
        const double x = ((double)index - (double)(count - 1u) / 2.0) * spacing;
        Digit(mesh, Add(PlateCentre(post, height, 2u), Scale(post->right, x)), post->right,
              post->up, digits[count - 1u - index]);
    }
}

static void JunctionSign(FoxMesh *mesh, const Post *post, FoxSign sign)
{
    if (sign != FOX_SIGN_PRIORITY && sign != FOX_SIGN_YIELD && sign != FOX_SIGN_STOP)
    {
        return;
    }
    const double height = 2.1;
    Pole(mesh, post, height);
    const V3 back = PlateCentre(post, height, 0u);
    const V3 front = PlateCentre(post, height, 1u);
    switch (sign)
    {
    case FOX_SIGN_PRIORITY:
        Polygon(mesh, back, post->right, post->up, 4u, 0.45, 0.0, COLOR_WHITE);
        Polygon(mesh, front, post->right, post->up, 4u, 0.31, 0.0, COLOR_YELLOW);
        break;
    case FOX_SIGN_YIELD:
        Polygon(mesh, back, post->right, post->up, 3u, 0.52, -PI / 2.0, COLOR_RED);
        Polygon(mesh, front, post->right, post->up, 3u, 0.32, -PI / 2.0, COLOR_WHITE);
        break;
    case FOX_SIGN_STOP:
        Polygon(mesh, back, post->right, post->up, 8u, 0.46, PI / 8.0, COLOR_WHITE);
        Polygon(mesh, front, post->right, post->up, 8u, 0.41, PI / 8.0, COLOR_RED);
        break;
    default:
        break;
    }
}

/* A traffic light: a pole with a housing whose lamps the view lights as the
 * signal changes. */
static void TrafficLight(FoxMesh *mesh, FoxSignalHeads *heads, const Post *post, FoxVec origin,
                         uint32_t road, uint32_t end)
{
    const double height = 3.3;
    Pole(mesh, post, height);
    const double depth = 0.13;
    const V3 housing = Along(post->base, post->up, 2.75, post->forward, -0.2);
    Box(mesh, housing, post->forward, post->right, post->up, depth, 0.17, 0.5, COLOR_HOUSING);
    FoxSignalHead head = {.road = road, .end = end};
    for (uint32_t lamp = 0u; lamp < 3u; ++lamp)
    {
        const V3 face = Along(housing, post->forward, -depth - 0.005, post->up,
                              0.3 - 0.3 * (double)lamp);
        Rectangle(mesh, face, post->right, post->up, -0.1, 0.1, -0.1, 0.1, COLOR_LAMP_OFF);
        head.lamps[lamp] = World(Add(face, Scale(post->forward, -0.04)), origin);
    }
    if (heads->count >= heads->capacity)
    {
        const uint32_t capacity = heads->count < 16u ? 16u : heads->count * 2u;
        FoxSignalHead *grown = realloc(heads->items, (size_t)capacity * sizeof(*grown));
        if (grown == NULL)
        {
            heads->failed = true;
            return;
        }
        heads->items = grown;
        heads->capacity = capacity;
    }
    heads->items[heads->count++] = head;
}

/* A post beside the curb lane of a lane frame, `back` metres before it. */
static Post CurbPost(const LaneFrame *frame, FoxDriveSide side, double back)
{
    const double curb = side == FOX_DRIVE_RIGHT ? 1.0 : -1.0;
    const double out = LaneWidth() / 2.0 + SHOULDER + POST_CLEARANCE;
    return (Post){
        .base = Along(frame->point, frame->forward, -back, frame->right, curb * out),
        .forward = frame->forward,
        .right = frame->right,
        .up = frame->up,
    };
}

/* ---- Roads -------------------------------------------------------------------------- */

static uint32_t RoadsAt(const FoxNetwork *network, uint32_t node)
{
    FoxNodeView view;
    return FoxNetworkNode(network, node, &view) ? view.roadCount : 0u;
}

/* Lines, arrows and the sign or light where a road's lanes meet a node. */
static void RoadEnd(const FoxNetwork *network, uint32_t roadIndex, const FoxRoadView *road,
                    uint32_t end, FoxVec origin, FoxMesh *detail, FoxSignalHeads *heads)
{
    FoxNodeView node;
    if (!FoxNetworkNode(network, road->nodes[end], &node) || node.roadCount < 3u)
    {
        return;
    }
    /* Lanes of direction 0 arrive at nodes[1]. */
    const uint32_t direction = end == 1u ? 0u : 1u;
    for (uint32_t lane = 0u; lane < road->laneCount[direction]; ++lane)
    {
        const uint32_t path = road->lanes[direction][lane];
        FoxPathView view;
        LaneFrame frame;
        if (!FoxNetworkPath(network, path, &view) ||
            !LaneFrameAt(network, path, view.length, origin, &frame))
        {
            continue;
        }
        if (node.signals || road->sign[end] == FOX_SIGN_STOP)
        {
            EndLine(detail, &frame, true);
        }
        else if (road->sign[end] == FOX_SIGN_YIELD)
        {
            EndLine(detail, &frame, false);
        }
        uint32_t turns = FoxNetworkLaneTurns(network, path);
        turns = turns == FOX_TURN_BACK ? 0u : (turns & ~FOX_TURN_BACK);
        LaneFrame arrow;
        if (turns != 0u && view.length > FOX_METRES(14) &&
            LaneFrameAt(network, path, view.length - FOX_METRES(7), origin, &arrow))
        {
            Arrow(detail, &arrow, turns);
        }
        if (lane == 0u)
        {
            const Post post = CurbPost(&frame, road->driveSide, 1.2);
            if (node.signals)
            {
                TrafficLight(detail, heads, &post, origin, roadIndex, end);
            }
            else
            {
                JunctionSign(detail, &post, road->sign[end]);
            }
        }
    }
}

void FoxRoadMeshRoad(const FoxNetwork *network, uint32_t roadIndex, FoxVec origin, FoxMesh *base,
                     FoxMesh *detail, FoxSignalHeads *heads)
{
    FoxRoadView road;
    if (!FoxNetworkRoad(network, roadIndex, &road) || road.centreCount < 2u)
    {
        return;
    }
    const double length = (double)road.length / FOX_UNITS_PER_METRE;
    const double from = (double)road.trim[0] / FOX_UNITS_PER_METRE;
    const double to = length - (double)road.trim[1] / FOX_UNITS_PER_METRE;
    if (to <= from)
    {
        return;
    }
    Station *stations = malloc(((size_t)road.centreCount + 2u) * sizeof(*stations));
    if (stations == NULL)
    {
        base->failed = true;
        return;
    }
    const uint32_t count = BuildStations(road.centre, road.centreCount, origin, from, to, stations);
    const double paved = FoxRoadMeshHalfWidth(road.type) + SHOULDER;
    Strip(base, stations, count, -paved, paved, ASPHALT_LIFT, from, to, 0.0, 0.0, COLOR_ASPHALT);
    const double dashFrom = from + (RoadsAt(network, road.nodes[0]) >= 3u ? SOLID_NEAR : 0.0);
    const double dashTo = to - (RoadsAt(network, road.nodes[1]) >= 3u ? SOLID_NEAR : 0.0);
    Markings(detail, stations, count, road.type, from, to, dashFrom, dashTo);
    free(stations);
    for (uint32_t end = 0u; end < 2u; ++end)
    {
        RoadEnd(network, roadIndex, &road, end, origin, detail, heads);
    }
    /* The limit, after every junction it starts from. */
    for (uint32_t direction = 0u; direction < 2u; ++direction)
    {
        FoxPathView view;
        LaneFrame frame;
        if (road.laneCount[direction] == 0u || to - from < SPEED_SIGN_MIN_LENGTH ||
            !FoxNetworkPath(network, road.lanes[direction][0], &view) ||
            !LaneFrameAt(network, road.lanes[direction][0], FOX_METRES(8), origin, &frame))
        {
            continue;
        }
        const Post post = CurbPost(&frame, road.driveSide, 0.0);
        SpeedSign(detail, &post, road.speedLimit);
    }
}

/* ---- Nodes ---------------------------------------------------------------------------- */

/* Between two roads closer than this the curb rounds the corner where their
 * edges meet; wider apart it sweeps round from one road to the other. The
 * simulation trims roads by the same rule. */
#define SWEEP_GAP (PI * 5.0 / 6.0)
#define SWEEP_SEGMENTS 12u
/* A turning circle: the arc round its far side and each flare into it. */
#define BULB_SEGMENTS 24u
#define FLARE_SEGMENTS 6u
#define OUTLINE_CAPACITY (FOX_NODE_MAX_ROADS * (SWEEP_SEGMENTS + 2u) + BULB_SEGMENTS + 16u)

/* A road where it meets a node: its station at the trim, the direction away
 * from the node, the right of that direction and its paved half width. */
typedef struct RoadEndShape
{
    Station station;
    V3 outward;
    V3 right;
    double paved;
    double angle;
    FoxRoadType type;
    FoxDriveSide side;
} RoadEndShape;

static bool EndShape(const FoxNetwork *network, uint32_t roadIndex, uint32_t node, FoxVec origin,
                     RoadEndShape *out)
{
    FoxRoadView road;
    if (!FoxNetworkRoad(network, roadIndex, &road) || road.centreCount < 2u)
    {
        return false;
    }
    const uint32_t end = road.nodes[0] == node ? 0u : 1u;
    const double length = (double)road.length / FOX_UNITS_PER_METRE;
    const double trim = (double)road.trim[end] / FOX_UNITS_PER_METRE;
    out->station =
        StationAt(road.centre, road.centreCount, origin, end == 0u ? trim : length - trim);
    const V3 tangent = Unit(Cross(out->station.up, out->station.right));
    out->outward = end == 0u ? tangent : Scale(tangent, -1.0);
    out->right = end == 0u ? out->station.right : Scale(out->station.right, -1.0);
    out->paved = FoxRoadMeshHalfWidth(road.type) + SHOULDER;
    out->type = road.type;
    out->side = road.driveSide;
    return true;
}

static V3 Lift(V3 point, V3 up, double lift)
{
    return Add(point, Scale(up, lift));
}

static V3 RightCorner(const RoadEndShape *end)
{
    return Add(end->station.point, Scale(end->right, end->paved));
}

static V3 LeftCorner(const RoadEndShape *end)
{
    return Sub(end->station.point, Scale(end->right, end->paved));
}

/* The outline of what a node paves, anticlockwise seen from above: the
 * mouths of its roads and the curbs between them. Corners at the mouths keep
 * the up of their road, so the asphalt meets the road's without a seam. */
typedef struct Outline
{
    V3 points[OUTLINE_CAPACITY];
    V3 lift[OUTLINE_CAPACITY];
    /* Whether the edge from a point to the next one is a curb. */
    bool curb[OUTLINE_CAPACITY];
    uint32_t count;
} Outline;

static void Put(Outline *outline, V3 point, V3 up, bool curb)
{
    if (outline->count < OUTLINE_CAPACITY)
    {
        outline->points[outline->count] = point;
        outline->lift[outline->count] = up;
        outline->curb[outline->count++] = curb;
    }
}

/* The mouth of a road: its right corner, then its left, from where the curb
 * goes on. */
static void Mouth(Outline *outline, const RoadEndShape *end)
{
    Put(outline, RightCorner(end), end->station.up, false);
    Put(outline, LeftCorner(end), end->station.up, true);
}

/* The curb between the left edge of one road and the right edge of the next
 * one round: a curve that leaves both edges smoothly where they would meet,
 * a straight line where they never do. */
static uint32_t Fillet(V3 from, V3 fromDirection, V3 to, V3 toDirection, V3 ex, V3 ey,
                       double reach, V3 *out)
{
    const V3 gap = Sub(from, to);
    const double ax = Dot(fromDirection, ex);
    const double ay = Dot(fromDirection, ey);
    const double bx = Dot(toDirection, ex);
    const double by = Dot(toDirection, ey);
    const double gx = Dot(gap, ex);
    const double gy = Dot(gap, ey);
    /* from - s * a = to - u * b, so s * a - u * b = from - to. */
    const double det = -ax * by + ay * bx;
    uint32_t written = 0u;
    if (fabs(det) > 1e-6)
    {
        const double s = (-gx * by + gy * bx) / det;
        const double u = (ax * gy - ay * gx) / det;
        if (s > 0.0 && u > 0.0 && s < reach && u < reach)
        {
            const V3 control = Sub(from, Scale(fromDirection, s));
            for (uint32_t step = 1u; step < FILLET_SEGMENTS; ++step)
            {
                const double t = (double)step / FILLET_SEGMENTS;
                const double a = (1.0 - t) * (1.0 - t);
                const double b = 2.0 * (1.0 - t) * t;
                const double c = t * t;
                out[written++] = Add(Add(Scale(from, a), Scale(control, b)), Scale(to, c));
            }
        }
    }
    return written;
}

/* A cubic that leaves p0 along t0 and arrives at p3 along t3, at t. */
static V3 CubicPoint(V3 p0, V3 t0, V3 p3, V3 t3, double t)
{
    const double reach = Length(Sub(p3, p0)) * 0.4;
    const V3 p1 = Add(p0, Scale(t0, reach));
    const V3 p2 = Sub(p3, Scale(t3, reach));
    const double s = 1.0 - t;
    return Add(Add(Scale(p0, s * s * s), Scale(p1, 3.0 * s * s * t)),
               Add(Scale(p2, 3.0 * s * t * t), Scale(p3, t * t * t)));
}

/* The centre line from one road's end through the node to another's: a cubic
 * that leaves and joins both straight, sampled into stations. */
static void CentreCurve(const RoadEndShape *a, const RoadEndShape *b, V3 up, Station *out)
{
    const V3 p0 = a->station.point;
    const V3 p3 = b->station.point;
    const V3 t0 = Scale(a->outward, -1.0);
    const V3 t3 = b->outward;
    double travelled = 0.0;
    for (uint32_t step = 0u; step <= SWEEP_SEGMENTS; ++step)
    {
        const double t = (double)step / SWEEP_SEGMENTS;
        const V3 point = CubicPoint(p0, t0, p3, t3, t);
        /* The direction across the neighbouring half steps; at the ends the
         * roads'. */
        V3 tangent = step == 0u ? t0 : t3;
        if (step != 0u && step != SWEEP_SEGMENTS)
        {
            tangent = Sub(CubicPoint(p0, t0, p3, t3, t + 0.5 / SWEEP_SEGMENTS),
                          CubicPoint(p0, t0, p3, t3, t - 0.5 / SWEEP_SEGMENTS));
        }
        if (step != 0u)
        {
            travelled += Length(Sub(point, out[step - 1u].point));
        }
        out[step] = MakeStation(point, tangent, up, travelled);
    }
    /* The ends take the roads' own frames, so the edges meet exactly. */
    out[0].point = p0;
    out[0].up = a->station.up;
    out[0].right = Scale(a->right, -1.0);
    out[SWEEP_SEGMENTS].point = p3;
    out[SWEEP_SEGMENTS].up = b->station.up;
    out[SWEEP_SEGMENTS].right = b->right;
}

/* The curb from the left corner of road a round to the right corner of the
 * next road b, `gap` radians further anticlockwise; the corners themselves
 * are not included. */
static void Curb(Outline *outline, const RoadEndShape *a, const RoadEndShape *b, double gap, V3 up,
                 V3 ex, V3 ey, double reach)
{
    if (gap < SWEEP_GAP)
    {
        V3 fillet[FILLET_SEGMENTS];
        const uint32_t count =
            Fillet(LeftCorner(a), a->outward, RightCorner(b), b->outward, ex, ey, reach, fillet);
        for (uint32_t index = 0u; index < count; ++index)
        {
            Put(outline, fillet[index], up, true);
        }
        return;
    }
    /* Wide apart, the curb keeps beside the way from one road into the other,
     * its width changing from the one road's to the other's. */
    Station stations[SWEEP_SEGMENTS + 1u];
    CentreCurve(a, b, up, stations);
    for (uint32_t step = 1u; step < SWEEP_SEGMENTS; ++step)
    {
        const double t = (double)step / SWEEP_SEGMENTS;
        Put(outline, Offset(&stations[step], a->paved + (b->paved - a->paved) * t, 0.0), up, true);
    }
}

/* Paves an outline as a fan from the node and paints a line along its curbs,
 * just inside them as along the roads' edges. */
static void PaveOutline(const Outline *outline, V3 centre, V3 up, FoxMesh *base, FoxMesh *detail)
{
    const uint32_t count = outline->count;
    if (count < 3u)
    {
        return;
    }
    const V3 middle = Lift(centre, up, ASPHALT_LIFT);
    for (uint32_t index = 0u; index < count; ++index)
    {
        const uint32_t next = (index + 1u) % count;
        Triangle(base, middle, Lift(outline->points[index], outline->lift[index], ASPHALT_LIFT),
                 Lift(outline->points[next], outline->lift[next], ASPHALT_LIFT), COLOR_ASPHALT);
    }
    if (detail == NULL)
    {
        return;
    }
    for (uint32_t index = 0u; index < count; ++index)
    {
        if (!outline->curb[index])
        {
            continue;
        }
        const uint32_t next = (index + 1u) % count;
        V3 inward[2];
        for (uint32_t side = 0u; side < 2u; ++side)
        {
            /* Inside is to the left going round. Between two curb edges the
             * normals are averaged; where a curb meets a mouth it keeps its
             * own. */
            const uint32_t at = side == 0u ? index : next;
            const uint32_t before = (at + count - 1u) % count;
            const uint32_t after = (at + 1u) % count;
            const V3 own = side == 0u ? Unit(Cross(up, Sub(outline->points[after],
                                                            outline->points[at])))
                                      : Unit(Cross(up, Sub(outline->points[at],
                                                            outline->points[before])));
            V3 normal = own;
            if (outline->curb[before] && outline->curb[at])
            {
                const V3 previous = Unit(Cross(up, Sub(outline->points[at], outline->points[before])));
                const V3 following = Unit(Cross(up, Sub(outline->points[after], outline->points[at])));
                const V3 mean = Unit(Add(previous, following));
                normal = Scale(mean, 1.0 / fmax(Dot(mean, following), 0.5));
            }
            inward[side] = normal;
        }
        const V3 a = Lift(outline->points[index], up, MARKING_LIFT);
        const V3 b = Lift(outline->points[next], up, MARKING_LIFT);
        Quad(detail, Add(a, Scale(inward[0], SHOULDER)), Add(b, Scale(inward[1], SHOULDER)),
             Add(b, Scale(inward[1], SHOULDER + 2.0 * EDGE_HALF)),
             Add(a, Scale(inward[0], SHOULDER + 2.0 * EDGE_HALF)), COLOR_LINE);
    }
}

/* An intersection, or a bend that a piece of road cannot carry: the mouths
 * of its roads joined by curbs. */
static void Junction(const RoadEndShape *ends, const uint32_t *order, uint32_t count, V3 centre,
                     V3 up, FoxMesh *base, FoxMesh *detail)
{
    const V3 ex = ends[0].outward;
    const V3 ey = Unit(Cross(up, ex));
    double reach = 0.0;
    for (uint32_t index = 0u; index < count; ++index)
    {
        reach = fmax(reach, Length(Sub(ends[index].station.point, centre)) + ends[index].paved);
    }
    reach *= 3.0;
    Outline outline = {.count = 0u};
    for (uint32_t index = 0u; index < count; ++index)
    {
        const RoadEndShape *a = &ends[order[index]];
        const RoadEndShape *b = &ends[order[(index + 1u) % count]];
        double gap = b->angle - a->angle;
        gap += gap <= 0.0 ? 2.0 * PI : 0.0;
        Mouth(&outline, a);
        Curb(&outline, a, b, gap, up, ex, ey, reach);
    }
    PaveOutline(&outline, centre, up, base, detail);
}

/* Whether a piece of road can carry the lanes from one road to the other:
 * where it turns too tightly for its width, its inner edge would fold. */
static bool RibbonFits(const RoadEndShape *a, const RoadEndShape *b, V3 up)
{
    if (a->type != b->type)
    {
        return false;
    }
    Station stations[SWEEP_SEGMENTS + 1u];
    CentreCurve(a, b, up, stations);
    for (uint32_t step = 0u; step < SWEEP_SEGMENTS; ++step)
    {
        const V3 along = Sub(stations[step + 1u].point, stations[step].point);
        for (int side = -1; side <= 1; side += 2)
        {
            const V3 edge = Sub(Offset(&stations[step + 1u], (double)side * a->paved, 0.0),
                                Offset(&stations[step], (double)side * a->paved, 0.0));
            if (Dot(edge, along) <= 0.0)
            {
                return false;
            }
        }
    }
    return true;
}

/* A bend: a piece of road from one road's end to the other's that carries
 * the asphalt and the markings. */
static void Bend(const RoadEndShape *a, const RoadEndShape *b, V3 up, FoxMesh *base, FoxMesh *detail)
{
    Station stations[SWEEP_SEGMENTS + 1u];
    CentreCurve(a, b, up, stations);
    const double length = stations[SWEEP_SEGMENTS].along;
    Strip(base, stations, SWEEP_SEGMENTS + 1u, -a->paved, a->paved, ASPHALT_LIFT, 0.0, length, 0.0,
          0.0, COLOR_ASPHALT);
    if (detail != NULL && length > 0.0)
    {
        Markings(detail, stations, SWEEP_SEGMENTS + 1u, a->type, 0.0, length, 0.0, length);
    }
}

/* Through a bend paved as an outline only the line between the directions
 * carries on: dashed between streets, double elsewhere. */
static void BendCentreLine(const RoadEndShape *a, const RoadEndShape *b, V3 up, FoxMesh *detail)
{
    const FoxRoadTypeInfo *first = FoxRoadTypeGet(a->type);
    const FoxRoadTypeInfo *second = FoxRoadTypeGet(b->type);
    if (first->backwardLanes == 0u || second->backwardLanes == 0u)
    {
        return;
    }
    Station stations[SWEEP_SEGMENTS + 1u];
    CentreCurve(a, b, up, stations);
    const double length = stations[SWEEP_SEGMENTS].along;
    if (first->forwardLanes == 1u && second->forwardLanes == 1u)
    {
        LaneLine(detail, stations, SWEEP_SEGMENTS + 1u, 0.0, 0.0, length, 0.0, length, true);
        return;
    }
    for (int side = -1; side <= 1; side += 2)
    {
        LaneLine(detail, stations, SWEEP_SEGMENTS + 1u, (double)side * DOUBLE_LINE_SPACING, 0.0,
                 length, 0.0, 0.0, false);
    }
}

/* A dead end widens into a turning circle about its node, as wide as the
 * lanes turn round on. */
static void Bulb(const RoadEndShape *end, V3 centre, V3 up, FoxMesh *base, FoxMesh *detail)
{
    const double radius =
        (double)FoxRoadTurnaroundRadius(end->type) / FOX_UNITS_PER_METRE + LaneWidth() / 2.0 +
        SHOULDER;
    const V3 ex = end->outward;
    const V3 ey = Scale(end->right, -1.0);
    const double trim = Dot(Sub(end->station.point, centre), ex);
    /* The flares join the circle where it still lies well short of the
     * road's end. */
    const double join = acos(fmax(fmin(0.5, (trim - 1.0) / radius), -0.3));
    Outline outline = {.count = 0u};
    Mouth(&outline, end);
    const V3 inward = Scale(ex, -1.0);
    for (uint32_t flare = 0u; flare < 2u; ++flare)
    {
        /* Out of the left edge onto the circle, round it, off it into the
         * right edge. */
        const double angle = flare == 0u ? join : 2.0 * PI - join;
        const V3 onCircle = Along(centre, ex, cos(angle) * radius, ey, sin(angle) * radius);
        const V3 tangent = Add(Scale(ex, -sin(angle)), Scale(ey, cos(angle)));
        if (flare == 1u)
        {
            for (uint32_t step = 0u; step <= BULB_SEGMENTS; ++step)
            {
                const double a = join + (2.0 * PI - 2.0 * join) * (double)step / BULB_SEGMENTS;
                Put(&outline, Along(centre, ex, cos(a) * radius, ey, sin(a) * radius), up, true);
            }
        }
        for (uint32_t step = 1u; step < FLARE_SEGMENTS; ++step)
        {
            const double t = (double)step / FLARE_SEGMENTS;
            Put(&outline,
                flare == 0u ? CubicPoint(LeftCorner(end), inward, onCircle, tangent, t)
                            : CubicPoint(onCircle, tangent, RightCorner(end), ex, t),
                up, true);
        }
    }
    PaveOutline(&outline, centre, up, base, detail);
}

void FoxRoadMeshNode(const FoxNetwork *network, uint32_t nodeIndex, FoxVec origin, FoxMesh *base,
                     FoxMesh *detail)
{
    FoxNodeView node;
    if (!FoxNetworkNode(network, nodeIndex, &node) || node.roadCount == 0u)
    {
        return;
    }
    const V3 centre = Local(node.position, origin);
    const V3 up = UpOf(node.position);
    RoadEndShape ends[FOX_NODE_MAX_ROADS];
    uint32_t count = 0u;
    for (uint32_t slot = 0u; slot < node.roadCount; ++slot)
    {
        if (EndShape(network, node.roads[slot], nodeIndex, origin, &ends[count]))
        {
            ++count;
        }
    }
    if (count == 0u)
    {
        return;
    }
    /* Round the node anticlockwise seen from above. */
    const V3 ex = ends[0].outward;
    const V3 ey = Unit(Cross(up, ex));
    uint32_t order[FOX_NODE_MAX_ROADS];
    for (uint32_t index = 0u; index < count; ++index)
    {
        ends[index].angle = atan2(Dot(ends[index].outward, ey), Dot(ends[index].outward, ex));
        uint32_t position = index;
        while (position > 0u && ends[order[position - 1u]].angle > ends[index].angle)
        {
            order[position] = order[position - 1u];
            --position;
        }
        order[position] = index;
    }
    if (count == 1u)
    {
        Bulb(&ends[0], centre, up, base, detail);
    }
    else if (count == 2u && RibbonFits(&ends[0], &ends[1], up))
    {
        Bend(&ends[0], &ends[1], up, base, detail);
    }
    else
    {
        Junction(ends, order, count, centre, up, base, detail);
        if (count == 2u && detail != NULL)
        {
            BendCentreLine(&ends[0], &ends[1], up, detail);
        }
    }
}

void FoxRoadMeshLine(const FoxVec *points, uint32_t count, double halfWidth, bool dashed,
                     uint32_t color, FoxVec origin, FoxMesh *mesh)
{
    if (points == NULL || count < 2u)
    {
        return;
    }
    Station *stations = malloc(((size_t)count + 2u) * sizeof(*stations));
    if (stations == NULL)
    {
        mesh->failed = true;
        return;
    }
    double length = 0.0;
    for (uint32_t index = 1u; index < count; ++index)
    {
        length += Length(Sub(Local(points[index], origin), Local(points[index - 1u], origin)));
    }
    const uint32_t written = BuildStations(points, count, origin, 0.0, length, stations);
    Strip(mesh, stations, written, -halfWidth, halfWidth, OVERLAY_LIFT, 0.0, length,
          dashed ? 2.0 : 0.0, dashed ? 1.5 : 0.0, color);
    free(stations);
}

void FoxRoadMeshRing(FoxVec centre, double radius, double width, uint32_t color, FoxVec origin,
                     FoxMesh *mesh)
{
    const V3 middle = Local(centre, origin);
    const V3 up = UpOf(centre);
    const V3 axis = fabs(up.z) < 0.9 ? (V3){0.0, 0.0, 1.0} : (V3){1.0, 0.0, 0.0};
    const V3 ex = Unit(Cross(up, axis));
    const V3 ey = Cross(up, ex);
    const V3 lift = Scale(up, OVERLAY_LIFT);
    for (uint32_t step = 0u; step < RING_SEGMENTS; ++step)
    {
        const double a = 2.0 * PI * (double)step / RING_SEGMENTS;
        const double b = 2.0 * PI * (double)(step + 1u) / RING_SEGMENTS;
        const double inner = radius - width / 2.0;
        const double outer = radius + width / 2.0;
        Quad(mesh, Add(Along(middle, ex, cos(a) * inner, ey, sin(a) * inner), lift),
             Add(Along(middle, ex, cos(a) * outer, ey, sin(a) * outer), lift),
             Add(Along(middle, ex, cos(b) * outer, ey, sin(b) * outer), lift),
             Add(Along(middle, ex, cos(b) * inner, ey, sin(b) * inner), lift), color);
    }
}

void FoxRoadMeshPreview(const FoxVec *centre, uint32_t count, FoxRoadType type, uint32_t color,
                        FoxVec origin, FoxMesh *mesh)
{
    if (centre == NULL || count < 2u || FoxRoadTypeGet(type) == NULL)
    {
        return;
    }
    Station *stations = malloc(((size_t)count + 2u) * sizeof(*stations));
    if (stations == NULL)
    {
        mesh->failed = true;
        return;
    }
    double length = 0.0;
    for (uint32_t index = 1u; index < count; ++index)
    {
        length += Length(Sub(Local(centre[index], origin), Local(centre[index - 1u], origin)));
    }
    const uint32_t written = BuildStations(centre, count, origin, 0.0, length, stations);
    const double paved = FoxRoadMeshHalfWidth(type) + SHOULDER;
    const double lift = MARKING_LIFT + 0.05;
    Strip(mesh, stations, written, -paved, paved, lift, 0.0, length, 0.0, 0.0, color);
    for (uint32_t end = 0u; end < 2u; ++end)
    {
        const Station *station = &stations[end == 0u ? 0u : written - 1u];
        const V3 tangent = Unit(Cross(station->up, station->right));
        const V3 outward = end == 0u ? tangent : Scale(tangent, -1.0);
        const V3 right = end == 0u ? station->right : Scale(station->right, -1.0);
        for (uint32_t step = 0u; step < CAP_SEGMENTS; ++step)
        {
            const double a = PI * (double)step / CAP_SEGMENTS;
            const double b = PI * (double)(step + 1u) / CAP_SEGMENTS;
            Triangle(mesh, Offset(station, 0.0, lift),
                     Add(Along(station->point, right, cos(a) * paved, outward, -sin(a) * paved),
                         Scale(station->up, lift)),
                     Add(Along(station->point, right, cos(b) * paved, outward, -sin(b) * paved),
                         Scale(station->up, lift)),
                     color);
        }
    }
    free(stations);
}
