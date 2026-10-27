#include "client/road_mesh.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* Heights above the ground: the asphalt, then everything painted on it. */
#define ASPHALT_LIFT 0.15
#define MARKING_LIFT 0.19
/* Concrete lanes lie over the asphalt; footways stand a kerb above it. */
#define CONCRETE_LIFT 0.17
#define FOOTWAY_LIFT 0.29
/* What the tools show lies over the markings. */
#define OVERLAY_LIFT 0.36
/* Concrete is laid in slabs this long. */
#define SLAB 5.0
#define JOINT 0.08
/* A zebra: its depth along the road, and its bars and the gaps between. */
#define ZEBRA_DEPTH 3.0
#define ZEBRA_BAR 0.5
#define ZEBRA_GAP 0.5
#define RING_SEGMENTS 32u
/* Paved beyond the outer lanes. */
#define SHOULDER ((double)FOX_ROAD_SHOULDER / FOX_UNITS_PER_METRE)
#define LINE_HALF 0.08
#define EDGE_HALF 0.07
#define DOUBLE_LINE_SPACING 0.14
#define DASH 3.0
#define DASH_GAP 6.0
/* Guide lines through an intersection: short dashes. */
#define GUIDE_DASH 0.5
#define GUIDE_GAP 0.5
/* Lane lines turn solid this far before an intersection. */
#define SOLID_NEAR 15.0
/* Lines across the lanes at a junction stand this far back from it: a stop
 * line this wide, the teeth of a give-way line reaching this far. */
#define LINE_BACK 0.3
#define STOP_WIDTH 0.5
#define TEETH_BACK 0.9
#define TOOTH_BASE 0.45
#define TOOTH_GAP 0.25
#define FILLET_SEGMENTS 8u
#define CAP_SEGMENTS 12u
/* Signs stand this far beyond the paved edge. */
#define POST_CLEARANCE 0.6
/* Roads shorter than this between their trims get no limit sign. */
#define SPEED_SIGN_MIN_LENGTH 30.0
#define PI 3.14159265358979323846

#define COLOR_ASPHALT UINT32_C(0xFF3C3A38)
#define COLOR_CONCRETE UINT32_C(0xFF8E9294)
#define COLOR_JOINT UINT32_C(0xFF5E6264)
#define COLOR_BIKE UINT32_C(0xFF3E4C9A)
#define COLOR_FOOTWAY UINT32_C(0xFFAEB2B4)
#define COLOR_KERB UINT32_C(0xFF8A8E90)
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
        LaiueGraphicsVertexV2 *grown = realloc(mesh->vertices, (size_t)capacity * sizeof(*grown));
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
            const double t =
                length > 0.0 ? fmin(fmax((along - travelled) / length, 0.0), 1.0) : 0.0;
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
 * dashed when `dash` is non-zero. The dashes are stretched a little to fit
 * a whole number into the stretch, which begins and ends with half a gap:
 * where two pieces of a line meet, the pattern runs on unbroken. A stretch
 * too short for one dash is all gap. */
static void Strip(FoxMesh *mesh, const Station *stations, uint32_t count, double left, double right,
                  double lift, double from, double to, double dash, double gap, uint32_t color)
{
    if (to <= from)
    {
        return;
    }
    if (dash > 0.0)
    {
        const double period = dash + gap;
        const double dashes = round((to - from) / period);
        if (dashes < 1.0)
        {
            return;
        }
        const double fitted = (to - from) / dashes;
        dash *= fitted / period;
        gap *= fitted / period;
    }
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
                /* The dash at or after the piece. */
                const double period = dash + gap;
                const double first = from + gap / 2.0;
                double start = first + floor((piece - first) / period) * period;
                if (piece >= start + dash)
                {
                    start += period;
                }
                if (start > piece)
                {
                    piece = start;
                    continue;
                }
                end = fmin(end, start + dash);
            }
            const Station p0 = Blend(a, b, (piece - a->along) / span);
            const Station p1 = Blend(a, b, (end - a->along) / span);
            Quad(mesh, Offset(&p0, left, lift), Offset(&p0, right, lift), Offset(&p1, right, lift),
                 Offset(&p1, left, lift), color);
            piece = end;
        }
    }
}

/* A painted line at an offset, solid except for a stretch of dashes. */
static void LaneLine(FoxMesh *mesh, const Station *stations, uint32_t count, double at, double from,
                     double to, double dashFrom, double dashTo, bool dashed, double dash,
                     double gap)
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
          dash, gap, COLOR_LINE);
    Strip(mesh, stations, count, at - LINE_HALF, at + LINE_HALF, MARKING_LIFT, dashTo, to, 0.0, 0.0,
          COLOR_LINE);
}

static double Metres(int64_t units)
{
    return (double)units / FOX_UNITS_PER_METRE;
}

static double LaneWidth(void)
{
    return Metres(FOX_LANE_WIDTH);
}

/* Half the width of the car lanes, metres; zero without any. */
static double CarHalf(const FoxRoadLayout *layout)
{
    return Metres(FoxRoadLayoutCarHalfWidth(layout));
}

/* Half the width a road meets its nodes with, metres: its car lanes and
 * their shoulders, or the strips of a path. */
static double PavedHalf(const FoxRoadLayout *layout)
{
    if (FoxRoadLayoutCarLanes(layout) != 0u)
    {
        return CarHalf(layout) + SHOULDER;
    }
    int64_t extent[2];
    FoxRoadLayoutExtent(layout, extent);
    return Metres(FoxMax(extent[0], extent[1]));
}

double FoxRoadMeshReach(const FoxRoadLayout *layout)
{
    int64_t extent[2];
    FoxRoadLayoutExtent(layout, extent);
    return Metres(FoxMax(extent[0], extent[1])) +
           (FoxRoadLayoutCarLanes(layout) != 0u ? SHOULDER : 0.0);
}

/* The width of what runs along one edge beyond the shoulder, metres. */
static double EdgeWidth(const FoxRoadLayout *layout, uint32_t side)
{
    return (layout->bike[side] != FOX_WAY_NONE ? Metres(FOX_LANE_WIDTH) : 0.0) +
           (layout->foot[side] ? Metres(FOX_FOOT_WIDTH) : 0.0);
}

/* A strip's edges in metres right of the centre line, as drawn: beside car
 * lanes the edge strips stand beyond the shoulder. */
static void StripEdges(const FoxRoadLayout *layout, const FoxRoadStrip *strip, double *outLeft,
                       double *outRight)
{
    double shift = 0.0;
    if (FoxRoadLayoutCarLanes(layout) != 0u &&
        (strip->kind == FOX_LANE_BIKE || strip->kind == FOX_LANE_FOOT))
    {
        shift = strip->left < 0 ? -SHOULDER : SHOULDER;
    }
    *outLeft = Metres(strip->left) + shift;
    *outRight = Metres(strip->right) + shift;
}

/* A kerb's face from the asphalt up to a footway, along an offset. */
static void KerbFace(FoxMesh *mesh, const Station *stations, uint32_t count, double at, double from,
                     double to)
{
    for (uint32_t index = 0u; index + 1u < count; ++index)
    {
        const Station *a = &stations[index];
        const Station *b = &stations[index + 1u];
        const double low = fmax(from, a->along);
        const double high = fmin(to, b->along);
        const double span = b->along - a->along;
        if (span <= 0.0 || high <= low)
        {
            continue;
        }
        const Station p0 = Blend(a, b, (low - a->along) / span);
        const Station p1 = Blend(a, b, (high - a->along) / span);
        Quad(mesh, Offset(&p0, at, ASPHALT_LIFT), Offset(&p1, at, ASPHALT_LIFT),
             Offset(&p1, at, FOOTWAY_LIFT), Offset(&p0, at, FOOTWAY_LIFT), COLOR_KERB);
    }
}

/* The surfaces of a road's strips over [from, to] of a centre line: the car
 * lanes' asphalt with concrete laid over in slabs, bike lanes, and footways
 * raised on a kerb. */
static void Surfaces(FoxMesh *base, FoxMesh *detail, const Station *stations, uint32_t count,
                     const FoxRoadLayout *layout, FoxDriveSide side, double from, double to)
{
    FoxRoadStrip strips[FOX_ROAD_MAX_STRIPS];
    const uint32_t stripCount = FoxRoadLayoutStrips(layout, side, strips);
    if (FoxRoadLayoutCarLanes(layout) != 0u)
    {
        const double half = CarHalf(layout) + SHOULDER;
        Strip(base, stations, count, -half, half, ASPHALT_LIFT, from, to, 0.0, 0.0, COLOR_ASPHALT);
    }
    for (uint32_t index = 0u; index < stripCount; ++index)
    {
        const FoxRoadStrip *strip = &strips[index];
        double left = 0.0;
        double right = 0.0;
        StripEdges(layout, strip, &left, &right);
        switch (strip->kind)
        {
        case FOX_LANE_CONCRETE:
            Strip(base, stations, count, left, right, CONCRETE_LIFT, from, to, 0.0, 0.0,
                  COLOR_CONCRETE);
            if (detail != NULL)
            {
                Strip(detail, stations, count, left, right, CONCRETE_LIFT + 0.01, from, to, JOINT,
                      SLAB - JOINT, COLOR_JOINT);
            }
            break;
        case FOX_LANE_BIKE:
            Strip(base, stations, count, left, right, ASPHALT_LIFT, from, to, 0.0, 0.0, COLOR_BIKE);
            break;
        case FOX_LANE_FOOT:
            Strip(base, stations, count, left, right, FOOTWAY_LIFT, from, to, 0.0, 0.0,
                  COLOR_FOOTWAY);
            /* The kerb faces the side nearer the road. */
            KerbFace(base, stations, count, strip->left < 0 ? right : left, from, to);
            break;
        default:
            break;
        }
    }
}

/* The lane markings of a layout over [from, to] of a centre line: edge lines
 * along the car lanes there, and over [linesFrom, linesTo] the line between
 * the two ways, single and dashed between two single lanes, double
 * elsewhere, and lane lines dashed in [dashFrom, dashTo]. */
static void Markings(FoxMesh *mesh, const Station *stations, uint32_t count,
                     const FoxRoadLayout *layout, FoxDriveSide side, double from, double to,
                     double linesFrom, double linesTo, double dashFrom, double dashTo)
{
    if (FoxRoadLayoutCarLanes(layout) == 0u)
    {
        return;
    }
    const double half = CarHalf(layout);
    for (int edgeSide = -1; edgeSide <= 1; edgeSide += 2)
    {
        const double edge = (double)edgeSide * (half - EDGE_HALF);
        Strip(mesh, stations, count, edge - EDGE_HALF, edge + EDGE_HALF, MARKING_LIFT, from, to,
              0.0, 0.0, COLOR_LINE);
    }
    if (layout->shared)
    {
        return;
    }
    FoxRoadStrip strips[FOX_ROAD_MAX_STRIPS];
    const uint32_t stripCount = FoxRoadLayoutStrips(layout, side, strips);
    const bool street = layout->lanes[0] == 1u && layout->lanes[1] == 1u;
    for (uint32_t index = 0u; index + 1u < stripCount; ++index)
    {
        const FoxRoadStrip *a = &strips[index];
        const FoxRoadStrip *b = &strips[index + 1u];
        if (a->kind > FOX_LANE_CONCRETE || b->kind > FOX_LANE_CONCRETE)
        {
            continue;
        }
        const double at = Metres(a->right);
        /* Lanes of one way are parted by a dashed line, and so are the two
         * ways of a street, to overtake across. */
        if (a->way == b->way || street)
        {
            LaneLine(mesh, stations, count, at, linesFrom, linesTo, dashFrom, dashTo, true, DASH,
                     DASH_GAP);
        }
        else
        {
            /* Bigger roads keep their directions apart with a double line. */
            LaneLine(mesh, stations, count, at - DOUBLE_LINE_SPACING, linesFrom, linesTo, 0.0, 0.0,
                     false, DASH, DASH_GAP);
            LaneLine(mesh, stations, count, at + DOUBLE_LINE_SPACING, linesFrom, linesTo, 0.0, 0.0,
                     false, DASH, DASH_GAP);
        }
    }
}

/* The bars of a zebra across the car lanes, centred `at` along the road. */
static void Zebra(FoxMesh *mesh, const Station *stations, uint32_t count,
                  const FoxRoadLayout *layout, double at)
{
    const double half = CarHalf(layout);
    const double room = 2.0 * half - 2.0 * EDGE_HALF;
    const uint32_t bars = (uint32_t)floor((room + ZEBRA_GAP) / (ZEBRA_BAR + ZEBRA_GAP));
    if (bars == 0u)
    {
        return;
    }
    const double used = (double)bars * ZEBRA_BAR + (double)(bars - 1u) * ZEBRA_GAP;
    double x = -used / 2.0;
    for (uint32_t bar = 0u; bar < bars; ++bar)
    {
        Strip(mesh, stations, count, x, x + ZEBRA_BAR, MARKING_LIFT, at - ZEBRA_DEPTH / 2.0,
              at + ZEBRA_DEPTH / 2.0, 0.0, 0.0, COLOR_LINE);
        x += ZEBRA_BAR + ZEBRA_GAP;
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

/* A painted band of half width `half` along points of a lane frame (ahead,
 * right), its edges mitred where it bends, ending in a head this wide and
 * long pointing on the way the band ends. */
#define ARROW_POINTS 12u
static void ArrowBand(FoxMesh *mesh, const LaneFrame *frame, const double (*points)[2],
                      uint32_t count, double half, double headHalf, double headLength)
{
    double normals[ARROW_POINTS][2];
    for (uint32_t index = 0u; index < count; ++index)
    {
        /* The normals of the segments either side, averaged into a mitre
         * that keeps the band as wide through the bend. */
        double sumX = 0.0;
        double sumY = 0.0;
        double segmentX = 0.0;
        double segmentY = 0.0;
        for (uint32_t side = 0u; side < 2u; ++side)
        {
            if ((side == 0u && index == 0u) || (side == 1u && index + 1u == count))
            {
                continue;
            }
            const double *from = points[side == 0u ? index - 1u : index];
            const double *to = points[side == 0u ? index : index + 1u];
            const double dx = to[0] - from[0];
            const double dy = to[1] - from[1];
            const double length = sqrt(dx * dx + dy * dy);
            segmentX = -dy / length;
            segmentY = dx / length;
            sumX += segmentX;
            sumY += segmentY;
        }
        const double length = sqrt(sumX * sumX + sumY * sumY);
        const double mitre = fmax((sumX * segmentX + sumY * segmentY) / length, 0.5);
        normals[index][0] = sumX / length / mitre;
        normals[index][1] = sumY / length / mitre;
    }
    for (uint32_t index = 0u; index + 1u < count; ++index)
    {
        const double *a = points[index];
        const double *b = points[index + 1u];
        const double *na = normals[index];
        const double *nb = normals[index + 1u];
        Quad(mesh, InLane(frame, a[0] - na[0] * half, a[1] - na[1] * half),
             InLane(frame, a[0] + na[0] * half, a[1] + na[1] * half),
             InLane(frame, b[0] + nb[0] * half, b[1] + nb[1] * half),
             InLane(frame, b[0] - nb[0] * half, b[1] - nb[1] * half), COLOR_LINE);
    }
    const double *end = points[count - 1u];
    const double *before = points[count - 2u];
    const double dx = end[0] - before[0];
    const double dy = end[1] - before[1];
    const double length = sqrt(dx * dx + dy * dy);
    const double ux = dx / length;
    const double uy = dy / length;
    Triangle(mesh, InLane(frame, end[0] + uy * headHalf, end[1] - ux * headHalf),
             InLane(frame, end[0] - uy * headHalf, end[1] + ux * headHalf),
             InLane(frame, end[0] + ux * headLength, end[1] + uy * headLength), COLOR_LINE);
}

/* An arrow for every way a lane leads, painted at its frame: a stem with
 * its head for straight on, and for a turn a branch that bends smoothly off
 * the stem, or the stem itself bending when the lane only turns. */
static void Arrow(FoxMesh *mesh, const LaneFrame *frame, uint32_t turns)
{
    const double half = 0.08;
    const double headHalf = 0.3;
    const double headLength = 0.8;
    const double bottom = -2.2;
    const bool straight = (turns & FOX_TURN_STRAIGHT) != 0u;
    if (straight)
    {
        const double stem[2][2] = {{bottom, 0.0}, {0.9, 0.0}};
        ArrowBand(mesh, frame, stem, 2u, half, headHalf, headLength);
    }
    for (int side = -1; side <= 1; side += 2)
    {
        if ((turns & (side < 0 ? FOX_TURN_LEFT : FOX_TURN_RIGHT)) == 0u)
        {
            continue;
        }
        /* Up the stem, round a bend of 60 degrees and a little on. */
        const double radius = 0.7;
        const double from = -0.9;
        const double turn = PI / 3.0;
        double points[ARROW_POINTS][2];
        uint32_t count = 0u;
        if (!straight)
        {
            points[count][0] = bottom;
            points[count++][1] = 0.0;
        }
        const uint32_t arc = 8u;
        for (uint32_t step = 0u; step <= arc; ++step)
        {
            const double angle = turn * (double)step / (double)arc;
            points[count][0] = from + radius * sin(angle);
            points[count++][1] = (double)side * radius * (1.0 - cos(angle));
        }
        points[count][0] = points[count - 1u][0] + 0.15 * cos(turn);
        points[count][1] = points[count - 1u][1] + (double)side * 0.15 * sin(turn);
        ++count;
        ArrowBand(mesh, frame, points, count, half, headHalf, headLength * 0.85);
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
        {-w, w, h - t, h},          {w - t, w, 0.0, h},    {w - t, w, -h, 0.0},
        {-w, w, -h, -h + t},        {-w, -w + t, -h, 0.0}, {-w, -w + t, 0.0, h},
        {-w, w, -t / 2.0, t / 2.0},
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
 * signal changes, and beside it on the side of the road a section with an
 * arrow for turning across the oncoming lanes. */
static void TrafficLight(FoxMesh *mesh, FoxSignalHeads *heads, const Post *post, FoxVec origin,
                         FoxDriveSide side, uint32_t road, uint32_t end)
{
    const double height = 3.3;
    Pole(mesh, post, height);
    const double depth = 0.13;
    const V3 housing = Along(post->base, post->up, 2.75, post->forward, -0.2);
    Box(mesh, housing, post->forward, post->right, post->up, depth, 0.17, 0.5, COLOR_HOUSING);
    FoxSignalHead head = {.road = road, .end = end};
    for (uint32_t lamp = 0u; lamp < 3u; ++lamp)
    {
        const V3 face =
            Along(housing, post->forward, -depth - 0.005, post->up, 0.3 - 0.3 * (double)lamp);
        Rectangle(mesh, face, post->right, post->up, -0.1, 0.1, -0.1, 0.1, COLOR_LAMP_OFF);
        head.lamps[lamp] = World(Add(face, Scale(post->forward, -0.04)), origin);
    }
    /* Towards the middle of the road, where the turn across it goes. */
    const double inward = side == FOX_DRIVE_RIGHT ? -1.0 : 1.0;
    const V3 section = Along(housing, post->right, inward * 0.32, post->up, -0.3);
    Box(mesh, section, post->forward, post->right, post->up, depth, 0.15, 0.15, COLOR_HOUSING);
    const V3 face = Along(section, post->forward, -depth - 0.005, post->up, 0.0);
    Rectangle(mesh, face, post->right, post->up, -0.1, 0.1, -0.1, 0.1, COLOR_LAMP_OFF);
    head.arrow = World(Add(face, Scale(post->forward, -0.04)), origin);
    /* The arrow itself, dark in front of the lamp: a shaft and a head. */
    const V3 plate = Add(face, Scale(post->forward, -0.16));
    Rectangle(mesh, plate, post->right, post->up, inward > 0.0 ? -0.07 : -0.01,
              inward > 0.0 ? 0.01 : 0.07, -0.02, 0.02, COLOR_BLACK);
    Polygon(mesh, Along(plate, post->right, inward * 0.03, post->up, 0.0), post->right, post->up,
            3u, 0.055, inward > 0.0 ? 0.0 : PI, COLOR_BLACK);
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

/* Roads with car lanes at a node: an intersection for cars has three. */
static uint32_t RoadsAt(const FoxNetwork *network, uint32_t node)
{
    FoxNodeView view;
    return FoxNetworkNode(network, node, &view) ? view.carRoadCount : 0u;
}

/* What is painted across the lanes arriving at a road's end. */
typedef enum EndMarking
{
    MARK_NONE = 0,
    MARK_STOP,
    MARK_TEETH,
} EndMarking;

static EndMarking EndMark(const FoxNetwork *network, const FoxRoadView *road, uint32_t end)
{
    FoxNodeView node;
    if (!FoxNetworkNode(network, road->nodes[end], &node) || node.carRoadCount < 3u)
    {
        return MARK_NONE;
    }
    if (node.signals || road->sign[end] == FOX_SIGN_STOP)
    {
        return MARK_STOP;
    }
    return road->sign[end] == FOX_SIGN_YIELD ? MARK_TEETH : MARK_NONE;
}

/* How far before a road's end the lines along it stop: at the line across
 * the lanes arriving there. */
static double LinesBack(EndMarking mark)
{
    return mark == MARK_STOP ? LINE_BACK + STOP_WIDTH : (mark == MARK_TEETH ? TEETH_BACK : 0.0);
}

/* Where the lanes arriving at a road's end lie across it, from the centre
 * line to the edge line: over the outer sides of both lines when `over`,
 * else between their inner sides. False when no lanes arrive there. */
static bool ArrivingSpan(const FoxRoadView *road, uint32_t end, bool over, double *outLeft,
                         double *outRight)
{
    /* Lanes of direction 0 arrive at nodes[1]. */
    const uint32_t direction = end == 1u ? 0u : 1u;
    if (road->laneCount[direction] == 0u)
    {
        return false;
    }
    FoxRoadStrip strips[FOX_ROAD_MAX_STRIPS];
    const uint32_t count = FoxRoadLayoutStrips(&road->layout, road->driveSide, strips);
    const uint8_t way = direction == 0u ? FOX_WAY_FORWARD : FOX_WAY_BACKWARD;
    double left = 0.0;
    double right = 0.0;
    bool any = false;
    for (uint32_t index = 0u; index < count; ++index)
    {
        if (strips[index].kind <= FOX_LANE_CONCRETE &&
            (strips[index].way == way || strips[index].way == FOX_WAY_BOTH))
        {
            left = any ? fmin(left, Metres(strips[index].left)) : Metres(strips[index].left);
            right = any ? fmax(right, Metres(strips[index].right)) : Metres(strips[index].right);
            any = true;
        }
    }
    if (!any)
    {
        return false;
    }
    /* Between the inner sides of the lines along them, or over them. */
    const double half = CarHalf(&road->layout);
    const double edge = over ? 0.0 : 2.0 * EDGE_HALF;
    const bool street = road->layout.lanes[0] == 1u && road->layout.lanes[1] == 1u;
    const double centre = (street ? 0.0 : DOUBLE_LINE_SPACING) + LINE_HALF;
    *outLeft = left <= -half + 1e-6 ? -half + edge : left + (over ? -centre : centre);
    *outRight = right >= half - 1e-6 ? half - edge : right - (over ? -centre : centre);
    return true;
}

/* The stop line across every lane arriving at a road's end, a little
 * before the junction. It reaches over the centre line and the edge line to
 * their outer sides, so the lines along the road run into it and end there
 * in one clean joint. */
static void StopLine(FoxMesh *mesh, const Station *stations, uint32_t count,
                     const FoxRoadView *road, uint32_t end, double at)
{
    double left = 0.0;
    double right = 0.0;
    if (!ArrivingSpan(road, end, true, &left, &right))
    {
        return;
    }
    const double near = end == 1u ? at - LINE_BACK - STOP_WIDTH : at + LINE_BACK;
    Strip(mesh, stations, count, left, right, MARKING_LIFT, near, near + STOP_WIDTH, 0.0, 0.0,
          COLOR_LINE);
}

/* A point `along` a road's centre line and `right` of it, on the markings. */
static V3 MarkingPoint(const Station *stations, uint32_t count, double along, double right)
{
    uint32_t index = 0u;
    while (index + 2u < count && stations[index + 1u].along < along)
    {
        ++index;
    }
    const Station *a = &stations[index];
    const Station *b = &stations[index + 1u];
    const double span = b->along - a->along;
    const Station station = Blend(a, b, span > 0.0 ? (along - a->along) / span : 0.0);
    return Offset(&station, right, MARKING_LIFT);
}

/* The give-way line: a row of equal teeth pointing at the arriving traffic,
 * centred between the centre line and the edge line with as much room at
 * either side as between two teeth. */
static void GiveWayTeeth(FoxMesh *mesh, const Station *stations, uint32_t count,
                         const FoxRoadView *road, uint32_t end, double at)
{
    double left = 0.0;
    double right = 0.0;
    if (count < 2u || !ArrivingSpan(road, end, false, &left, &right))
    {
        return;
    }
    const double span = right - left;
    const uint32_t teeth = (uint32_t)floor(fmax(0.0, span / (TOOTH_BASE + TOOTH_GAP)));
    if (teeth == 0u)
    {
        return;
    }
    const double room = (span - (double)teeth * TOOTH_BASE) / (double)teeth;
    /* Towards the junction the base, away from it the point. */
    const double base = end == 1u ? at - LINE_BACK : at + LINE_BACK;
    const double point = end == 1u ? at - TEETH_BACK : at + TEETH_BACK;
    for (uint32_t tooth = 0u; tooth < teeth; ++tooth)
    {
        const double low = left + room / 2.0 + (double)tooth * (TOOTH_BASE + room);
        const double high = low + TOOTH_BASE;
        const V3 tip = MarkingPoint(stations, count, point, (low + high) / 2.0);
        const V3 a = MarkingPoint(stations, count, base, low);
        const V3 b = MarkingPoint(stations, count, base, high);
        if (end == 1u)
        {
            Triangle(mesh, tip, b, a, COLOR_LINE);
        }
        else
        {
            Triangle(mesh, a, b, tip, COLOR_LINE);
        }
    }
}

/* Lines, arrows and the sign or light where a road's lanes meet a node. */
static void RoadEnd(const FoxNetwork *network, uint32_t roadIndex, const FoxRoadView *road,
                    uint32_t end, FoxVec origin, FoxMesh *detail, FoxSignalHeads *heads)
{
    FoxNodeView node;
    if (!FoxNetworkNode(network, road->nodes[end], &node) || node.carRoadCount < 3u)
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
            /* A light stands level with the stop line, a sign a little
             * before the junction. */
            const Post post = CurbPost(&frame, road->driveSide,
                                       node.signals ? LINE_BACK + STOP_WIDTH / 2.0 : 1.2);
            if (node.signals)
            {
                TrafficLight(detail, heads, &post, origin, road->driveSide, roadIndex, end);
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
    Surfaces(base, detail, stations, count, &road.layout, road.driveSide, from, to);
    const double dashFrom = from + (RoadsAt(network, road.nodes[0]) >= 3u ? SOLID_NEAR : 0.0);
    const double dashTo = to - (RoadsAt(network, road.nodes[1]) >= 3u ? SOLID_NEAR : 0.0);
    const EndMarking marks[2] = {EndMark(network, &road, 0u), EndMark(network, &road, 1u)};
    Markings(detail, stations, count, &road.layout, road.driveSide, from, to,
             from + LinesBack(marks[0]), to - LinesBack(marks[1]), dashFrom, dashTo);
    for (uint32_t crossing = 0u; crossing < road.crossingCount; ++crossing)
    {
        const double at = Metres(road.crossings[crossing]);
        if (at > from + ZEBRA_DEPTH / 2.0 && at < to - ZEBRA_DEPTH / 2.0)
        {
            Zebra(detail, stations, count, &road.layout, at);
        }
    }
    for (uint32_t end = 0u; end < 2u; ++end)
    {
        if (marks[end] == MARK_STOP)
        {
            StopLine(detail, stations, count, &road, end, end == 0u ? from : to);
        }
        else if (marks[end] == MARK_TEETH)
        {
            GiveWayTeeth(detail, stations, count, &road, end, end == 0u ? from : to);
        }
    }
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
#define FLARE_SEGMENTS 12u
#define OUTLINE_CAPACITY (FOX_NODE_MAX_ROADS * (SWEEP_SEGMENTS + 2u) + CAP_SEGMENTS + 16u)

/* A road where it meets a node: its station at the trim, the direction away
 * from the node, the right of that direction and its paved half width. */
typedef struct RoadEndShape
{
    Station station;
    V3 outward;
    V3 right;
    double paved;
    double angle;
    FoxRoadLayout layout;
    /* What runs beyond the paved edge on the left and the right seen going
     * out of the node, metres, and whether it has a footway. */
    double edge[2];
    bool foot[2];
    /* Which end of its road meets the node. */
    uint32_t end;
    FoxDriveSide side;
    /* The sign in force for the traffic arriving at this end. */
    FoxSign sign;
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
    out->paved = PavedHalf(&road.layout);
    out->layout = road.layout;
    /* Going out from nodes[0] its left is the road's side 0. */
    const uint32_t left = end == 0u ? 0u : 1u;
    const bool cars = FoxRoadLayoutCarLanes(&road.layout) != 0u;
    out->edge[0] = cars ? EdgeWidth(&road.layout, left) : 0.0;
    out->edge[1] = cars ? EdgeWidth(&road.layout, 1u - left) : 0.0;
    out->foot[0] = cars && road.layout.foot[left];
    out->foot[1] = cars && road.layout.foot[1u - left];
    out->end = end;
    out->side = road.driveSide;
    out->sign = road.sign[end];
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
 * the up of their road, so the asphalt meets the road's without a seam, and
 * the way into the road, so the line along the curb meets the road's edge
 * line square. */
typedef struct Outline
{
    V3 points[OUTLINE_CAPACITY];
    V3 lift[OUTLINE_CAPACITY];
    /* Into the paving across the road at a mouth's corner; zero elsewhere. */
    V3 across[OUTLINE_CAPACITY];
    /* Whether the edge from a point to the next one is a curb. */
    bool curb[OUTLINE_CAPACITY];
    uint32_t count;
} Outline;

static void PutCorner(Outline *outline, V3 point, V3 up, V3 across, bool curb)
{
    if (outline->count < OUTLINE_CAPACITY)
    {
        outline->points[outline->count] = point;
        outline->lift[outline->count] = up;
        outline->across[outline->count] = across;
        outline->curb[outline->count++] = curb;
    }
}

static void Put(Outline *outline, V3 point, V3 up, bool curb)
{
    PutCorner(outline, point, up, (V3){0.0, 0.0, 0.0}, curb);
}

/* The mouth of a road: its right corner, then its left, from where the curb
 * goes on. */
static void Mouth(Outline *outline, const RoadEndShape *end)
{
    PutCorner(outline, RightCorner(end), end->station.up, Scale(end->right, -1.0), false);
    PutCorner(outline, LeftCorner(end), end->station.up, end->right, true);
}

/* The curb between the left edge of one road and the right edge of the next
 * one round: a curve that leaves both edges smoothly where they would meet,
 * a straight line where they never do. */
static uint32_t Fillet(V3 from, V3 fromDirection, V3 to, V3 toDirection, V3 ex, V3 ey, double reach,
                       V3 *out)
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

/* Paves an outline as a fan from the node, in `color` at `lift`, and paints
 * a line along its curbs, just inside them as along the roads' edges, where
 * cars drive. */
static void PaveOutline(const Outline *outline, V3 centre, V3 up, uint32_t color, double lift,
                        FoxMesh *base, FoxMesh *detail)
{
    const uint32_t count = outline->count;
    if (count < 3u)
    {
        return;
    }
    const V3 middle = Lift(centre, up, lift);
    for (uint32_t index = 0u; index < count; ++index)
    {
        const uint32_t next = (index + 1u) % count;
        Triangle(base, middle, Lift(outline->points[index], outline->lift[index], lift),
                 Lift(outline->points[next], outline->lift[next], lift), color);
    }
    if (detail == NULL || color != COLOR_ASPHALT)
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
             * normals are averaged; where a curb meets a mouth it goes in
             * square to the road, as the road's own edge line does. */
            const uint32_t at = side == 0u ? index : next;
            if (Length(outline->across[at]) > 0.0)
            {
                inward[side] = outline->across[at];
                continue;
            }
            const uint32_t before = (at + count - 1u) % count;
            const uint32_t after = (at + 1u) % count;
            const V3 own = side == 0u
                               ? Unit(Cross(up, Sub(outline->points[after], outline->points[at])))
                               : Unit(Cross(up, Sub(outline->points[at], outline->points[before])));
            V3 normal = own;
            if (outline->curb[before] && outline->curb[at])
            {
                const V3 previous =
                    Unit(Cross(up, Sub(outline->points[at], outline->points[before])));
                const V3 following =
                    Unit(Cross(up, Sub(outline->points[after], outline->points[at])));
                const V3 mean = Unit(Add(previous, following));
                normal = Scale(mean, 1.0 / fmax(Dot(mean, following), 0.5));
            }
            inward[side] = normal;
        }
        const V3 a = Lift(outline->points[index], outline->lift[index], MARKING_LIFT);
        const V3 b = Lift(outline->points[next], outline->lift[next], MARKING_LIFT);
        Quad(detail, Add(a, Scale(inward[0], SHOULDER)), Add(b, Scale(inward[1], SHOULDER)),
             Add(b, Scale(inward[1], SHOULDER + 2.0 * EDGE_HALF)),
             Add(a, Scale(inward[0], SHOULDER + 2.0 * EDGE_HALF)), COLOR_LINE);
    }
}

/* A road's layout seen along the way from it into a node and on: as it is
 * where its end at the node is nodes[1], turned round where it is nodes[0]. */
static FoxRoadLayout Inward(const RoadEndShape *end)
{
    FoxRoadLayout layout = end->layout;
    if (end->end == 1u)
    {
        return layout;
    }
    FoxRoadLayout turned = layout;
    turned.lanes[0] = layout.lanes[1];
    turned.lanes[1] = layout.lanes[0];
    for (uint32_t side = 0u; side < 2u; ++side)
    {
        for (uint32_t lane = 0u; lane < FOX_ROAD_MAX_LANES; ++lane)
        {
            turned.surface[side][lane] = layout.surface[1u - side][lane];
        }
        const uint8_t way = layout.bike[1u - side];
        turned.bike[side] = way == FOX_WAY_FORWARD    ? (uint8_t)FOX_WAY_BACKWARD
                            : way == FOX_WAY_BACKWARD ? (uint8_t)FOX_WAY_FORWARD
                                                      : way;
        turned.foot[side] = layout.foot[1u - side];
    }
    return turned;
}

/* The same layout seen going out of the node along a road. */
static FoxRoadLayout Outward(const RoadEndShape *end)
{
    RoadEndShape other = *end;
    other.end = 1u - end->end;
    return Inward(&other);
}

static bool SameLayout(const FoxRoadLayout *a, const FoxRoadLayout *b)
{
    bool same = a->lanes[0] == b->lanes[0] && a->lanes[1] == b->lanes[1] && a->shared == b->shared;
    for (uint32_t side = 0u; side < 2u; ++side)
    {
        same = same && a->bike[side] == b->bike[side] && a->foot[side] == b->foot[side];
    }
    return same;
}

#define CORNER_POINTS 10u

/* The line round a corner from road a's left edge to road b's right edge,
 * `outA` and `outB` metres beyond their paved edges: from corner to corner,
 * evenly sampled into CORNER_POINTS. */
static void CornerLine(const RoadEndShape *a, const RoadEndShape *b, double gap, V3 up, V3 ex,
                       V3 ey, double reach, double outA, double outB, V3 *out)
{
    RoadEndShape wideA = *a;
    RoadEndShape wideB = *b;
    wideA.paved += outA;
    wideB.paved += outB;
    V3 raw[FILLET_SEGMENTS + SWEEP_SEGMENTS + 2u];
    uint32_t count = 0u;
    raw[count++] = LeftCorner(&wideA);
    if (gap < SWEEP_GAP)
    {
        count += Fillet(LeftCorner(&wideA), a->outward, RightCorner(&wideB), b->outward, ex, ey,
                        reach, raw + count);
    }
    else
    {
        Station stations[SWEEP_SEGMENTS + 1u];
        CentreCurve(a, b, up, stations);
        for (uint32_t step = 1u; step < SWEEP_SEGMENTS; ++step)
        {
            const double t = (double)step / SWEEP_SEGMENTS;
            raw[count++] =
                Offset(&stations[step], wideA.paved + (wideB.paved - wideA.paved) * t, 0.0);
        }
    }
    raw[count++] = RightCorner(&wideB);
    double lengths[FILLET_SEGMENTS + SWEEP_SEGMENTS + 2u];
    lengths[0] = 0.0;
    for (uint32_t index = 1u; index < count; ++index)
    {
        lengths[index] = lengths[index - 1u] + Length(Sub(raw[index], raw[index - 1u]));
    }
    uint32_t segment = 0u;
    for (uint32_t point = 0u; point < CORNER_POINTS; ++point)
    {
        const double at = lengths[count - 1u] * (double)point / (double)(CORNER_POINTS - 1u);
        while (segment + 2u < count && lengths[segment + 1u] < at)
        {
            ++segment;
        }
        const double span = lengths[segment + 1u] - lengths[segment];
        const double t = span > 0.0 ? (at - lengths[segment]) / span : 0.0;
        out[point] = Add(raw[segment], Scale(Sub(raw[segment + 1u], raw[segment]), t));
    }
}

/* A quad seen from either side. */
static void TwoSided(FoxMesh *mesh, V3 a, V3 b, V3 c, V3 d, uint32_t color)
{
    Quad(mesh, a, b, c, d, color);
    Quad(mesh, d, c, b, a, color);
}

/* The footway round a corner, from the kerb along `inner` to `outer`, with
 * the kerb's face. */
static void CornerFootway(const V3 *inner, const V3 *outer, V3 up, FoxMesh *base)
{
    for (uint32_t index = 0u; index + 1u < CORNER_POINTS; ++index)
    {
        TwoSided(base, Lift(inner[index], up, FOOTWAY_LIFT), Lift(outer[index], up, FOOTWAY_LIFT),
                 Lift(outer[index + 1u], up, FOOTWAY_LIFT),
                 Lift(inner[index + 1u], up, FOOTWAY_LIFT), COLOR_FOOTWAY);
        TwoSided(base, Lift(inner[index], up, ASPHALT_LIFT),
                 Lift(inner[index + 1u], up, ASPHALT_LIFT),
                 Lift(inner[index + 1u], up, FOOTWAY_LIFT), Lift(inner[index], up, FOOTWAY_LIFT),
                 COLOR_KERB);
    }
}

/* Whether every road at a node is only a path: then it is paved as one. */
static bool PathsOnly(const RoadEndShape *ends, uint32_t count)
{
    for (uint32_t index = 0u; index < count; ++index)
    {
        if (FoxRoadLayoutCarLanes(&ends[index].layout) != 0u)
        {
            return false;
        }
    }
    return true;
}

/* An intersection, or a bend that a piece of road cannot carry: the mouths
 * of its roads joined by curbs, and the footways round its corners. */
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
        if (a->foot[0] && b->foot[1])
        {
            V3 inner[CORNER_POINTS];
            V3 outer[CORNER_POINTS];
            CornerLine(a, b, gap, up, ex, ey, reach, 0.0, 0.0, inner);
            CornerLine(a, b, gap, up, ex, ey, reach, a->edge[0], b->edge[1], outer);
            CornerFootway(inner, outer, up, base);
        }
    }
    const bool paths = PathsOnly(ends, count);
    PaveOutline(&outline, centre, up, paths ? COLOR_FOOTWAY : COLOR_ASPHALT,
                paths ? FOOTWAY_LIFT : ASPHALT_LIFT, base, detail);
}

/* Whether a piece of road can carry the lanes from one road to the other:
 * where it turns too tightly for its width, its inner edge would fold. */
static bool RibbonFits(const RoadEndShape *a, const RoadEndShape *b, V3 up)
{
    const FoxRoadLayout from = Inward(a);
    const FoxRoadLayout to = Outward(b);
    if (!SameLayout(&from, &to))
    {
        return false;
    }
    Station stations[SWEEP_SEGMENTS + 1u];
    CentreCurve(a, b, up, stations);
    int64_t extent[2];
    FoxRoadLayoutExtent(&from, extent);
    const double reach = Metres(FoxMax(extent[0], extent[1])) + SHOULDER;
    for (uint32_t step = 0u; step < SWEEP_SEGMENTS; ++step)
    {
        const V3 along = Sub(stations[step + 1u].point, stations[step].point);
        for (int side = -1; side <= 1; side += 2)
        {
            const V3 edge = Sub(Offset(&stations[step + 1u], (double)side * reach, 0.0),
                                Offset(&stations[step], (double)side * reach, 0.0));
            if (Dot(edge, along) <= 0.0)
            {
                return false;
            }
        }
    }
    return true;
}

/* A bend: a piece of road from one road's end to the other's that carries
 * its strips and markings. */
static void Bend(const RoadEndShape *a, const RoadEndShape *b, V3 up, FoxMesh *base,
                 FoxMesh *detail)
{
    Station stations[SWEEP_SEGMENTS + 1u];
    CentreCurve(a, b, up, stations);
    const double length = stations[SWEEP_SEGMENTS].along;
    const FoxRoadLayout layout = Inward(a);
    Surfaces(base, detail, stations, SWEEP_SEGMENTS + 1u, &layout, a->side, 0.0, length);
    if (detail != NULL && length > 0.0)
    {
        Markings(detail, stations, SWEEP_SEGMENTS + 1u, &layout, a->side, 0.0, length, 0.0, length,
                 0.0, length);
    }
}

/* Where the two ways of a layout part, metres right of its centre line;
 * false where it carries only one way. */
static bool WaysPart(const FoxRoadLayout *layout, FoxDriveSide side, double *outAt, bool *outStreet)
{
    if (layout->lanes[0] == 0u || layout->lanes[1] == 0u || layout->shared)
    {
        return false;
    }
    FoxRoadStrip strips[FOX_ROAD_MAX_STRIPS];
    const uint32_t count = FoxRoadLayoutStrips(layout, side, strips);
    for (uint32_t index = 0u; index + 1u < count; ++index)
    {
        if (strips[index].kind <= FOX_LANE_CONCRETE &&
            strips[index + 1u].kind <= FOX_LANE_CONCRETE &&
            strips[index].way != strips[index + 1u].way)
        {
            *outAt = Metres(strips[index].right);
            *outStreet = layout->lanes[0] == 1u && layout->lanes[1] == 1u;
            return true;
        }
    }
    return false;
}

/* The line between the two ways along a way through a node: dashed between
 * streets, double elsewhere, in short dashes for a guide line. */
static void WayLine(FoxMesh *detail, const Station *stations, uint32_t points, double length,
                    double at, bool street, bool guide)
{
    const double dash = guide ? GUIDE_DASH : DASH;
    const double gap = guide ? GUIDE_GAP : DASH_GAP;
    if (street || guide)
    {
        LaneLine(detail, stations, points, at, 0.0, length, 0.0, length, true, dash, gap);
        return;
    }
    for (int side = -1; side <= 1; side += 2)
    {
        LaneLine(detail, stations, points, at + (double)side * DOUBLE_LINE_SPACING, 0.0, length,
                 0.0, 0.0, false, dash, gap);
    }
}

/* Through a bend paved as an outline only the line between the directions
 * carries on. */
static void BendCentreLine(const RoadEndShape *a, const RoadEndShape *b, V3 up, FoxMesh *detail)
{
    const FoxRoadLayout from = Inward(a);
    const FoxRoadLayout to = Outward(b);
    double at = 0.0;
    double other = 0.0;
    bool street = false;
    bool otherStreet = false;
    if (!WaysPart(&from, a->side, &at, &street) || !WaysPart(&to, b->side, &other, &otherStreet) ||
        fabs(at - other) > 0.01)
    {
        return;
    }
    Station stations[SWEEP_SEGMENTS + 1u];
    CentreCurve(a, b, up, stations);
    WayLine(detail, stations, SWEEP_SEGMENTS + 1u, stations[SWEEP_SEGMENTS].along, at,
            street && otherStreet, false);
}

/* Whether, going round the node from the end `first` of an order to the
 * end `second`, the end `other` is met on the way. */
static bool Between(uint32_t first, uint32_t second, uint32_t other, uint32_t count)
{
    return other != first && other != second &&
           (other + count - first) % count < (second + count - first) % count;
}

/* Where a main road runs straight through an intersection without lights,
 * the line between its ways carries on across it as a guide line of short
 * dashes, and so does its edge across the mouths of the roads that give way
 * to it. */
static void GuideLines(const RoadEndShape *ends, const uint32_t *order, uint32_t count, V3 up,
                       FoxMesh *detail)
{
    for (uint32_t first = 0u; first < count; ++first)
    {
        for (uint32_t second = first + 1u; second < count; ++second)
        {
            const RoadEndShape *a = &ends[order[first]];
            const RoadEndShape *b = &ends[order[second]];
            const bool gives = a->sign == FOX_SIGN_YIELD || a->sign == FOX_SIGN_STOP ||
                               b->sign == FOX_SIGN_YIELD || b->sign == FOX_SIGN_STOP;
            const FoxRoadLayout from = Inward(a);
            const FoxRoadLayout to = Outward(b);
            if (!SameLayout(&from, &to) || Dot(a->outward, b->outward) > -0.94 || gives ||
                FoxRoadLayoutCarLanes(&from) == 0u)
            {
                continue;
            }
            bool main = true;
            for (uint32_t other = 0u; other < count; ++other)
            {
                const FoxSign sign = ends[order[other]].sign;
                main = main && (other == first || other == second || sign == FOX_SIGN_YIELD ||
                                sign == FOX_SIGN_STOP);
            }
            if (!main)
            {
                continue;
            }
            Station stations[SWEEP_SEGMENTS + 1u];
            CentreCurve(a, b, up, stations);
            const uint32_t points = SWEEP_SEGMENTS + 1u;
            const double length = stations[SWEEP_SEGMENTS].along;
            double at = 0.0;
            bool street = false;
            if (WaysPart(&from, a->side, &at, &street))
            {
                WayLine(detail, stations, points, length, at, street, true);
            }
            /* The roads met going round from a to b join on the right of the
             * way from a to b, the others on its left. */
            bool joined[2] = {false, false};
            for (uint32_t other = 0u; other < count; ++other)
            {
                if (other != first && other != second)
                {
                    joined[Between(first, second, other, count) ? 1u : 0u] = true;
                }
            }
            const double half = CarHalf(&from);
            for (uint32_t side = 0u; side < 2u; ++side)
            {
                const double edge = (side == 1u ? 1.0 : -1.0) * (half - EDGE_HALF);
                if (joined[side])
                {
                    Strip(detail, stations, points, edge - EDGE_HALF, edge + EDGE_HALF,
                          MARKING_LIFT, 0.0, length, GUIDE_DASH, GUIDE_GAP, COLOR_LINE);
                }
            }
            return;
        }
    }
}

/* A dead end runs on to its node and rounds off there in a half circle, the
 * edge line following the curb round, and its footways round it too. */
static void Cap(const RoadEndShape *end, V3 centre, V3 up, FoxMesh *base, FoxMesh *detail)
{
    Outline outline = {.count = 0u};
    Mouth(&outline, end);
    for (uint32_t step = 0u; step <= CAP_SEGMENTS; ++step)
    {
        /* From the left side round the far end to the right side. */
        const double a = PI * (double)step / CAP_SEGMENTS;
        Put(&outline,
            Along(centre, end->right, -cos(a) * end->paved, end->outward, -sin(a) * end->paved), up,
            true);
    }
    const bool path = FoxRoadLayoutCarLanes(&end->layout) == 0u;
    PaveOutline(&outline, centre, up, path ? COLOR_FOOTWAY : COLOR_ASPHALT,
                path ? FOOTWAY_LIFT : ASPHALT_LIFT, base, detail);
    if (!end->foot[0] || !end->foot[1])
    {
        return;
    }
    const double outer = end->paved + fmin(end->edge[0], end->edge[1]);
    for (uint32_t step = 0u; step < CAP_SEGMENTS; ++step)
    {
        const double a = PI * (double)step / CAP_SEGMENTS;
        const double b = PI * (double)(step + 1u) / CAP_SEGMENTS;
        const V3 innerA =
            Along(centre, end->right, -cos(a) * end->paved, end->outward, -sin(a) * end->paved);
        const V3 innerB =
            Along(centre, end->right, -cos(b) * end->paved, end->outward, -sin(b) * end->paved);
        const V3 outerA = Along(centre, end->right, -cos(a) * outer, end->outward, -sin(a) * outer);
        const V3 outerB = Along(centre, end->right, -cos(b) * outer, end->outward, -sin(b) * outer);
        TwoSided(base, Lift(innerA, up, FOOTWAY_LIFT), Lift(outerA, up, FOOTWAY_LIFT),
                 Lift(outerB, up, FOOTWAY_LIFT), Lift(innerB, up, FOOTWAY_LIFT), COLOR_FOOTWAY);
        TwoSided(base, Lift(innerA, up, ASPHALT_LIFT), Lift(innerB, up, ASPHALT_LIFT),
                 Lift(innerB, up, FOOTWAY_LIFT), Lift(innerA, up, FOOTWAY_LIFT), COLOR_KERB);
    }
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
        Cap(&ends[0], centre, up, base, detail);
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
        if (node.carRoadCount >= 3u && !node.signals && detail != NULL)
        {
            GuideLines(ends, order, count, up, detail);
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

void FoxRoadMeshPreview(const FoxVec *centre, uint32_t count, const FoxRoadLayout *layout,
                        uint32_t color, FoxVec origin, FoxMesh *mesh)
{
    if (centre == NULL || count < 2u || layout == NULL || !FoxRoadLayoutValid(layout))
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
    int64_t extent[2];
    FoxRoadLayoutExtent(layout, extent);
    const double shoulder = FoxRoadLayoutCarLanes(layout) != 0u ? SHOULDER : 0.0;
    const double leftReach = Metres(extent[0]) + shoulder;
    const double rightReach = Metres(extent[1]) + shoulder;
    const double paved = fmax(leftReach, rightReach);
    const double lift = OVERLAY_LIFT - 0.02;
    Strip(mesh, stations, written, -leftReach, rightReach, lift, 0.0, length, 0.0, 0.0, color);
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
