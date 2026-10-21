#include "client/assist.h"

#include <math.h>
#include <stddef.h>

#define PI 3.14159265358979323846
/* Nodes this near the pointer, metres, lend their guides. */
#define GUIDE_RADIUS 250.0
#define MAX_GUIDES 64u
/* The lines shown run on this far past the snapped point, metres. */
#define LINE_OVERSHOOT 14.0
/* A road's direction at a node is measured this far along it, metres. */
#define DIRECTION_PROBE 4.0

/* The plane touching the ground at a point, with east and north in it; its
 * coordinates are metres. */
typedef struct Plane
{
    FoxVec origin;
    double east[3];
    double north[3];
} Plane;

static double Dot3(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void Normalize3(double v[3])
{
    const double length = sqrt(Dot3(v, v));
    if (length > 0.0)
    {
        v[0] /= length;
        v[1] /= length;
        v[2] /= length;
    }
}

static Plane PlaneAt(FoxVec origin)
{
    Plane plane = {.origin = origin};
    double up[3] = {(double)origin.x, (double)origin.y, (double)origin.z};
    Normalize3(up);
    /* North is the world's y axis laid into the plane, as the demo town
     * around the pole uses it. */
    plane.north[0] = -up[0] * up[1];
    plane.north[1] = 1.0 - up[1] * up[1];
    plane.north[2] = -up[2] * up[1];
    if (Dot3(plane.north, plane.north) < 1e-9)
    {
        plane.north[0] = 1.0;
        plane.north[1] = 0.0;
        plane.north[2] = 0.0;
    }
    Normalize3(plane.north);
    plane.east[0] = plane.north[1] * up[2] - plane.north[2] * up[1];
    plane.east[1] = plane.north[2] * up[0] - plane.north[0] * up[2];
    plane.east[2] = plane.north[0] * up[1] - plane.north[1] * up[0];
    return plane;
}

static void ToPlane(const Plane *plane, FoxVec point, double out[2])
{
    const double d[3] = {(double)(point.x - plane->origin.x) / FOX_UNITS_PER_METRE,
                         (double)(point.y - plane->origin.y) / FOX_UNITS_PER_METRE,
                         (double)(point.z - plane->origin.z) / FOX_UNITS_PER_METRE};
    out[0] = Dot3(d, plane->east);
    out[1] = Dot3(d, plane->north);
}

static FoxVec FromPlane(const Plane *plane, const double p[2])
{
    FoxVec point = plane->origin;
    point.x += llround((plane->east[0] * p[0] + plane->north[0] * p[1]) * FOX_UNITS_PER_METRE);
    point.y += llround((plane->east[1] * p[0] + plane->north[1] * p[1]) * FOX_UNITS_PER_METRE);
    point.z += llround((plane->east[2] * p[0] + plane->north[2] * p[1]) * FOX_UNITS_PER_METRE);
    return point;
}

static double Cross2(const double a[2], const double b[2])
{
    return a[0] * b[1] - a[1] * b[0];
}

static double Dot2(const double a[2], const double b[2])
{
    return a[0] * b[0] + a[1] * b[1];
}

/* Where the line through p along d meets the one through q along e. */
static bool Meet(const double p[2], const double d[2], const double q[2], const double e[2],
                 double out[2])
{
    const double det = Cross2(d, e);
    if (fabs(det) < 1e-3)
    {
        return false;
    }
    const double diff[2] = {q[0] - p[0], q[1] - p[1]};
    const double t = Cross2(diff, e) / det;
    out[0] = p[0] + d[0] * t;
    out[1] = p[1] + d[1] * t;
    return true;
}

/* The direction a road leaves a node in, in the plane. */
static bool Leaving(const Plane *plane, const FoxRoadView *road, uint32_t node, double out[2])
{
    const uint32_t count = road->centreCount;
    if (count < 2u)
    {
        return false;
    }
    const bool fromStart = road->nodes[0] == node;
    const FoxVec origin = road->centre[fromStart ? 0u : count - 1u];
    FoxVec along = road->centre[fromStart ? 1u : count - 2u];
    for (uint32_t step = 1u; step < count; ++step)
    {
        along = road->centre[fromStart ? step : count - 1u - step];
        if ((double)FoxVecDistance(origin, along) >= DIRECTION_PROBE * FOX_UNITS_PER_METRE)
        {
            break;
        }
    }
    double a[2];
    double b[2];
    ToPlane(plane, origin, a);
    ToPlane(plane, along, b);
    const double length = hypot(b[0] - a[0], b[1] - a[1]);
    if (length <= 0.0)
    {
        return false;
    }
    out[0] = (b[0] - a[0]) / length;
    out[1] = (b[1] - a[1]) / length;
    return true;
}

/* The way a road from a point would carry on the road it starts on: at a
 * node the road it leaves most nearly straight on along `towards`, on a road
 * that road's direction. False when it starts on open ground. With
 * `onlyRoad`, a node must have a single road. */
static bool Continuing(const FoxNetwork *network, const Plane *plane, FoxVec start,
                       const double towards[2], bool onlyRoad, double out[2])
{
    const uint32_t nodeIndex = FoxNetworkNearestNode(network, start, FOX_METRES(1) / 2);
    FoxNodeView node;
    if (nodeIndex != FOX_NETWORK_NONE && FoxNetworkNode(network, nodeIndex, &node))
    {
        if (node.roadCount == 0u || (onlyRoad && node.roadCount != 1u))
        {
            return false;
        }
        double best = -2.0;
        for (uint32_t slot = 0u; slot < node.roadCount; ++slot)
        {
            FoxRoadView road;
            double leaving[2];
            if (!FoxNetworkRoad(network, node.roads[slot], &road) ||
                !Leaving(plane, &road, nodeIndex, leaving))
            {
                continue;
            }
            const double onward[2] = {-leaving[0], -leaving[1]};
            const double score = Dot2(onward, towards);
            if (score > best)
            {
                best = score;
                out[0] = onward[0];
                out[1] = onward[1];
            }
        }
        return best > -2.0;
    }
    if (onlyRoad)
    {
        return false;
    }
    int64_t along = 0;
    const uint32_t roadIndex = FoxNetworkNearestRoad(network, start, FOX_METRES(1) / 2, &along);
    FoxRoadView road;
    if (roadIndex == FOX_NETWORK_NONE || !FoxNetworkRoad(network, roadIndex, &road))
    {
        return false;
    }
    /* The segment holding the point gives the road's direction there. */
    double best = INFINITY;
    for (uint32_t segment = 0u; segment + 1u < road.centreCount; ++segment)
    {
        double a[2];
        double b[2];
        ToPlane(plane, road.centre[segment], a);
        ToPlane(plane, road.centre[segment + 1u], b);
        const double length = hypot(b[0] - a[0], b[1] - a[1]);
        if (length <= 0.0)
        {
            continue;
        }
        const double d[2] = {(b[0] - a[0]) / length, (b[1] - a[1]) / length};
        const double t = fmin(fmax(-Dot2(a, d), 0.0), length);
        const double distance = hypot(a[0] + d[0] * t, a[1] + d[1] * t);
        if (distance < best)
        {
            best = distance;
            const double sign = Dot2(d, towards) >= 0.0 ? 1.0 : -1.0;
            out[0] = d[0] * sign;
            out[1] = d[1] * sign;
        }
    }
    return best < INFINITY;
}

typedef struct Guide
{
    double point[2];
    double direction[2];
} Guide;

static void AddGuide(Guide *guides, uint32_t *count, const double point[2], double dx, double dy)
{
    const double direction[2] = {dx, dy};
    for (uint32_t index = 0u; index < *count; ++index)
    {
        const Guide *guide = &guides[index];
        const double offset[2] = {point[0] - guide->point[0], point[1] - guide->point[1]};
        /* The same line already: parallel and through the point. */
        if (fabs(Cross2(guide->direction, direction)) < 0.02 &&
            fabs(Cross2(guide->direction, offset)) < 0.5)
        {
            return;
        }
    }
    if (*count < MAX_GUIDES)
    {
        guides[(*count)++] = (Guide){{point[0], point[1]}, {dx, dy}};
    }
}

/* The lines of the roads at nodes near a point, and their perpendiculars. */
static uint32_t CollectGuides(const FoxNetwork *network, const Plane *plane, FoxVec near,
                              Guide *guides)
{
    uint32_t count = 0u;
    const double radius = GUIDE_RADIUS * FOX_UNITS_PER_METRE;
    for (uint32_t index = 0u; index < FoxNetworkNodeCapacity(network); ++index)
    {
        FoxNodeView node;
        if (!FoxNetworkNode(network, index, &node) ||
            (double)FoxVecDistance(node.position, near) > radius)
        {
            continue;
        }
        double point[2];
        ToPlane(plane, node.position, point);
        for (uint32_t slot = 0u; slot < node.roadCount; ++slot)
        {
            FoxRoadView road;
            double d[2];
            if (FoxNetworkRoad(network, node.roads[slot], &road) && Leaving(plane, &road, index, d))
            {
                AddGuide(guides, &count, point, d[0], d[1]);
                AddGuide(guides, &count, point, -d[1], d[0]);
            }
        }
    }
    return count;
}

/* The nearest point of any road's centre line to p within reach, in the
 * plane; or, given a direction, the nearest point where the line through p
 * along it crosses one. */
static bool RoadPoint(const FoxNetwork *network, const Plane *plane, const double p[2],
                      const double *along, double reach, double out[2])
{
    const FoxVec near = FromPlane(plane, p);
    double best = reach;
    bool found = false;
    for (uint32_t index = 0u; index < FoxNetworkRoadCapacity(network); ++index)
    {
        FoxRoadView road;
        if (!FoxNetworkRoad(network, index, &road) || road.centreCount < 2u ||
            (double)FoxVecDistance(near, road.centre[0]) >
                (double)road.length + reach * FOX_UNITS_PER_METRE)
        {
            continue;
        }
        for (uint32_t segment = 0u; segment + 1u < road.centreCount; ++segment)
        {
            double a[2];
            double b[2];
            ToPlane(plane, road.centre[segment], a);
            ToPlane(plane, road.centre[segment + 1u], b);
            const double d[2] = {b[0] - a[0], b[1] - a[1]};
            const double lengthSquared = Dot2(d, d);
            if (lengthSquared <= 0.0)
            {
                continue;
            }
            double candidate[2];
            if (along != NULL)
            {
                const double unit[2] = {d[0] / sqrt(lengthSquared), d[1] / sqrt(lengthSquared)};
                if (!Meet(p, along, a, unit, candidate))
                {
                    continue;
                }
                const double offset[2] = {candidate[0] - a[0], candidate[1] - a[1]};
                const double t = Dot2(offset, d) / lengthSquared;
                if (t < 0.0 || t > 1.0)
                {
                    continue;
                }
            }
            else
            {
                const double offset[2] = {p[0] - a[0], p[1] - a[1]};
                const double t = fmin(fmax(Dot2(offset, d) / lengthSquared, 0.0), 1.0);
                candidate[0] = a[0] + d[0] * t;
                candidate[1] = a[1] + d[1] * t;
            }
            const double distance = hypot(candidate[0] - p[0], candidate[1] - p[1]);
            if (distance < best)
            {
                best = distance;
                out[0] = candidate[0];
                out[1] = candidate[1];
                found = true;
            }
        }
    }
    return found;
}

static void AddLine(FoxAssist *out, const Plane *plane, const double from[2], const double to[2],
                    bool guide)
{
    if (out->lineCount >= FOX_ASSIST_MAX_LINES)
    {
        return;
    }
    const double length = hypot(to[0] - from[0], to[1] - from[1]);
    double end[2] = {to[0], to[1]};
    if (length > 0.0)
    {
        end[0] += (to[0] - from[0]) / length * LINE_OVERSHOOT;
        end[1] += (to[1] - from[1]) / length * LINE_OVERSHOOT;
    }
    out->lines[out->lineCount++] = (FoxAssistLine){
        .from = FromPlane(plane, from),
        .to = FromPlane(plane, end),
        .guide = guide,
    };
}

/* Follows the guides: onto the nearest within reach, and onto where it
 * crosses another within reach. */
static bool FollowGuides(const Guide *guides, uint32_t count, const double p[2], double reach,
                         double out[2], uint32_t *outFirst, uint32_t *outSecond)
{
    uint32_t first = MAX_GUIDES;
    double best = reach;
    for (uint32_t index = 0u; index < count; ++index)
    {
        const double offset[2] = {p[0] - guides[index].point[0], p[1] - guides[index].point[1]};
        const double distance = fabs(Cross2(guides[index].direction, offset));
        if (distance < best)
        {
            best = distance;
            first = index;
        }
    }
    if (first == MAX_GUIDES)
    {
        return false;
    }
    const Guide *guide = &guides[first];
    const double offset[2] = {p[0] - guide->point[0], p[1] - guide->point[1]};
    const double t = Dot2(offset, guide->direction);
    out[0] = guide->point[0] + guide->direction[0] * t;
    out[1] = guide->point[1] + guide->direction[1] * t;
    *outFirst = first;
    *outSecond = MAX_GUIDES;
    best = reach;
    for (uint32_t index = 0u; index < count; ++index)
    {
        double crossing[2];
        if (index != first && Meet(guide->point, guide->direction, guides[index].point,
                                   guides[index].direction, crossing))
        {
            const double distance = hypot(crossing[0] - p[0], crossing[1] - p[1]);
            if (distance < best)
            {
                best = distance;
                out[0] = crossing[0];
                out[1] = crossing[1];
                *outSecond = index;
            }
        }
    }
    return true;
}

bool FoxAssistPoint(const FoxNetwork *network, const FoxPlanet *planet,
                    const FoxAssistRequest *request, FoxAssist *out)
{
    if (network == NULL || planet == NULL || request == NULL || out == NULL)
    {
        return false;
    }
    *out = (FoxAssist){.point = request->raw, .kind = FOX_SNAPPED_FREE};
    const uint32_t snaps = request->snaps;
    const double reach = request->reach;
    const Plane plane = PlaneAt(request->hasStart ? request->start : request->raw);
    double p[2];
    ToPlane(&plane, request->raw, p);
    const double start[2] = {0.0, 0.0};
    double q[2] = {p[0], p[1]};
    bool exact = false;
    FoxVec exactPoint = request->raw;
    double reference[2] = {0.0, 1.0};
    bool referenced = false;

    /* A node near the pointer wins over everything. */
    const uint32_t near = (snaps & FOX_SNAP_NETWORK) != 0u
                              ? FoxNetworkNearestNode(network, request->raw,
                                                      llround(reach * FOX_UNITS_PER_METRE))
                              : FOX_NETWORK_NONE;
    FoxNodeView node;
    if (near != FOX_NETWORK_NONE && FoxNetworkNode(network, near, &node))
    {
        out->kind = FOX_SNAPPED_NODE;
        exact = true;
        exactPoint = node.position;
        ToPlane(&plane, node.position, q);
    }
    Guide guides[MAX_GUIDES];
    const uint32_t guideCount = (snaps & FOX_SNAP_GUIDES) != 0u
                                    ? CollectGuides(network, &plane, request->raw, guides)
                                    : 0u;
    if (request->hasStart)
    {
        referenced = Continuing(network, &plane, request->start, p, false, reference);
    }
    if (out->kind == FOX_SNAPPED_FREE && request->hasStart)
    {
        double onward[2];
        const double v[2] = {p[0], p[1]};
        if (request->curveControl && (snaps & FOX_SNAP_TANGENT) != 0u &&
            Continuing(network, &plane, request->start, v, true, onward))
        {
            /* A curve leaves the road's end the way the road was going. */
            const double t = fmax(Dot2(v, onward), 2.0);
            q[0] = onward[0] * t;
            q[1] = onward[1] * t;
            out->kind = FOX_SNAPPED_ANGLE;
            AddLine(out, &plane, start, q, false);
        }
        else if ((snaps & FOX_SNAP_ANGLE) != 0u)
        {
            const double step = request->angleStep * PI / 180.0;
            const double angle = atan2(Cross2(reference, v), Dot2(reference, v));
            const double snapped = round(angle / step) * step;
            const double d[2] = {reference[0] * cos(snapped) - reference[1] * sin(snapped),
                                 reference[0] * sin(snapped) + reference[1] * cos(snapped)};
            double t = Dot2(v, d);
            if ((snaps & FOX_SNAP_LENGTH) != 0u)
            {
                t = round(t / request->lengthStep) * request->lengthStep;
            }
            t = fmax(t, (snaps & FOX_SNAP_LENGTH) != 0u ? request->lengthStep : 1.0);
            q[0] = d[0] * t;
            q[1] = d[1] * t;
            out->kind = FOX_SNAPPED_ANGLE;
            /* What the line meets near there: a node on it, a road across it
             * or a guide. */
            const uint32_t onLine = (snaps & FOX_SNAP_NETWORK) != 0u
                                        ? FoxNetworkNearestNode(network, FromPlane(&plane, q),
                                                                llround(reach * FOX_UNITS_PER_METRE))
                                        : FOX_NETWORK_NONE;
            double crossing[2];
            if (onLine != FOX_NETWORK_NONE && FoxNetworkNode(network, onLine, &node))
            {
                double n[2];
                ToPlane(&plane, node.position, n);
                if (fabs(Cross2(d, n)) < reach / 2.0 && Dot2(d, n) > 0.0)
                {
                    q[0] = n[0];
                    q[1] = n[1];
                    exact = true;
                    exactPoint = node.position;
                    out->kind = FOX_SNAPPED_NODE;
                }
            }
            if (out->kind == FOX_SNAPPED_ANGLE && (snaps & FOX_SNAP_NETWORK) != 0u &&
                RoadPoint(network, &plane, q, d, reach, crossing))
            {
                q[0] = crossing[0];
                q[1] = crossing[1];
                out->kind = FOX_SNAPPED_ROAD;
            }
            double best = reach;
            uint32_t followed = MAX_GUIDES;
            double onGuide[2] = {q[0], q[1]};
            for (uint32_t index = 0u; out->kind == FOX_SNAPPED_ANGLE && index < guideCount; ++index)
            {
                if (Meet(start, d, guides[index].point, guides[index].direction, crossing) &&
                    hypot(crossing[0] - q[0], crossing[1] - q[1]) < best &&
                    Dot2(crossing, d) > 0.0)
                {
                    best = hypot(crossing[0] - q[0], crossing[1] - q[1]);
                    followed = index;
                    onGuide[0] = crossing[0];
                    onGuide[1] = crossing[1];
                }
            }
            if (followed != MAX_GUIDES)
            {
                q[0] = onGuide[0];
                q[1] = onGuide[1];
                out->kind = FOX_SNAPPED_GUIDE;
                AddLine(out, &plane, guides[followed].point, q, true);
            }
            AddLine(out, &plane, start, q, false);
        }
        else if ((snaps & FOX_SNAP_LENGTH) != 0u)
        {
            const double length = hypot(v[0], v[1]);
            if (length > 0.0)
            {
                const double t =
                    fmax(round(length / request->lengthStep), 1.0) * request->lengthStep;
                q[0] = v[0] / length * t;
                q[1] = v[1] / length * t;
            }
        }
    }
    double point[2];
    if (out->kind == FOX_SNAPPED_FREE && (snaps & FOX_SNAP_NETWORK) != 0u &&
        RoadPoint(network, &plane, q, NULL, reach, point))
    {
        q[0] = point[0];
        q[1] = point[1];
        out->kind = FOX_SNAPPED_ROAD;
    }
    uint32_t first = MAX_GUIDES;
    uint32_t second = MAX_GUIDES;
    if (out->kind == FOX_SNAPPED_FREE &&
        FollowGuides(guides, guideCount, q, reach, point, &first, &second))
    {
        q[0] = point[0];
        q[1] = point[1];
        out->kind = FOX_SNAPPED_GUIDE;
        AddLine(out, &plane, guides[first].point, q, true);
        if (second != MAX_GUIDES)
        {
            AddLine(out, &plane, guides[second].point, q, true);
        }
    }
    if (out->kind == FOX_SNAPPED_FREE && (snaps & FOX_SNAP_GRID) != 0u)
    {
        /* The grid lies square to the world's axes around the pole. */
        const FoxVec raw = FromPlane(&plane, q);
        const double step = request->lengthStep * FOX_UNITS_PER_METRE;
        exactPoint = (FoxVec){llround(round((double)raw.x / step) * step),
                              llround(round((double)raw.y / step) * step), raw.z};
        exact = true;
        ToPlane(&plane, exactPoint, q);
        out->kind = FOX_SNAPPED_GRID;
    }
    if (exact && out->kind == FOX_SNAPPED_NODE)
    {
        out->point = exactPoint;
    }
    else if (!FoxPlanetGround(planet, exact ? exactPoint : FromPlane(&plane, q), &out->point))
    {
        return false;
    }
    if (request->hasStart)
    {
        double end[2];
        ToPlane(&plane, out->point, end);
        out->metres = hypot(end[0], end[1]);
        const double angle = atan2(Cross2(reference, end), Dot2(reference, end)) * 180.0 / PI;
        /* Against the road continued, how far it turns either way; against
         * north, the bearing round to the east. */
        out->degrees = referenced ? fabs(angle) : fmod(360.0 - angle, 360.0);
    }
    return true;
}
