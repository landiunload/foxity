#include "client/hud.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define GLYPH_WIDTH 5u
#define GLYPH_HEIGHT 7u
/* A character cell and a line, in font pixels. */
#define CELL_WIDTH 6.0f
#define LINE_HEIGHT 9.0f
#define PADDING 4.0f
/* Between the text and the graphs, and the size of a graph. */
#define COLUMN_GAP 10.0f
#define GRAPH_HEIGHT 28.0f
#define GRAPH_MIN_WIDTH 160.0f
/* The view is this many font pixels high. */
#define VIEW_PIXELS 460.0f
#define MARGIN 6.0f

#define COLOR_PANEL UINT32_C(0xFF1C1C1C)
#define COLOR_GRAPH UINT32_C(0xFF2E2E2E)
#define COLOR_GRID UINT32_C(0xFF454545)
#define COLOR_TEXT UINT32_C(0xFFEEEEEE)
#define COLOR_HEADING UINT32_C(0xFF30C8F0)

typedef struct Glyph
{
    uint32_t code;
    uint8_t rows[GLYPH_HEIGHT];
} Glyph;

/* Rows top to bottom, the high bit of five on the left; Latin and Cyrillic
 * capitals, digits and the signs the panel uses. */
static const Glyph glyphs[] = {
    {'0', {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}}, {'1', {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E}},
    {'2', {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}}, {'3', {0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E}},
    {'4', {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}}, {'5', {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E}},
    {'6', {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}}, {'7', {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}},
    {'8', {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}}, {'9', {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C}},
    {'A', {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}}, {'B', {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E}},
    {'C', {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}}, {'D', {0x1C, 0x12, 0x11, 0x11, 0x11, 0x12, 0x1C}},
    {'E', {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}}, {'F', {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10}},
    {'G', {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F}}, {'H', {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}},
    {'I', {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}}, {'J', {0x07, 0x02, 0x02, 0x02, 0x02, 0x12, 0x0C}},
    {'K', {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}}, {'L', {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F}},
    {'M', {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11}}, {'N', {0x11, 0x11, 0x19, 0x15, 0x13, 0x11, 0x11}},
    {'O', {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}}, {'P', {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10}},
    {'Q', {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D}}, {'R', {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11}},
    {'S', {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E}}, {'T', {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}},
    {'U', {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}}, {'V', {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04}},
    {'W', {0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A}}, {'X', {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11}},
    {'Y', {0x11, 0x11, 0x11, 0x0A, 0x04, 0x04, 0x04}}, {'Z', {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F}},
    {'.', {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C}}, {',', {0x00, 0x00, 0x00, 0x00, 0x0C, 0x04, 0x08}},
    {':', {0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00}}, {'-', {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00}},
    {'+', {0x00, 0x04, 0x04, 0x1F, 0x04, 0x04, 0x00}}, {'/', {0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x00}},
    {'%', {0x18, 0x19, 0x02, 0x04, 0x08, 0x13, 0x03}}, {'(', {0x02, 0x04, 0x08, 0x08, 0x08, 0x04, 0x02}},
    {')', {0x08, 0x04, 0x02, 0x02, 0x02, 0x04, 0x08}}, {'[', {0x0E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0E}},
    {']', {0x0E, 0x02, 0x02, 0x02, 0x02, 0x02, 0x0E}}, {'=', {0x00, 0x00, 0x1F, 0x00, 0x1F, 0x00, 0x00}},
    {'>', {0x08, 0x04, 0x02, 0x01, 0x02, 0x04, 0x08}}, {'<', {0x02, 0x04, 0x08, 0x10, 0x08, 0x04, 0x02}},
    {'#', {0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}}, {'*', {0x00, 0x04, 0x15, 0x0E, 0x15, 0x04, 0x00}},
    {'_', {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F}}, {'|', {0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}},
    {'\'', {0x04, 0x04, 0x08, 0x00, 0x00, 0x00, 0x00}}, {'!', {0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x04}},
    {'?', {0x0E, 0x11, 0x01, 0x02, 0x04, 0x00, 0x04}},
    {0x00B1, {0x04, 0x04, 0x1F, 0x04, 0x04, 0x00, 0x1F}}, {0x00D7, {0x00, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x00}},
    {0x0410, {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}}, {0x0411, {0x1F, 0x10, 0x10, 0x1E, 0x11, 0x11, 0x1E}},
    {0x0412, {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E}}, {0x0413, {0x1F, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10}},
    {0x0414, {0x06, 0x0A, 0x0A, 0x0A, 0x0A, 0x1F, 0x11}}, {0x0415, {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}},
    {0x0401, {0x0A, 0x00, 0x1F, 0x10, 0x1E, 0x10, 0x1F}}, {0x0416, {0x15, 0x15, 0x15, 0x0E, 0x15, 0x15, 0x15}},
    {0x0417, {0x0E, 0x11, 0x01, 0x06, 0x01, 0x11, 0x0E}}, {0x0418, {0x11, 0x11, 0x13, 0x15, 0x19, 0x11, 0x11}},
    {0x0419, {0x0A, 0x04, 0x11, 0x13, 0x15, 0x19, 0x11}}, {0x041A, {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}},
    {0x041B, {0x07, 0x09, 0x09, 0x09, 0x09, 0x09, 0x11}}, {0x041C, {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11}},
    {0x041D, {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}}, {0x041E, {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}},
    {0x041F, {0x1F, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11}}, {0x0420, {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10}},
    {0x0421, {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}}, {0x0422, {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}},
    {0x0423, {0x11, 0x11, 0x11, 0x0F, 0x01, 0x11, 0x0E}}, {0x0424, {0x04, 0x0E, 0x15, 0x15, 0x15, 0x0E, 0x04}},
    {0x0425, {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11}}, {0x0426, {0x12, 0x12, 0x12, 0x12, 0x12, 0x1F, 0x01}},
    {0x0427, {0x11, 0x11, 0x11, 0x0F, 0x01, 0x01, 0x01}}, {0x0428, {0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x1F}},
    {0x0429, {0x15, 0x15, 0x15, 0x15, 0x15, 0x1F, 0x01}}, {0x042A, {0x18, 0x08, 0x08, 0x0E, 0x09, 0x09, 0x0E}},
    {0x042B, {0x11, 0x11, 0x11, 0x19, 0x15, 0x15, 0x19}}, {0x042C, {0x10, 0x10, 0x10, 0x1E, 0x11, 0x11, 0x1E}},
    {0x042D, {0x0E, 0x11, 0x01, 0x07, 0x01, 0x11, 0x0E}}, {0x042E, {0x12, 0x15, 0x15, 0x1D, 0x15, 0x15, 0x12}},
    {0x042F, {0x0F, 0x11, 0x11, 0x0F, 0x05, 0x09, 0x11}},
};

struct FoxHud
{
    LaiueGraphicsDeviceV2 *device;
    LaiueGraphicsHandle buffer;
    uint32_t count;
    LaiueGraphicsVertexV2 *vertices;
    uint32_t capacity;
    bool failed;
};

FoxHud *FoxHudCreate(LaiueGraphicsDeviceV2 *device)
{
    if (device == NULL || device->createBuffer == NULL || device->uploadBuffer == NULL ||
        device->destroyHandle == NULL || device->structSize < LAIUE_GRAPHICS_DEVICE_V2_INSTANCES_SIZE ||
        device->getCapabilities == NULL || device->submitInstances == NULL ||
        (device->getCapabilities(device) & LAIUE_GRAPHICS_CAP_GENERIC_INSTANCES) == 0u)
    {
        return NULL;
    }
    FoxHud *hud = calloc(1u, sizeof(*hud));
    if (hud != NULL)
    {
        hud->device = device;
    }
    return hud;
}

void FoxHudDestroy(FoxHud *hud)
{
    if (hud == NULL)
    {
        return;
    }
    if (hud->buffer != 0u)
    {
        hud->device->destroyHandle(hud->device, hud->buffer);
    }
    free(hud->vertices);
    free(hud);
}

/* The next code point of UTF-8 text; a broken sequence reads as '?'. */
static uint32_t NextCode(const char **cursor)
{
    const uint8_t *bytes = (const uint8_t *)*cursor;
    if (bytes[0] < 0x80u)
    {
        *cursor += 1;
        return bytes[0];
    }
    if ((bytes[0] & 0xE0u) == 0xC0u && (bytes[1] & 0xC0u) == 0x80u)
    {
        *cursor += 2;
        return ((uint32_t)(bytes[0] & 0x1Fu) << 6u) | (uint32_t)(bytes[1] & 0x3Fu);
    }
    if ((bytes[0] & 0xF0u) == 0xE0u && (bytes[1] & 0xC0u) == 0x80u && (bytes[2] & 0xC0u) == 0x80u)
    {
        *cursor += 3;
        return ((uint32_t)(bytes[0] & 0x0Fu) << 12u) | ((uint32_t)(bytes[1] & 0x3Fu) << 6u) |
               (uint32_t)(bytes[2] & 0x3Fu);
    }
    *cursor += 1;
    return '?';
}

static const Glyph *Find(uint32_t code)
{
    /* Small letters show as capitals; dashes as the hyphen. */
    if (code >= 'a' && code <= 'z')
    {
        code -= 'a' - 'A';
    }
    else if (code >= 0x0430u && code <= 0x044Fu)
    {
        code -= 0x20u;
    }
    else if (code == 0x0451u)
    {
        code = 0x0401u;
    }
    else if (code == 0x2013u || code == 0x2014u)
    {
        code = '-';
    }
    for (size_t index = 0u; index < sizeof(glyphs) / sizeof(glyphs[0]); ++index)
    {
        if (glyphs[index].code == code)
        {
            return &glyphs[index];
        }
    }
    return NULL;
}

/* A rectangle in panel space: x to the right, y up, the top left at the
 * origin; depth runs towards the eye. */
static void Rectangle(FoxHud *hud, float x0, float y0, float x1, float y1, float depth,
                      uint32_t color)
{
    if (hud->failed)
    {
        return;
    }
    if (hud->count + 6u > hud->capacity)
    {
        if (hud->capacity > UINT32_MAX / 4u)
        {
            hud->failed = true;
            return;
        }
        const uint32_t capacity = hud->capacity < 4096u ? 8192u : hud->capacity * 2u;
        LaiueGraphicsVertexV2 *grown = realloc(hud->vertices, (size_t)capacity * sizeof(*grown));
        if (grown == NULL)
        {
            hud->failed = true;
            return;
        }
        hud->vertices = grown;
        hud->capacity = capacity;
    }
    const float corners[6][2] = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y0}, {x1, y1}, {x0, y1}};
    for (uint32_t corner = 0u; corner < 6u; ++corner)
    {
        hud->vertices[hud->count++] = (LaiueGraphicsVertexV2){
            .position = {corners[corner][0], corners[corner][1], depth},
            .colorRGBA = color,
        };
    }
}

/* The lit pixels of a glyph, each row's runs merged into one rectangle. */
static void Character(FoxHud *hud, const Glyph *glyph, float left, float top, uint32_t color)
{
    for (uint32_t row = 0u; row < GLYPH_HEIGHT; ++row)
    {
        uint32_t column = 0u;
        while (column < GLYPH_WIDTH)
        {
            if ((glyph->rows[row] & (1u << (GLYPH_WIDTH - 1u - column))) == 0u)
            {
                ++column;
                continue;
            }
            uint32_t end = column;
            while (end < GLYPH_WIDTH && (glyph->rows[row] & (1u << (GLYPH_WIDTH - 1u - end))) != 0u)
            {
                ++end;
            }
            Rectangle(hud, left + (float)column, top - (float)row - 1.0f, left + (float)end,
                      top - (float)row, 0.0f, color);
            column = end;
        }
    }
}

/* Writes one line from `cursor` up to a newline; returns past it. */
static const char *Line(FoxHud *hud, const char *cursor, float left, float top, uint32_t color)
{
    while (*cursor != '\0' && *cursor != '\n')
    {
        const Glyph *glyph = Find(NextCode(&cursor));
        if (glyph != NULL)
        {
            Character(hud, glyph, left, top, color);
        }
        left += CELL_WIDTH;
    }
    return *cursor == '\n' ? cursor + 1 : cursor;
}

/* The longest line in characters and the number of lines. */
static void Measure(const char *text, uint32_t *outColumns, uint32_t *outLines)
{
    uint32_t lines = 1u;
    uint32_t longest = 0u;
    uint32_t current = 0u;
    for (const char *cursor = text; *cursor != '\0';)
    {
        if (*cursor == '\n')
        {
            ++lines;
            current = 0u;
            ++cursor;
            continue;
        }
        (void)NextCode(&cursor);
        ++current;
        longest = current > longest ? current : longest;
    }
    *outColumns = longest;
    *outLines = lines;
}

static void Graph(FoxHud *hud, const FoxHudGraph *graph, float left, float top, float width)
{
    Line(hud, graph->title != NULL ? graph->title : "", left, top, COLOR_TEXT);
    const float bottom = top - LINE_HEIGHT - GRAPH_HEIGHT;
    const float upper = top - LINE_HEIGHT;
    Rectangle(hud, left, bottom, left + width, upper, -0.25f, COLOR_GRAPH);
    /* Quarter lines to read values by. */
    for (uint32_t quarter = 1u; quarter < 4u; ++quarter)
    {
        const float y = bottom + GRAPH_HEIGHT * (float)quarter / 4.0f;
        Rectangle(hud, left, y, left + width, y + 0.5f, -0.2f, COLOR_GRID);
    }
    if (graph->values == NULL || graph->count == 0u || !(graph->maximum > 0.0f))
    {
        return;
    }
    const float bar = width / (float)graph->count;
    for (uint32_t index = 0u; index < graph->count; ++index)
    {
        const float x0 = left + bar * (float)index;
        const float x1 = x0 + bar;
        const float first = fminf(fmaxf(graph->values[index], 0.0f) / graph->maximum, 1.0f);
        if (first > 0.0f)
        {
            Rectangle(hud, x0, bottom, x1, bottom + GRAPH_HEIGHT * first, -0.1f, graph->color);
        }
        if (graph->stacked != NULL)
        {
            const float second = fminf(fmaxf(graph->stacked[index], 0.0f) / graph->maximum,
                                       1.0f - first);
            if (second > 0.0f)
            {
                Rectangle(hud, x0, bottom + GRAPH_HEIGHT * first, x1,
                          bottom + GRAPH_HEIGHT * (first + second), -0.1f, graph->stackedColor);
            }
        }
    }
}

static void Upload(FoxHud *hud)
{
    if (hud->buffer != 0u)
    {
        hud->device->destroyHandle(hud->device, hud->buffer);
        hud->buffer = 0u;
    }
    if (hud->failed || hud->count == 0u)
    {
        hud->count = 0u;
        return;
    }
    const uint64_t bytes = (uint64_t)hud->count * sizeof(*hud->vertices);
    const LaiueGraphicsBufferDescV1 description = {
        .structSize = sizeof(description),
        .usageFlags = LAIUE_GRAPHICS_BUFFER_USAGE_VERTEX,
        .sizeBytes = bytes,
    };
    LaiueGraphicsHandle handle = 0u;
    if (hud->device->createBuffer(hud->device, &description, &handle) == 0u)
    {
        hud->count = 0u;
        return;
    }
    const LaiueGraphicsBufferUploadV1 upload = {
        .structSize = sizeof(upload),
        .buffer = handle,
        .data = hud->vertices,
        .sizeBytes = bytes,
    };
    if (hud->device->uploadBuffer(hud->device, &upload) == 0u)
    {
        hud->device->destroyHandle(hud->device, handle);
        hud->count = 0u;
        return;
    }
    hud->buffer = handle;
}

void FoxHudSetContent(FoxHud *hud, const char *text, const FoxHudGraph *graphs,
                      uint32_t graphCount)
{
    if (hud == NULL || text == NULL || (graphs == NULL && graphCount != 0u))
    {
        return;
    }
    hud->count = 0u;
    hud->failed = false;
    uint32_t columns = 0u;
    uint32_t lines = 0u;
    Measure(text, &columns, &lines);
    const float textWidth = (float)columns * CELL_WIDTH;
    const float textHeight = (float)lines * LINE_HEIGHT;
    float graphWidth = 0.0f;
    for (uint32_t index = 0u; index < graphCount; ++index)
    {
        uint32_t titleColumns = 0u;
        uint32_t titleLines = 0u;
        Measure(graphs[index].title != NULL ? graphs[index].title : "", &titleColumns, &titleLines);
        graphWidth = fmaxf(graphWidth, fmaxf(GRAPH_MIN_WIDTH, (float)graphs[index].count));
        graphWidth = fmaxf(graphWidth, (float)titleColumns * CELL_WIDTH);
    }
    const float graphsHeight = (float)graphCount * (LINE_HEIGHT + GRAPH_HEIGHT + 3.0f);
    const float width = textWidth + (graphCount != 0u ? COLUMN_GAP + graphWidth : 0.0f) +
                        2.0f * PADDING;
    const float height = fmaxf(textHeight, graphsHeight) + 2.0f * PADDING;
    /* The panel sits a little behind everything on it, away from the eye. */
    Rectangle(hud, 0.0f, -height, width, 0.0f, -0.5f, COLOR_PANEL);
    float top = -PADDING;
    for (const char *cursor = text; *cursor != '\0'; top -= LINE_HEIGHT)
    {
        const bool heading = cursor[0] == '-' && cursor[1] == '-';
        cursor = Line(hud, cursor, PADDING, top, heading ? COLOR_HEADING : COLOR_TEXT);
    }
    const float graphLeft = PADDING + textWidth + COLUMN_GAP;
    for (uint32_t index = 0u; index < graphCount; ++index)
    {
        Graph(hud, &graphs[index], graphLeft,
              -PADDING - (float)index * (LINE_HEIGHT + GRAPH_HEIGHT + 3.0f), graphWidth);
    }
    Upload(hud);
}

/* The rotation whose columns are the given axes, as a unit quaternion. */
static void Quaternion(const double x[3], const double y[3], const double z[3], float out[4])
{
    const double m00 = x[0], m01 = y[0], m02 = z[0];
    const double m10 = x[1], m11 = y[1], m12 = z[1];
    const double m20 = x[2], m21 = y[2], m22 = z[2];
    const double trace = m00 + m11 + m22;
    double q[4];
    if (trace > 0.0)
    {
        const double s = sqrt(trace + 1.0) * 2.0;
        q[0] = (m21 - m12) / s;
        q[1] = (m02 - m20) / s;
        q[2] = (m10 - m01) / s;
        q[3] = 0.25 * s;
    }
    else if (m00 > m11 && m00 > m22)
    {
        const double s = sqrt(1.0 + m00 - m11 - m22) * 2.0;
        q[0] = 0.25 * s;
        q[1] = (m01 + m10) / s;
        q[2] = (m02 + m20) / s;
        q[3] = (m21 - m12) / s;
    }
    else if (m11 > m22)
    {
        const double s = sqrt(1.0 + m11 - m00 - m22) * 2.0;
        q[0] = (m01 + m10) / s;
        q[1] = 0.25 * s;
        q[2] = (m12 + m21) / s;
        q[3] = (m02 - m20) / s;
    }
    else
    {
        const double s = sqrt(1.0 + m22 - m00 - m11) * 2.0;
        q[0] = (m02 + m20) / s;
        q[1] = (m12 + m21) / s;
        q[2] = 0.25 * s;
        q[3] = (m10 - m01) / s;
    }
    const double length = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (uint32_t index = 0u; index < 4u; ++index)
    {
        out[index] = (float)(q[index] / length);
    }
}

void FoxHudDraw(FoxHud *hud, const FoxCameraView *camera)
{
    if (hud == NULL || camera == NULL || hud->buffer == 0u)
    {
        return;
    }
    /* Just beyond the near plane, where nothing of the city can be. */
    const double distance = (double)camera->nearPlane * 1.5;
    const double halfHeight = distance * tan((double)FOX_CAMERA_FIELD_OF_VIEW / 2.0);
    const double halfWidth = halfHeight * (double)camera->aspectRatio;
    const double pixel = 2.0 * halfHeight / (double)VIEW_PIXELS;
    const double margin = (double)MARGIN * pixel;
    LaiueGraphicsInstanceV2 instance = {.scale = (float)pixel};
    for (uint32_t axis = 0u; axis < 3u; ++axis)
    {
        instance.originRelative[axis] =
            (float)((double)camera->eye[axis] + camera->look[axis] * distance +
                    camera->screenRight[axis] * (margin - halfWidth) +
                    camera->screenUp[axis] * (halfHeight - margin));
    }
    /* Towards the eye the panel's depth runs backwards. */
    const double back[3] = {-camera->look[0], -camera->look[1], -camera->look[2]};
    Quaternion(camera->screenRight, camera->screenUp, back, instance.rotation);
    const LaiueGraphicsDrawItemV2 draw = {
        .structSize = LAIUE_GRAPHICS_DRAW_ITEM_V2_LEGACY_SIZE,
        .vertexBuffer = hud->buffer,
        .indexCount = hud->count,
        .scale = 1.0f,
    };
    (void)hud->device->submitInstances(hud->device, &draw, &instance, 1u);
}
