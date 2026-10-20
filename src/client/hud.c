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
/* The view is this many font pixels high. */
#define VIEW_PIXELS 400.0f
#define MARGIN 6.0f
#define MAX_TEXT 4096u

#define COLOR_PANEL UINT32_C(0xFF1C1C1C)
#define COLOR_TEXT UINT32_C(0xFFEEEEEE)
#define COLOR_HEADING UINT32_C(0xFF30C8F0)

typedef struct Glyph
{
    char character;
    uint8_t rows[GLYPH_HEIGHT];
} Glyph;

/* Rows top to bottom, the high bit of five on the left. */
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
};

struct FoxHud
{
    LaiueGraphicsDeviceV2 *device;
    LaiueGraphicsHandle buffer;
    uint32_t count;
    float width;
    float height;
    char text[MAX_TEXT];
    LaiueGraphicsVertexV2 *vertices;
    uint32_t capacity;
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

static const Glyph *Find(char character)
{
    const char upper = character >= 'a' && character <= 'z' ? (char)(character - 'a' + 'A') : character;
    for (size_t index = 0u; index < sizeof(glyphs) / sizeof(glyphs[0]); ++index)
    {
        if (glyphs[index].character == upper)
        {
            return &glyphs[index];
        }
    }
    return NULL;
}

/* A rectangle in panel space: x to the right, y up, the top left at the
 * origin. */
static bool Rectangle(FoxHud *hud, float x0, float y0, float x1, float y1, float depth,
                      uint32_t color)
{
    if (hud->count + 6u > hud->capacity)
    {
        const uint32_t capacity = hud->capacity < 4096u ? 4096u : hud->capacity * 2u;
        LaiueGraphicsVertexV2 *grown = realloc(hud->vertices, (size_t)capacity * sizeof(*grown));
        if (grown == NULL)
        {
            return false;
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
    return true;
}

/* The lit pixels of a glyph, each row's runs merged into one rectangle. */
static bool Character(FoxHud *hud, const Glyph *glyph, float left, float top, uint32_t color)
{
    for (uint32_t row = 0u; row < GLYPH_HEIGHT; ++row)
    {
        uint32_t column = 0u;
        while (column < GLYPH_WIDTH)
        {
            const uint32_t bit = 1u << (GLYPH_WIDTH - 1u - column);
            if ((glyph->rows[row] & bit) == 0u)
            {
                ++column;
                continue;
            }
            uint32_t end = column;
            while (end < GLYPH_WIDTH && (glyph->rows[row] & (1u << (GLYPH_WIDTH - 1u - end))) != 0u)
            {
                ++end;
            }
            if (!Rectangle(hud, left + (float)column, top - (float)row - 1.0f, left + (float)end,
                           top - (float)row, 0.0f, color))
            {
                return false;
            }
            column = end;
        }
    }
    return true;
}

void FoxHudSetText(FoxHud *hud, const char *text)
{
    if (hud == NULL || text == NULL || strncmp(hud->text, text, sizeof(hud->text)) == 0)
    {
        return;
    }
    const size_t length = strlen(text);
    const size_t kept = length < sizeof(hud->text) - 1u ? length : sizeof(hud->text) - 1u;
    memcpy(hud->text, text, kept);
    hud->text[kept] = '\0';

    /* The size of the panel first: the longest line and the line count. */
    uint32_t lines = 1u;
    uint32_t longest = 0u;
    uint32_t current = 0u;
    for (size_t index = 0u; index < kept; ++index)
    {
        if (hud->text[index] == '\n')
        {
            ++lines;
            current = 0u;
            continue;
        }
        ++current;
        longest = current > longest ? current : longest;
    }
    hud->width = (float)longest * CELL_WIDTH + 2.0f * PADDING;
    hud->height = (float)lines * LINE_HEIGHT + 2.0f * PADDING - (LINE_HEIGHT - (float)GLYPH_HEIGHT);
    hud->count = 0u;
    /* The panel sits a little behind the letters, away from the eye. */
    bool ok = Rectangle(hud, 0.0f, -hud->height, hud->width, 0.0f, -0.5f, COLOR_PANEL);
    float top = -PADDING;
    float left = PADDING;
    bool heading = kept >= 2u && hud->text[0] == '-' && hud->text[1] == '-';
    for (size_t index = 0u; ok && index < kept; ++index)
    {
        const char character = hud->text[index];
        if (character == '\n')
        {
            top -= LINE_HEIGHT;
            left = PADDING;
            heading = index + 2u < kept && hud->text[index + 1u] == '-' && hud->text[index + 2u] == '-';
            continue;
        }
        const Glyph *glyph = Find(character);
        if (glyph != NULL)
        {
            ok = Character(hud, glyph, left, top, heading ? COLOR_HEADING : COLOR_TEXT);
        }
        left += CELL_WIDTH;
    }
    if (hud->buffer != 0u)
    {
        hud->device->destroyHandle(hud->device, hud->buffer);
        hud->buffer = 0u;
    }
    if (!ok || hud->count == 0u)
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
