#include "client/review.h"

#include "client/hud.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)
#include <direct.h>
#else
#include <sys/stat.h>
#endif

#define PATH_CAPACITY 1024u
#define COMMENT_CAPACITY 4096u
#define PANEL_CAPACITY 8192u
/* Code points to a line of the panel. */
#define LINE_WIDTH 56u
/* How long the word that a note was saved stays. */
#define SAVED_SECONDS 2.5

struct FoxReview
{
    FoxHud *hud;
    char directory[PATH_CAPACITY];
    char shot[PATH_CAPACITY];
    uint32_t number;
    bool typing;
    char comment[COMMENT_CAPACITY];
    size_t commentBytes;
    char context[PATH_CAPACITY];
    double savedFor;
    bool changed;
    char panel[PANEL_CAPACITY];
};

static bool Exists(const char *path)
{
    FILE *file = NULL;
#if defined(_WIN32)
    if (fopen_s(&file, path, "rb") != 0)
    {
        file = NULL;
    }
#else
    file = fopen(path, "rb");
#endif
    if (file == NULL)
    {
        return false;
    }
    (void)fclose(file);
    return true;
}

static FILE *Create(const char *path)
{
#if defined(_WIN32)
    FILE *file = NULL;
    return fopen_s(&file, path, "wb") == 0 ? file : NULL;
#else
    return fopen(path, "wb");
#endif
}

static void MakeDirectory(const char *path)
{
#if defined(_WIN32)
    (void)_mkdir(path);
#else
    (void)mkdir(path, 0755);
#endif
}

/* The first number whose shot is not taken yet. */
static void FindFreeShot(FoxReview *review)
{
    for (;;)
    {
        ++review->number;
        (void)snprintf(review->shot, sizeof(review->shot), "%s/shot-%03u.bmp", review->directory,
                       review->number);
        if (!Exists(review->shot))
        {
            return;
        }
    }
}

FoxReview *FoxReviewCreate(LaiueGraphicsDeviceV2 *device, const char *directory)
{
    if (directory == NULL || strlen(directory) + 16u >= PATH_CAPACITY)
    {
        return NULL;
    }
    FoxReview *review = calloc(1u, sizeof(*review));
    if (review == NULL)
    {
        return NULL;
    }
    (void)snprintf(review->directory, sizeof(review->directory), "%s", directory);
    MakeDirectory(review->directory);
    review->hud = FoxHudCreate(device);
    FindFreeShot(review);
    return review;
}

void FoxReviewDestroy(FoxReview *review)
{
    if (review != NULL)
    {
        FoxHudDestroy(review->hud);
        free(review);
    }
}

const char *FoxReviewNextShot(FoxReview *review)
{
    return review == NULL ? NULL : review->shot;
}

void FoxReviewOpenNote(FoxReview *review, const char *context)
{
    if (review == NULL)
    {
        return;
    }
    review->typing = true;
    review->comment[0] = '\0';
    review->commentBytes = 0u;
    review->savedFor = 0.0;
    review->changed = true;
    (void)snprintf(review->context, sizeof(review->context), "%s", context != NULL ? context : "");
}

bool FoxReviewTyping(const FoxReview *review)
{
    return review != NULL && review->typing;
}

/* A code point as UTF-8; how many bytes. */
static size_t Encode(uint32_t codepoint, char out[4])
{
    if (codepoint < 0x80u)
    {
        out[0] = (char)codepoint;
        return 1u;
    }
    if (codepoint < 0x800u)
    {
        out[0] = (char)(0xC0u | (codepoint >> 6u));
        out[1] = (char)(0x80u | (codepoint & 0x3Fu));
        return 2u;
    }
    if (codepoint < 0x10000u)
    {
        out[0] = (char)(0xE0u | (codepoint >> 12u));
        out[1] = (char)(0x80u | ((codepoint >> 6u) & 0x3Fu));
        out[2] = (char)(0x80u | (codepoint & 0x3Fu));
        return 3u;
    }
    out[0] = (char)(0xF0u | (codepoint >> 18u));
    out[1] = (char)(0x80u | ((codepoint >> 12u) & 0x3Fu));
    out[2] = (char)(0x80u | ((codepoint >> 6u) & 0x3Fu));
    out[3] = (char)(0x80u | (codepoint & 0x3Fu));
    return 4u;
}

void FoxReviewType(FoxReview *review, const uint32_t *codepoints, uint32_t count)
{
    if (review == NULL || !review->typing || codepoints == NULL)
    {
        return;
    }
    for (uint32_t index = 0u; index < count; ++index)
    {
        char bytes[4];
        const size_t length = Encode(codepoints[index], bytes);
        if (review->commentBytes + length + 1u >= COMMENT_CAPACITY)
        {
            break;
        }
        memcpy(review->comment + review->commentBytes, bytes, length);
        review->commentBytes += length;
        review->comment[review->commentBytes] = '\0';
        review->changed = true;
    }
}

void FoxReviewErase(FoxReview *review)
{
    if (review == NULL || !review->typing || review->commentBytes == 0u)
    {
        return;
    }
    /* Back over the continuation bytes to the start of the last character. */
    do
    {
        --review->commentBytes;
    } while (review->commentBytes > 0u &&
             ((unsigned char)review->comment[review->commentBytes] & 0xC0u) == 0x80u);
    review->comment[review->commentBytes] = '\0';
    review->changed = true;
}

void FoxReviewFinish(FoxReview *review, bool withComment)
{
    if (review == NULL || !review->typing)
    {
        return;
    }
    char note[PATH_CAPACITY + 8u];
    (void)snprintf(note, sizeof(note), "%.*s.txt", (int)(strlen(review->shot) - 4u), review->shot);
    FILE *file = Create(note);
    if (file != NULL)
    {
        (void)fprintf(file, "%s\n\n%s\n", withComment ? review->comment : "", review->context);
        (void)fclose(file);
    }
    review->typing = false;
    review->savedFor = SAVED_SECONDS;
    review->changed = true;
}

/* The comment broken into lines of LINE_WIDTH code points. */
static void Wrap(const char *text, char *out, size_t capacity)
{
    size_t used = 0u;
    uint32_t column = 0u;
    for (const char *cursor = text; *cursor != '\0' && used + 2u < capacity; ++cursor)
    {
        const bool starts = ((unsigned char)*cursor & 0xC0u) != 0x80u;
        if (starts && column == LINE_WIDTH)
        {
            out[used++] = '\n';
            column = 0u;
        }
        out[used++] = *cursor;
        column += starts ? 1u : 0u;
    }
    out[used] = '\0';
}

void FoxReviewDraw(FoxReview *review, const FoxCameraView *camera, double seconds)
{
    if (review == NULL || review->hud == NULL || camera == NULL)
    {
        return;
    }
    if (!review->typing && review->savedFor > 0.0)
    {
        review->savedFor -= seconds;
        review->changed = review->changed || review->savedFor <= 0.0;
    }
    if (!review->typing && review->savedFor <= 0.0)
    {
        return;
    }
    if (review->changed)
    {
        review->changed = false;
        if (review->typing)
        {
            char wrapped[COMMENT_CAPACITY + COMMENT_CAPACITY / LINE_WIDTH + 8u];
            Wrap(review->comment, wrapped, sizeof(wrapped));
            (void)snprintf(review->panel, sizeof(review->panel),
                           "-- СНИМОК %03u: КОММЕНТАРИЙ\n%s_\n\nENTER - СОХРАНИТЬ, ESC - БЕЗ "
                           "КОММЕНТАРИЯ",
                           review->number, wrapped);
        }
        else
        {
            (void)snprintf(review->panel, sizeof(review->panel), "-- СНИМОК %03u СОХРАНЁН",
                           review->number);
            FindFreeShot(review);
        }
        FoxHudSetContent(review->hud, review->panel, NULL, 0u);
    }
    FoxHudDraw(review->hud, camera);
}
