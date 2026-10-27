#ifndef FOX_CLIENT_REVIEW_H
#define FOX_CLIENT_REVIEW_H

#include "client/camera.h"
#include "graphics/graphics_device_v2.h"

#include <stdbool.h>
#include <stdint.h>

/* Notes for whoever builds the game, in the review build only: P saves the
 * frame as review/shot-NNN.bmp and opens a line to type a comment on it,
 * which Enter saves beside it as shot-NNN.txt, with where the camera stood
 * and when; Escape saves it without a comment. The game shows the line in a
 * panel over the view. Nothing of this is built into the game itself. */

typedef struct FoxReview FoxReview;

/* Notes go to `directory`, made if missing; NULL when it cannot be. */
FoxReview *FoxReviewCreate(LaiueGraphicsDeviceV2 *device, const char *directory);
void FoxReviewDestroy(FoxReview *review);

/* The path the next shot is saved to, and the note opened on it once saved;
 * `context` is written under the comment. */
const char *FoxReviewNextShot(FoxReview *review);
void FoxReviewOpenNote(FoxReview *review, const char *context);

/* While a comment is typed the game's keys rest. */
bool FoxReviewTyping(const FoxReview *review);
/* Typed code points, a removed character, Enter and Escape. */
void FoxReviewType(FoxReview *review, const uint32_t *codepoints, uint32_t count);
void FoxReviewErase(FoxReview *review);
void FoxReviewFinish(FoxReview *review, bool withComment);

/* The panel with the line being typed, or a moment's word that it was
 * saved; `seconds` since the last frame. */
void FoxReviewDraw(FoxReview *review, const FoxCameraView *camera, double seconds);

#endif
