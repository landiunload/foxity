#include "client/client.h"

#include "app/log.h"
#include "app/modules.h"
#include "client/scene.h"
#include "graphics/graphics_device_service.h"
#include "input/input_service.h"
#include "mod/application.h"
#include "platform/window_service.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FOX_CLIENT_WIDTH 1280
#define FOX_CLIENT_HEIGHT 720

typedef struct GraphicsProvider
{
    FoxGraphicsChoice choice;
    const char *module;
    const char *id;
} GraphicsProvider;

/* Preferred first: the platform's native API, then the portable one. */
static const GraphicsProvider graphicsProviders[] = {
#if defined(_WIN32)
    {FOX_GRAPHICS_D3D12, "graphics_d3d12", "laiue.graphics.d3d12"},
#endif
    {FOX_GRAPHICS_VULKAN, "graphics_vulkan", "laiue.graphics.vulkan"},
};

typedef struct ClientSession
{
    const LaiueWindowServiceV1 *windowService;
    Window *window;
    const LaiueInputServiceV1 *inputService;
    Input *input;
    const LaiueGraphicsDeviceServiceV2 *graphicsService;
    uint32_t graphicsServiceSize;
    LaiueGraphicsDeviceV2 *device;
    FoxScene *scene;
    double previousSeconds;
    uint32_t smokeFrames;
    uint32_t presentedFrames;
    bool frameFailed;
    /* Saves this frame to a file and closes: a picture of the city. */
    const char *capturePath;
    uint32_t captureFrame;
    bool captured;
} ClientSession;

static bool FieldPresent(uint32_t publishedSize, uint32_t structSize, size_t offset, size_t size)
{
    return offset + size <= (size_t)publishedSize && offset + size <= (size_t)structSize;
}

#define DEVICE_HAS(device, field)                                                                  \
    ((device)->structSize >= offsetof(LaiueGraphicsDeviceV2, field) + sizeof((device)->field) &&   \
     (device)->field != NULL)

static const LaiueWindowServiceV1 *QueryWindow(const FoxModules *modules)
{
    uint32_t size = 0u;
    const LaiueWindowServiceV1 *service = (const LaiueWindowServiceV1 *)FoxModulesQuery(
        modules, LAIUE_WINDOW_SERVICE_NAME, LAIUE_WINDOW_SERVICE_ABI_VERSION_1,
        (uint32_t)sizeof(*service), &size);
    if (service == NULL || service->structSize < sizeof(*service) ||
        service->abiVersion != LAIUE_WINDOW_SERVICE_ABI_VERSION_1 || service->create == NULL ||
        service->destroy == NULL || service->getNativeHandle == NULL ||
        service->getClientSize == NULL || service->runLoop == NULL ||
        service->requestClose == NULL || service->setRawInputCallback == NULL)
    {
        return NULL;
    }
    return service;
}

static const LaiueInputServiceV1 *QueryInput(const FoxModules *modules)
{
    uint32_t size = 0u;
    const LaiueInputServiceV1 *service = (const LaiueInputServiceV1 *)FoxModulesQuery(
        modules, LAIUE_INPUT_SERVICE_NAME, LAIUE_INPUT_SERVICE_ABI_VERSION_1,
        (uint32_t)sizeof(*service), &size);
    if (service == NULL || service->structSize < sizeof(*service) ||
        service->abiVersion != LAIUE_INPUT_SERVICE_ABI_VERSION_1 || service->create == NULL ||
        service->destroy == NULL || service->handleRawInput == NULL || service->endFrame == NULL ||
        service->resetState == NULL || service->consumeKeyPress == NULL ||
        service->isKeyDown == NULL || service->isMouseButtonDown == NULL ||
        service->getMouseDelta == NULL)
    {
        return NULL;
    }
    return service;
}

static const LaiueGraphicsDeviceServiceV2 *QueryGraphics(const FoxModules *modules,
                                                         uint32_t *outSize)
{
    const LaiueGraphicsDeviceServiceV2 *service =
        (const LaiueGraphicsDeviceServiceV2 *)FoxModulesQuery(
            modules, LAIUE_GRAPHICS_DEVICE_SERVICE_NAME_V2,
            LAIUE_GRAPHICS_DEVICE_SERVICE_ABI_VERSION_2,
            LAIUE_GRAPHICS_DEVICE_SERVICE_V2_LEGACY_SIZE, outSize);
    if (service == NULL || service->structSize < LAIUE_GRAPHICS_DEVICE_SERVICE_V2_LEGACY_SIZE ||
        service->abiVersion != LAIUE_GRAPHICS_DEVICE_SERVICE_ABI_VERSION_2 ||
        service->createDevice == NULL || service->destroyDevice == NULL)
    {
        return NULL;
    }
    return service;
}

static LaiueGraphicsDeviceV2 *CreateDevice(const LaiueGraphicsDeviceServiceV2 *service,
                                           uint32_t serviceSize, void *nativeWindow)
{
    LaiueGraphicsDeviceV2 *device = NULL;
    const bool withContext =
        FieldPresent(serviceSize, service->structSize,
                     offsetof(LaiueGraphicsDeviceServiceV2, context), sizeof(service->context)) &&
        service->createDeviceWithContext != NULL && service->context != NULL;
    const uint32_t created =
        withContext ? service->createDeviceWithContext(service->context, nativeWindow,
                                                       FOX_CLIENT_WIDTH, FOX_CLIENT_HEIGHT,
                                                       LAIUE_GRAPHICS_BACKEND_AUTO, &device)
                    : service->createDevice(nativeWindow, FOX_CLIENT_WIDTH, FOX_CLIENT_HEIGHT,
                                            LAIUE_GRAPHICS_BACKEND_AUTO, &device);
    if (created != 0u && device != NULL &&
        device->structSize >=
            offsetof(LaiueGraphicsDeviceV2, abiVersion) + sizeof(device->abiVersion) &&
        device->abiVersion == LAIUE_GRAPHICS_DEVICE_V2_ABI_VERSION &&
        DEVICE_HAS(device, beginFrame) && DEVICE_HAS(device, endFrame))
    {
        return device;
    }
    /* A partial result still belongs to the provider's destroy callback. */
    if (device != NULL)
    {
        service->destroyDevice(device);
    }
    return NULL;
}

static void OnRawInput(void *opaque, void *rawInput)
{
    ClientSession *session = (ClientSession *)opaque;
    if (session->input != NULL)
    {
        session->inputService->handleRawInput(session->input, rawInput);
    }
}

/* Keyboard and mouse to player intent. Without input the city still runs and
 * is shown; it just cannot be steered. */
/* Whether the device lets vertical sync be switched. */
static bool HasVerticalSync(const LaiueGraphicsDeviceV2 *device)
{
    return device->structSize >= LAIUE_GRAPHICS_DEVICE_V2_VERTICAL_SYNC_SIZE &&
           device->setVerticalSync != NULL && device->getVerticalSync != NULL;
}

static FoxControls ReadControls(ClientSession *session)
{
    FoxControls controls = {.tool = -1};
    LaiueGraphicsDeviceV2 *device = session->device;
    controls.verticalSync = HasVerticalSync(device) ? (int32_t)device->getVerticalSync(device) : -1;
    const LaiueWindowServiceV1 *window = session->windowService;
    if (window->consumeMouseWheelSteps != NULL)
    {
        controls.zoomSteps = window->consumeMouseWheelSteps(session->window);
    }
    const LaiueInputServiceV1 *input = session->inputService;
    Input *state = session->input;
    if (state == NULL)
    {
        return controls;
    }
    controls.rotating = input->isMouseButtonDown(state, INPUT_MOUSE_BUTTON_RIGHT);
    input->getMouseDelta(state, &controls.rotateX, &controls.rotateY);
    controls.moveForward = (input->isKeyDown(state, INPUT_KEY_W) ? 1.0f : 0.0f) -
                           (input->isKeyDown(state, INPUT_KEY_S) ? 1.0f : 0.0f);
    controls.moveRight = (input->isKeyDown(state, INPUT_KEY_D) ? 1.0f : 0.0f) -
                         (input->isKeyDown(state, INPUT_KEY_A) ? 1.0f : 0.0f);
    controls.moveFast = input->isKeyDown(state, INPUT_KEY_SHIFT);
    int32_t width = 0;
    int32_t height = 0;
    window->getClientSize(session->window, &width, &height);
    if (window->getCursorClientPosition != NULL && width > 0 && height > 0)
    {
        int32_t cursorX = 0;
        int32_t cursorY = 0;
        window->getCursorClientPosition(session->window, &cursorX, &cursorY);
        /* Pixel centres to the view's -1..1 range; window rows grow downwards. */
        controls.pointer = cursorX >= 0 && cursorY >= 0 && cursorX < width && cursorY < height;
        controls.pointerX = 2.0f * ((float)cursorX + 0.5f) / (float)width - 1.0f;
        controls.pointerY = 1.0f - 2.0f * ((float)cursorY + 0.5f) / (float)height;
    }
    controls.click = input->wasMouseButtonPressed != NULL &&
                     input->wasMouseButtonPressed(state, INPUT_MOUSE_BUTTON_LEFT);
    controls.held = input->isMouseButtonDown(state, INPUT_MOUSE_BUTTON_LEFT);
    static const InputKey toolKeys[] = {INPUT_KEY_1, INPUT_KEY_2, INPUT_KEY_3,
                                        INPUT_KEY_4, INPUT_KEY_5, INPUT_KEY_6,
                                        INPUT_KEY_7, INPUT_KEY_8, INPUT_KEY_9};
    for (int32_t index = 0; index < (int32_t)(sizeof(toolKeys) / sizeof(toolKeys[0])); ++index)
    {
        if (input->consumeKeyPress(state, toolKeys[index]))
        {
            controls.tool = index;
        }
    }
    controls.nextTimeScale = input->consumeKeyPress(state, INPUT_KEY_V);
    controls.nextRoadType = input->consumeKeyPress(state, INPUT_KEY_T);
    controls.togglePause = input->consumeKeyPress(state, INPUT_KEY_SPACE);
    controls.toggleStatistics = input->consumeKeyPress(state, INPUT_KEY_F3);
    if (input->consumeKeyPress(state, INPUT_KEY_F7) && controls.verticalSync >= 0)
    {
        const uint32_t wanted = controls.verticalSync != 0 ? 0u : 1u;
        if (device->setVerticalSync(device, wanted) != 0u)
        {
            controls.verticalSync = (int32_t)wanted;
            FoxLog(FOX_LOG_INFO, "vertical sync %s", wanted != 0u ? "on" : "off");
        }
    }
    controls.carsChange = (input->consumeKeyPress(state, INPUT_KEY_E) ? 1 : 0) -
                          (input->consumeKeyPress(state, INPUT_KEY_G) ? 1 : 0);
    return controls;
}

static void Put32(uint8_t *bytes, uint32_t value)
{
    for (uint32_t index = 0u; index < 4u; ++index)
    {
        bytes[index] = (uint8_t)(value >> (8u * index));
    }
}

static FILE *OpenForWriting(const char *path)
{
#if defined(_WIN32)
    FILE *file = NULL;
    return fopen_s(&file, path, "wb") == 0 ? file : NULL;
#else
    return fopen(path, "wb");
#endif
}

/* Writes the last presented frame as a top-down 32-bit BMP. */
static bool CaptureFrame(LaiueGraphicsDeviceV2 *device, const char *path, uint32_t frameWidth,
                         uint32_t frameHeight)
{
    if (device->structSize < LAIUE_GRAPHICS_DEVICE_V2_READBACK_SIZE || device->readbackFrame == NULL ||
        device->getDiagnostics == NULL)
    {
        return false;
    }
    LaiueGraphicsDiagnosticsV2 diagnostics = {.structSize = sizeof(diagnostics)};
    if (device->getDiagnostics(device, &diagnostics) == 0u)
    {
        return false;
    }
    /* Captured frames are tightly packed RGBA8. */
    const uint64_t bytes = (uint64_t)frameWidth * frameHeight * 4u;
    uint8_t *pixels = malloc((size_t)bytes);
    if (pixels == NULL)
    {
        return false;
    }
    LaiueGraphicsFrameReadbackV2 request = {
        .structSize = sizeof(request),
        .expectedFrameIndex = diagnostics.frameIndex,
        .pixels = pixels,
        .capacityBytes = bytes,
    };
    bool ok = device->readbackFrame(device, &request) != 0u;
    FILE *file = ok ? OpenForWriting(path) : NULL;
    ok = file != NULL;
    if (ok)
    {
        const uint32_t width = request.width;
        const uint32_t height = request.height;
        uint8_t header[54] = {'B', 'M'};
        Put32(header + 2, 54u + width * height * 4u);
        Put32(header + 10, 54u);
        Put32(header + 14, 40u);
        Put32(header + 18, width);
        /* A negative height stores rows top-down, as captured. */
        Put32(header + 22, (uint32_t)(-(int32_t)height));
        header[26] = 1u;
        header[28] = 32u;
        ok = fwrite(header, 1u, sizeof(header), file) == sizeof(header);
        for (uint32_t row = 0u; ok && row < height; ++row)
        {
            uint8_t *line = pixels + (size_t)row * request.rowPitchBytes;
            for (uint32_t x = 0u; x < width; ++x)
            {
                /* RGBA to the BGRA of BMP. */
                uint8_t *pixel = line + (size_t)x * 4u;
                const uint8_t red = pixel[0];
                pixel[0] = pixel[2];
                pixel[2] = red;
            }
            ok = fwrite(line, 4u, width, file) == width;
        }
        ok = fclose(file) == 0 && ok;
    }
    free(pixels);
    return ok;
}

static void OnFrame(void *opaque)
{
    ClientSession *session = (ClientSession *)opaque;
    const LaiueWindowServiceV1 *window = session->windowService;
    if (window->consumeFocusLoss != NULL && window->consumeFocusLoss(session->window) &&
        session->input != NULL)
    {
        /* Releases that happened outside the window are never delivered. */
        session->inputService->resetState(session->input);
    }
    if (window->consumeResize != NULL && window->consumeResize(session->window) &&
        FieldPresent(session->graphicsServiceSize, session->graphicsService->structSize,
                     offsetof(LaiueGraphicsDeviceServiceV2, resize),
                     sizeof(session->graphicsService->resize)) &&
        session->graphicsService->resize != NULL)
    {
        int32_t width = 0;
        int32_t height = 0;
        window->getClientSize(session->window, &width, &height);
        if (width > 0 && height > 0)
        {
            session->graphicsService->resize(session->device, width, height);
        }
    }
    /* Escape drops a road being placed, and quits otherwise. */
    if (session->input != NULL &&
        session->inputService->consumeKeyPress(session->input, INPUT_KEY_ESCAPE) &&
        !FoxSceneCancel(session->scene))
    {
        window->requestClose(session->window);
    }

    const double now = LaiueApplicationMonotonicSeconds();
    const double elapsed = now - session->previousSeconds;
    session->previousSeconds = now;

    int32_t width = 0;
    int32_t height = 0;
    window->getClientSize(session->window, &width, &height);
    const FoxControls controls = ReadControls(session);
    LaiueGraphicsDeviceV2 *device = session->device;
    LaiueGraphicsCameraV2 camera = {.structSize = sizeof(camera)};
    const float aspect = width > 0 && height > 0 ? (float)width / (float)height : 1.0f;
    if (FoxSceneUpdate(session->scene, &controls, elapsed, aspect, camera.viewProjection) &&
        DEVICE_HAS(device, setCamera))
    {
        (void)device->setCamera(device, &camera);
    }
    const bool capturing = session->capturePath != NULL && !session->captured &&
                           session->presentedFrames + 1u >= session->captureFrame;
    if (capturing && device->structSize >= LAIUE_GRAPHICS_DEVICE_V2_READBACK_REQUEST_SIZE &&
        device->requestFrameReadback != NULL)
    {
        (void)device->requestFrameReadback(device);
    }
    if (width > 0 && height > 0)
    {
        if (device->beginFrame(device, (uint32_t)width, (uint32_t)height) != 0u)
        {
            FoxSceneDraw(session->scene);
            if (device->endFrame(device) != 0u)
            {
                ++session->presentedFrames;
                if (capturing)
                {
                    session->captured = true;
                    const bool saved = CaptureFrame(device, session->capturePath,
                                                    (uint32_t)width, (uint32_t)height);
                    FoxLog(saved ? FOX_LOG_INFO : FOX_LOG_ERROR, "%s %s",
                           saved ? "saved the frame to" : "could not save the frame to",
                           session->capturePath);
                    window->requestClose(session->window);
                }
            }
            else if (session->smokeFrames != 0u)
            {
                session->frameFailed = true;
            }
        }
        else if (session->smokeFrames != 0u)
        {
            session->frameFailed = true;
        }
    }
    if (session->input != NULL)
    {
        session->inputService->endFrame(session->input);
    }
    if (session->frameFailed ||
        (session->smokeFrames != 0u && session->presentedFrames >= session->smokeFrames))
    {
        window->requestClose(session->window);
    }
}

/* Opens a window and a device on the loaded modules and runs until the window
 * closes. False means this provider cannot present here, and nothing is left
 * open. */
static bool RunPresentation(const FoxModules *modules, const FoxClientOptions *options,
                            int *outExitCode)
{
    static ClientSession session;
    memset(&session, 0, sizeof(session));
    session.windowService = QueryWindow(modules);
    session.graphicsService = QueryGraphics(modules, &session.graphicsServiceSize);
    if (session.windowService == NULL || session.graphicsService == NULL)
    {
        return false;
    }

    const WindowConfiguration configuration = {
        .title = L"Foxity",
        .width = FOX_CLIENT_WIDTH,
        .height = FOX_CLIENT_HEIGHT,
    };
    session.window = session.windowService->create(&configuration);
    if (session.window == NULL)
    {
        FoxLog(FOX_LOG_WARNING, "window could not be created");
        return false;
    }
    void *nativeWindow = session.windowService->getNativeHandle(session.window);
    session.device =
        CreateDevice(session.graphicsService, session.graphicsServiceSize, nativeWindow);
    if (session.device == NULL)
    {
        FoxLog(FOX_LOG_WARNING, "graphics device could not be created");
        session.windowService->destroy(session.window);
        return false;
    }
    session.inputService = QueryInput(modules);
    session.input =
        session.inputService == NULL ? NULL : session.inputService->create(nativeWindow);
    if (session.input == NULL)
    {
        FoxLog(FOX_LOG_WARNING, "input is unavailable; the window cannot be controlled");
    }

    if (options->noVerticalSync)
    {
        if (HasVerticalSync(session.device) &&
            session.device->setVerticalSync(session.device, 0u) != 0u)
        {
            FoxLog(FOX_LOG_INFO, "vertical sync off");
        }
        else
        {
            FoxLog(FOX_LOG_WARNING, "this device cannot switch vertical sync");
        }
    }
    /* The sky stays bright: a clear blue behind everything. */
    if (session.device->structSize >= LAIUE_GRAPHICS_DEVICE_V2_CLEAR_COLOR_SIZE &&
        session.device->setClearColor != NULL)
    {
        static const float sky[3] = {0.55f, 0.75f, 0.95f};
        (void)session.device->setClearColor(session.device, sky);
    }
    FoxSceneDesc sceneDesc = {
        .seed = options->seed,
        .driveSide = options->driveSide,
        .cars = options->cars,
        .demo = options->demo,
        .gridLines = options->gridLines,
        .gridSpacing = options->gridSpacing,
        .cameraDistance = options->cameraDistance,
        .focusEast = options->focusEast,
        .focusNorth = options->focusNorth,
        .modules = modules,
        .device = session.device,
    };
    sceneDesc.math = (const LaiueSceneMathServiceV1 *)FoxModulesQuery(
        modules, LAIUE_SCENE_MATH_SERVICE_NAME, LAIUE_SCENE_MATH_SERVICE_ABI_VERSION_1,
        LAIUE_SCENE_MATH_SERVICE_V1_LEGACY_SIZE, &sceneDesc.mathServiceSize);
    sceneDesc.mesher = (const LaiueMesherServiceV1 *)FoxModulesQuery(
        modules, LAIUE_MESHER_SERVICE_NAME, LAIUE_MESHER_SERVICE_ABI_VERSION_1,
        LAIUE_MESHER_SERVICE_V1_LEGACY_SIZE, &sceneDesc.mesherServiceSize);
    session.scene = FoxSceneCreate(&sceneDesc);
    if (session.scene == NULL)
    {
        FoxLog(FOX_LOG_ERROR, "the city could not be created");
    }
    else
    {
        session.smokeFrames = options->smokeFrames;
        session.capturePath = options->capturePath;
        session.captureFrame = options->captureFrame;
        session.previousSeconds = LaiueApplicationMonotonicSeconds();
        session.windowService->setRawInputCallback(session.window, OnRawInput, &session);
        session.windowService->runLoop(session.window, OnFrame, &session);
        session.windowService->setRawInputCallback(session.window, NULL, NULL);
        FoxSceneDestroy(session.scene);
    }

    if (session.input != NULL)
    {
        session.inputService->destroy(session.input);
    }
    session.graphicsService->destroyDevice(session.device);
    session.windowService->destroy(session.window);

    if (session.scene == NULL)
    {
        *outExitCode = FOX_CLIENT_EXIT_NO_WORLD;
    }
    else if (options->smokeFrames != 0u)
    {
        const bool passed = !session.frameFailed && session.presentedFrames >= options->smokeFrames;
        FoxLog(passed ? FOX_LOG_INFO : FOX_LOG_ERROR, "smoke: %u of %u frames presented",
               (unsigned)session.presentedFrames, (unsigned)options->smokeFrames);
        *outExitCode = passed ? 0 : FOX_CLIENT_EXIT_FRAME_FAILED;
    }
    else
    {
        *outExitCode = 0;
    }
    return true;
}

int FoxClientRun(const FoxClientOptions *options)
{
    if (options == NULL)
    {
        return FOX_CLIENT_EXIT_INVALID_OPTIONS;
    }
    FoxModules modules;
    if (!FoxModulesOpen(&modules))
    {
        return FOX_CLIENT_EXIT_NO_ENGINE;
    }
    int exitCode = FOX_CLIENT_EXIT_NO_PRESENTATION;
    bool presented = false;
    for (size_t index = 0u;
         !presented && index < sizeof(graphicsProviders) / sizeof(graphicsProviders[0]); ++index)
    {
        const GraphicsProvider *provider = &graphicsProviders[index];
        if (options->graphics != FOX_GRAPHICS_ANY && options->graphics != provider->choice)
        {
            continue;
        }
        const char *const names[] = {"window",     "input",  provider->module,
                                     "scene_math", "mesher", "navigation"};
        const LaiueModuleProviderSelectionV1 selection = {
            .structSize = sizeof(selection),
            .serviceName = LAIUE_GRAPHICS_DEVICE_SERVICE_NAME_V2,
            .moduleId = provider->id,
        };
        FoxLog(FOX_LOG_INFO, "trying %s", provider->id);
        if (FoxModulesLoad(&modules, names, (uint32_t)(sizeof(names) / sizeof(names[0])),
                           &selection, 1u))
        {
            presented = RunPresentation(&modules, options, &exitCode);
        }
        FoxModulesUnload(&modules);
    }
    FoxModulesClose(&modules);
    if (!presented)
    {
        FoxLog(FOX_LOG_ERROR, "no window and graphics provider could present on this machine");
    }
    return exitCode;
}
