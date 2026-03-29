// WoodenHaptics real-time 3D linkage visualizer using raylib
// Shows all body segments, joints and end-effector in real-time
// Press 1/2 to toggle between woodenhaptics_v2015 / default_woody config
// Press Q or ESC to quit
#ifndef LINUX
#define LINUX
#endif

#include <cstdio>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

// ---- Kinematic configs ----
struct WoodyConfig {
    const char* name;
    double Ln, Lb, Lc;                     // link lengths
    double capstan[3], body_diam[3];        // capstan/body diameters
    double cpr[3];                          // counts per revolution
    double dir[3];                          // motor direction (+1 or -1)
    int enc_home[6];                        // encoder home offsets
    double ws_origin[3];                    // workspace origin
};

static const WoodyConfig config = {
    "woodenhaptics_v2015",
    0.080, 0.205, 0.245,
    {0.010, 0.010, 0.010}, {0.160, 0.120, 0.120},
    {2000, 2000, 2000},
    {1.0, -1.0, -1.0},       // motor_and_body_aligned = {0,1,1}
    {0, 0, -6000, 0, 0, 0},
    {0.220, 0.000, 0.080}
};

// ---- Forward kinematics (matches hfab_kinematics.cpp for variant!=3) ----
struct LinkageState {
    double tA, tB, tC;
    Vector3 base;       // (0, 0, 0)
    Vector3 joint1;     // top of body A
    Vector3 joint2;     // end of body B
    Vector3 endeff;     // end of body C (end effector)
    Vector3 endeff_ws;  // end effector in workspace coords (after origin subtraction)
};

// Hfab coords: x=forward, y=right, z=up
// Raylib coords: x=right, y=up, z=forward
// Mapping: hfab(x,y,z) -> raylib(y_hfab, z_hfab, x_hfab) = raylib(right, up, fwd)
// Actually simpler: raylib x = hfab y, raylib y = hfab z, raylib z = hfab x
static Vector3 toRaylib(double hx, double hy, double hz) {
    return {(float)hy, (float)hz, (float)hx};
}

LinkageState computeLinkage(const WoodyConfig& cfg, const int raw_enc[6]) {
    LinkageState s;
    double pi = 3.141592653589793;

    int enc[6];
    for (int i = 0; i < 6; i++) enc[i] = raw_enc[i] + cfg.enc_home[i];

    double motorAngle[3];
    for (int i = 0; i < 3; i++)
        motorAngle[i] = 2.0 * pi * (double)enc[i] / cfg.cpr[i];

    s.tA = cfg.dir[0] * motorAngle[0] * cfg.capstan[0] / cfg.body_diam[0];
    s.tB = cfg.dir[1] * motorAngle[1] * cfg.capstan[1] / cfg.body_diam[1];
    s.tC = cfg.dir[2] * motorAngle[2] * cfg.capstan[2] / cfg.body_diam[2];

    // Joint positions in hfab coords (x=fwd, y=right, z=up)
    // Base
    s.base = toRaylib(0, 0, 0);

    // Joint 1: top of body A (vertical shaft)
    s.joint1 = toRaylib(0, 0, cfg.Ln);

    // Joint 2: end of body B
    double j2_x = cos(s.tA) * cfg.Lb * sin(s.tB);
    double j2_y = sin(s.tA) * cfg.Lb * sin(s.tB);
    double j2_z = cfg.Ln + cfg.Lb * cos(s.tB);
    s.joint2 = toRaylib(j2_x, j2_y, j2_z);

    // End effector: end of body C
    double ee_x = cos(s.tA) * (cfg.Lb * sin(s.tB) + cfg.Lc * sin(s.tC));
    double ee_y = sin(s.tA) * (cfg.Lb * sin(s.tB) + cfg.Lc * sin(s.tC));
    double ee_z = cfg.Ln + cfg.Lb * cos(s.tB) - cfg.Lc * cos(s.tC);
    s.endeff = toRaylib(ee_x, ee_y, ee_z);

    s.endeff_ws = toRaylib(
        ee_x - cfg.ws_origin[0],
        ee_y - cfg.ws_origin[1],
        ee_z - cfg.ws_origin[2]);

    return s;
}

// ---- Draw a thick line segment as a cylinder ----
void DrawLinkSegment(Vector3 from, Vector3 to, float radius, Color color) {
    Vector3 dir = Vector3Subtract(to, from);
    float len = Vector3Length(dir);
    if (len < 0.0001f) return;

    // Midpoint
    Vector3 mid = {(from.x+to.x)/2, (from.y+to.y)/2, (from.z+to.z)/2};

    // DrawCylinderEx draws from startPos to endPos
    DrawCylinderEx(from, to, radius, radius, 8, color);
}

// ---- Serial communication ----
int openSerial(const char* port) {
    int fd = open(port, O_RDWR);
    if (fd < 0) return -1;

    struct termios tinfo;
    tcgetattr(fd, &tinfo);
    cfmakeraw(&tinfo);
    cfsetspeed(&tinfo, B115200);
    tcsetattr(fd, TCSANOW, &tinfo);

    struct serial_struct kss;
    if (ioctl(fd, TIOCGSERIAL, &kss) >= 0) {
        kss.flags |= ASYNC_LOW_LATENCY;
        ioctl(fd, TIOCSSERIAL, &kss);
    }
    return fd;
}

// Send command, read response (non-blocking)
bool sendAndReceive(int fd, int enc_out[6], int* model_out, int* err_out) {
    char sendbuf[16] = {};
    sprintf(sendbuf, "[0,0,0]");
    write(fd, sendbuf, 16);

    // Non-blocking read
    char buf[256] = {};
    int total = 0;
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
    for (int retry = 0; retry < 300; retry++) {
        int n = read(fd, buf + total, sizeof(buf) - total - 1);
        if (n > 0) {
            total += n;
            for (int j = 0; j < total; j++)
                if (buf[j] == ']') goto done;
        }
        usleep(200);
    }
done:
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) & ~O_NONBLOCK);

    int model, enc[6], err;
    if (sscanf(buf, "[%d,%d,%d,%d,%d,%d,%d,%d]",
               &model, &enc[0], &enc[1], &enc[2], &enc[3], &enc[4], &enc[5], &err) == 8) {
        for (int i = 0; i < 6; i++) enc_out[i] = enc[i];
        *model_out = model;
        *err_out = err;
        return true;
    }
    return false;
}

int main() {
    // Open serial
    int fd = openSerial("/dev/ttyACM0");
    if (fd < 0) {
        printf("Cannot open /dev/ttyACM0\n");
        return 1;
    }
    printf("Waiting for device boot...\n");
    usleep(1500000);
    tcflush(fd, TCIOFLUSH);

    // Warm up serial - get a few responses
    int raw_enc[6] = {};
    int model = 0, err = 0;
    for (int i = 0; i < 5; i++) {
        sendAndReceive(fd, raw_enc, &model, &err);
        usleep(20000);
    }

    // Init raylib
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    const int screenW = 1200, screenH = 800;
    InitWindow(screenW, screenH, "WoodenHaptics Linkage Visualizer");
    SetTargetFPS(60);

    Camera3D camera = {};
    camera.position = {0.35f, 0.25f, 0.35f};
    camera.target = {0.0f, 0.15f, 0.1f};
    camera.up = {0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    // Create "Woody" text texture for platform surface
    Image woodyImg = GenImageColor(256, 64, BLANK);
    ImageDrawText(&woodyImg, "Woody", 30, 10, 44, YELLOW);
    Texture2D woodyTex = LoadTextureFromImage(woodyImg);
    UnloadImage(woodyImg);

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_Q)) break;

        // Manual orbit camera around target
        // Scroll to zoom, right-drag to orbit, middle-drag to pan
        {
            float wheel = GetMouseWheelMove();
            if (wheel != 0) {
                Vector3 dir = Vector3Subtract(camera.position, camera.target);
                float dist = Vector3Length(dir);
                dist -= wheel * 0.05f;
                if (dist < 0.05f) dist = 0.05f;
                camera.position = Vector3Add(camera.target, Vector3Scale(Vector3Normalize(dir), dist));
            }
            if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
                Vector2 delta = GetMouseDelta();
                Vector3 dir = Vector3Subtract(camera.position, camera.target);
                float dist = Vector3Length(dir);
                // Horizontal rotation (around Y)
                Matrix rotY = MatrixRotateY(-delta.x * 0.005f);
                // Vertical rotation (around right vector)
                Vector3 right = Vector3Normalize(Vector3CrossProduct(
                    Vector3Subtract(camera.target, camera.position), camera.up));
                Matrix rotR = MatrixRotate(right, -delta.y * 0.005f);
                dir = Vector3Transform(Vector3Transform(dir, rotY), rotR);
                camera.position = Vector3Add(camera.target, Vector3Scale(Vector3Normalize(dir), dist));
            }
            if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
                Vector2 delta = GetMouseDelta();
                Vector3 fwd = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
                Vector3 right = Vector3Normalize(Vector3CrossProduct(fwd, camera.up));
                Vector3 up = Vector3CrossProduct(right, fwd);
                Vector3 pan = Vector3Add(Vector3Scale(right, -delta.x * 0.001f),
                                         Vector3Scale(up, delta.y * 0.001f));
                camera.position = Vector3Add(camera.position, pan);
                camera.target = Vector3Add(camera.target, pan);
            }
        }

        // Read from device
        sendAndReceive(fd, raw_enc, &model, &err);

        // Compute linkage
        LinkageState st = computeLinkage(config, raw_enc);

        // Draw
        BeginDrawing();
        ClearBackground({30, 30, 35, 255});

        BeginMode3D(camera);

        // Grid floor
        DrawGrid(20, 0.05f);

        // Draw active linkage
        float r = 0.006f; // link radius

        // Base platform (wider in hfab Y = raylib X)
        DrawCube(st.base, 0.20f, 0.01f, 0.10f, DARKGRAY);

        // "Woody" text as texture on platform surface
        // Quad lying on platform top (y=0.006), text reads along raylib X (= hfab Y)
        // Positioned at +x side of platform
        rlSetTexture(woodyTex.id);
        rlBegin(RL_QUADS);
            rlColor4ub(255,255,255,255);
            float tx0 = 0.02f, tx1 = 0.09f;   // raylib X span (hfab Y)
            float tz0 = 0.01f, tz1 = 0.045f;  // raylib Z span (hfab X)
            float ty = 0.006f;
            rlTexCoord2f(0, 1); rlVertex3f(tx0, ty, tz0);
            rlTexCoord2f(1, 1); rlVertex3f(tx1, ty, tz0);
            rlTexCoord2f(1, 0); rlVertex3f(tx1, ty, tz1);
            rlTexCoord2f(0, 0); rlVertex3f(tx0, ty, tz1);
        rlEnd();
        rlSetTexture(0);

        // Motor: black cylinder standing in Z direction (= raylib Y), behind base at hfab y=0
        Vector3 motorBot = toRaylib(-0.04, 0.0, -0.01);
        Vector3 motorTop = toRaylib(-0.04, 0.0, 0.06);
        DrawCylinderEx(motorBot, motorTop, 0.020f, 0.020f, 12, BLACK);

        // Body A (vertical shaft) - gray
        DrawLinkSegment(st.base, st.joint1, r * 1.2f, GRAY);
        DrawSphere(st.joint1, 0.010f, LIGHTGRAY);

        // Body B - green
        DrawLinkSegment(st.joint1, st.joint2, r, GREEN);
        DrawSphere(st.joint2, 0.008f, LIME);

        // Body C - blue
        DrawLinkSegment(st.joint2, st.endeff, r, BLUE);

        // End effector - red sphere
        DrawSphere(st.endeff, 0.012f, RED);

        // Axes: hfab X (fwd)=red, Y (right)=green, Z (up)=blue
        // hfab X -> raylib Z, hfab Y -> raylib X, hfab Z -> raylib Y
        float axLen = 0.15f;
        float axR = 0.005f;
        float tipLen = 0.025f;
        float tipR = axR * 3.0f;
        // X axis (red) = raylib +Z
        DrawCylinderEx({0,0,0}, {0, 0, axLen}, axR, axR, 8, RED);
        DrawCylinderEx({0, 0, axLen}, {0, 0, axLen+tipLen}, tipR, 0, 8, RED);
        // Y axis (green) = raylib +X
        DrawCylinderEx({0,0,0}, {axLen, 0, 0}, axR, axR, 8, GREEN);
        DrawCylinderEx({axLen, 0, 0}, {axLen+tipLen, 0, 0}, tipR, 0, 8, GREEN);
        // Z axis (blue) = raylib +Y
        DrawCylinderEx({0,0,0}, {0, axLen, 0}, axR, axR, 8, BLUE);
        DrawCylinderEx({0, axLen, 0}, {0, axLen+tipLen, 0}, tipR, 0, 8, BLUE);

        EndMode3D();

        // Project axis labels to screen
        float labelOff = axLen + tipLen + 0.02f;
        Vector2 labelX = GetWorldToScreen({0, 0, labelOff}, camera);
        Vector2 labelY = GetWorldToScreen({labelOff, 0, 0}, camera);
        Vector2 labelZ = GetWorldToScreen({0, labelOff, 0}, camera);
        DrawText("X", (int)labelX.x-6, (int)labelX.y-10, 24, RED);
        DrawText("Y", (int)labelY.x-6, (int)labelY.y-10, 24, GREEN);
        DrawText("Z", (int)labelZ.x-6, (int)labelZ.y-10, 24, BLUE);

        // Text overlay
        int y = 10;
        DrawText(TextFormat("Raw enc: [%d, %d, %d, %d, %d, %d]",
                 raw_enc[0], raw_enc[1], raw_enc[2], raw_enc[3], raw_enc[4], raw_enc[5]),
                 10, y, 18, WHITE); y += 22;
        DrawText(TextFormat("tA: %6.1f    tB: %6.1f    tC: %6.1f deg",
                 st.tA * 180.0/3.14159, st.tB * 180.0/3.14159, st.tC * 180.0/3.14159),
                 10, y, 18, LIGHTGRAY); y += 22;
        DrawText(TextFormat("End eff: (%.3f, %.3f, %.3f) m",
                 st.endeff_ws.z, st.endeff_ws.x, st.endeff_ws.y),
                 10, y, 18, RED); y += 22;

        DrawFPS(GetScreenWidth() - 100, 10);

        EndDrawing();
    }

    CloseWindow();
    close(fd);
    return 0;
}
