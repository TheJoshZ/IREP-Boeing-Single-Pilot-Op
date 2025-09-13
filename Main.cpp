#include "XPLMMenus.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#define _USE_MATH_DEFINES
#include "XPLMProcessing.h"
#include <cmath>
#include <array>
#include <tuple>
#include <fstream>
#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"
#include "XPLMPlanes.h"  // For multiplayer aircraft
#include "XPLMDataAccess.h"
#include <map>
#include <deque>
#include <sstream> // For loading waypoints from file

#if IBM
    #include <windows.h>
    #include <GL/gl.h>
#endif

#ifndef XPLM300
    #error This must be compiled against the XPLM300+ SDK
#endif

// ──────────────────────────────────
// Global state variables
// ──────────────────────────────────

static int        g_menu_container_idx;
static XPLMMenuID g_menu_id;
static bool       g_hud_visible = false;
static bool g_font_baked = false;
static float      g_ladder_spacing = 30.0f;  // Default pitch ladder spacing in pixels per degree
 
// DataRefs for aircraft parameters
static XPLMDataRef  g_airspeed_ias_ref = NULL;   // Indicated airspeed (knots)
static XPLMDataRef  g_airspeed_tas_ref = NULL;   // True airspeed (knots)
static XPLMDataRef  g_altitude_pa_ref  = NULL;   // Pressure altitude (meters)
static XPLMDataRef  g_radar_alt_ref    = NULL;   // Radar altitude (meters above ground level)
static XPLMDataRef  g_pitch_ref        = NULL;   // Pitch angle (degrees)
static XPLMDataRef  g_roll_ref         = NULL;   // Roll angle (degrees)
static XPLMDataRef  gHeadingRef       = NULL;   // Aircraft heading (degrees)
static XPLMDataRef g_aoa_ref = NULL; // Angle of Attack (degrees)

double lat1 = 47.4602, lon1 = -122.3078; // North end
double lat2 = 47.4294, lon2 = -122.3080; // South end
float elev = 400.0f; // Approx field elevation in feet

// for landing assist
static double radout_init = 0.0;
static bool radout_init_set = false;
static bool g_landing_assist_visible = false;
static bool g_seattle_to_kelowna_visible = false;
static bool g_zones_visible = false;

// for traffic 
static bool g_aircraft_highlight_visible = false;
static int draw_aircraft_highlight_callback(XPLMDrawingPhase inPhase, int inIsBefore, void* inRefcon); // callback for drawing aircraft highlights
static void draw_highlight_box(float x, float y, float z, const float color[3], const char* tailnum);  // function to draw a highlight box around an aircraft
static std::map<int, std::deque<std::array<float, 3>>> g_ai_trails; //store trails for each AI aircraft

// for waypoints
static bool g_custom_waypoints_visible = false;
static bool LoadCustomWaypoints(const char* filename);
static float draw_custom_waypoints_callback(XPLMDrawingPhase inPhase, int inIsBefore, void* inRefcon);
static char g_waypoint_status[64] = "No load attempted";


//for custom zones
static std::vector<std::tuple<double, double, double>> g_custom_zone_points;
static char g_zone_status[64] = "No zone load attempted";


// ──────────────────────────────────
// Forward declarations
// ──────────────────────────────────

static void menu_handler(void* in_menu_ref, void* in_item_ref);
static float draw_hud_callback(
    XPLMDrawingPhase inPhase,
    int              inIsBefore,
    void*            inRefcon);

// ──────────────────────────────────
// Utility: Draw text with black shadow for better readability
// ──────────────────────────────────
static void DrawTextWithShadow(
    float color[3],
    int   x,
    int   y,
    const char* text)
{
    float shadow_col[3] = { 0.0f, 0.0f, 0.0f };
    float scale = 1.2f; // Scale factor for text size

    glPushMatrix();
    glScalef(scale, scale, 1.0f);

    // Draw shadow in 8 directions
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue; // skip center
            XPLMDrawString(shadow_col, (int)((x + dx) / scale), (int)((y + dy) / scale), (char*)text, NULL, xplmFont_Basic);
        }
    }
    // Draw main text
    XPLMDrawString(color, (int)(x / scale), (int)(y / scale), (char*)text, NULL, xplmFont_Basic);

    glPopMatrix();
}

// ──────────────────────────────────
// Plugin API functions: Start, Stop, Enable, Disable, ReceiveMessage
// ──────────────────────────────────


// actual API start function
PLUGIN_API int
XPluginStart(
    char* outName,
    char* outSig,
    char* outDesc)
{

    
    // Initialize the plugin
    // for the font..
    strcpy(outName, "HUDPlugin");
    strcpy(outSig, "xpsdk.examples.hud");
    strcpy(outDesc, "Advanced HUD with landing assist");

    
    strcpy(outSig,  "xpsdk.examples.hud_with_reversed_ladder_zero_and_horizon");
    strcpy(outDesc, "Shows IAS/TAS, PA, RAD ALT, reversed pitch ladder with zero line, "
                    "a fixed ||__|| nose‐line, plus a real horizon that stays level, and debug info.");

    g_menu_container_idx = XPLMAppendMenuItem(
        XPLMFindPluginsMenu(),
        "SVS HUD J",
        0,
        0);

    g_menu_id = XPLMCreateMenu(
        "SVS HUD J",
        XPLMFindPluginsMenu(),
        g_menu_container_idx,
        menu_handler,
        NULL);

    XPLMAppendMenuItem(g_menu_id, "Landing Assist", (void*)"Landing Assist Item", 1);
    XPLMAppendMenuSeparator(g_menu_id);
    
    XPLMAppendMenuItem(g_menu_id, "Seattle to Kelowna", (void*)"S to K", 1);
    XPLMAppendMenuItem(g_menu_id, "Load Custom Waypoints", (void*)"Load Custom Waypoints", 1);
    XPLMAppendMenuItem(g_menu_id, "Show Custom Waypoints", (void*)"Show Custom Waypoints", 1);
    XPLMAppendMenuSeparator(g_menu_id);

    XPLMAppendMenuItem(g_menu_id, "Show zones", (void*)"Zones", 1);
    XPLMAppendMenuItem(g_menu_id, "Load Custom Zone", (void*)"Load Custom Zone", 1);
    XPLMAppendMenuItem(g_menu_id, "Show Custom Zone", (void*)"Show Custom Zone", 1);
    XPLMAppendMenuSeparator(g_menu_id);

    XPLMAppendMenuItem(g_menu_id, "HUD", (void*)"HUD Item", 1);
    XPLMAppendMenuItem(g_menu_id, "Toggle Aircraft Highlight", (void*)"Toggle Aircraft Highlight", 1);

    // If loaded in an aircraft folder, add extra menu item to Aircraft menu
    {
        XPLMMenuID aircraft_menu = XPLMFindAircraftMenu();
        if (aircraft_menu) {
            XPLMAppendMenuItemWithCommand(
                aircraft_menu,
                "Toggle Settings (Command-Based)",
                XPLMFindCommand("sim/operation/toggle_settings_window"));
        }
    }

    // Find DataRefs once for later use
    g_airspeed_ias_ref = XPLMFindDataRef("sim/flightmodel/position/indicated_airspeed");
    g_airspeed_tas_ref = XPLMFindDataRef("sim/flightmodel/position/true_airspeed");
    g_altitude_pa_ref  = XPLMFindDataRef("sim/flightmodel/position/elevation");
    g_radar_alt_ref    = XPLMFindDataRef("sim/flightmodel/position/y_agl");
    g_pitch_ref        = XPLMFindDataRef("sim/flightmodel/position/theta");
    g_roll_ref         = XPLMFindDataRef("sim/flightmodel/position/phi");
    gHeadingRef        = XPLMFindDataRef("sim/flightmodel/position/psi");
    g_aoa_ref = XPLMFindDataRef("sim/flightmodel2/misc/AoA_angle_degrees");
    float ac_lat = XPLMGetDataf(XPLMFindDataRef("sim/flightmodel/position/latitude"));
    float ac_lon = XPLMGetDataf(XPLMFindDataRef("sim/flightmodel/position/longitude"));
    return 1;
}

PLUGIN_API void
XPluginStop(void)
{
    XPLMDestroyMenu(g_menu_id);

    if (g_hud_visible) {
        XPLMUnregisterDrawCallback(
            (XPLMDrawCallback_f)draw_hud_callback,
            xplm_Phase_Window,
            0,
            NULL);
    }
}


PLUGIN_API void
XPluginDisable(void)
{
    // No special action required on disable
}

PLUGIN_API int
XPluginEnable(void)
{
    // Plugin is always enabled successfully
    return 1;
}

PLUGIN_API void
XPluginReceiveMessage(
    XPLMPluginID inFrom,
    int          inMsg,
    void*        inParam)
{
    // Not used in this plugin
}

// ──────────────────────────────────
// functions
// ─────────────────────────────────
// draw seattle city zone

// Draws a 3D volumetric zone with height
void DrawSeattleZone(
    const std::vector<std::tuple<double, double, double>>& points, 
    float base_alt_m = 0.0f, 
    float top_alt_m = 2000.0f,
    bool draw_wireframe = true
) {
    if (points.size() < 3) return;

    std::vector<std::array<float, 3>> base_points;
    std::vector<std::array<float, 3>> top_points;

    // Convert WGS84 to local coordinates (base and top)
    for (const auto& pt : points) {
        double lat = std::get<0>(pt);
        double lon = std::get<1>(pt);
        
        double x_base, y_base, z_base;
        XPLMWorldToLocal(lat, lon, base_alt_m, &x_base, &y_base, &z_base);
        base_points.push_back({ (float)x_base, (float)y_base, (float)z_base });

        double x_top, y_top, z_top;
        XPLMWorldToLocal(lat, lon, top_alt_m, &x_top, &y_top, &z_top);
        top_points.push_back({ (float)x_top, (float)y_top, (float)z_top });
    }

    // Enable transparency and depth
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);

    // ─── 1. Draw Solid Red Faces ───
    glColor4f(1.0f, 0.0f, 0.0f, 0.2f);  // Red, semi-transparent

    // Base (polygon)
    glDisable(GL_CULL_FACE); // Draw both sides of polygons
    glBegin(GL_POLYGON);
    for (const auto& p : base_points) glVertex3fv(p.data());
    glEnd();

    // Top (polygon, now closed!)
    glBegin(GL_POLYGON);
    for (const auto& p : top_points) glVertex3fv(p.data());
    glEnd();

    // Sides (quad strip)
    glBegin(GL_QUAD_STRIP);
    for (size_t i = 0; i < base_points.size(); ++i) {
        glVertex3fv(base_points[i].data());
        glVertex3fv(top_points[i].data());
    }
    // Close the loop
    glVertex3fv(base_points[0].data());
    glVertex3fv(top_points[0].data());
    glEnd();

    // ─── 2. Optional Wireframe ───
    if (draw_wireframe) {
        glColor4f(1.0f, 1.0f, 0.0f, 0.7f);  // Yellow wireframe
        glLineWidth(2.0f);

        // Base outline
        glBegin(GL_LINE_LOOP);
        for (const auto& p : base_points) glVertex3fv(p.data());
        glEnd();

        // Top outline
        glBegin(GL_LINE_LOOP);
        for (const auto& p : top_points) glVertex3fv(p.data());
        glEnd();

        // Vertical connectors
        glBegin(GL_LINES);
        for (size_t i = 0; i < base_points.size(); ++i) {
            glVertex3fv(base_points[i].data());
            glVertex3fv(top_points[i].data());
        }
        glEnd();
        glLineWidth(1.0f);
    }

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
}

// runway distance calculation
double haversine_m(double lat1, double lon1, double lat2, double lon2) {
        const double R = 6371000.0; // Earth radius in meters
        double dLat = (lat2 - lat1) * M_PI / 180.0;
        double dLon = (lon2 - lon1) * M_PI / 180.0;
        double a = sin(dLat/2) * sin(dLat/2) +
                cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
                sin(dLon/2) * sin(dLon/2);
        double c = 2 * atan2(sqrt(a), sqrt(1-a));
        return R * c;
}

// function for drawing boxes
// direction: 0 = dot (circle, green), 1 = up arrow (blue), -1 = down arrow (red)
void DrawLandingBox(
    float box_x, float box_y, float box_z,
    float box_w, float box_h,
    int direction,
    float next_box_x, float next_box_y, float next_box_z,
    float heading_to_next // in degrees
)
{
    // Set box color based on direction
    if (direction == 0) {
        glColor4f(0.0f, 1.0f, 0.0f, 1.0f); // Green
    } else if (direction == 1) {
        glColor4f(0.2f, 0.5f, 1.0f, 1.0f); // Blue
    } else if (direction == -1) {
        glColor4f(1.0f, 0.2f, 0.2f, 1.0f); // Red
    } else {
        glColor4f(1.0f, 1.0f, 0.0f, 1.0f); // Default Yellow
    }

    glLineWidth(3.0f);
    glBegin(GL_LINE_LOOP);
        glVertex3f((float)(box_x - box_w/2), (float)(box_y - box_h/2), (float)box_z);
        glVertex3f((float)(box_x + box_w/2), (float)(box_y - box_h/2), (float)box_z);
        glVertex3f((float)(box_x + box_w/2), (float)(box_y + box_h/2), (float)box_z);
        glVertex3f((float)(box_x - box_w/2), (float)(box_y + box_h/2), (float)box_z);
    glEnd();
    glLineWidth(1.0f);

    float cx = box_x;
    float cy = box_y;
    float cz = box_z;
    float sym_size = box_h * 0.4f;

    // Draw up/down arrow above/below box
    if (direction == 1) {
        // Up arrow above box (blue)
        glColor4f(0.2f, 0.5f, 1.0f, 1.0f);
        float ay = cy + box_h/2 + sym_size * 0.7f;
        glBegin(GL_TRIANGLES);
            glVertex3f(cx, ay + sym_size * 0.5f, cz);
            glVertex3f(cx - sym_size * 0.4f, ay - sym_size * 0.3f, cz);
            glVertex3f(cx + sym_size * 0.4f, ay - sym_size * 0.3f, cz);
        glEnd();
    } else if (direction == -1) {
        // Down arrow below box (red)
        glColor4f(1.0f, 0.2f, 0.2f, 1.0f);
        float ay = cy - box_h/2 - sym_size * 0.7f;
        glBegin(GL_TRIANGLES);
            glVertex3f(cx, ay - sym_size * 0.5f, cz);
            glVertex3f(cx - sym_size * 0.4f, ay + sym_size * 0.3f, cz);
            glVertex3f(cx + sym_size * 0.4f, ay + sym_size * 0.3f, cz);
        glEnd();
    }

    // Draw left/right arrow beside box, pointing to next box
    if (next_box_x != box_x || next_box_y != box_y) {
        float dx = next_box_x - box_x;
        float dy = next_box_y - box_y;
        // If mostly to the right, draw right arrow; if mostly to the left, draw left arrow
        if (fabs(dx) > fabs(dy)) {
            if (dx > 0) {
                // Right arrow
                glColor4f(1.0f, 1.0f, 0.0f, 1.0f);
                float ax = cx + box_w/2 + sym_size * 0.7f;
                glBegin(GL_TRIANGLES);
                    glVertex3f(ax + sym_size * 0.5f, cy, cz);
                    glVertex3f(ax - sym_size * 0.3f, cy - sym_size * 0.4f, cz);
                    glVertex3f(ax - sym_size * 0.3f, cy + sym_size * 0.4f, cz);
                glEnd();
            } else {
                // Left arrow
                glColor4f(1.0f, 1.0f, 0.0f, 1.0f);
                float ax = cx - box_w/2 - sym_size * 0.7f;
                glBegin(GL_TRIANGLES);
                    glVertex3f(ax - sym_size * 0.5f, cy, cz);
                    glVertex3f(ax + sym_size * 0.3f, cy - sym_size * 0.4f, cz);
                    glVertex3f(ax + sym_size * 0.3f, cy + sym_size * 0.4f, cz);
                glEnd();
            }
        }
    }

    // Draw heading in center of box
    char heading_buf[16];
    snprintf(heading_buf, sizeof(heading_buf), "%.0f", heading_to_next);
    // You may want to use your DrawTextWithShadow here for better visibility
    // For 3D text, you need a 3D text drawing function; for now, this is a placeholder:
    // Draw3DText(cx, cy, cz, heading_buf);
}

// Function to load custom zone points from a file
bool LoadCustomZonePoints(const char* filename) {
    g_custom_zone_points.clear();
    std::ifstream infile(filename);
    if (!infile) {
        strcpy(g_zone_status, "Zone file not found");
        return false;
    }
    double lat, lon, alt;
    while (infile >> lat >> lon >> alt) {
        g_custom_zone_points.emplace_back(lat, lon, alt);
    }
    if (g_custom_zone_points.size() < 3) {
        strcpy(g_zone_status, "Too few zone points");
        return false;
    }
    strcpy(g_zone_status, "Zone loaded");
    return true;
}

// ──────────────────────────────────
// Menu handler: handles menu item selections
// ──────────────────────────────────

//callbacks
static float draw_hud_callback(XPLMDrawingPhase, int, void*);
static float draw_landing_assist_callback(XPLMDrawingPhase, int, void*);
static float draw_seattle_to_kelowna_callback(XPLMDrawingPhase inPhase, int inIsBefore, void* inRefcon);
// Update your callback to call DrawSeattleZone with your desired points:
static float draw_seattle_zone_callback(XPLMDrawingPhase inPhase, int inIsBefore, void* inRefcon) {
    if (!g_zones_visible) return 1.0f;

    // Define zone boundaries (latitude, longitude, unused altitude)
    std::vector<std::tuple<double, double, double>> seattle_zone = {
        {47.591114, -122.341964, 0},  // NW
        {47.591565, -122.286117, 0},  // NE
        {47.642684, -122.278412, 0},  // SE
        {47.663445, -122.431298, 0},  // SW
        {47.630000, -122.400000, 0}   
    };

    // Draw from 0m to 2500m altitude
    DrawSeattleZone(seattle_zone, 0.0f, 2500.0f, true);

    return 1.0f;
}

// Callback for drawing the custom zone
static float draw_custom_zone_callback(XPLMDrawingPhase inPhase, int inIsBefore, void* inRefcon) {
    if (!g_zones_visible || g_custom_zone_points.size() < 3) return 1.0f;
    // Draw from lowest to highest alt in file, or use fixed values if you want
    float base_alt = std::get<2>(g_custom_zone_points[0]);
    float top_alt = base_alt + 2500.0f;
    DrawSeattleZone(g_custom_zone_points, base_alt, top_alt, true);
    return 1.0f;
}

static void
menu_handler(
    void* in_menu_ref,
    void* in_item_ref)
{
    const char* item = (const char*)in_item_ref;

    if (!strcmp(item, "Menu Item 2")) {
        XPLMCommandOnce(XPLMFindCommand("sim/operation/toggle_key_shortcuts_window"));
    }
    else if (!strcmp(item, "HUD Item")) {
        // Toggle HUD visibility on/off
        g_hud_visible = !g_hud_visible;

        if (g_hud_visible) {
            XPLMRegisterDrawCallback(
                (XPLMDrawCallback_f)draw_hud_callback,
                xplm_Phase_Window, // note  xplm_Phase_Window for HUD and xplm_Phase_Airplanes for landing line
                0,
                NULL);
        }
        else {
            XPLMUnregisterDrawCallback(
                (XPLMDrawCallback_f)draw_hud_callback,
                xplm_Phase_Window, // note
                0,
                NULL);
        }
    }
    else if (!strcmp(item, "Landing Assist Item")) 
    {
        g_landing_assist_visible = !g_landing_assist_visible;
        radout_init_set = false; // <-- Reset so it will capture new value next time
        if (g_landing_assist_visible) {
            XPLMRegisterDrawCallback(
                (XPLMDrawCallback_f)draw_landing_assist_callback,
                xplm_Phase_Airplanes, // 3D world
                0,
                NULL);
        } else {
            XPLMUnregisterDrawCallback(
                (XPLMDrawCallback_f)draw_landing_assist_callback,
                xplm_Phase_Airplanes,
                0,
                NULL);
        }
    }
    else if(!strcmp(item, "S to K"))
    {
        g_seattle_to_kelowna_visible = !g_seattle_to_kelowna_visible;
        if (g_seattle_to_kelowna_visible) {
            XPLMRegisterDrawCallback(
                (XPLMDrawCallback_f)draw_seattle_to_kelowna_callback,
                xplm_Phase_Airplanes,
                0,
                NULL);
        } else {
            XPLMUnregisterDrawCallback(
                (XPLMDrawCallback_f)draw_seattle_to_kelowna_callback,
                xplm_Phase_Airplanes,
                0,
                NULL);
        }
    }
    else if(!strcmp(item, "Zones"))
    {
        g_zones_visible = !g_zones_visible;
        if (g_zones_visible) {
            XPLMRegisterDrawCallback(
                (XPLMDrawCallback_f)draw_seattle_zone_callback,
                xplm_Phase_Airplanes,
                0,
                NULL);
        } else {
            XPLMUnregisterDrawCallback(
                (XPLMDrawCallback_f)draw_seattle_zone_callback,
                xplm_Phase_Airplanes,
                0,
                NULL);
        }
    }
    else if (!strcmp(item, "Load Custom Waypoints")) {
        // if (LoadCustomWaypoints("C:\\X-Plane 11\\Resources\\plugins\\custom_waypoints.txt")) {
        if (LoadCustomWaypoints("C:\\Users\\fsr_v\\Desktop\\X-Plane 11\\Resources\\plugins\\custom_waypoints.txt")) {
            XPLMDebugString("Custom waypoints loaded successfully.\n");
            strcpy(g_waypoint_status, "Loaded");
        } else {
            XPLMDebugString("Failed to load custom waypoints.\n");
            strcpy(g_waypoint_status, "Failed");
        }
    }
    else if (!strcmp(item, "Show Custom Waypoints")) {
        g_custom_waypoints_visible = !g_custom_waypoints_visible;
        if (g_custom_waypoints_visible) {
            XPLMRegisterDrawCallback(
                (XPLMDrawCallback_f)draw_custom_waypoints_callback,
                xplm_Phase_Airplanes,
                0,
                NULL);
        } else {
            XPLMUnregisterDrawCallback(
                (XPLMDrawCallback_f)draw_custom_waypoints_callback,
                xplm_Phase_Airplanes,
                0,
                NULL);
        }
    }
    else if (!strcmp(item, "Toggle Aircraft Highlight")) {
        g_aircraft_highlight_visible = !g_aircraft_highlight_visible;
        
        if (g_aircraft_highlight_visible) {
            XPLMRegisterDrawCallback(
                draw_aircraft_highlight_callback,
                xplm_Phase_Airplanes,
                0,
                NULL
            );
        } else {
            XPLMUnregisterDrawCallback(
                draw_aircraft_highlight_callback,
                xplm_Phase_Airplanes,
                0,
                NULL
            );
        }
        
    }
    else if (!strcmp(item, "Load Custom Zone")) {
        // "C:\\X-Plane 11\\Resources\\plugins\\custom_zones.txt"
        // "C:\\Users\\fsr_v\\Desktop\\X-Plane 11\\Resources\\plugins\\custom_zone.txt"
        if (LoadCustomZonePoints("C:\\Users\\fsr_v\\Desktop\\X-Plane 11\\Resources\\plugins\\custom_zones.txt")) {
            XPLMDebugString("Custom zone loaded successfully.\n");
        } else {
            XPLMDebugString("Failed to load custom zone.\n");
        }
    }
    else if (!strcmp(item, "Show Custom Zone")) {
        g_zones_visible = !g_zones_visible;
        if (g_zones_visible) {
            XPLMRegisterDrawCallback(
                (XPLMDrawCallback_f)draw_custom_zone_callback,
                xplm_Phase_Airplanes,
                0,
                NULL);
        } else {
            XPLMUnregisterDrawCallback(
                (XPLMDrawCallback_f)draw_custom_zone_callback,
                xplm_Phase_Airplanes,
                0,
                NULL);
        }
    }

    
}
// ──────────────────────────────────
// Draw HUD callback: draws the HUD elements every frame
// ──────────────────────────────────

// ──────────────────────────────
// for drawing CAT III Runway Edge Lines for KSEA 16L/34R
// ──────────────────────────────
static float draw_landing_assist_callback(
    XPLMDrawingPhase inPhase,
    int inIsBefore,
    void* inRefcon)
{
    if (!g_landing_assist_visible) return 1.0f;

    {
        // KSEA 16L/34R endpoints (approximate, WGS84)
        // 16L: 47.4502, -122.3088
        // 34R: 47.4180, -122.3088
        // Runway width: 150 ft = 45.72 m

        // double lat1 = 47.4602, lon1 = -122.3076; // North end, redudent
        // double lat2 = 47.4280, lon2 = -122.3076; // South end, redundant
        // float elev = 410.0f; // Approx field elevation in feet

        // Convert feet to meters for XPLMWorldToLocal
        float elev_m = elev * 0.3048f;

        // Calculate left/right edge offsets (perpendicular to runway heading)
        double heading_deg = 180.0; // 16L/34R is roughly north-south
        double heading_rad = heading_deg * M_PI / 180.0;
        double offset_lat = (45.72 / 2.0) / 111111.0; // meters to degrees latitude
        double offset_lon = (45.72 / 2.0) / (111111.0 * cos(lat1 * M_PI / 180.0));

        // Left edge (west)
        double lat1L = lat1;
        double lon1L = lon1 - offset_lon;
        double lat2L = lat2;
        double lon2L = lon2 - offset_lon;

        // Right edge (east)
        double lat1R = lat1;
        double lon1R = lon1 + offset_lon;
        double lat2R = lat2;
        double lon2R = lon2 + offset_lon;

        // Convert to local OpenGL coordinates
        double x1L, y1L, z1L, x2L, y2L, z2L;
        double x1R, y1R, z1R, x2R, y2R, z2R;
        XPLMWorldToLocal(lat1L, lon1L, elev_m, &x1L, &y1L, &z1L);
        XPLMWorldToLocal(lat2L, lon2L, elev_m, &x2L, &y2L, &z2L);
        XPLMWorldToLocal(lat1R, lon1R, elev_m, &x1R, &y1R, &z1R);
        XPLMWorldToLocal(lat2R, lon2R, elev_m, &x2R, &y2R, &z2R);

        // Draw the two edge lines in 3D
        glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
        glLineWidth(4.0f);
        glBegin(GL_LINES);
        glVertex3f((float)x1L, (float)y1L, (float)z1L);
        glVertex3f((float)x2L, (float)y2L, (float)z2L);
        glVertex3f((float)x1R, (float)y1R, (float)z1R);
        glVertex3f((float)x2R, (float)y2R, (float)z2R);
        glEnd();
        glLineWidth(1.0f);

        // ──────────────────────────────
        // Drawing centerline
        // ──────────────────────────────

        // Calculate the centerline coordinates (midpoint between left and right edges)
        double lat1C = (lat1L + lat1R) / 2.0;
        double lon1C = (lon1L + lon1R) / 2.0;
        double lat2C = (lat2L + lat2R) / 2.0;
        double lon2C = (lon2L + lon2R) / 2.0;

        double x1C, y1C, z1C, x2C, y2C, z2C;
        XPLMWorldToLocal(lat1C, lon1C, elev_m, &x1C, &y1C, &z1C);
        XPLMWorldToLocal(lat2C, lon2C, elev_m, &x2C, &y2C, &z2C);

        // Draw the centerline in a different color (e.g., white)
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f); // White
        glLineWidth(2.0f);  
        glBegin(GL_LINES);
            glVertex3f((float)x1C, (float)y1C, (float)z1C);
            glVertex3f((float)x2C, (float)y2C, (float)z2C);
        glEnd();
        glLineWidth(1.0f);

        // ──────────────────────────────
        // Draw a 2D "fly-through" box above the runway threshold
        // ──────────────────────────────

        // Box center: 100 meters above north threshold (lat1, lon1)
        // Box center: 100 meters above north threshold (lat1, lon1)
        double box_lat = 47.495;
        double box_lon = lon1;
        double box_elev_m = elev_m + 300.0; // 100 meters above runway

        // Convert box center to local coordinates
        double box_x, box_y, box_z;
        XPLMWorldToLocal(box_lat, box_lon, box_elev_m, &box_x, &box_y, &box_z);

        // Box size (meters)
        float box_w = 30.0f;
        float box_h = 30.0f;

        // Draw the box as a wireframe square in the XY plane (facing forward)
        glColor4f(1.0f, 1.0f, 0.0f, 1.0f); // Yellow
        glLineWidth(3.0f);
        glBegin(GL_LINE_LOOP);
            glVertex3f((float)(box_x - box_w/2), (float)(box_y - box_h/2), (float)box_z);
            glVertex3f((float)(box_x + box_w/2), (float)(box_y - box_h/2), (float)box_z);
            glVertex3f((float)(box_x + box_w/2), (float)(box_y + box_h/2), (float)box_z);
            glVertex3f((float)(box_x - box_w/2), (float)(box_y + box_h/2), (float)box_z);
        glEnd();
        glLineWidth(1.0f);

        // Calculate center point between runway endpoints
        double box2_lat = 47.485;
        double box2_lon = lon1;
        double box2_elev_m = elev_m + 200.0; // 300 meters above runway

        // Convert box center to local coordinates
        double box2_x, box2_y, box2_z;
        XPLMWorldToLocal(box2_lat, box2_lon, box2_elev_m, &box2_x, &box2_y, &box2_z);

        // Box size (meters)
        float box2_w = 30.0f;
        float box2_h = 30.0f;

        // Draw the box as a wireframe square in the XY plane (facing forward)
        glColor4f(1.0f, 0.5f, 0.0f, 1.0f); // Orange for distinction
        glLineWidth(3.0f);
        glBegin(GL_LINE_LOOP);
            glVertex3f((float)(box2_x - box2_w/2), (float)(box2_y - box2_h/2), (float)box2_z);
            glVertex3f((float)(box2_x + box2_w/2), (float)(box2_y - box2_h/2), (float)box2_z);
            glVertex3f((float)(box2_x + box2_w/2), (float)(box2_y + box2_h/2), (float)box2_z);
            glVertex3f((float)(box2_x - box2_w/2), (float)(box2_y + box2_h/2), (float)box2_z);
        glEnd();
        glLineWidth(1.0f);
    }

    return 1.0f;
}
// ──────────────────────────────────
// traffic
// ──────────────────────────────────
static int draw_aircraft_highlight_callback(XPLMDrawingPhase inPhase, int inIsBefore, void* inRefcon) {
    if (!g_aircraft_highlight_visible) return 1;

    double user_lat = XPLMGetDataf(XPLMFindDataRef("sim/flightmodel/position/latitude"));
    double user_lon = XPLMGetDataf(XPLMFindDataRef("sim/flightmodel/position/longitude"));
    double user_alt = XPLMGetDataf(XPLMFindDataRef("sim/flightmodel/position/elevation"));
    double user_x, user_y, user_z;
    XPLMWorldToLocal(user_lat, user_lon, user_alt, &user_x, &user_y, &user_z);

    for (int i = 1; i <= 19; i++) {
        char lat_refname[64], lon_refname[64], alt_refname[64];
        snprintf(lat_refname, sizeof(lat_refname), "sim/multiplayer/position/plane%d_lat", i);
        snprintf(lon_refname, sizeof(lon_refname), "sim/multiplayer/position/plane%d_lon", i);
        snprintf(alt_refname, sizeof(alt_refname), "sim/multiplayer/position/plane%d_el", i);

        XPLMDataRef lat_ref = XPLMFindDataRef(lat_refname);
        XPLMDataRef lon_ref = XPLMFindDataRef(lon_refname);
        XPLMDataRef alt_ref = XPLMFindDataRef(alt_refname);

        if (lat_ref && lon_ref && alt_ref) {
            double lat = XPLMGetDataf(lat_ref);
            double lon = XPLMGetDataf(lon_ref);
            double alt = XPLMGetDataf(alt_ref);
            double x, y, z;
            XPLMWorldToLocal(lat, lon, alt, &x, &y, &z);

            float dx = (float)(x - user_x);
            float dy = (float)(y - user_y);
            float dz = (float)(z - user_z);
            float dist_km = sqrtf(dx*dx + dy*dy + dz*dz) / 1000.0f;

            // Determine color based on distance
            float color[3];
            if (dist_km < 1.0f) {
                color[0] = 1.0f; color[1] = 0.0f; color[2] = 0.0f;
            }
            else if (dist_km < 2.0f) {
                float t = (dist_km - 1.0f) / 1.0f;
                color[0] = 1.0f;
                color[1] = 0.5f * t;
                color[2] = 0.0f;
            }
            else {
                color[0] = 0.0f; color[1] = 1.0f; color[2] = 0.0f;
            }

            char tailnum[32] = "N12345";
            draw_highlight_box((float)x, (float)y, (float)z, color, tailnum);

            // ────── TRAIL LOGIC ──────
            // Scale trail length: closer = shorter, farther = longer
            int max_trail_points = 10 + (int)(dist_km * 600.0f);
            if (max_trail_points > 1000) max_trail_points = 1000;
            if (max_trail_points < 10) max_trail_points = 10;
            // ...existing code...

            auto& trail = g_ai_trails[i];
            trail.push_back({(float)x, (float)y, (float)z});
            while ((int)trail.size() > max_trail_points) trail.pop_front();

            // Draw the trail as a line strip
            glColor4f(color[0], color[1], color[2], 0.7f);
            glLineWidth(2.0f);
            glBegin(GL_LINE_STRIP);
            for (const auto& pt : trail) {
                glVertex3f(pt[0], pt[1], pt[2]);
            }
            glEnd();
            glLineWidth(1.0f);
        }
    }
    return 1;
}

// Modified to accept color parameter
static void draw_highlight_box(float x, float y, float z, const float color[3], const char* tailnum) {
    const float size = 5.0f; // Box size in meters
    
    glColor3fv(color); // Use the passed color
    glLineWidth(2.0f);
    
    // Bottom square
    glBegin(GL_LINE_LOOP);
    glVertex3f(x-size, y-size, z-size);
    glVertex3f(x+size, y-size, z-size);
    glVertex3f(x+size, y+size, z-size);
    glVertex3f(x-size, y+size, z-size);
    glEnd();
    
    // Top square
    glBegin(GL_LINE_LOOP);
    glVertex3f(x-size, y-size, z+size);
    glVertex3f(x+size, y-size, z+size);
    glVertex3f(x+size, y+size, z+size);
    glVertex3f(x-size, y+size, z+size);
    glEnd();
    
    // Vertical connectors
    glBegin(GL_LINES);
    glVertex3f(x-size, y-size, z-size);
    glVertex3f(x-size, y-size, z+size);
    glVertex3f(x+size, y-size, z-size);
    glVertex3f(x+size, y-size, z+size);
    glVertex3f(x+size, y+size, z-size);
    glVertex3f(x+size, y+size, z+size);
    glVertex3f(x-size, y+size, z-size);
    glVertex3f(x-size, y+size, z+size);
    glEnd();
}
// ──────────────────────────────────
// for drawing S to K
// ──────────────────────────────────
// --- Replace all highlighted lines with this ---

// Use static/global waypoints so state is preserved between frames
struct Waypoint {
    double lat, lon, alt_m;
    int direction; // 1 = up arrow, 0 = dot, -1 = down arrow
};
std::vector<Waypoint> g_custom_waypoints;

//for custom waypoints
std::vector<Waypoint> g_loaded_waypoints;

bool LoadCustomWaypoints(const char* filename) {
    g_custom_waypoints.clear();
    XPLMDebugString("Trying to load waypoints from: ");
    XPLMDebugString(filename);
    XPLMDebugString("\n");
    std::ifstream infile(filename);
    if (!infile) {
        XPLMDebugString("File not found or could not be opened.\n");
        return false;
    }
    double lat, lon, alt;
    int dir;
    while (infile >> lat >> lon >> alt >> dir) {
        g_custom_waypoints.push_back({lat, lon, alt, dir});
    }
    if (g_custom_waypoints.empty()) {
        XPLMDebugString("File loaded but no waypoints found.\n");
        return false;
    }
    XPLMDebugString("Waypoints loaded successfully.\n");
    return true;
}

// callback for drawing custom waypoints
static float draw_custom_waypoints_callback(
    XPLMDrawingPhase inPhase,
    int inIsBefore,
    void* inRefcon)
{
    if (!g_custom_waypoints_visible || g_custom_waypoints.empty()) return 1.0f;

    // Draw lines and boxes similar to Seattle to Kelowna
    int n = (int)g_custom_waypoints.size();
    std::vector<std::array<double, 3>> box_xyz(n);

    for (int i = 0; i < n; ++i) {
        XPLMWorldToLocal(
            g_custom_waypoints[i].lat,
            g_custom_waypoints[i].lon,
            g_custom_waypoints[i].alt_m,
            &box_xyz[i][0], &box_xyz[i][1], &box_xyz[i][2]);
    }

    // Draw line strip
    glColor4f(0.0f, 1.0f, 1.0f, 0.7f); // Cyan, semi-transparent
    glLineWidth(2.0f);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < n; ++i) {
        glVertex3f((float)box_xyz[i][0], (float)box_xyz[i][1], (float)box_xyz[i][2]);
    }
    glEnd();
    glLineWidth(1.0f);

    // Draw boxes
    for (int i = 0; i < n; ++i) {
        double box_x = box_xyz[i][0];
        double box_y = box_xyz[i][1];
        double box_z = box_xyz[i][2];

        double next_box_x = box_x, next_box_y = box_y, next_box_z = box_z;
        float heading_to_next = 0.0f;
        if (i < n - 1) {
            next_box_x = box_xyz[i+1][0];
            next_box_y = box_xyz[i+1][1];
            next_box_z = box_xyz[i+1][2];
            double dlat = g_custom_waypoints[i+1].lat - g_custom_waypoints[i].lat;
            double dlon = g_custom_waypoints[i+1].lon - g_custom_waypoints[i].lon;
            heading_to_next = (float)(atan2(dlon, dlat) * 180.0 / M_PI);
            if (heading_to_next < 0) heading_to_next += 360.0f;
        }

        DrawLandingBox(
            (float)box_x, (float)box_y, (float)box_z,
            60.0f, 30.0f,
            g_custom_waypoints[i].direction,
            (float)next_box_x, (float)next_box_y, (float)next_box_z,
            heading_to_next
        );
    }

    return 1.0f;
}

// formatted in lat, long, alt_meters, and direction, 1 = up arrow, 0 = dot, -1 = down arrow
static const Waypoint g_waypoints[] = {
    {47.4476, -122.3078, 433 * 0.3048, 1},
    {47.4988, -122.307, 3016 * 0.3048, 1},
    {47.5554, -122.31, 5766 * 0.3048, 1},
    {48.5815, -121.191, 26999 * 0.3048, 1},
    {48.8338, -120.937, 31111 * 0.3048, 0},
    {48.9321, -120.837, 32656 * 0.3048, 0},
    {49.0000, -120.768, 33705 * 0.3048, 0},
    {49.0516, -120.712, 34492 * 0.3048, 0},
    {49.8416, -119.434, 4234 * 0.3048, -1},
    {49.8596, -119.388, 3509 * 0.3048, -1},
    {49.8939, -119.369, 2759 * 0.3048, -1},
    {49.9572, -119.378, 1409 * 0.3048, -1},
};

// Add at file scope:
static const int g_num_waypoints = sizeof(g_waypoints) / sizeof(g_waypoints[0]);
static int g_active_waypoint = -1;
static double g_prev_dist = 1e9;
static bool g_passed_waypoint[sizeof(g_waypoints)/sizeof(g_waypoints[0])] = {0};

static float draw_seattle_to_kelowna_callback(
    XPLMDrawingPhase inPhase,
    int inIsBefore,
    void* inRefcon)
{
    if (!g_seattle_to_kelowna_visible) return 1.0f;

    // Get aircraft position
    float ac_lat = XPLMGetDataf(XPLMFindDataRef("sim/flightmodel/position/latitude"));
    float ac_lon = XPLMGetDataf(XPLMFindDataRef("sim/flightmodel/position/longitude"));

    // If all waypoints passed, do nothing
    bool any_left = false;
    for (int i = 0; i < g_num_waypoints; ++i) {
        if (!g_passed_waypoint[i]) {
            any_left = true;
            break;
        }
    }
    if (!any_left) return 1.0f;

    // If no active waypoint or the current one is passed, pick the closest unpassed waypoint
    if (g_active_waypoint == -1 || g_passed_waypoint[g_active_waypoint]) {
        double min_dist = 1e9;
        int min_idx = -1;
        for (int i = 0; i < g_num_waypoints; ++i) {
            if (g_passed_waypoint[i]) continue;
            double d = haversine_m(ac_lat, ac_lon, g_waypoints[i].lat, g_waypoints[i].lon);
            if (d < min_dist) {
                min_dist = d;
                min_idx = i;
            }
        }
        g_active_waypoint = min_idx;
        g_prev_dist = 1e9;
    }

    // Check distance to active waypoint
    double dist = haversine_m(ac_lat, ac_lon, g_waypoints[g_active_waypoint].lat, g_waypoints[g_active_waypoint].lon);

    // If distance starts increasing (after decreasing), mark as passed and pick next closest
    if (dist < g_prev_dist) {
        g_prev_dist = dist;
    } else if (dist > g_prev_dist + 5.0) { // 5m hysteresis
        g_passed_waypoint[g_active_waypoint] = true;
        g_active_waypoint = -1; // Force picking a new one next frame
        g_prev_dist = 1e9;
        return 1.0f; // Don't draw the just-passed box this frame
    }

    // Flashing logic (1 Hz flash)
    double now = XPLMGetElapsedTime();
    bool flash = ((int)(now * 1.0) % 2) == 0;

    // --- Store all box positions for line drawing ---
    double box_xyz[g_num_waypoints][3];
    for (int i = 0; i < g_num_waypoints; ++i) {
        XPLMWorldToLocal(g_waypoints[i].lat, g_waypoints[i].lon, g_waypoints[i].alt_m, &box_xyz[i][0], &box_xyz[i][1], &box_xyz[i][2]);
    }

    // --- Draw lines connecting the boxes ---
    glColor4f(1.0f, 1.0f, 0.0f, 0.7f); // Yellow, semi-transparent
    glLineWidth(2.0f);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < g_num_waypoints; ++i) {
        glVertex3f((float)box_xyz[i][0], (float)box_xyz[i][1], (float)box_xyz[i][2]);
    }
    glEnd();
    glLineWidth(1.0f);

    // --- Draw the boxes as before ---
    for (int i = 0; i < g_num_waypoints; ++i) {
        double box_x = box_xyz[i][0];
        double box_y = box_xyz[i][1];
        double box_z = box_xyz[i][2];

        // Next box (or repeat current if last)
        double next_box_x = box_x, next_box_y = box_y, next_box_z = box_z;
        float heading_to_next = 0.0f;
        if (i < g_num_waypoints - 1) {
            next_box_x = box_xyz[i+1][0];
            next_box_y = box_xyz[i+1][1];
            next_box_z = box_xyz[i+1][2];
            double dlat = g_waypoints[i+1].lat - g_waypoints[i].lat;
            double dlon = g_waypoints[i+1].lon - g_waypoints[i].lon;
            heading_to_next = (float)(atan2(dlon, dlat) * 180.0 / M_PI);
            if (heading_to_next < 0) heading_to_next += 360.0f;
        }

        if (i == g_active_waypoint) {
            if (flash) {
                DrawLandingBox(
                    (float)box_x, (float)box_y, (float)box_z,
                    60.0f, 30.0f,
                    g_waypoints[i].direction,
                    (float)next_box_x, (float)next_box_y, (float)next_box_z,
                    heading_to_next
                );
            }
        } else {
            DrawLandingBox(
                (float)box_x, (float)box_y, (float)box_z,
                60.0f, 30.0f,
                g_waypoints[i].direction,
                (float)next_box_x, (float)next_box_y, (float)next_box_z,
                heading_to_next
            );
        }
    }

    return 1.0f;
}

static float
draw_hud_callback(
    XPLMDrawingPhase inPhase,
    int              inIsBefore,
    void*            inRefcon)
{

    if (!g_hud_visible) {
        return 1.0f;
    }



    // 1) Setup 2D graphics state (disable fog, textures, lighting; enable alpha blending)
    XPLMSetGraphicsState(
        0,  // fog
        0,  // texture units
        0,  // lighting
        0,  // alpha testing
        1,  // alpha blending
        0,  // depth testing
        0   // depth writing
    );

    // 2) Read flight data from DataRefs
    float ias_knots = 0.0f, tas_knots = 0.0f;
    float pa_ft = 0.0f, radalt_ft = 0.0f, radalt_m = 0.0f;
    float pitch_deg = 0.0f, roll_deg = 0.0f;

    if (g_airspeed_ias_ref) ias_knots = XPLMGetDataf(g_airspeed_ias_ref);
    if (g_airspeed_tas_ref) tas_knots = XPLMGetDataf(g_airspeed_tas_ref);
    if (g_altitude_pa_ref) pa_ft = XPLMGetDataf(g_altitude_pa_ref) * 3.28084f;     // meters to feet
    if (g_radar_alt_ref) radalt_ft = XPLMGetDataf(g_radar_alt_ref) * 3.28084f;  
    if (g_radar_alt_ref) radalt_m = XPLMGetDataf(g_radar_alt_ref);
    if (g_pitch_ref) pitch_deg = XPLMGetDataf(g_pitch_ref);
    if (g_roll_ref) roll_deg = XPLMGetDataf(g_roll_ref);

    // 3) Get screen center coordinates
    int screen_w = 0, screen_h = 0;
    XPLMGetScreenSize(&screen_w, &screen_h);
    float cx = screen_w * 0.5f;
    float cy = screen_h * 0.5f;

    // 3.5) Draw the "real" horizon line (counter-pitch + counter-roll)
    {
        glPushMatrix();

            const float earth_radius_ft = 20925524.9f; // Earth radius in feet (~6371 km)
            float horizon_angle_rad = acosf(earth_radius_ft / (earth_radius_ft + pa_ft));
            float horizon_angle_deg = horizon_angle_rad * (180.0f / 3.14159265f);

            float adjusted_pitch = pitch_deg - horizon_angle_deg;

            // Move origin to screen center
            glTranslatef(cx, cy, 0.0f);

            // Rotate by roll angle around screen center
            glRotatef(roll_deg, 0.0f, 0.0f, 1.0f);

            // Translate vertically by negative pitch offset (in rotated space)
            glTranslatef(0.0f, -((pitch_deg + horizon_angle_deg) * g_ladder_spacing), 0.0f);


            // Draw the horizon line centered at origin (now rotated and translated)
            glColor4f(0.0f, 1.0f, 0.0f, 0.9f);
            glLineWidth(2.0f);
            glBegin(GL_LINES);
                glVertex2f(-cx, 0.0f);
                glVertex2f(cx, 0.0f);
            glEnd();
            glLineWidth(1.0f);

        glPopMatrix();
    };
    
    // ──────────────────────────────
    // 4) Draw reversed pitch ladder lines (+30° to -30°) in 5° increments
    //    with split segments on left and right, gap in middle, with shadows.
    // ──────────────────────────────
    struct LabelInfo 
        {
            float screen_x, screen_y;
            char label[8];
        };

        {
            const float gap = 25.0f;

            // Calculate compass bottom edge y coordinate
            float compass_y = 60.0f;       // same as in your compass code
            float compass_radius = 50.0f;  // same as in your compass code
            const float minY = compass_y + compass_radius + 150.0f;  // 10px padding above compass top edge
            const float maxY = screen_h - 100.0f;  // top boundary for ladder lines, same as compass_top_y

            std::vector<LabelInfo> label_list;

                    glPushMatrix();
                    glTranslatef(cx, cy, 0.0f);
                    glRotatef(roll_deg, 0.0f, 0.0f, 1.0f);

                    for (int deg = -30; deg <= 90; deg += 5) {
                        float y_line = (deg - pitch_deg) * g_ladder_spacing;
                        float abs_y = cy + y_line;

                        // Only draw ladder lines within vertical bounds
                        if (abs_y < minY || abs_y > maxY)
                            continue;

                        float total_len = (deg % 10 == 0) ? 80.0f : 40.0f;
                        float half_len  = total_len * 0.5f;
                        float half_gap  = gap * 0.5f;

                        float x1 = -half_len;
                        float x2 = -half_gap;
                        float x3 = half_gap;
                        float x4 = half_len;

                        // Shadow lines
                        glColor4f(0.0f, 0.0f, 0.0f, 0.5f);
                        glBegin(GL_LINES);
                        glVertex2f(x1 + 1, y_line + 1);
                        glVertex2f(x2 + 1, y_line + 1);
                        glEnd();
                        glBegin(GL_LINES);
                        glVertex2f(x3 + 1, y_line + 1);
                        glVertex2f(x4 + 1, y_line + 1);
                        glEnd();

                        // Main ladder lines
                        glColor4f(0.0f, 1.0f, 0.0f, 0.9f);
                        glBegin(GL_LINES);
                        glVertex2f(x1, y_line);
                        glVertex2f(x2, y_line);
                        glEnd();
                        glBegin(GL_LINES);
                        glVertex2f(x3, y_line);
                        glVertex2f(x4, y_line);
                        glEnd();

                        // Store label position in screen coordinates
                        float x_label = x4 + 5.0f;
                        float y_label = y_line - 6.0f;

                        // Rotate label point by roll
                        float angle_rad = roll_deg * (3.1415926f / 180.0f);
                        float rot_x = cosf(angle_rad) * x_label - sinf(angle_rad) * y_label;
                        float rot_y = sinf(angle_rad) * x_label + cosf(angle_rad) * y_label;

                        LabelInfo info;
                        info.screen_x = cx + rot_x;
                        info.screen_y = cy + rot_y;
                        sprintf_s(info.label, sizeof(info.label), (deg > 0 ? "+%d" : "%d"), deg);
                        label_list.push_back(info);
                    }

                glPopMatrix();

                // Draw labels (after pop, in screen coords)
                for (const auto& label : label_list) {
                    float shadow_color[] = { 0.0f, 0.0f, 0.0f };
                    float text_color[]   = { 0.0f, 1.0f, 0.0f };

                    DrawTextWithShadow(shadow_color, (int)(label.screen_x + 1), (int)(label.screen_y + 1), label.label);
                    DrawTextWithShadow(text_color,   (int)(label.screen_x),     (int)(label.screen_y),     label.label);
                }
        }
            

    // ──────────────────────────────
    // 5) Draw "||__||" fixed nose line at screen center
    // ──────────────────────────────
    {
        glColor4f(0.0f, 1.0f, 0.0f, 0.9f);
        glLineWidth(2.0f);

        // Left vertical line
        glBegin(GL_LINES);
        glVertex2f(cx - 10.0f, cy - 2.0f);
        glVertex2f(cx - 10.0f, cy + 2.0f);
        glEnd();

        // Horizontal line connecting two vertical lines
        glBegin(GL_LINES);
        glVertex2f(cx - 10.0f, cy);
        glVertex2f(cx + 10.0f, cy);
        glEnd();

        // Right vertical line
        glBegin(GL_LINES);
        glVertex2f(cx + 10.0f, cy - 2.0f);
        glVertex2f(cx + 10.0f, cy + 2.0f);
        glEnd();

        glLineWidth(1.0f);
    }

    // ──────────────────────────────
    // 6) Draw Indicated Airspeed (IAS) and True Airspeed (TAS) on the left side
    // ──────────────────────────────
    {
        // Add these before the HUD drawing code (ideally in your update or draw function)
        static XPLMDataRef gClimbRateRef = XPLMFindDataRef("sim/flightmodel/position/vh_ind_fpm"); // ft/min
        static XPLMDataRef gMachRef = XPLMFindDataRef("sim/flightmodel/misc/machno");

        // --- Get values each frame
        float mach = XPLMGetDataf(gMachRef);

        float green[] = { 0.0f, 1.0f, 0.0f };

        // Constants
        float max_airspeed = 488.0f;
        float bar_width = 8.0f;
        float bar_height = 300.0f;
        float scale_x = cx - 300.0f;
        float scale_top_y = cy + bar_height / 2;
        float scale_bottom_y = cy - bar_height / 2;

        // Draw vertical scale line
        glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
        glBegin(GL_LINES);
        glVertex2f(scale_x, scale_bottom_y);
        glVertex2f(scale_x, scale_top_y);
        glEnd();

        // Draw top (max) and bottom (0) labels
        char label_top[16], label_bottom[16];
        sprintf_s(label_top, sizeof(label_top), "%.0f", max_airspeed);
        sprintf_s(label_bottom, sizeof(label_bottom), "0");
        DrawTextWithShadow(green, (int)(scale_x - 30), (int)(scale_top_y - 6), label_top);
        DrawTextWithShadow(green, (int)(scale_x - 20), (int)(scale_bottom_y - 6), label_bottom);

        // Draw IAS indicator (arrow + number in brackets)
        float ias_y = scale_bottom_y + (ias_knots / max_airspeed) * bar_height;
        glBegin(GL_TRIANGLES);
        glVertex2f(scale_x - 15, ias_y);
        glVertex2f(scale_x - 5, ias_y + 6);
        glVertex2f(scale_x - 5, ias_y - 6);
        glEnd();

        char ias_display[32];
        sprintf_s(ias_display, sizeof(ias_display), "(%.0f)", ias_knots);
        DrawTextWithShadow(green, (int)(scale_x - 60), (int)(ias_y - 5), ias_display);

        // Draw low-speed bar (e.g. from 0 to 130 kt)
        float low_speed_top = scale_bottom_y + (130.0f / max_airspeed) * bar_height;
        float low_speed_bottom = scale_bottom_y;
        glBegin(GL_QUADS);
        glVertex2f(scale_x - bar_width, low_speed_bottom);
        glVertex2f(scale_x + bar_width, low_speed_bottom);
        glVertex2f(scale_x + bar_width, low_speed_top);
        glVertex2f(scale_x - bar_width, low_speed_top);
        glEnd();

        // Draw climb rate above the bar
        float climb_rate_fpm = XPLMGetDataf(gClimbRateRef);
        float climb_rate_ms = climb_rate_fpm * 0.00508f;  // Convert ft/min to m/s
        char climb_rate_text[32];
        sprintf_s(climb_rate_text, sizeof(climb_rate_text), "V/S: %.1f m/s", climb_rate_ms);
        DrawTextWithShadow(green, (int)(scale_x - 40), (int)(scale_top_y + 20), climb_rate_text);

        // Draw Mach number below the bar
        char mach_text[32];
        sprintf_s(mach_text, sizeof(mach_text), "Mach %.2f", mach);
        DrawTextWithShadow(green, (int)(scale_x - 40), (int)(scale_bottom_y - 30), mach_text);

        // IAS and TAS on the right side of the bar
        char ias_label[32], tas_label[32];
        sprintf_s(ias_label, sizeof(ias_label), "IAS: %.0f kt", ias_knots);
        sprintf_s(tas_label, sizeof(tas_label), "TAS: %.0f kt", tas_knots);

        float label_x = scale_x + 60.0f;
        float y_ias  = cy + 20.0f;
        float y_tas  = cy - 10.0f;

        DrawTextWithShadow(green, (int)label_x, (int)y_ias, ias_label);
        DrawTextWithShadow(green, (int)label_x, (int)y_tas, tas_label);
    }

    // ──────────────────────────────
    // 7) Draw Pressure Altitude (P ALT) and Radar Altitude (R ALT) on the right side
    // ──────────────────────────────
    {
        char pa_buf[32], radalt_buf[32];
        sprintf_s(pa_buf, sizeof(pa_buf), "P ALT: %.0f ft", pa_ft);
        sprintf_s(radalt_buf, sizeof(radalt_buf), "R ALT: %.0f ft", radalt_ft);

        float right_color[] = { 0.0f, 1.0f, 0.0f };
        float x_right = cx + 200.0f;
        float y_pa    = cy + 20.0f;
        float y_radar = cy - 10.0f;

        DrawTextWithShadow(right_color, (int)x_right, (int)y_pa, pa_buf);
        DrawTextWithShadow(right_color, (int)x_right, (int)y_radar, radalt_buf);

        // Draw V/S 
        static XPLMDataRef gClimbRateRef = XPLMFindDataRef("sim/flightmodel/position/vh_ind_fpm"); // ft/min
        float climb_rate_fpm = XPLMGetDataf(gClimbRateRef);
        float climb_rate_ms = climb_rate_fpm * 0.00508f;  // Convert ft/min to m/s
        char climb_rate_text[32];
        sprintf_s(climb_rate_text, sizeof(climb_rate_text), "V/S: %.1f m/s", climb_rate_ms);
        float y_vs = y_pa + 30.0f; // height control
        DrawTextWithShadow(right_color, (int)x_right, (int)y_vs, climb_rate_text);


        // draw AOA 
        if (g_aoa_ref) {
            float aoa = XPLMGetDataf(g_aoa_ref);
            char aoa_buf[32];
            sprintf_s(aoa_buf, sizeof(aoa_buf), "AOA: %.1f°", aoa);
            float y_aoa = y_pa + 60.0f; // height control 
            DrawTextWithShadow(right_color, (int)x_right, (int)y_aoa, aoa_buf);
        }
    }

    // ──────────────────────────────
    // 8) Draw Debug Information lower left with ladder spacing and slider visualization
    // ──────────────────────────────
    {
        float debug_color[] = { 1.0f, 1.0f, 0.0f }; // Yellow
        int debug_x = 30;
        int debug_y = 80;

        char debug_text[128];

        // Print load status
        sprintf_s(debug_text, sizeof(debug_text), "Waypoint Load: %s", g_waypoint_status);
        DrawTextWithShadow(debug_color, debug_x, debug_y, debug_text);

        // Example: print number of custom waypoints loaded
        sprintf_s(debug_text, sizeof(debug_text), "Custom Waypoints: %zu", g_custom_waypoints.size());
        DrawTextWithShadow(debug_color, debug_x, debug_y + 20, debug_text);
        // debug text for custom zone
        sprintf_s(debug_text, sizeof(debug_text), "Zone Load: %s", g_zone_status);
        DrawTextWithShadow(debug_color, debug_x, debug_y + 40, debug_text);
    }
    // ──────────────────────────────
    // 9) Rotating Compass
    // ──────────────────────────────

    {
        float green[] = { 0.0f, 1.0f, 0.0f };
        float heading_deg = XPLMGetDataf(gHeadingRef);
        float heading_rad = heading_deg * (float)(M_PI / 180.0f);

        float compass_radius = 50.0f;
        float compass_y = 80.0f;  // From bottom
        float compass_center_x = cx;

        // Heading text
        char heading_text[32];
        sprintf_s(heading_text, sizeof(heading_text), "HDG: %.0f°", heading_deg);
        int text_width = static_cast<int>(strlen(heading_text)) * 8;
        float text_y = compass_y + compass_radius + 30.0f;
        DrawTextWithShadow(green, (int)(compass_center_x - text_width / 2 + 10), (int)text_y, heading_text);

        // Outer circle
        glColor4f(0.0f, 1.0f, 0.0f, 0.9f);
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < 64; ++i) {
            float angle = i * 2.0f * (float)M_PI / 64;
            glVertex2f(compass_center_x + cosf(angle) * compass_radius,
                    compass_y + sinf(angle) * compass_radius);
        }
        glEnd();

        // Inner ring for depth
        glColor4f(0.0f, 1.0f, 0.0f, 0.3f);
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < 64; ++i) {
            float angle = i * 2.0f * (float)M_PI / 64;
            glVertex2f(compass_center_x + cosf(angle) * (compass_radius - 4.0f),
                    compass_y + sinf(angle) * (compass_radius - 4.0f));
        }
        glEnd();

        // Tick marks every 30°
        for (int i = 0; i < 360; i += 30) {
            float rel_angle = (i - heading_deg) * (float)(M_PI / 180.0f);
            float x1 = compass_center_x + sinf(rel_angle) * (compass_radius - 2.0f);
            float y1 = compass_y - cosf(rel_angle) * (compass_radius - 2.0f);
            float x2 = compass_center_x + sinf(rel_angle) * (compass_radius + 4.0f);
            float y2 = compass_y - cosf(rel_angle) * (compass_radius + 4.0f);

            glBegin(GL_LINES);
            glVertex2f(x1, y1);
            glVertex2f(x2, y2);
            glEnd();
        }

        // NESW labels
        const char* labels[] = {"N", "E", "S", "W"};
        float angles[] = {0.0f, 90.0f, 180.0f, 270.0f};

        for (int i = 0; i < 4; ++i) {
            float rel_angle = (angles[i] - heading_deg) * (float)(M_PI / 180.0f);
            float x = compass_center_x + sinf(rel_angle) * (compass_radius + 10.0f);
            float y = compass_y - cosf(rel_angle) * (compass_radius + 10.0f);
            DrawTextWithShadow(green, (int)(x - 4), (int)(y - 4), labels[i]);
        }

        // Heading indicator (bold triangle)
        glColor4f(0.0f, 1.0f, 0.0f, 0.9f);
        glBegin(GL_TRIANGLES);
        glVertex2f(compass_center_x, compass_y + compass_radius + 12);
        glVertex2f(compass_center_x - 6, compass_y + compass_radius);
        glVertex2f(compass_center_x + 6, compass_y + compass_radius);
        glEnd();
    }

    // ──────────────────────────────
    // 10) meme
    // ──────────────────────────────

//⠀⢀⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣀⣀⠀⠀⠀
//⢰⡿⠋⠙⠦⣄⡀⠀⠀⠀⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡋⠀⠀⢳⣄⠀
//⠸⣧⠀⠀⠀⠈⣷⣤⣤⠛⠋⠛⣦⠀⠀⠀⠀⠀⠀⠀⣀⠀⢀⣤⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣤⣤⣤⣷⡄⠀⠀⠛⡆
//⠀⠙⣷⠀⠀⠀⠀⠀⡇⠀⠀⣴⠛⠀⠀⠀⠀⠀⣠⣴⡿⠓⠛⠛⢿⣟⠒⢤⣄⠀⠀⠀⠀⢾⡏⠀⠈⢿⣿⠃⠀⠠⠀⢸
//⠀⠀⢸⡄⠀⠀⠚⠋⠁⠀⣼⠁⠀⠀⠀⢀⡴⠛⠛⣋⣤⣄⠀⠠⠤⢤⣀⠀⠈⠳⢦⡀⠀⠈⢳⡀⠀⣾⠃⠀⣀⡀⠀⢺
//⠀⠀⠀⢿⡀⠀⠀⠀⡀⠀⣿⠀⠀⠀⣴⠋⠡⠂⠉⠀⠀⠀⣀⣀⡀⠀⠉⠙⠀⠀⠈⠱⣆⠀⢸⡇⢀⠀⠀⠠⠋⠁⢀⡿
//⠀⠀⠀⠈⢧⡀⠀⠀⠈⣰⠏⠀⠀⡾⠁⠀⠀⠀⢀⣴⣿⣿⣿⣿⣿⣷⣦⡀⠀⠀⠀⠀⠙⡆⠸⣦⡀⠀⠀⠀⢀⣠⠟⠁
//⠀⠀⠀⠀⠀⠙⢦⣤⣴⠟⠀⠀⣸⠁⠀⠀⠀⣠⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡄⠀⠀⠀⠀⢿⡄⠈⠻⣦⣤⠾⠋⠀⠀⠀
//⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⠀⠀⠀⢠⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡄⠀⠀⠀⢸⣧⠀⠀⠀⠀⠀⠀⠀⠀⠀
//⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⠀⠀⠀⣾⣿⣿⣿⡿⠟⠛⠛⠿⣿⣿⣿⣿⣷⡀⠀⠀⢸⡏⠀⠀⠀⠀⠀⠀⠀⠀⠀
//⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠻⡄⠀⣸⣿⣿⡿⠋⠀⠀⠀⠀⠀⠈⠻⣿⣿⣿⣇⠀⠀⣿⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀
//⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣷⠀⢻⣿⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠈⠻⣿⣿⠀⣴⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
//⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⣇⠀⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⠀⢀⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
//⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠛⠶⣄⡀⠀⠀⠀⠀⠀⠀⠀⣠⣴⠞⠿⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
//⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠉⠉⠉⠉⠉⠉⠉⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀


    
    // ──────────────────────────────
    // 11) meatball for landing assist
    // ──────────────────────────────

    // Draw E-Bracket/Meatball indicator (only in landing assist mode)
    if (g_landing_assist_visible) {
        // Position (below AoA/altitude)
        float x_bracket = cx + 220.0f;
        float y_bracket = cy - 100.0f;

        // Bracket size
        float bracket_width = 12.0f;
        float bracket_height = 40.0f;

        // ──────────────────────────────
        // Arrow logic
        // ──────────────────────────────

        // find distance to runway
        // Get aircraft position (add this before using ac_lat/ac_lon)
        float ac_lat = XPLMGetDataf(XPLMFindDataRef("sim/flightmodel/position/latitude"));
        float ac_lon = XPLMGetDataf(XPLMFindDataRef("sim/flightmodel/position/longitude"));
        double dist1 = haversine_m(ac_lat, ac_lon, lat1, lon1);
        double dist2 = haversine_m(ac_lat, ac_lon, lat2, lon2);
        // find the shortest distance to runway
        double runway_dist_m = (dist1 < dist2) ? dist1 : dist2 - 400;

        // Format as meters and feet
        char runway_dist_text[64];
        sprintf_s(runway_dist_text, sizeof(runway_dist_text), "Runway Dist: %.0f m (%.0f ft)",
            runway_dist_m, runway_dist_m * 3.28084);

        // Draw at lower left
        float left_color[] = { 0.0f, 1.0f, 0.0f };
        int x = 30;
        int y = 40;
        DrawTextWithShadow(left_color, x, y, runway_dist_text);

        // Set radout_init only the first frame after assist is enabled
        if (!radout_init_set) {
            radout_init = radalt_m;
            radout_init_set = true;
        }

        // Calculate target altitude based on distance to runway
        double target_alt = radout_init * (runway_dist_m / (runway_dist_m + 1.0)); // Avoid div by zero
        // Compute the vertical deviation (positive = too high, negative = too low)
        double deviation = radalt_m - target_alt;
        
        // Map deviation to arrow offset: center = on path, up = too high, down = too low
        // Clamp deviation to [-radout_init, radout_init] for display
        if (deviation > radout_init) deviation = radout_init;
        if (deviation < -radout_init) deviation = -radout_init;
        float arrow_offset = (float)(-(deviation / radout_init) * bracket_height);
            
        // Draw bracket (fixed)
        glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
        glLineWidth(3.0f);
        glBegin(GL_LINES);
            // Top bar
            glVertex2f(x_bracket - bracket_width, y_bracket + bracket_height);
            glVertex2f(x_bracket + bracket_width, y_bracket + bracket_height);
            // Left vertical
            glVertex2f(x_bracket - bracket_width, y_bracket + bracket_height);
            glVertex2f(x_bracket - bracket_width, y_bracket - bracket_height);
            // Middle bar
            glVertex2f(x_bracket - bracket_width, y_bracket);
            glVertex2f(x_bracket + bracket_width, y_bracket);
            // Bottom bar
            glVertex2f(x_bracket - bracket_width, y_bracket - bracket_height);
            glVertex2f(x_bracket + bracket_width, y_bracket - bracket_height);
        glEnd();

        // Draw arrow (moves up/down)
        glColor4f(1.0f, 1.0f, 0.0f, 1.0f); // Yellow
        glLineWidth(3.0f);
        glBegin(GL_LINES);
            // Arrow stem
            glVertex2f(x_bracket - bracket_width, y_bracket + arrow_offset);
            glVertex2f(x_bracket + bracket_width, y_bracket + arrow_offset);
        glEnd();
        glLineWidth(1.0f);
    }

    // 12) Horizontal meatball for landing assist (line style)
    if (g_landing_assist_visible) {
        // Horizontal line position (below the vertical one)
        float y_horiz = cy - 170.0f;
        float x_horiz = cx + 220.0f;

        float line_half = 40.0f; // half-length of the horizontal line

        // Calculate lateral offset from runway centerline (in meters)
        float ac_lat = XPLMGetDataf(XPLMFindDataRef("sim/flightmodel/position/latitude"));
        float ac_lon = XPLMGetDataf(XPLMFindDataRef("sim/flightmodel/position/longitude"));

        double dist1 = haversine_m(ac_lat, ac_lon, lat1, lon1);
        double dist2 = haversine_m(ac_lat, ac_lon, lat2, lon2);

        // Interpolate centerline longitude at aircraft's latitude
        double frac = (ac_lat - lat2) / (lat1 - lat2);
        double centerline_lon = lon2 + frac * (lon1 - lon2);

        // Lateral offset in meters
        double meters_per_deg_lon = 111320.0 * cos(ac_lat * M_PI / 180.0);
        double lateral_offset_m = (ac_lon - centerline_lon) * meters_per_deg_lon;

        // Clamp and scale for display
        float max_offset_m = 30.0f; // +/- 30 meters = line edge
        if (lateral_offset_m > max_offset_m) lateral_offset_m = max_offset_m;
        if (lateral_offset_m < -max_offset_m) lateral_offset_m = -max_offset_m;
        float arrow_x = x_horiz + (float)(lateral_offset_m / max_offset_m) * line_half;

        // Draw horizontal centerline
        glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
        glLineWidth(3.0f);
        glBegin(GL_LINES);
            glVertex2f(x_horiz - line_half, y_horiz);
            glVertex2f(x_horiz + line_half, y_horiz);
        glEnd();

        // Draw vertical centerline (shows perfect center)
        glColor4f(0.0f, 1.0f, 0.0f, 1.0f); // Green
        glLineWidth(2.0f);
        glBegin(GL_LINES);
            glVertex2f(x_horiz, y_horiz - 14.0f);
            glVertex2f(x_horiz, y_horiz + 14.0f);
        glEnd();

        // Draw vertical "meatball" line
        glColor4f(1.0f, 1.0f, 0.0f, 1.0f); // Yellow
        glLineWidth(3.0f);
        glBegin(GL_LINES);
            glVertex2f(arrow_x, y_horiz - 12.0f);
            glVertex2f(arrow_x, y_horiz + 12.0f);
        glEnd();
        glLineWidth(1.0f);
    }

    // draw text using the baked font texture
    // Inside draw_landing_assist_callback or draw_hud_callback


    return 1.0f;
}
