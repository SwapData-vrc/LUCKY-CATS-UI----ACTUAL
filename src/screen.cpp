/* Lucky Cats brain screen · 480x240
 *
 * Everything is positioned ABSOLUTELY on the screen. Nothing is a child of a
 * card, because a child that overflows its parent's content area is silently
 * clipped -- which is what cut the bottom off the last build.
 *
 * Every touch target is hit-tested by hand against the same constants used to
 * draw it. LVGL's own input path is not used at all: its indev is serviced by
 * the timer system, and that system does not deliver on this robot. That is
 * also why the routine picker is drawn and driven here rather than being an
 * lv_dropdown -- a real dropdown would open a list nothing could tap.
 *
 * Palette is the old selector's, kept so the two builds look like one robot.
 */

#include "screen.hpp"

#include "autons.hpp"
#include "field_img.h"
#include "liblvgl/lvgl.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "subsystems.hpp"

#include <cmath>
#include <cstdio>

namespace screen {
namespace {

namespace ink {
constexpr uint32_t BG = 0x0b0e13;
constexpr uint32_t CARD = 0x151a21;
constexpr uint32_t EDGE = 0x232a33;
constexpr uint32_t CTRL = 0x1d232b;
constexpr uint32_t SUNK = 0x0e1218;
constexpr uint32_t TEXT = 0xe6edf3;
constexpr uint32_t DIM = 0x7d8590;
constexpr uint32_t ACCENT = 0x4cc9f0;
constexpr uint32_t WARN = 0xd29922;
constexpr uint32_t GOOD = 0x3fb950;
} // namespace ink

// ---------------------------------------------------------------- routines
struct Routine {
  const char* name;
  void (*run)();
};

const Routine ROUTINES[] = {
    {"Do nothing", auton::do_nothing},
    {"My route", auton::my_route},
    {"Blue toggle", auton::blue_toggle},
    {"Example", auton::example},
};
constexpr int COUNT = static_cast<int>(sizeof(ROUTINES) / sizeof(ROUTINES[0]));

int g_selected = 0;
bool g_open = false; // is the picker list dropped down

// ------------------------------------------------------------------ layout
// Verified against 480x240. Every number below is both drawn and hit-tested.
constexpr int SCR_W = 480, SCR_H = 240;

constexpr int LP_X = 8, LP_Y = 8, LP_W = 220, LP_H = 224;
constexpr int RP_X = 236, RP_Y = 8, RP_W = 236, RP_H = 224;

constexpr int SEL_X = LP_X + 8, SEL_Y = LP_Y + 24, SEL_W = LP_W - 16, SEL_H = 36;
constexpr int ITEM_H = 34; // picker list rows, drawn over the RUN button
constexpr int RUN_X = LP_X + 8, RUN_Y = LP_Y + 72, RUN_W = LP_W - 16, RUN_H = 58;

constexpr int FIELD_PX = 176;
constexpr int FIELD_SRC = 232;
constexpr int FIELD_ZOOM = 256 * FIELD_PX / FIELD_SRC;
constexpr int FIELD_X = RP_X + (RP_W - FIELD_PX) / 2;
constexpr int FIELD_Y = RP_Y + 20;
constexpr float FIELD_IN = 144.0f;
constexpr float PX_PER_IN = FIELD_PX / FIELD_IN;

lv_obj_t* g_sel_box = nullptr;
lv_obj_t* g_sel_lbl = nullptr;
lv_obj_t* g_caret = nullptr;
lv_obj_t* g_items[COUNT] = {};
lv_obj_t* g_item_lbl[COUNT] = {};
lv_obj_t* g_run = nullptr;
lv_obj_t* g_run_lbl = nullptr;
lv_obj_t* g_hint = nullptr;
lv_obj_t* g_trail = nullptr;
lv_obj_t* g_bot = nullptr;
lv_obj_t* g_pose = nullptr;
lv_obj_t* g_state = nullptr;
lv_obj_t* g_dot = nullptr;

// --------------------------------------------------------------- the robot
// A rectangle with a visible FRONT, not a dot: heading is half of what a route
// goes wrong by, and a dot cannot show it.
constexpr int BOT_PX = 26;
constexpr float BOT_IN = 15.0f;
LV_ATTRIBUTE_MEM_ALIGN uint8_t g_bot_buf[LV_CANVAS_BUF_SIZE_TRUE_COLOR_ALPHA(BOT_PX, BOT_PX)];

void draw_robot(float theta_deg) {
  if (g_bot == nullptr) return;
  lv_canvas_fill_bg(g_bot, lv_color_hex(0x000000), LV_OPA_TRANSP);

  const float t = theta_deg * 3.14159265f / 180.0f;
  const float s = std::sin(t), c = std::cos(t);
  const float half = BOT_IN * PX_PER_IN * 0.5f;
  const float mid = BOT_PX * 0.5f;

  auto put = [&](float fx, float fy, lv_point_t& p) {
    p.x = static_cast<lv_coord_t>(mid + (fx * c + fy * s));
    p.y = static_cast<lv_coord_t>(mid - (-fx * s + fy * c));
  };

  lv_point_t body[4];
  put(-half, -half, body[0]);
  put(half, -half, body[1]);
  put(half, half, body[2]);
  put(-half, half, body[3]);

  lv_draw_rect_dsc_t fill;
  lv_draw_rect_dsc_init(&fill);
  fill.bg_color = lv_color_hex(ink::TEXT);
  fill.bg_opa = LV_OPA_60;
  fill.border_color = lv_color_hex(ink::TEXT);
  fill.border_width = 1;
  fill.border_opa = LV_OPA_COVER;
  lv_canvas_draw_polygon(g_bot, body, 4, &fill);

  lv_point_t nose[4];
  put(-half, half * 0.45f, nose[0]);
  put(half, half * 0.45f, nose[1]);
  put(half, half, nose[2]);
  put(-half, half, nose[3]);

  lv_draw_rect_dsc_t front;
  lv_draw_rect_dsc_init(&front);
  front.bg_color = lv_color_hex(ink::ACCENT);
  front.bg_opa = LV_OPA_COVER;
  front.border_width = 0;
  lv_canvas_draw_polygon(g_bot, nose, 4, &front);
}

// --------------------------------------------------------------- the trail
constexpr int TRAIL_MAX = 120;
lv_point_t g_pts[TRAIL_MAX];
int g_pts_n = 0;

volatile bool g_running = false;
pros::Task* g_run_task = nullptr;

lv_coord_t px_x(float x_in) { return static_cast<lv_coord_t>(FIELD_PX * 0.5f + x_in * PX_PER_IN); }
lv_coord_t px_y(float y_in) { return static_cast<lv_coord_t>(FIELD_PX * 0.5f - y_in * PX_PER_IN); }

void trail_clear() {
  g_pts_n = 0;
  if (g_trail != nullptr) lv_line_set_points(g_trail, g_pts, 0);
}

void sample() {
  const lemlib::Pose p = chassis.getPose();
  const lv_coord_t x = px_x(static_cast<float>(p.x));
  const lv_coord_t y = px_y(static_cast<float>(p.y));

  if (g_bot != nullptr) {
    lv_obj_set_pos(g_bot, FIELD_X + x - BOT_PX / 2, FIELD_Y + y - BOT_PX / 2);
    draw_robot(static_cast<float>(p.theta));
  }
  if (g_pose != nullptr) {
    char buf[48];
    std::snprintf(buf, sizeof(buf), "X%6.1f  Y%6.1f  H%4.0f", p.x, p.y, p.theta);
    lv_label_set_text(g_pose, buf);
  }

  if (g_pts_n > 0) {
    const lv_coord_t dx = x - g_pts[g_pts_n - 1].x, dy = y - g_pts[g_pts_n - 1].y;
    if (dx * dx + dy * dy < 4) return;
  }
  if (g_pts_n >= TRAIL_MAX) return;

  g_pts[g_pts_n].x = x;
  g_pts[g_pts_n].y = y;
  ++g_pts_n;
  if (g_trail != nullptr && g_pts_n >= 2) lv_line_set_points(g_trail, g_pts, g_pts_n);
}

// ------------------------------------------------------------------ paint
void paint() {
  const bool live = g_running;

  if (g_sel_lbl != nullptr) lv_label_set_text(g_sel_lbl, ROUTINES[g_selected].name);
  if (g_caret != nullptr) lv_label_set_text(g_caret, g_open ? LV_SYMBOL_UP : LV_SYMBOL_DOWN);

  // The picker list only exists while it is open.
  for (int i = 0; i < COUNT; ++i) {
    if (g_items[i] == nullptr) continue;
    // The label is a separate object from its box -- both have to move, or the
    // list opens as four blank rectangles.
    if (g_open) {
      lv_obj_clear_flag(g_items[i], LV_OBJ_FLAG_HIDDEN);
      lv_obj_clear_flag(g_item_lbl[i], LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(g_items[i], LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(g_item_lbl[i], LV_OBJ_FLAG_HIDDEN);
    }

    const bool on = (i == g_selected);
    lv_obj_set_style_bg_color(g_items[i], lv_color_hex(on ? ink::ACCENT : ink::CTRL), LV_PART_MAIN);
    lv_obj_set_style_text_color(g_item_lbl[i], lv_color_hex(on ? ink::BG : ink::TEXT), LV_PART_MAIN);
  }

  // RUN hides under the open list rather than being tappable through it.
  if (g_run != nullptr) {
    if (g_open) {
      lv_obj_add_flag(g_run, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(g_hint, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_clear_flag(g_run, LV_OBJ_FLAG_HIDDEN);
      lv_obj_clear_flag(g_hint, LV_OBJ_FLAG_HIDDEN);
      lv_obj_set_style_bg_color(g_run, lv_color_hex(live ? ink::WARN : ink::ACCENT), LV_PART_MAIN);
      lv_obj_set_style_border_color(g_run, lv_color_hex(live ? ink::WARN : ink::ACCENT), LV_PART_MAIN);
      lv_label_set_text(g_run_lbl, live ? "STOP" : "RUN");
      lv_label_set_text(g_hint, live ? "running - tap to cancel" : "or hold LEFT + press A");
    }
  }

  if (g_state != nullptr) {
    lv_label_set_text(g_state, live ? "DRIVING" : (g_pts_n > 1 ? "LAST RUN" : "READY"));
    lv_obj_set_style_text_color(g_state, lv_color_hex(live ? ink::WARN : ink::DIM), LV_PART_MAIN);
  }
  if (g_dot != nullptr)
    lv_obj_set_style_bg_color(g_dot, lv_color_hex(live ? ink::WARN : ink::GOOD), LV_PART_MAIN);
}

void select(int i) {
  if (i < 0 || i >= COUNT) return;
  g_selected = i;
  std::printf("auton selected: %s\n", ROUTINES[i].name);
}

// ------------------------------------------------------------------ touch
bool in(int x, int y, int rx, int ry, int rw, int rh) {
  return x >= rx && x < rx + rw && y >= ry && y < ry + rh;
}

void poll_touch() {
  static bool was_down = false;

  const pros::screen_touch_status_s_t t = pros::screen::touch_status();
  const bool down = (t.touch_status == pros::E_TOUCH_PRESSED || t.touch_status == pros::E_TOUCH_HELD);

  if (down && !was_down) {
    const int x = t.x, y = t.y;
    std::printf("touch %d,%d\n", x, y); // so a dead touchscreen is visible, not guessed

    if (g_open) {
      // While open, the list owns the whole left panel: a tap that misses an
      // item closes it rather than falling through to whatever is underneath.
      for (int i = 0; i < COUNT; ++i) {
        if (in(x, y, SEL_X, SEL_Y + SEL_H + i * ITEM_H, SEL_W, ITEM_H)) {
          select(i);
          break;
        }
      }
      g_open = false;
      paint();
    } else if (in(x, y, SEL_X, SEL_Y, SEL_W, SEL_H)) {
      g_open = true;
      paint();
    } else if (in(x, y, RUN_X, RUN_Y, RUN_W, RUN_H)) {
      if (g_running) request_stop();
      else request_run();
    }
  }

  was_down = down;
}

// ------------------------------------------------------------------- pump
// Nothing LVGL draws reaches the panel on its own here: liblvgl's own daemon
// is alive but schedules no redraw, and lv_timer_handler() alone did not fix
// it. lv_refr_now() refreshes the display directly and is the one call
// observed to actually put pixels on this screen.
void pump(void*) {
  int tick = 0;
  while (true) {
    if ((tick % 4) == 0) sample();

    // Where the robot actually was, four times a second, only while running. A
    // route that ends in the wrong place is far easier to read as a list of
    // poses than as a memory of the robot moving.
    if (g_running && (tick % 12) == 0) {
      const lemlib::Pose p = chassis.getPose();
      std::printf("  ..     X %.1f  Y %.1f  H %.0f\n", p.x, p.y, p.theta);
    }
    ++tick;
    lv_timer_handler();
    lv_refr_now(NULL);
    poll_touch();
    pros::delay(20);
  }
}

// --------------------------------------------------------------- build bits
lv_obj_t* label(lv_obj_t* p, int x, int y, const char* s, uint32_t c, const lv_font_t* f) {
  lv_obj_t* l = lv_label_create(p);
  lv_label_set_text(l, s);
  lv_obj_set_style_text_color(l, lv_color_hex(c), LV_PART_MAIN);
  lv_obj_set_style_text_font(l, f, LV_PART_MAIN);
  lv_obj_set_pos(l, x, y);
  return l;
}

lv_obj_t* caption(lv_obj_t* p, int x, int y, const char* s) {
  lv_obj_t* l = label(p, x, y, s, ink::DIM, &lv_font_montserrat_12);
  lv_obj_set_style_text_letter_space(l, 2, LV_PART_MAIN);
  return l;
}

lv_obj_t* box(lv_obj_t* scr, int x, int y, int w, int h, uint32_t bg, uint32_t edge, int radius) {
  lv_obj_t* o = lv_obj_create(scr);
  lv_obj_set_pos(o, x, y);
  lv_obj_set_size(o, w, h);
  lv_obj_set_style_bg_color(o, lv_color_hex(bg), LV_PART_MAIN);
  lv_obj_set_style_border_width(o, edge ? 1 : 0, LV_PART_MAIN);
  if (edge) lv_obj_set_style_border_color(o, lv_color_hex(edge), LV_PART_MAIN);
  lv_obj_set_style_radius(o, radius, LV_PART_MAIN);
  lv_obj_set_style_pad_all(o, 0, LV_PART_MAIN);
  lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
  return o;
}

} // namespace

// ------------------------------------------------------------------- build
void init() {
  // LVGL races this task. lv_is_initialized() alone is not enough: PROS
  // registers the display separately and later, so there is a window where
  // LVGL claims ready and lv_scr_act() is still null. Drawing onto null is a
  // data abort at boot.
  int waited = 0;
  while (waited < 5000 && (!lv_is_initialized() || lv_scr_act() == nullptr)) {
    pros::delay(10);
    waited += 10;
  }

  lv_obj_t* scr = lv_scr_act();
  if (scr == nullptr) {
    std::printf("screen: LVGL never came up after %d ms -- selector disabled\n", waited);
    return;
  }
  std::printf("screen: display ready after %d ms\n", waited);

  lv_obj_set_style_bg_color(scr, lv_color_hex(ink::BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

  // Panels are backdrops only -- nothing is parented to them, so nothing can
  // be clipped by them.
  box(scr, LP_X, LP_Y, LP_W, LP_H, ink::CARD, ink::EDGE, 8);
  box(scr, RP_X, RP_Y, RP_W, RP_H, ink::CARD, ink::EDGE, 8);

  caption(scr, LP_X + 10, LP_Y + 8, "AUTONOMOUS");
  caption(scr, RP_X + 10, RP_Y + 6, "FIELD");

  // Picker: closed bar.
  g_sel_box = box(scr, SEL_X, SEL_Y, SEL_W, SEL_H, ink::CTRL, ink::EDGE, 6);
  g_sel_lbl = label(scr, SEL_X + 12, SEL_Y + 9, ROUTINES[0].name, ink::TEXT, &lv_font_montserrat_16);
  g_caret = label(scr, SEL_X + SEL_W - 24, SEL_Y + 10, LV_SYMBOL_DOWN, ink::DIM, &lv_font_montserrat_14);

  // Picker: the dropped list. Built once and hidden, so opening it costs
  // nothing at the moment somebody taps.
  for (int i = 0; i < COUNT; ++i) {
    const int iy = SEL_Y + SEL_H + i * ITEM_H;
    g_items[i] = box(scr, SEL_X, iy, SEL_W, ITEM_H, ink::CTRL, ink::EDGE, 0);
    g_item_lbl[i] = label(scr, SEL_X + 12, iy + 8, ROUTINES[i].name, ink::TEXT, &lv_font_montserrat_16);
    lv_obj_add_flag(g_items[i], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(g_item_lbl[i], LV_OBJ_FLAG_HIDDEN);
  }

  // RUN: filled, and the biggest thing on the screen. It is the only control
  // that makes the robot move, so it is the only one with a solid fill.
  g_run = box(scr, RUN_X, RUN_Y, RUN_W, RUN_H, ink::ACCENT, ink::ACCENT, 8);
  g_run_lbl = lv_label_create(g_run);
  lv_obj_set_style_text_font(g_run_lbl, &lv_font_montserrat_30, LV_PART_MAIN);
  lv_obj_set_style_text_color(g_run_lbl, lv_color_hex(ink::BG), LV_PART_MAIN);
  lv_obj_set_style_text_letter_space(g_run_lbl, 2, LV_PART_MAIN);
  lv_label_set_text(g_run_lbl, "RUN");
  lv_obj_center(g_run_lbl);

  g_hint = label(scr, RUN_X, RUN_Y + RUN_H + 8, "or hold LEFT + press A", ink::DIM,
                 &lv_font_montserrat_12);

  g_dot = box(scr, LP_X + 10, LP_Y + LP_H - 24, 8, 8, ink::GOOD, 0, 4);
  g_state = label(scr, LP_X + 24, LP_Y + LP_H - 28, "READY", ink::DIM, &lv_font_montserrat_14);
  lv_obj_set_style_text_letter_space(g_state, 1, LV_PART_MAIN);

  // Field.
  lv_obj_t* fld = lv_img_create(scr);
  lv_img_set_src(fld, &field_img);
  lv_img_set_zoom(fld, FIELD_ZOOM);
  lv_obj_set_pos(fld, FIELD_X, FIELD_Y);

  g_trail = lv_line_create(scr);
  lv_obj_set_pos(g_trail, FIELD_X, FIELD_Y);
  lv_obj_set_size(g_trail, FIELD_PX, FIELD_PX);
  lv_obj_set_style_line_color(g_trail, lv_color_hex(ink::WARN), LV_PART_MAIN);
  lv_obj_set_style_line_width(g_trail, 2, LV_PART_MAIN);
  lv_obj_set_style_line_rounded(g_trail, true, LV_PART_MAIN);

  g_bot = lv_canvas_create(scr);
  lv_canvas_set_buffer(g_bot, g_bot_buf, BOT_PX, BOT_PX, LV_IMG_CF_TRUE_COLOR_ALPHA);
  lv_obj_set_pos(g_bot, FIELD_X + px_x(0) - BOT_PX / 2, FIELD_Y + px_y(0) - BOT_PX / 2);
  draw_robot(0);

  // Pose, monospaced so the digits stop jittering sideways as they change.
  g_pose = label(scr, RP_X + 10, FIELD_Y + FIELD_PX + 8, "X   0.0  Y   0.0  H   0", ink::TEXT,
                 &lv_font_unscii_8);
  lv_obj_set_style_text_letter_space(g_pose, 1, LV_PART_MAIN);

  paint();
  lv_refr_now(NULL);

  pros::Task pump_task(pump, nullptr, "lvgl_pump");
  std::printf("screen: ready, %d routines\n", COUNT);
}

// -------------------------------------------------------------------- run
void run_selected() {
  trail_clear();
  g_running = true;
  paint();

  const lemlib::Pose before = chassis.getPose();
  std::printf("auton: running %s\n", ROUTINES[g_selected].name);
  std::printf("  start  X %.1f  Y %.1f  H %.0f\n", before.x, before.y, before.theta);

  const uint32_t t0 = pros::millis();
  if (ROUTINES[g_selected].run != nullptr) ROUTINES[g_selected].run();

  const lemlib::Pose after = chassis.getPose();
  std::printf("  end    X %.1f  Y %.1f  H %.0f  (%lu ms)\n", after.x, after.y,
              after.theta, static_cast<unsigned long>(pros::millis() - t0));

  g_running = false;
  paint();
  std::printf("auton: done\n");
}

bool request_run() {
  // A screen button must never drive the robot at an event. Under field
  // control the only thing that starts autonomous is the field.
  if (pros::competition::is_connected()) {
    std::printf("run refused: under competition control\n");
    return false;
  }
  if (g_running) {
    std::printf("run refused: already running\n");
    return false;
  }

  // calibrate() zeroes the pose and starts the odometry task. A route that
  // calls setPose before that finishes has its start position thrown away and
  // then drives from wherever LemLib believes it is -- which looks exactly
  // like a route swerving off on its own.
  if (!chassis_ready) {
    std::printf("run refused: still calibrating -- wait a moment\n");
    return false;
  }

  std::printf("run: starting %s\n", ROUTINES[g_selected].name);
  delete g_run_task;
  g_run_task = new pros::Task([] { run_selected(); }, "auton_run");
  return true;
}

void request_stop() {
  if (!g_running) return;
  chassis.cancelAllMotions();
  g_running = false;
  paint();
  std::printf("run stopped\n");
}

bool running() { return g_running; }

const char* selected_name() { return ROUTINES[g_selected].name; }

} // namespace screen
