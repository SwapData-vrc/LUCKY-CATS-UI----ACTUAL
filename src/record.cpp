#include "record.hpp"
#include "subsystems.hpp"

#include <cmath>
#include <cstdarg>
#include <cstdio>

// records what you drive and prints it back as lemlib code

namespace record {
namespace {

const int MAX = 500;        // about a min of driving
const double PT_IN = 6.0;   // new waypoint after this many in
const double TURN_DEG = 20.0;
const int MS_PER_IN = 50;   // same rough numbers the hand routes use
const int MS_PER_DEG = 6;
const int MIN_MS = 400;
const double PI = 3.14159265358979;
const int MAX_LINES = 160;

// kind: p = point, t = turn, c = claw, l = lift, i = intake
struct Step {
  char kind;
  float x, y, h;
  int a, b;
};

Step steps[MAX];
int n = 0;
bool on = false;
bool full = false;
uint32_t t_start = 0, t_len = 0, t_last = 0;

// everything is relative to where you started, same as the hand routes, so it
// replays from setPose(0,0,0) whichever way the robot is pointing.
double x, y;                 // position in the start frame
double x_start, y_start, h_start;
double lx, ly, lh;

char out[MAX_LINES][LINE_LEN];
int out_n = 0;

int last_claw = 0, last_lift = 0, last_intake = 0, last_spin = 0;
uint32_t lift_started = 0;

double wrap(double d) { return std::fmod(d + 540.0, 360.0) - 180.0; }

// heading relative to where we started
double rel(double h) { return wrap(h - h_start); }

void add(Step s) {
  if (n >= MAX) { full = true; return; }
  steps[n++] = s;
}

// if the lift was still moving when we stopped, log what it did
void flush_lift() {
  if (last_lift != 0 && lift_started != 0) {
    int held = pros::millis() - lift_started;
    if (held > 40) add({'l', 0,0,0, last_lift, held});
  }
  last_lift = 0;
  lift_started = 0;
}

void put(const char *fmt, ...) {
  if (out_n >= MAX_LINES) return;
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(out[out_n], LINE_LEN, fmt, ap);
  va_end(ap);
  out_n++;
}

void dump() {
  out_n = 0;
  put("// %d steps, %.1f s", n, t_len / 1000.0);
  if (full) put("// ran out of room at %d", MAX);
  put("void recorded() {");
  put("  chassis.setPose(0, 0, 0);");

  double px = 0, py = 0, ph = 0;

  for (int i = 0; i < n; i++) {
    Step s = steps[i];

    if (s.kind == 0x63) {
      put("  spinclaw(%d);", s.a);
    }
    else if (s.kind == 0x69) {
      put("  intake.move(%d);", s.a);
      put("  claw_spin.move(%d);", s.b);
    }
    else if (s.kind == 0x6c) {
      put("  lift.move(%d);", s.a);
      put("  pros::delay(%d);", s.b);
      put("  lift.brake();");
    }
    else if (s.kind == 0x74) {
      int ms = std::fabs(wrap(s.h - ph)) * MS_PER_DEG;
      if (ms < MIN_MS) ms = MIN_MS;
      put("  chassis.turnToHeading(%.0f, %d, {}, false);", s.h, ms);
      ph = s.h;
    }
    else {
      double d = std::hypot(s.x - px, s.y - py);
      int ms = d * MS_PER_IN;
      if (ms < MIN_MS) ms = MIN_MS;
      put("  chassis.moveToPoint(%.1f, %.1f, %d, {}, false);", s.x, s.y, ms);
      px = s.x; py = s.y; ph = s.h;
    }
  }
  put("}");

  // also send it down the wire in case a terminal is listening
  for (int i = 0; i < out_n; i++) std::printf("%s\n", out[i]);
  std::fflush(stdout);
}

void stop(const char *why) {
  flush_lift();
  t_len = pros::millis() - t_start;
  on = false;
  std::printf("rec: %s, %d steps. hit RIGHT+Y to print\n", why, n);
}

}

void print_last() {
  if (n == 0) { std::printf("rec: nothing saved\n"); return; }
  dump();
}

int line_count() { return out_n; }
const char *line(int i) { return (i >= 0 && i < out_n) ? out[i] : ""; }

bool active() { return on; }
int count() { return n; }
uint32_t elapsed() { return on ? pros::millis() - t_start : t_len; }

void toggle() {
  if (on) { stop("stopped"); return; }

  lemlib::Pose p = chassis.getPose();
  x_start = p.x;
  y_start = p.y;
  h_start = p.theta;

  x = y = 0;
  lx = ly = lh = 0;

  n = 0;
  out_n = 0;   // drop last run's text or RIGHT+Y shows the old route
  full = false;
  t_start = t_last = pros::millis();
  t_len = 0;
  last_claw = claw_at();
  last_lift = last_intake = last_spin = 0;
  lift_started = 0;
  on = true;
  std::printf("rec: go\n");
}

void update(int lift_cmd, int intake_cmd, int spin_cmd) {
  if (!on) return;

  if (pros::millis() - t_start >= RECORD_MAX_MS) { stop("time up"); return; }

  uint32_t now = pros::millis();
  t_last = now;

  // where we are now, turned into "from where we started" coords
  lemlib::Pose p = chassis.getPose();
  double dx = p.x - x_start;
  double dy = p.y - y_start;
  double r = h_start * PI / 180.0;
  x = dx * std::cos(r) - dy * std::sin(r);
  y = dx * std::sin(r) + dy * std::cos(r);

  double rh = rel(p.theta);

  if (claw_at() != last_claw) {
    last_claw = claw_at();
    add({'c', 0,0,0, last_claw, 0});
  }

  if (intake_cmd != last_intake || spin_cmd != last_spin) {
    last_intake = intake_cmd;
    last_spin = spin_cmd;
    add({'i', 0,0,0, intake_cmd, spin_cmd});
  }

  // only log a lift move once it stops, so we know how long it ran
  if (lift_cmd != last_lift) {
    if (last_lift != 0 && lift_started != 0) {
      int held = now - lift_started;
      if (held > 40) add({'l', 0,0,0, last_lift, held});
    }
    lift_started = lift_cmd != 0 ? now : 0;
    last_lift = lift_cmd;
  }

  double moved = std::hypot(x - lx, y - ly);
  double turned = std::fabs(wrap(rh - lh));

  if (moved >= PT_IN)          add({'p', (float)x, (float)y, (float)rh, 0, 0});
  else if (turned >= TURN_DEG) add({'t', (float)x, (float)y, (float)rh, 0, 0});
  else return;

  lx = x; ly = y; lh = rh;
}

}
