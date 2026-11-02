#include "hub_console.h"

#include <stdio.h>
#include <string.h>
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"

#include "fan_bus.h"
#include "hub_mode.h"
#include "probes.h"

// Sets the XIAO's mode when it owns the modes.
static void set_mode(const char *name) {
  int m = hub_mode_from_name(name);
  if (!hub_mode_owned()) printf("the ATtiny owns the modes: use BTN\n");
  else if (m == HUB_MODE_UNKNOWN) printf("? mode \"%s\"  usage: mode auto|low|mid|high\n", name);
  else hub_mode_set(m);
}

// Turns the one-second resend on or off.
static void set_resend(bool on) {
  fan_bus_set_resend(on);
  printf("frames %s\n", on ? "resent every second" : "sent on step changes only");
}

// Runs one console line.
static void run_line(const char *line) {
  float v;
  char name[8];
  int n;
  if (sscanf(line, "nodes %d", &n) == 1 && n >= 1 && n <= FAN_BUS_NODES) {
    fan_bus_set_nodes(n);
    printf("frames carry %d nodes\n", n);
  } else if (sscanf(line, "mode %7s", name) == 1) set_mode(name);
  else if (strcmp(line, "resend on") == 0) set_resend(true);
  else if (strcmp(line, "resend off") == 0) set_resend(false);
  else if (strcmp(line, "flow probe") == 0) {
    probes_flow_from_probe();
    printf("flow from the probe\n");
  } else if (sscanf(line, "flow %f", &v) == 1) {
    probes_fix_flow(v);
    printf("flow fixed at %.2f C\n", v);
  } else printf("? \"%s\"  usage: flow <C> | flow probe | mode <name> | nodes <1-50> | resend on|off\n", line);
}

void hub_console_start() {
  usb_serial_jtag_driver_config_t console = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&console));
  usb_serial_jtag_vfs_use_driver();
}

void hub_console_poll() {
  static char line[32];
  static size_t len = 0;
  uint8_t c;
  while (usb_serial_jtag_read_bytes(&c, 1, 0) == 1) {
    bool end_of_line = c == '\n' || c == '\r';
    if (!end_of_line && len < sizeof(line) - 1) line[len++] = (char) c;
    else if (end_of_line && len > 0) {
      line[len] = 0;
      run_line(line);
      len = 0;
    }
  }
}
