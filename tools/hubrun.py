"""Runs the hub test jig on one hub: `hubrun.py <jig port> <hub port> <label> [seconds] [learn]`.

The hub's XIAO runs hub-test. During the jig's frames phase the script steps the hub's flow
input through 20, 30, 33, 37 and 20 C so the frames carry every curve step, then hands the
flow back to the probe. Jig lines print as they come, hub lines prefixed "hub |".
"""
import sys
import threading
import time

import serial

FLOWS = ("20", "30", "33", "37", "20")

jig_port, hub_port, label = sys.argv[1:4]
seconds = int(sys.argv[4]) if len(sys.argv) > 4 else 30
command = "learn" if "learn" in sys.argv[5:] else "run"

jig = serial.Serial(jig_port, 115200, timeout=0.2)
hub = serial.Serial(hub_port, 115200, timeout=0.2)
done = threading.Event()


def echo_hub():
    buf = b""
    while not done.is_set():
        buf += hub.read(4096)
        *lines, buf = buf.split(b"\n")
        for line in lines:
            text = line.decode(errors="replace").rstrip()
            if text and not text.startswith(("W (", "I (")):
                print("hub |", text, flush=True)


def step_flow():
    for flow in FLOWS:
        hub.write(f"flow {flow}\n".encode())
        time.sleep(seconds / len(FLOWS))
    hub.write(b"flow probe\n")


threading.Thread(target=echo_hub, daemon=True).start()
hub.write(b"nodes 1\n")
jig.read(65536)
jig.write(f"{command} {label} {seconds}\n".encode())

buf = b""
while True:
    buf += jig.read(4096)
    *lines, buf = buf.split(b"\n")
    for line in lines:
        text = line.decode(errors="replace").rstrip()
        if "bit " in text:
            continue
        print(text, flush=True)
        if "== 3. frames" in text:
            threading.Thread(target=step_flow, daemon=True).start()
        if text.startswith("######## end"):
            done.set()
            sys.exit(0)
