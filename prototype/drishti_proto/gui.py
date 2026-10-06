# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Live tkinter view of the running simulation.

tkinter is in the Python standard library, so this needs no install on
Windows, macOS or a Linux box with python3-tk. That was the deciding factor:
a demo that needs a pip install is a demo that fails in the room.

Three panels, left to right:

  TRUTH        the world as it actually is, which the vehicle cannot see
  BELIEF       the traversability cost the REAL cost function produced from
               what has been observed so far
  STATUS       what the REAL supervisor decided this tick, and why

Watching TRUTH and BELIEF diverge is the point. The vehicle plans over the
middle panel, and the difference between the two is exactly the ignorance that
SPEC.md §6.2 prices as expensive.
"""
from __future__ import annotations

import math
import tkinter as tk
from typing import Optional

from .sim import Simulation
from .supervisor import Action
from .world import (CLASS_DITCH, CLASS_GRASS, CLASS_GRAVEL, CLASS_MUD,
                    CLASS_ROCK, CLASS_TREE, CLASS_WATER)

BG = "#12161c"
PANEL = "#1a1f27"
TEXT = "#d8dee9"
DIM = "#7a8699"

ACTION_COLOUR = {Action.PASS: "#3ddc84", Action.SLOW: "#f2c94c",
                 Action.STOP: "#eb5757"}

TRUTH_COLOUR = {
    CLASS_ROCK: "#5b5f66", CLASS_TREE: "#4a3a24", CLASS_DITCH: "#0a0c10",
    CLASS_MUD: "#6b4f2a", CLASS_WATER: "#2a4a6b", CLASS_GRAVEL: "#6e6a5e",
    CLASS_GRASS: "#33502f",
}


def _cost_colour(cost: float, lethal: bool, observed: bool) -> str:
    """Green (cheap) through amber to red (expensive); grey when unobserved."""
    if lethal:
        return "#d32f2f"
    if not observed:
        return "#2a2f38"          # ignorance: dark, and priced at 0.85
    c = max(0.0, min(1.0, cost))
    if c < 0.5:
        r = int(60 + (255 - 60) * (c / 0.5))
        g = 190
    else:
        r = 235
        g = int(190 - 150 * ((c - 0.5) / 0.5))
    return "#%02x%02x%02x" % (r, g, 60)


class DemoWindow:
    CELL = 7          # screen pixels per world cell

    def __init__(self, sim: Simulation, title: str = "DRISHTI-UGV",
                 speed_ms: int = 60, autostart: bool = True):
        self.sim = sim
        self.speed_ms = speed_ms
        self.running = autostart
        self.trail = []

        w = sim.world
        self.map_w = w.width * self.CELL
        self.map_h = w.height * self.CELL
        panel_w = 330

        self.root = tk.Tk()
        self.root.title("%s — %s" % (title, w.name))
        self.root.configure(bg=BG)

        header = tk.Frame(self.root, bg=BG)
        header.grid(row=0, column=0, columnspan=3, sticky="w", padx=10, pady=(8, 0))
        tk.Label(header, text="DRISHTI-UGV", bg=BG, fg=TEXT,
                 font=("Consolas", 15, "bold")).pack(side="left")
        tk.Label(header, text="  vision-first, GPS-denied terrain reasoning",
                 bg=BG, fg=DIM, font=("Consolas", 10)).pack(side="left")

        self.truth = self._canvas(1, 0, "GROUND TRUTH  (the vehicle cannot see this)")
        self.belief = self._canvas(1, 1, "BELIEF  ·  real traversability cost")

        side = tk.Frame(self.root, bg=PANEL, width=panel_w)
        side.grid(row=1, column=2, sticky="n", padx=(6, 10), pady=6)
        self.status = tk.Label(side, text="", bg=PANEL, fg=TEXT, justify="left",
                               anchor="nw", font=("Consolas", 10), width=42)
        self.status.pack(fill="both", expand=True, padx=10, pady=10)

        controls = tk.Frame(self.root, bg=BG)
        controls.grid(row=2, column=0, columnspan=3, sticky="w", padx=10, pady=(0, 8))
        self.play_button = tk.Button(controls, text="Pause", width=8,
                                     command=self.toggle)
        self.play_button.pack(side="left")
        tk.Button(controls, text="Step", width=8,
                  command=lambda: self.tick(force=True)).pack(side="left", padx=4)
        tk.Button(controls, text="Restart", width=8,
                  command=self.restart).pack(side="left")
        tk.Label(controls, text="   space = pause/resume,  s = single step,  "
                               "r = restart,  q = quit",
                 bg=BG, fg=DIM, font=("Consolas", 9)).pack(side="left")

        self.root.bind("<space>", lambda _e: self.toggle())
        self.root.bind("s", lambda _e: self.tick(force=True))
        self.root.bind("r", lambda _e: self.restart())
        self.root.bind("q", lambda _e: self.root.destroy())

        self._restart_factory = None
        self.draw()
        self.root.after(self.speed_ms, self._loop)

    def _canvas(self, row: int, column: int, caption: str) -> tk.Canvas:
        frame = tk.Frame(self.root, bg=BG)
        frame.grid(row=row, column=column, padx=(10, 0), pady=6, sticky="n")
        tk.Label(frame, text=caption, bg=BG, fg=DIM,
                 font=("Consolas", 9)).pack(anchor="w")
        canvas = tk.Canvas(frame, width=self.map_w, height=self.map_h,
                           bg="#0c0f14", highlightthickness=0)
        canvas.pack()
        return canvas

    # ------------------------------------------------------------- control
    def set_restart_factory(self, factory):
        self._restart_factory = factory

    def toggle(self):
        self.running = not self.running
        self.play_button.config(text="Pause" if self.running else "Play")

    def restart(self):
        if self._restart_factory:
            self.sim = self._restart_factory()
            self.trail = []
            self.running = True
            self.play_button.config(text="Pause")
            self.draw()

    def _loop(self):
        self.tick()
        self.root.after(self.speed_ms, self._loop)

    def tick(self, force: bool = False):
        if (self.running or force) and not self.sim.finished:
            self.sim.step()
            self.trail.append(self.sim.pose[:2])
            self.draw()

    # -------------------------------------------------------------- render
    def draw(self):
        self._draw_truth()
        self._draw_belief()
        self._draw_status()

    def _draw_truth(self):
        c = self.truth
        c.delete("all")
        w = self.sim.world
        s = self.CELL
        for cy in range(w.height):
            for cx in range(w.width):
                cls = w.class_at(cx, cy)
                colour = TRUTH_COLOUR.get(cls)
                if colour is None:
                    h = w.height_at(cx, cy)
                    shade = max(0, min(255, int(70 + h * 90)))
                    colour = "#%02x%02x%02x" % (shade, int(shade * 0.86),
                                                int(shade * 0.62))
                c.create_rectangle(cx * s, cy * s, cx * s + s, cy * s + s,
                                   fill=colour, outline="")
        self._overlay(c)

    def _draw_belief(self):
        c = self.belief
        c.delete("all")
        sim = self.sim
        w = sim.world
        s = self.CELL
        for cy in range(w.height):
            for cx in range(w.width):
                c.create_rectangle(
                    cx * s, cy * s, cx * s + s, cy * s + s,
                    fill=_cost_colour(sim.cost[cy][cx], sim.lethal[cy][cx],
                                      sim.observed[cy][cx]),
                    outline="")
        if sim.path:
            pts = []
            for x, y in sim.path:
                pts += [x / w.resolution * s, y / w.resolution * s]
            if len(pts) >= 4:
                c.create_line(*pts, fill="#4aa3ff", width=2)
        self._overlay(c)

    def _overlay(self, c: tk.Canvas):
        """Trail, vehicle, sensor cone and goal, on either canvas."""
        sim = self.sim
        w = sim.world
        s = self.CELL
        px = lambda v: v / w.resolution * s          # noqa: E731

        if len(self.trail) > 1:
            pts = []
            for x, y in self.trail[-400:]:
                pts += [px(x), px(y)]
            c.create_line(*pts, fill="#8fb8ff", width=1, dash=(2, 2))

        gx, gy = w.goal
        c.create_oval(px(gx) - 7, px(gy) - 7, px(gx) + 7, px(gy) + 7,
                      outline="#3ddc84", width=2)
        c.create_text(px(gx), px(gy) - 14, text="GOAL", fill="#3ddc84",
                      font=("Consolas", 8, "bold"))

        x, y, th = sim.pose
        cone = math.degrees(sim.SENSOR_FOV)
        r = px(sim.SENSOR_RANGE)
        c.create_arc(px(x) - r, px(y) - r, px(x) + r, px(y) + r,
                     start=-math.degrees(th) - cone / 2, extent=cone,
                     outline="", fill="#ffffff", stipple="gray12")

        colour = ACTION_COLOUR.get(sim.telemetry.action, "#ffffff")
        nose = 9
        c.create_polygon(
            px(x) + nose * math.cos(th), px(y) + nose * math.sin(th),
            px(x) + 6 * math.cos(th + 2.4), px(y) + 6 * math.sin(th + 2.4),
            px(x) + 6 * math.cos(th - 2.4), px(y) + 6 * math.sin(th - 2.4),
            fill=colour, outline="#0c0f14")

    def _draw_status(self):
        t = self.sim.telemetry
        sim = self.sim

        bar = {Action.PASS: "PASS", Action.SLOW: "SLOW", Action.STOP: "STOP"}
        lines = [
            "  t = %6.1f s        %s" % (t.t, sim.world.name),
            "",
            "  SUPERVISOR    %s" % bar.get(t.action, "?"),
            "  reason        %s" % (t.reason_text or "-"),
            "",
            "  Nav2 wants    %5.2f m/s  %5.2f rad/s" % t.cmd_in,
            "  wheels get    %5.2f m/s  %5.2f rad/s" % t.cmd_out,
            "  speed limit   %5.2f m/s" % t.v_limit,
            "",
            "  EVIDENCE",
            "   rgb age      %6.2f s" % min(t.rgb_age, 999),
            "   depth age    %6.2f s" % min(t.depth_age, 999),
            "   frame static %6.2f s" % t.rgb_static_for,
            "   confidence   %6.2f" % t.confidence,
            "   nearest haz  %s" % ("   none" if math.isnan(t.nearest_obstacle)
                                    else "%6.2f m" % t.nearest_obstacle),
            "",
            "  MAP",
            "   observed     %5.1f %%" % (100 * t.observed_fraction),
            "   lethal cells %5d" % t.lethal_cells,
            "   to goal      %6.2f m" % t.distance_to_goal,
        ]
        if t.active_faults:
            lines += ["", "  FAULT INJECTED",
                      "   " + ", ".join(t.active_faults)]
        if t.outcome:
            lines += ["", "  OUTCOME: %s" % t.outcome.upper()]

        self.status.config(text="\n".join(lines),
                           fg=ACTION_COLOUR.get(t.action, TEXT))

    def run(self):
        self.root.mainloop()
