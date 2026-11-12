"""Single-window driver station (pygame): 3D view of the robot + virtual RC transmitter.

The 3D view is rendered offscreen by MuJoCo and drawn into this window, so there is
only one window and every key goes to the transmitter.

Keyboard:
  Space (hold 1 s) CH6 button: home -> stand -> crouch
  W/S A/D     right stick (walk forward/back, left/right; diagonals work)
  Left/Right  left stick X (spin CCW / CW)
  Up/Down     left stick Y (full forward = roll demo, full back = pitch demo)
  1 / 2 / 3   CH5 switch forward / middle / back
  Z / X / C   CH7 switch forward / middle / back
  + / -       CH8 knob
  Backspace   transmitter on/off
  H           pick the robot up / set it down
  F11         fullscreen on/off (the window can also be resized/maximised)
  Esc         leave fullscreen / quit
Mouse in the 3D view: left-drag rotate, wheel zoom.
Gamepad (Xbox layout): sticks = sticks, A = button, RB/LB = CH5 up/down,
  D-pad up/down = CH7, Start = transmitter on/off, Y = pick up / set down.
"""
from __future__ import annotations

import os

os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import mujoco  # noqa: E402
import pygame  # noqa: E402

STATE_NAMES = {0: "0 not homed", 1: "1 crouched", 2: "2 IDLE (motors off)", 3: "3 standing"}
VIEW_W, VIEW_H, SIDE_W = 760, 640, 440       # initial 3D view size, sidebar width
MAX_RENDER_W = 1280                            # render at most this wide, then scale to fit
BG, FG, DIM, ACC, OK, BAD, GRID = ((24, 26, 30), (230, 230, 230), (130, 135, 145), (90, 170, 255),
                                   (90, 200, 120), (230, 90, 80), (60, 64, 70))
HELP = [
    ("Space (hold 1 s)", "home -> stand -> crouch"),
    ("W A S D", "walk"),
    ("Left / Right", "spin"),
    ("Up / Down", "roll / pitch demo"),
    ("1  2  3", "step in place / normal / low"),
    ("Z or C + Space", "kinematicsDemo / jump"),
    ("Backspace", "transmitter on/off"),
    ("H", "pick up / set down"),
    ("mouse", "drag = rotate, wheel = zoom"),
    ("F11", "fullscreen"),
]


class TransmitterPanel:
    def __init__(self, tx, sim=None, use_gamepad=True):
        pygame.init()
        pygame.display.set_caption("CARA 2.0 simulator")
        self.screen = pygame.display.set_mode((VIEW_W + SIDE_W, VIEW_H), pygame.RESIZABLE)
        self.fullscreen = False
        self.view_w, self.view_h = VIEW_W, VIEW_H
        self.font = pygame.font.SysFont("consolas", 15)
        self.big = pygame.font.SysFont("consolas", 18, bold=True)
        self.huge = pygame.font.SysFont("consolas", 22, bold=True)
        self.tx = tx
        self.pad = None
        if use_gamepad:
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.pad = pygame.joystick.Joystick(0)
                self.pad.init()
        self.quit = False
        self.hand_toggle = False
        # 3D view
        self.renderer = None
        if sim is not None:
            from ..model.lod import load_display_model
            m = load_display_model()          # same bodies as sim.m, low-poly meshes (fast to draw)
            m.vis.global_.offwidth = max(m.vis.global_.offwidth, MAX_RENDER_W)
            m.vis.global_.offheight = max(m.vis.global_.offheight, 1440)
            self._disp_model = m
            self._make_renderer()
            self.cam = mujoco.MjvCamera()
            self.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            self.cam.trackbodyid = m.body("base").id
            self.cam.distance, self.cam.azimuth, self.cam.elevation = 1.5, 135.0, -18.0
        self._drag = None

    # ------------------------------------------------------------------ window
    def _make_renderer(self):
        if self.renderer is not None:
            self.renderer.close()
        rw = min(self.view_w, MAX_RENDER_W)
        rh = min(1440, max(1, round(self.view_h * rw / self.view_w)))
        self.render_size = (rw, rh)
        self.renderer = mujoco.Renderer(self._disp_model, rh, rw)

    def _resized(self):
        self.screen = pygame.display.get_surface()
        w, h = self.screen.get_size()
        self.view_w, self.view_h = max(200, w - SIDE_W), max(200, h)
        if self.renderer is not None:
            self._make_renderer()

    def toggle_fullscreen(self):
        self.fullscreen = not self.fullscreen
        if self.fullscreen:
            pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        else:
            pygame.display.set_mode((VIEW_W + SIDE_W, VIEW_H), pygame.RESIZABLE)
        self._resized()

    # ------------------------------------------------------------------ input
    def poll(self):
        st = self.tx.state
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                self.quit = True
            elif ev.type in (pygame.VIDEORESIZE, pygame.WINDOWSIZECHANGED):
                self._resized()
            elif ev.type == pygame.KEYDOWN:
                k = ev.key
                if k == pygame.K_ESCAPE:
                    if self.fullscreen: self.toggle_fullscreen()
                    else: self.quit = True
                elif k == pygame.K_F11: self.toggle_fullscreen()
                elif k == pygame.K_1: self.tx.set(sw5=1)
                elif k == pygame.K_2: self.tx.set(sw5=0)
                elif k == pygame.K_3: self.tx.set(sw5=-1)
                elif k == pygame.K_z: self.tx.set(sw7=1)
                elif k == pygame.K_x: self.tx.set(sw7=0)
                elif k == pygame.K_c: self.tx.set(sw7=-1)
                elif k in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS): self.tx.set(knob=min(100, st.knob + 10))
                elif k in (pygame.K_MINUS, pygame.K_KP_MINUS): self.tx.set(knob=max(0, st.knob - 10))
                elif k == pygame.K_BACKSPACE: self.tx.set(on=not st.on)
                elif k == pygame.K_h: self.hand_toggle = True
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1 and ev.pos[0] < self.view_w:
                self._drag = ev.pos
            elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
                self._drag = None
            elif ev.type == pygame.MOUSEMOTION and self._drag and self.renderer:
                dx, dy = ev.pos[0] - self._drag[0], ev.pos[1] - self._drag[1]
                self._drag = ev.pos
                self.cam.azimuth -= 0.4 * dx
                self.cam.elevation = max(-89.0, min(10.0, self.cam.elevation - 0.4 * dy))
            elif ev.type == pygame.MOUSEWHEEL and self.renderer:
                self.cam.distance = max(0.4, min(6.0, self.cam.distance * (0.9 ** ev.y)))
            elif ev.type == pygame.JOYBUTTONDOWN:
                b = ev.button
                if b == 5: self.tx.set(sw5=min(1, st.sw5 + 1))
                elif b == 4: self.tx.set(sw5=max(-1, st.sw5 - 1))
                elif b == 7: self.tx.set(on=not st.on)
                elif b == 3: self.hand_toggle = True
            elif ev.type == pygame.JOYHATMOTION:
                if ev.value[1] == 1: self.tx.set(sw7=min(1, st.sw7 + 1))
                elif ev.value[1] == -1: self.tx.set(sw7=max(-1, st.sw7 - 1))
            elif ev.type == pygame.JOYDEVICEADDED and self.pad is None:
                self.pad = pygame.joystick.Joystick(ev.device_index)
                self.pad.init()

        keys = pygame.key.get_pressed()

        def kaxis(pos, neg):
            return float(keys[pos]) - float(keys[neg])
        rx, ry = kaxis(pygame.K_d, pygame.K_a), kaxis(pygame.K_w, pygame.K_s)
        lx, ly = kaxis(pygame.K_RIGHT, pygame.K_LEFT), kaxis(pygame.K_UP, pygame.K_DOWN)
        button = bool(keys[pygame.K_SPACE])
        if self.pad is not None:
            def ax(i, inv=False):
                if i >= self.pad.get_numaxes():
                    return 0.0
                val = self.pad.get_axis(i) * (-1 if inv else 1)
                return 0.0 if abs(val) < 0.12 else val
            lx = lx or ax(0); ly = ly or ax(1, True); rx = rx or ax(2); ry = ry or ax(3, True)
            button = button or bool(self.pad.get_numbuttons() > 0 and self.pad.get_button(0))
        self.tx.set(right_x=rx, right_y=ry, left_x=lx, left_y=ly, button=button)

    def take_hand_toggle(self):
        t, self.hand_toggle = self.hand_toggle, False
        return t

    # ------------------------------------------------------------------ display
    def _text(self, s, x, y, col=FG, font=None):
        self.screen.blit((font or self.font).render(s, True, col), (x, y))

    def _stick(self, x, y, sx, sy, label):
        r = 55
        pygame.draw.rect(self.screen, DIM, (x, y, 2 * r, 2 * r), 1)
        pygame.draw.line(self.screen, GRID, (x + r, y), (x + r, y + 2 * r))
        pygame.draw.line(self.screen, GRID, (x, y + r), (x + 2 * r, y + r))
        pygame.draw.circle(self.screen, ACC, (int(x + r + sx * (r - 8)), int(y + r - sy * (r - 8))), 8)
        self._text(label, x, y + 2 * r + 4, DIM)

    def _switch(self, x, y, val, label):
        for i, v in enumerate((1, 0, -1)):
            pygame.draw.rect(self.screen, ACC if val == v else GRID, (x, y + i * 15, 24, 12))
        self._text(label, x + 32, y + 13, DIM)

    def _draw_view(self, sim):
        if self.renderer is None:
            return
        self.renderer.update_scene(sim.d, self.cam)
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0        # shadows cost ~25 ms/frame
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
        img = self.renderer.render()
        surf = pygame.image.frombuffer(img.tobytes(), self.render_size, "RGB")
        if self.render_size != (self.view_w, self.view_h):
            surf = pygame.transform.smoothscale(surf, (self.view_w, self.view_h))
        self.screen.blit(surf, (0, 0))
        fw = sim.fw
        msg = None
        if not self.tx.state.on:
            msg = "Transmitter OFF - press Backspace"
        elif fw.state == 0:
            msg = "Hold SPACE for 1 s to home the robot"
        elif fw.state == 1:
            msg = "Hold SPACE for 1 s to stand up"
        elif fw.state == 2:
            msg = "Motors off - press 1 to re-arm"
        if msg:
            surf = self.huge.render(msg, True, (255, 255, 255))
            box = surf.get_rect(center=(self.view_w // 2, 34))
            pygame.draw.rect(self.screen, (0, 0, 0), box.inflate(24, 14), border_radius=6)
            self.screen.blit(surf, box)

    def draw(self, sim, rtf=None):
        st, fw = self.tx.state, sim.fw
        sc = self.screen
        sc.fill(BG)
        self._draw_view(sim)
        x0 = self.view_w + 16
        self._text("Transmitter", x0, 12, FG, self.big)
        self._text("ON" if st.on else "OFF", x0 + 140, 12, OK if st.on else BAD, self.big)
        pad = f"gamepad: {self.pad.get_name()[:26]}" if self.pad else "gamepad: none (keyboard)"
        self._text(pad, x0, 36, DIM)
        self._stick(x0, 60, st.left_x, st.left_y, "left  (arrows)")
        self._stick(x0 + 140, 60, st.right_x, st.right_y, "right (WASD)")
        self._switch(x0 + 280, 60, st.sw5, "CH5 1/2/3")
        self._switch(x0 + 280, 114, st.sw7, "CH7 Z/X/C")
        pygame.draw.circle(sc, OK if st.button else GRID, (x0 + 292, 186), 10)
        self._text("CH6 Space", x0 + 312, 178, DIM)
        y = 205
        self._text("chnVal " + " ".join(f"{c:+d}" for c in fw.chnVal), x0, y, DIM)
        self._text(f"state  {STATE_NAMES.get(fw.state, fw.state)}", x0, y + 22, FG, self.big)
        hold = min(1.0, fw.btnTime / 1000.0)
        pygame.draw.rect(sc, GRID, (x0, y + 50, 200, 10))
        pygame.draw.rect(sc, OK, (x0, y + 50, int(200 * hold), 10))
        self._text("button hold (1 s)", x0 + 210, y + 46, DIM)
        r, p = sim.roll_pitch_deg()
        self._text(f"roll {r:+5.1f}  pitch {p:+5.1f} deg", x0, y + 70)
        self._text(f"height {sim.base_height_mm():4.0f} mm   " + ("held in hand" if sim.hand.active else "on its own"),
                   x0, y + 90)
        extra = f"  overspeed {sim.bank.overspeed_trips}" if sim.bank.overspeed_trips else ""
        self._text(f"sim t {sim.time:6.1f} s" + (f"  x{rtf:.2f} real time" if rtf else "") + extra, x0, y + 110, DIM)
        y = 345
        self._text("Controls", x0, y, FG, self.big)
        for i, (k, what) in enumerate(HELP):
            self._text(k, x0, y + 26 + i * 21, ACC)
            self._text(what, x0 + 150, y + 26 + i * 21)
        pygame.display.flip()

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
        pygame.quit()
