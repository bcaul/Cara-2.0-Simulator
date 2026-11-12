"""Transmitter window (pygame): reads keyboard + gamepad into a Transmitter and shows
the sticks, switches, decoded channel values (chnVal) and robot state.

Keyboard (click this window first so it has focus):
  W/S A/D     right stick (walk forward/back, right/left; diagonals work)
  Left/Right  left stick X (spin CCW / CW)
  Up/Down     left stick Y (full forward = roll demo, full back = pitch demo)
  1 / 2 / 3   CH5 switch forward / middle / back
  Space       CH6 button (hold 1 s: home -> stand -> sit)
  Z / X / C   CH7 switch forward / middle / back
  + / -       CH8 knob
  Backspace   transmitter on/off
  H           pick the robot up / set it down
  Esc         quit
Gamepad (Xbox layout): sticks = sticks, A = button, RB/LB = CH5 up/down,
  D-pad up/down = CH7, Start = transmitter on/off, Y = pick up / set down.
"""
from __future__ import annotations

import os

os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import pygame  # noqa: E402

STATE_NAMES = {0: "0 not homed", 1: "1 starting (crouched)", 2: "2 IDLE (motors off)", 3: "3 standing"}
W, H = 560, 430
BG, FG, DIM, ACC, OK, BAD = (24, 26, 30), (230, 230, 230), (130, 135, 145), (90, 170, 255), (90, 200, 120), (230, 90, 80)


class TransmitterPanel:
    def __init__(self, tx, use_gamepad=True):
        pygame.init()
        pygame.display.set_caption("CARA transmitter - click here to drive")
        self.screen = pygame.display.set_mode((W, H))
        self.font = pygame.font.SysFont("consolas", 15)
        self.big = pygame.font.SysFont("consolas", 18, bold=True)
        self.tx = tx
        self.pad = None
        if use_gamepad:
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.pad = pygame.joystick.Joystick(0)
                self.pad.init()
        self.quit = False
        self.hand_toggle = False      # set True for one poll when H / Y pressed

    # ------------------------------------------------------------------ input
    def poll(self):
        st = self.tx.state
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                self.quit = True
            elif ev.type == pygame.KEYDOWN:
                k = ev.key
                if k == pygame.K_ESCAPE: self.quit = True
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
            button = button or (self.pad.get_numbuttons() > 0 and self.pad.get_button(0))
        self.tx.set(right_x=rx, right_y=ry, left_x=lx, left_y=ly, button=button)

    def take_hand_toggle(self):
        t, self.hand_toggle = self.hand_toggle, False
        return t

    # ------------------------------------------------------------------ display
    def _text(self, s, x, y, col=FG, font=None):
        self.screen.blit((font or self.font).render(s, True, col), (x, y))

    def _stick(self, x, y, sx, sy, label):
        r = 60
        pygame.draw.rect(self.screen, DIM, (x, y, 2 * r, 2 * r), 1)
        pygame.draw.line(self.screen, (60, 64, 70), (x + r, y), (x + r, y + 2 * r))
        pygame.draw.line(self.screen, (60, 64, 70), (x, y + r), (x + 2 * r, y + r))
        pygame.draw.circle(self.screen, ACC, (int(x + r + sx * (r - 8)), int(y + r - sy * (r - 8))), 8)
        self._text(label, x, y + 2 * r + 4, DIM)

    def _switch(self, x, y, val, label):
        for i, v in enumerate((1, 0, -1)):
            col = ACC if val == v else (60, 64, 70)
            pygame.draw.rect(self.screen, col, (x, y + i * 16, 26, 13))
        self._text(label, x + 34, y + 14, DIM)

    def draw(self, sim, rtf=None):
        st, fw = self.tx.state, sim.fw
        sc = self.screen
        sc.fill(BG)
        self._text("CARA 2.0 transmitter", 16, 12, FG, self.big)
        self._text("ON" if st.on else "OFF (Backspace)", 250, 14, OK if st.on else BAD, self.big)
        pad = f"gamepad: {self.pad.get_name()[:22]}" if self.pad else "gamepad: none (keyboard)"
        self._text(pad, 16, 38, DIM)
        self._stick(30, 64, st.left_x, st.left_y, "left  (arrows)")
        self._stick(190, 64, st.right_x, st.right_y, "right (WASD)")
        self._switch(350, 66, st.sw5, "CH5 1/2/3")
        self._switch(350, 124, st.sw7, "CH7 Z/X/C")
        pygame.draw.circle(sc, OK if st.button else (60, 64, 70), (363, 196), 11)
        self._text("CH6 Space", 384, 188, DIM)
        self._text(f"CH8 knob {st.knob:.0f}", 350, 218, DIM)

        y = 250
        self._text("chnVal " + " ".join(f"{c:+3d}" for c in fw.chnVal), 16, y)
        self._text(f"state  {STATE_NAMES.get(fw.state, fw.state)}", 16, y + 22, FG, self.big)
        hold = min(1.0, fw.btnTime / 1000.0)
        pygame.draw.rect(sc, (60, 64, 70), (16, y + 50, 200, 10))
        pygame.draw.rect(sc, OK, (16, y + 50, int(200 * hold), 10))
        self._text("button hold (1 s)", 226, y + 46, DIM)
        r, p = sim.roll_pitch_deg()
        self._text(f"roll {r:+5.1f}  pitch {p:+5.1f} deg   height {sim.base_height_mm():5.0f} mm", 16, y + 72)
        hand = "held in hand (H to set down)" if sim.hand.active else "on its own (H to pick up)"
        self._text(f"robot  {hand}", 16, y + 94)
        extra = f"   overspeed trips {sim.bank.overspeed_trips}" if sim.bank.overspeed_trips else ""
        self._text(f"sim t {sim.time:6.1f} s" + (f"   x{rtf:.2f} real time" if rtf else "") + extra, 16, y + 116, DIM)
        self._text("Hold Space 1 s: home -> stand.  WASD walk, arrows spin/tilt.", 16, y + 146, DIM)
        pygame.display.flip()

    def close(self):
        pygame.quit()
