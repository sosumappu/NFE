import json

import cv2
import numpy as np

# --- Colors (BGR, OpenCV convention) ---
GREEN = (0, 255, 0)  # wall 1 (left)
BLUE = (255, 0, 0)  # wall 2 (right)
TURQUOISE = (235, 180, 0)  # waypoints
GOLD = (0, 215, 255)  # start position / direction
RED = (0, 0, 255)  # start / finish line
WHITE = (255, 255, 255)  # scale line
GRAY = (180, 180, 180)  # live preview while dragging & sketch colour

SAVE_PRECISION = 4  # number of decimals in saved data (ex : 4 <=> 0.1mm precision)


def show(F, titre=""):
    cv2.imshow(titre, F)


def _round(x, ndigits=SAVE_PRECISION):
    return None if x is None else round(x, ndigits)


def _round_pt(p, ndigits=SAVE_PRECISION):
    return None if p is None else (round(p[0], ndigits), round(p[1], ndigits))


class env:
    """
    Interactive tool to design a training track for a self-driving-car
    project: two walls (track limits), a start position + heading, a
    start/finish line (for lap counting), and a pixel->meter scale.

    Controls
    --------
    w : switch to WALL mode    (left-drag = wall1 green, right-drag = wall2 blue)
    s : switch to START mode   (drag from start point to where the car should face)
    f : switch to FINISH mode  (drag to draw the start/finish line, used for laps)
    c : switch to SCALE mode   (drag along a feature of known length, then type
                                the real-world length in m -eters in the console)
    p : switch to WAYPOINT mode(left-drag = draw turquoise waypoints)
    d : switch to SKETCH mode  (left-drag = sketch, used for reference when
                                drawing a layout)
    u : undo last action for the current mode
    ESC / SPACE : finish and close the window
    """

    def __init__(self, template):
        self.template = template

        F = cv2.imread(self.template)
        if F is None:
            raise FileNotFoundError(f"Could not read template image: {template}")
        self.base = F

        self.size = (F.shape[0], F.shape[1])
        self.M = np.zeros(self.size)

        # walls & waypoints (freehand polylines)
        self.wall1 = []
        self.wall2 = []
        self.waypoint = []
        self.wall_events = []  # [('wall1', point), ...] in chronological order, for undo
        self.drawingp = False
        self.drawingm = False

        # general drawing
        self.sketch = []
        self.sketch_events = []

        # start position + heading
        self.start_pos = None  # (x, y)
        self.start_dir = None  # (x, y), point the car faces towards

        # start / finish line (for lap counting)
        self.finish_line = None  # ((x1, y1), (x2, y2))

        # pixel -> meter scale
        self.scale_line = None  # ((x1, y1), (x2, y2))
        self.px_per_m = None  # pixels per meter
        self.scale_factor = None  # meters per pixel -- multiply pixel coords by this

        # interaction state
        self.mode = "wall"
        self.dragging = False
        self.drag_start = None
        self.current_mouse = None

        self.ix = -1
        self.iy = -1

        self.img = self.base.copy()

    # ------------------------------------------------------------------
    # Mouse handling
    # ------------------------------------------------------------------
    def draw_tool(self, event, x, y, flags, param):
        self.current_mouse = (x, y)

        if self.mode in ("wall", "waypoint", "sketch"):
            if event == cv2.EVENT_LBUTTONDOWN:
                self.drawingp = True
            elif event == cv2.EVENT_LBUTTONUP:
                self.drawingp = False
            if event == cv2.EVENT_RBUTTONDOWN:
                self.drawingm = True
            elif event == cv2.EVENT_RBUTTONUP:
                self.drawingm = False

            if self.mode == "waypoint":
                if self.drawingp:
                    self.waypoint.append((x, y))
                    self.wall_events.append(("waypoint", (x, y)))

            elif self.mode == "sketch":
                if self.drawingp:
                    self.sketch.append((x, y))
                    self.sketch_events.append(("sketch", (x, y)))
            else:
                if self.drawingp:
                    self.wall1.append((x, y))
                    self.wall_events.append(("wall1", (x, y)))
                if self.drawingm:
                    self.wall2.append((x, y))
                    self.wall_events.append(("wall2", (x, y)))

        elif self.mode in ("start", "finish", "scale"):
            if event == cv2.EVENT_LBUTTONDOWN:
                self.dragging = True
                self.drag_start = (x, y)
            elif event == cv2.EVENT_LBUTTONUP and self.dragging:
                self.dragging = False
                end = (x, y)
                if self.mode == "start":
                    self.start_pos = self.drag_start
                    self.start_dir = end
                elif self.mode == "finish":
                    self.finish_line = (self.drag_start, end)
                elif self.mode == "scale":
                    self.scale_line = (self.drag_start, end)
                    self._calibrate_scale()

    def _calibrate_scale(self):
        p1, p2 = self.scale_line
        pix_len = float(np.hypot(p2[0] - p1[0], p2[1] - p1[1]))
        if pix_len == 0:
            print("Scale line has zero length, try again.")
            self.scale_line = None
            return
        try:
            meters = float(
                input(
                    f"Scale line is {pix_len:.1f} px long. "
                    "Enter the real-world length it represents, in meters: "
                )
            )
            if meters <= 0:
                raise ValueError
        except ValueError:
            print("Invalid value entered, scale not set.")
            self.scale_line = None
            return

        self.px_per_m = pix_len / meters
        self.scale_factor = meters / pix_len  # meters per pixel
        print(f"Scale set: {self.px_per_m:.2f} px/m ({self.scale_factor:.5f} m/px)")

    # ------------------------------------------------------------------
    # Undo
    # ------------------------------------------------------------------
    def undo(self):
        if self.mode in ("wall", "waypoint"):
            if self.wall_events:
                which, _ = self.wall_events.pop()
                getattr(self, which).pop()
        elif self.mode in ("sketch"):
            if self.sketch_events:
                which, _ = self.sketch_events.pop()
                getattr(self, which).pop()
        elif self.mode == "start":
            self.start_pos = None
            self.start_dir = None
        elif self.mode == "finish":
            self.finish_line = None
        elif self.mode == "scale":
            self.scale_line = None
            self.px_per_m = None
            self.scale_factor = None

    # ------------------------------------------------------------------
    # Rendering
    # ------------------------------------------------------------------
    def render(self):
        img = self.base.copy()

        for p in self.wall1:
            cv2.circle(img, p, 3, GREEN, -1)
        if len(self.wall1) > 1:
            cv2.polylines(img, [np.array(self.wall1)], False, GREEN, 1)

        for p in self.wall2:
            cv2.circle(img, p, 3, BLUE, -1)
        if len(self.wall2) > 1:
            cv2.polylines(img, [np.array(self.wall2)], False, BLUE, 1)

        for p in self.waypoint:
            cv2.circle(img, p, 3, TURQUOISE, -1)
        if len(self.waypoint) > 1:
            cv2.polylines(img, [np.array(self.waypoint)], False, TURQUOISE, 1)

        for p in self.sketch:
            cv2.circle(img, p, 1, GRAY, -1)

        if self.start_pos is not None:
            cv2.circle(img, self.start_pos, 6, GOLD, -1)
            if self.start_dir is not None:
                cv2.arrowedLine(
                    img, self.start_pos, self.start_dir, GOLD, 2, tipLength=0.3
                )

        if self.finish_line is not None:
            cv2.line(img, self.finish_line[0], self.finish_line[1], RED, 3)

        if self.scale_line is not None:
            cv2.line(img, self.scale_line[0], self.scale_line[1], WHITE, 2)
            if self.px_per_m is not None:
                mid = (
                    (self.scale_line[0][0] + self.scale_line[1][0]) // 2,
                    (self.scale_line[0][1] + self.scale_line[1][1]) // 2,
                )
                cv2.putText(
                    img,
                    f"{self.px_per_m:.1f}px/m",
                    mid,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    WHITE,
                    1,
                )

        if (
            self.dragging
            and self.drag_start is not None
            and self.current_mouse is not None
            and self.mode in ("start", "finish", "scale")
        ):
            cv2.line(img, self.drag_start, self.current_mouse, GRAY, 1)

        cv2.putText(
            img,
            f"Mode: {self.mode}  (w/s/f/c/p/d to switch, u undo, ESC/SPACE done)",
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )

        self.img = img

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def draw_walls(self, window_name="Track designer"):
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, self.draw_tool)

        while True:
            self.render()
            cv2.imshow(window_name, self.img)
            key = cv2.waitKey(10) & 0xFF

            if key == 27 or key == 32:  # ESC or SPACE
                break
            elif key == ord("w"):
                self.mode = "wall"
            elif key == ord("p"):
                self.mode = "waypoint"
            elif key == ord("s"):
                self.mode = "start"
            elif key == ord("f"):
                self.mode = "finish"
            elif key == ord("c"):
                self.mode = "scale"
            elif key == ord("d"):
                self.mode = "sketch"
            elif key == ord("u"):
                self.undo()

        cv2.destroyAllWindows()

    # ------------------------------------------------------------------
    # Export helpers
    # ------------------------------------------------------------------
    def get_start_angle(self):
        """Heading of the car in radians (image coords: x right, y down)."""
        if self.start_pos is None or self.start_dir is None:
            return None
        dx = self.start_dir[0] - self.start_pos[0]
        dy = self.start_dir[1] - self.start_pos[1]
        return float(np.arctan2(dy, dx))

    def to_meters(self, point):
        if self.scale_factor is None:
            raise ValueError("Scale not calibrated yet (use mode 'c' to draw it).")
        return (point[0] * self.scale_factor, point[1] * self.scale_factor)

    def get_scaled_data(self):
        """Every drawn element converted from pixels to meters, using the
        calibrated scale_factor (meters per pixel)."""
        if self.scale_factor is None:
            raise ValueError("Scale not calibrated yet (use mode 'c' to draw it).")

        sf = self.scale_factor
        return {
            "wall1": [(x * sf, y * sf) for x, y in self.wall1],
            "wall2": [(x * sf, y * sf) for x, y in self.wall2],
            "start_pos": self.to_meters(self.start_pos) if self.start_pos else None,
            "start_angle_rad": self.get_start_angle(),
            "finish_line": [self.to_meters(p) for p in self.finish_line]
            if self.finish_line
            else None,
            "px_per_m": self.px_per_m,
            "scale_factor_m_per_px": sf,
        }

    def save(self, directory, name):
        """Save the data for use as a simulation environment as well as
        the raw pixel layout + scale to a JSON file, so a track can then
        be later reloaded and edited without redrawing everything.

        directory : str (example : 'DATA/')
        -> the folder where the track will be saved in

        name : str (example : 'track1')
        -> the name of the track (be careful, it will overwrite tracks with the same name)
        """

        sf = self.scale_factor
        """
        data = {
            "inner_wall": [(x * sf, y * sf) for x, y in self.wall1],
            "outer_wall": [(x * sf, y * sf) for x, y in self.wall2],
            "start": {
                "x" : self.start_pos[0] * sf,
                "y" : self.start_pos[1] * sf,
                "yaw_rad" : self.get_start_angle(),
                      },
            "waypoints": [(x * sf, y * sf) for x, y in self.waypoint],
        }

        """
        data = {
            "inner_wall": [_round_pt((x * sf, y * sf)) for x, y in self.wall1],
            "outer_wall": [_round_pt((x * sf, y * sf)) for x, y in self.wall2],
            "start": {
                "x": _round(self.start_pos[0] * sf),
                "y": _round(self.start_pos[1] * sf),
                "yaw_rad": _round(self.get_start_angle()),
            },
            "waypoints": [_round_pt((x * sf, y * sf)) for x, y in self.waypoint],
        }
        with open(directory + "#" + name + ".json", "w") as f:
            json.dump(data, f, indent=2)

        raw_data = {
            "template": self.template,
            "wall1": self.wall1,
            "wall2": self.wall2,
            "waypoint": self.waypoint,
            "start_pos": self.start_pos,
            "start_dir": self.start_dir,
            "finish_line": self.finish_line,
            "scale_line": self.scale_line,
            "px_per_m": self.px_per_m,
            "scale_factor": self.scale_factor,
        }
        with open(directory + "#" + name + "(raw).json", "w") as f:
            json.dump(raw_data, f, indent=2)

    def load(self, directory, name):
        """Load an existing track to vizualise and edit.

        directory : str (example : 'DATA/')
        -> the folder where the track will be saved in

        name : str (example : 'track1')
        -> the name of the track (no need to add the extension '(raw).json')
        """
        with open(directory + "#" + name + "(raw).json", "r") as f:
            data = json.load(f)
        self.wall1 = [tuple(p) for p in data.get("wall1", [])]
        self.wall2 = [tuple(p) for p in data.get("wall2", [])]
        self.waypoint = [tuple(p) for p in data.get("waypoint", [])]
        self.start_pos = tuple(data["start_pos"]) if data.get("start_pos") else None
        self.start_dir = tuple(data["start_dir"]) if data.get("start_dir") else None
        self.finish_line = (
            (tuple(data["finish_line"][0]), tuple(data["finish_line"][1]))
            if data.get("finish_line")
            else None
        )
        self.scale_line = (
            (tuple(data["scale_line"][0]), tuple(data["scale_line"][1]))
            if data.get("scale_line")
            else None
        )
        self.px_per_m = data.get("px_per_m")
        self.scale_factor = data.get("scale_factor")
        self.wall_events = [("wall1", p) for p in self.wall1] + [
            ("wall2", p) for p in self.wall2
        ]


# %%
if __name__ == "__main__":
    e = env("DATA/anthracite.png")
    e.draw_walls()

    if e.scale_factor is not None:
        print(e.get_scaled_data())
    else:
        print("No scale calibrated; skipping meter conversion.")
