#!/usr/bin/env python3
"""
bag_to_vdb_gui.py
Tkinter GUI front-end for the bag-to-vdb Docker pipeline.
"""

import subprocess
import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, font, messagebox, scrolledtext, ttk

BG          = "#1e1e2e"
SURFACE     = "#252535"
SURFACE2    = "#2d2d42"
BORDER      = "#3a3a52"
TEXT        = "#cdd6f4"
TEXT_MUTED  = "#7f849c"
ACCENT      = "#89b4fa"
SUCCESS     = "#a6e3a1"
WARNING     = "#f9e2af"
ERROR       = "#f38ba8"

BTN_RUN     = "#40a02b"
BTN_RUN_H   = "#2d7a1e"
BTN_STOP    = "#d20f39"
BTN_STOP_H  = "#a30d2c"
BTN_BUILD   = "#04a5e5"
BTN_BUILD_H = "#0390c8"
BTN_COPY    = "#8839ef"
BTN_COPY_H  = "#6c2ec0"

MONO_FAMILIES = ("Cascadia Code", "JetBrains Mono", "Consolas", "Courier New")


def mono_font(size=10, bold=False):
    for fam in MONO_FAMILIES:
        try:
            f = font.Font(family=fam, size=size,
                          weight="bold" if bold else "normal")
            return f
        except Exception:
            pass
    return font.Font(family="Courier New", size=size,
                     weight="bold" if bold else "normal")


def styled_label(parent, text, size=10, color=TEXT, bold=False, **kw):
    return tk.Label(parent, text=text, bg=parent["bg"], fg=color,
                    font=mono_font(size, bold), **kw)


def styled_entry(parent, textvariable, width=40, **kw):
    return tk.Entry(parent, textvariable=textvariable, width=width,
                    bg=SURFACE2, fg=TEXT, insertbackground=TEXT,
                    relief="flat", bd=4,
                    highlightthickness=1, highlightbackground=BORDER,
                    highlightcolor=ACCENT,
                    font=mono_font(10), **kw)


def styled_button(parent, text, command, bg=SURFACE2, fg=TEXT,
                  hover_bg=BORDER, width=None, **kw):
    btn = tk.Button(parent, text=text, command=command,
                    bg=bg, fg=fg, activebackground=hover_bg, activeforeground=fg,
                    relief="flat", bd=0, padx=10, pady=6,
                    font=mono_font(10, bold=True),
                    cursor="hand2", **kw)
    if width:
        btn.config(width=width)
    btn.bind("<Enter>", lambda e: btn.config(bg=hover_bg))
    btn.bind("<Leave>", lambda e: btn.config(bg=bg))
    return btn


def section_frame(parent, title):
    outer = tk.Frame(parent, bg=SURFACE, bd=0,
                     highlightthickness=1, highlightbackground=BORDER)
    header = tk.Frame(outer, bg=SURFACE2, padx=10, pady=6)
    header.pack(fill="x")
    styled_label(header, "  " + title, size=10, color=ACCENT, bold=True).pack(side="left")
    inner = tk.Frame(outer, bg=SURFACE, padx=14, pady=10)
    inner.pack(fill="x")
    return outer, inner


def row_frame(parent):
    f = tk.Frame(parent, bg=parent["bg"])
    f.pack(fill="x", pady=3)
    return f


def tooltip(widget, text):
    tip = None

    def show(e):
        nonlocal tip
        tip = tk.Toplevel(widget)
        tip.wm_overrideredirect(True)
        tip.wm_geometry("+%d+%d" % (e.x_root + 12, e.y_root + 18))
        lbl = tk.Label(tip, text=text, bg="#313244", fg=TEXT_MUTED,
                       font=mono_font(9), relief="flat",
                       padx=8, pady=4, wraplength=340, justify="left")
        lbl.pack()

    def hide(e):
        nonlocal tip
        if tip:
            tip.destroy()
            tip = None

    widget.bind("<Enter>", show)
    widget.bind("<Leave>", hide)


class BagToVdbGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("bag-to-vdb  |  Docker GUI")
        self.root.configure(bg=BG)
        self.root.minsize(1280, 720)
        self.root.resizable(True, True)

        self.dockerfile_dir  = tk.StringVar()
        self.image_name      = tk.StringVar(value="bag-to-vdb")
        self.bag_path        = tk.StringVar()
        self.output_dir      = tk.StringVar()

        self.pc_topic          = tk.StringVar(value="points")
        self.odom_topic        = tk.StringVar()
        self.imu_topic         = tk.StringVar()
        self.odom_max_latency  = tk.StringVar(value="0.5")

        self.voxel_size         = tk.StringVar(value="0.05")
        self.icp_dist_thresh    = tk.StringVar(value="0.2")
        self.icp_fitness_thresh = tk.StringVar(value="0.6")

        self.enable_loop_closure          = tk.BooleanVar(value=False)
        self.loop_closure_radius          = tk.StringVar(value="10.0")
        self.loop_closure_fitness_thresh  = tk.StringVar(value="0.3")
        self.loop_closure_search_interval = tk.StringVar(value="10")

        self.level_floor    = tk.BooleanVar(value=False)

        self.vdb_voxel_size = tk.StringVar()
        self.vdb_half_width = tk.StringVar(value="3")

        self._proc = None
        self.lc_slider_rows = []

        self._build_ui()
        self._trace_all()

    def _build_ui(self):
        # ── Title bar ──────────────────────────────────────────────────────
        title_bar = tk.Frame(self.root, bg=SURFACE2, pady=12)
        title_bar.pack(fill="x")
        tk.Label(title_bar, text="  bag-to-vdb",
                 bg=SURFACE2, fg=ACCENT,
                 font=mono_font(16, bold=True)).pack(side="left", padx=16)
        tk.Label(title_bar,
                 text="ROS 2 Bag  ->  Registered Point Cloud  ->  OpenVDB Level-Set",
                 bg=SURFACE2, fg=TEXT_MUTED,
                 font=mono_font(9)).pack(side="left", padx=4)

        # ── Two-column body ────────────────────────────────────────────────
        body = tk.Frame(self.root, bg=BG)
        body.pack(fill="both", expand=True)

        # ── LEFT: scrollable parameters ────────────────────────────────────
        left_container = tk.Frame(body, bg=BG)
        left_container.pack(side="left", fill="both", expand=False)

        canvas = tk.Canvas(left_container, bg=BG, highlightthickness=0, width=620)
        lscroll = ttk.Scrollbar(left_container, orient="vertical",
                                command=canvas.yview)
        canvas.configure(yscrollcommand=lscroll.set)
        lscroll.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        self.scroll_frame = tk.Frame(canvas, bg=BG)
        win_id = canvas.create_window((0, 0), window=self.scroll_frame, anchor="nw")

        def on_configure(e):
            canvas.configure(scrollregion=canvas.bbox("all"))
        def on_canvas_resize(e):
            canvas.itemconfig(win_id, width=e.width)

        self.scroll_frame.bind("<Configure>", on_configure)
        canvas.bind("<Configure>", on_canvas_resize)

        def _on_mousewheel(e):
            canvas.yview_scroll(int(-1 * (e.delta / 120)), "units")
        def _on_scroll_up(e):
            canvas.yview_scroll(-1, "units")
        def _on_scroll_down(e):
            canvas.yview_scroll(1, "units")
        canvas.bind_all("<MouseWheel>", _on_mousewheel)
        canvas.bind_all("<Button-4>", _on_scroll_up)
        canvas.bind_all("<Button-5>", _on_scroll_down)

        # ── Vertical divider ───────────────────────────────────────────────
        tk.Frame(body, bg=BORDER, width=2).pack(side="left", fill="y")

        # ── RIGHT: command + console ───────────────────────────────────────
        right_panel = tk.Frame(body, bg=BG)
        right_panel.pack(side="left", fill="both", expand=True)

        P = {"padx": 12, "pady": 5, "fill": "x"}

        # ────────────────────────────────────────────────────────────────────
        # LEFT COLUMN: parameter sections
        # ────────────────────────────────────────────────────────────────────

        # ── Section 1: Docker Image ────────────────────────────────────────
        sec, inner = section_frame(self.scroll_frame, "Docker Image")
        sec.pack(**P)

        r = row_frame(inner)
        styled_label(r, "Dockerfile directory", width=24).pack(side="left")
        styled_entry(r, self.dockerfile_dir, width=32).pack(side="left", padx=4)
        styled_button(r, "Browse...",
                      lambda: self._browse_dir(self.dockerfile_dir),
                      bg=SURFACE2, hover_bg=BORDER).pack(side="left")

        r2 = row_frame(inner)
        styled_label(r2, "Image name / tag", width=24).pack(side="left")
        styled_entry(r2, self.image_name, width=20).pack(side="left", padx=4)
        self.btn_build = styled_button(
            r2, "  Build Image  ", self._build_image,
            bg=BTN_BUILD, fg="#ffffff", hover_bg=BTN_BUILD_H)
        self.btn_build.pack(side="left", padx=4)
        self.build_status = styled_label(r2, "", color=TEXT_MUTED)
        self.build_status.pack(side="left", padx=6)

        # ── Section 2: Required ────────────────────────────────────────────
        sec, inner = section_frame(self.scroll_frame, "Required Arguments  (* mandatory)")
        sec.pack(**P)

        r = row_frame(inner)
        styled_label(r, "bag_path  *", width=22, color=WARNING).pack(side="left")
        styled_entry(r, self.bag_path, width=32).pack(side="left", padx=4)
        b = styled_button(r, "Browse...",
                      lambda: self._browse_file(
                          self.bag_path,
                          [("ROS 2 bag", "*.db3 *.mcap *"), ("All", "*.*")]),
                      bg=SURFACE2, hover_bg=BORDER)
        b.pack(side="left")
        tooltip(b, "Path to the ROS 2 bag file on the host machine.")

        r = row_frame(inner)
        styled_label(r, "output_dir  *", width=22, color=WARNING).pack(side="left")
        styled_entry(r, self.output_dir, width=32).pack(side="left", padx=4)
        b = styled_button(r, "Browse...",
                      lambda: self._browse_dir(self.output_dir),
                      bg=SURFACE2, hover_bg=BORDER)
        b.pack(side="left")
        tooltip(b, "Host directory where .ply and .vdb output files will be saved.")

        # ── Section 3: Topic Arguments ─────────────────────────────────────
        sec, inner = section_frame(self.scroll_frame, "Topic Arguments")
        sec.pack(**P)

        topic_defs = [
            ("--pc_topic", self.pc_topic,
             "Default: points\n"
             "sensor_msgs/PointCloud2 topic name in the bag."),
            ("--odom_topic", self.odom_topic,
             "Default: None (disabled)\n"
             "nav_msgs/Odometry topic. Improves ICP initial guess.\n"
             "Leave blank to disable."),
            ("--imu_topic", self.imu_topic,
             "Default: None (disabled)\n"
             "IMU topic. Parsed but currently unused in the pipeline.\n"
             "Leave blank to disable."),
            ("--odom_max_latency", self.odom_max_latency,
             "Default: 0.5 s\n"
             "Staleness cutoff (seconds) for odom<->pointcloud timestamp matching."),
        ]
        for lbl_txt, var, tip_txt in topic_defs:
            r = row_frame(inner)
            styled_label(r, lbl_txt, width=24).pack(side="left")
            e = styled_entry(r, var, width=28)
            e.pack(side="left", padx=4)
            tooltip(e, tip_txt)

        # ── Section 4: Point Cloud Processing ─────────────────────────────
        sec, inner = section_frame(self.scroll_frame, "Point Cloud Processing")
        sec.pack(**P)

        slider_defs = [
            ("--voxel_size", self.voxel_size, 0.001, 1.0, 0.001,
             "Default: 0.05  |  Range: 0.001 - 1.0\n"
             "Downsampling resolution for registration and cleaning (metres).\n"
             "Smaller = more detail but slower. Larger = faster preview."),
            ("--icp_dist_thresh", self.icp_dist_thresh, 0.01, 10.0, 0.01,
             "Default: 0.2  |  Range: 0.01 - 10.0\n"
             "Max point correspondence distance for ICP (metres).\n"
             "Increase if registration fails on sparse data."),
            ("--icp_fitness_thresh", self.icp_fitness_thresh, 0.0, 1.0, 0.01,
             "Default: 0.6  |  Range: 0.0 - 1.0\n"
             "Min fraction of points aligned to accept a frame.\n"
             "Lower = accept more frames (may include bad ones).\n"
             "Higher = stricter quality; may skip many frames."),
        ]
        for lbl_txt, var, mn, mx, res, tip_txt in slider_defs:
            self._slider_row(inner, lbl_txt, var, mn, mx, res, tip_txt)

        # ── Section 5: Loop Closure ────────────────────────────────────────
        sec, inner = section_frame(self.scroll_frame, "Loop Closure")
        sec.pack(**P)

        r = row_frame(inner)
        styled_label(r, "--enable_loop_closure", width=26).pack(side="left")
        ck = tk.Checkbutton(
            r, variable=self.enable_loop_closure,
            text="Enable  (FPFH + RANSAC + ICP)",
            bg=inner["bg"], fg=TEXT, selectcolor=SURFACE2,
            activebackground=inner["bg"], activeforeground=TEXT,
            font=mono_font(10), cursor="hand2",
            command=self._toggle_loop_closure)
        ck.pack(side="left")
        tooltip(ck,
                "Default: False\n"
                "Enable loop closure detection using FPFH feature matching.\n"
                "Only useful for large maps where areas are revisited.\n"
                "Adds 2-5x processing time.")

        lc_slider_defs = [
            ("--loop_closure_radius", self.loop_closure_radius, 0.1, 100.0, 0.1,
             "Default: 10.0\n"
             "Spatial radius (metres) to search for loop closure candidates."),
            ("--loop_closure_fitness_thresh", self.loop_closure_fitness_thresh,
             0.0, 1.0, 0.01,
             "Default: 0.3\n"
             "Min ICP fitness to accept a loop closure constraint."),
        ]
        for lbl_txt, var, mn, mx, res, tip_txt in lc_slider_defs:
            row = self._slider_row(inner, lbl_txt, var, mn, mx, res, tip_txt,
                                   return_row=True)
            self.lc_slider_rows.append(row)

        r = row_frame(inner)
        styled_label(r, "--loop_closure_search_interval", width=30).pack(side="left")
        e = styled_entry(r, self.loop_closure_search_interval, width=8)
        e.pack(side="left", padx=4)
        styled_label(r, "frames  (default: 10)", color=TEXT_MUTED).pack(
            side="left", padx=6)
        tooltip(e, "Default: 10\nRun loop closure search every N registered frames.")
        self.lc_slider_rows.append(r)

        self._toggle_loop_closure()

        # ── Section 6: Map Post-Processing ────────────────────────────────
        sec, inner = section_frame(self.scroll_frame, "Map Post-Processing")
        sec.pack(**P)

        r = row_frame(inner)
        styled_label(r, "--level_floor", width=20).pack(side="left")
        ck = tk.Checkbutton(
            r, variable=self.level_floor,
            text="Detect ground plane via RANSAC and rotate map so Z points up",
            bg=inner["bg"], fg=TEXT, selectcolor=SURFACE2,
            activebackground=inner["bg"], activeforeground=TEXT,
            font=mono_font(10), cursor="hand2",
            command=self._update_command)
        ck.pack(side="left")
        tooltip(ck,
                "Default: False\n"
                "Fits a RANSAC plane to the floor and rotates the map so Z is up.\n"
                "Use for flat indoor/outdoor environments.\n"
                "Avoid for multi-floor buildings, ramps, or hilly terrain.")

        # ── Section 7: VDB Output ──────────────────────────────────────────
        sec, inner = section_frame(self.scroll_frame, "VDB Output")
        sec.pack(**P)

        r = row_frame(inner)
        styled_label(r, "--vdb_voxel_size", width=22).pack(side="left")
        e = styled_entry(r, self.vdb_voxel_size, width=10)
        e.pack(side="left", padx=4)
        styled_label(r, "m  (blank = same as --voxel_size)", color=TEXT_MUTED).pack(
            side="left", padx=4)
        tooltip(e,
                "Default: same as --voxel_size\n"
                "VDB grid voxel edge length in metres.\n"
                "Set coarser for large outdoor maps to reduce file size.\n"
                "Set finer for high-detail object scans.")

        r = row_frame(inner)
        styled_label(r, "--vdb_half_width", width=22).pack(side="left")
        e = styled_entry(r, self.vdb_half_width, width=6)
        e.pack(side="left", padx=4)
        styled_label(r,
                     "voxels  |  truncation = half_width x vdb_voxel_size",
                     color=TEXT_MUTED).pack(side="left", padx=4)
        tooltip(e,
                "Default: 3\n"
                "Narrow-band half-width in voxels.\n"
                "Truncation distance = vdb_half_width x vdb_voxel_size.\n"
                "Increase for thicker surfaces or planners that query\n"
                "distances far from surfaces. Rarely needs changing.")

        tk.Frame(self.scroll_frame, bg=BG, height=20).pack()

        # ────────────────────────────────────────────────────────────────────
        # RIGHT COLUMN: command preview + run controls + console
        # ────────────────────────────────────────────────────────────────────
        RP = {"padx": 10, "pady": 5, "fill": "x"}

        # ── Generated Command ──────────────────────────────────────────────
        sec_r, inner_r = section_frame(right_panel, "Generated Docker Command")
        sec_r.pack(**RP)

        self.cmd_text = scrolledtext.ScrolledText(
            inner_r, height=7, bg="#11111b", fg=SUCCESS,
            font=mono_font(9), relief="flat", bd=0, wrap="word",
            highlightthickness=1, highlightbackground=BORDER)
        self.cmd_text.pack(fill="x", pady=(0, 6))
        self.cmd_text.config(state="disabled")

        btn_row = tk.Frame(inner_r, bg=inner_r["bg"])
        btn_row.pack(fill="x")
        styled_button(btn_row, "  Copy Command  ", self._copy_command,
                      bg=BTN_COPY, fg="#ffffff", hover_bg=BTN_COPY_H).pack(side="left")

        # ── Run Controls ───────────────────────────────────────────────────
        ctrl = tk.Frame(right_panel, bg=BG, pady=8)
        ctrl.pack(fill="x", padx=10)

        self.btn_run = styled_button(
            ctrl, "  Run Conversion  ", self._run,
            bg=BTN_RUN, fg="#ffffff", hover_bg=BTN_RUN_H)
        self.btn_run.pack(side="left", padx=(0, 8))

        self.btn_stop = styled_button(
            ctrl, "  Stop  ", self._stop,
            bg=BTN_STOP, fg="#ffffff", hover_bg=BTN_STOP_H)
        self.btn_stop.pack(side="left")
        self.btn_stop.config(state="disabled")

        self.status_lbl = styled_label(ctrl, "Ready.", color=TEXT_MUTED)
        self.status_lbl.pack(side="left", padx=12)

        # ── Console Output (fills remaining right-panel height) ────────────
        console_outer = tk.Frame(right_panel, bg=SURFACE, bd=0,
                                 highlightthickness=1, highlightbackground=BORDER)
        console_outer.pack(padx=10, pady=5, fill="both", expand=True)

        console_header = tk.Frame(console_outer, bg=SURFACE2, padx=10, pady=6)
        console_header.pack(fill="x")
        styled_label(console_header, "  Console Output",
                     size=10, color=ACCENT, bold=True).pack(side="left")

        console_inner = tk.Frame(console_outer, bg=SURFACE, padx=14, pady=10)
        console_inner.pack(fill="both", expand=True)

        self.console = scrolledtext.ScrolledText(
            console_inner, bg="#11111b", fg=TEXT,
            font=mono_font(9), relief="flat", bd=0,
            highlightthickness=1, highlightbackground=BORDER)
        self.console.pack(fill="both", expand=True)
        self.console.config(state="disabled")
        self.console.tag_config("err",  foreground=ERROR)
        self.console.tag_config("ok",   foreground=SUCCESS)
        self.console.tag_config("warn", foreground=WARNING)
        self.console.tag_config("info", foreground=ACCENT)

        btn_row2 = tk.Frame(console_inner, bg=console_inner["bg"])
        btn_row2.pack(fill="x", pady=(6, 0))
        styled_button(btn_row2, "  Clear Console  ",
                      self._clear_console,
                      bg=SURFACE2, hover_bg=BORDER).pack(side="left")

        self._update_command()

    def _slider_row(self, parent, label, var, mn, mx, resolution, tip_txt,
                    return_row=False):
        r = row_frame(parent)
        styled_label(r, label, width=30).pack(side="left")

        precision = len(str(resolution).rstrip("0").split(".")[-1])

        def on_slider(val):
            var.set(("{:." + str(precision) + "f}").format(float(val)))
            self._update_command()

        scale = tk.Scale(
            r, from_=mn, to=mx, resolution=resolution,
            orient="horizontal", length=200,
            bg=parent["bg"], fg=TEXT, troughcolor=SURFACE2,
            highlightthickness=0, bd=0, sliderrelief="flat",
            activebackground=ACCENT, font=mono_font(8),
            command=on_slider, showvalue=False)
        try:
            scale.set(float(var.get()))
        except Exception:
            scale.set(mn)
        scale.pack(side="left", padx=4)

        e = styled_entry(r, var, width=9)
        e.pack(side="left", padx=4)

        def sync_slider(event, s=scale, v=var):
            try:
                s.set(float(v.get()))
            except Exception:
                pass
            self._update_command()

        e.bind("<FocusOut>", sync_slider)
        e.bind("<Return>", sync_slider)
        tooltip(e, tip_txt)

        if return_row:
            return r

    def _toggle_loop_closure(self):
        state = "normal" if self.enable_loop_closure.get() else "disabled"
        for row in self.lc_slider_rows:
            for child in row.winfo_children():
                try:
                    child.config(state=state)
                except Exception:
                    pass
        self._update_command()

    def _trace_all(self):
        for var in [
            self.dockerfile_dir, self.image_name,
            self.bag_path, self.output_dir,
            self.pc_topic, self.odom_topic, self.imu_topic, self.odom_max_latency,
            self.voxel_size, self.icp_dist_thresh, self.icp_fitness_thresh,
            self.loop_closure_radius, self.loop_closure_fitness_thresh,
            self.loop_closure_search_interval,
            self.vdb_voxel_size, self.vdb_half_width,
        ]:
            var.trace_add("write", lambda *_: self._update_command())

    def _browse_file(self, var, filetypes):
        path = filedialog.askopenfilename(filetypes=filetypes)
        if path:
            var.set(path)

    def _browse_dir(self, var):
        path = filedialog.askdirectory()
        if path:
            var.set(path)

    # ── command builder ──────────────────────────────────────────────────────

    def _build_docker_cmd(self):
        bag   = self.bag_path.get().strip()
        outd  = self.output_dir.get().strip()
        image = self.image_name.get().strip() or "bag-to-vdb"

        cmd = ["docker", "run", "--rm"]

        if bag:
            bag_p = Path(bag)
            cmd += ["-v", str(bag_p.parent) + ":/app/input:ro"]
            bag_container = "/app/input/" + bag_p.name
        else:
            bag_container = "/app/input/<bag_file>"

        if outd:
            cmd += ["-v", outd + ":/app/output"]

        cmd.append(image)
        cmd.append(bag_container)
        cmd.append("/app/output")

        pc = self.pc_topic.get().strip()
        if pc and pc != "points":
            cmd += ["--pc_topic", pc]

        odom = self.odom_topic.get().strip()
        if odom:
            cmd += ["--odom_topic", odom]

        imu = self.imu_topic.get().strip()
        if imu:
            cmd += ["--imu_topic", imu]

        lat = self.odom_max_latency.get().strip()
        if lat and lat != "0.5":
            cmd += ["--odom_max_latency", lat]

        vs = self.voxel_size.get().strip()
        if vs and vs != "0.05":
            cmd += ["--voxel_size", vs]

        icp_d = self.icp_dist_thresh.get().strip()
        if icp_d and icp_d != "0.2":
            cmd += ["--icp_dist_thresh", icp_d]

        icp_f = self.icp_fitness_thresh.get().strip()
        if icp_f and icp_f != "0.6":
            cmd += ["--icp_fitness_thresh", icp_f]

        if self.enable_loop_closure.get():
            cmd.append("--enable_loop_closure")
            lcr = self.loop_closure_radius.get().strip()
            if lcr and lcr != "10.0":
                cmd += ["--loop_closure_radius", lcr]
            lcf = self.loop_closure_fitness_thresh.get().strip()
            if lcf and lcf != "0.3":
                cmd += ["--loop_closure_fitness_thresh", lcf]
            lci = self.loop_closure_search_interval.get().strip()
            if lci and lci != "10":
                cmd += ["--loop_closure_search_interval", lci]

        if self.level_floor.get():
            cmd.append("--level_floor")

        vvs = self.vdb_voxel_size.get().strip()
        if vvs:
            cmd += ["--vdb_voxel_size", vvs]

        vhw = self.vdb_half_width.get().strip()
        if vhw and vhw != "3":
            cmd += ["--vdb_half_width", vhw]

        return cmd

    def _update_command(self, *_):
        if not hasattr(self, "cmd_text"):
            return
        cmd = self._build_docker_cmd()
        parts = []
        i = 0
        while i < len(cmd):
            if cmd[i].startswith("--") and i + 1 < len(cmd) and not cmd[i+1].startswith("--"):
                parts.append(cmd[i] + " " + cmd[i+1])
                i += 2
            else:
                parts.append(cmd[i])
                i += 1
        pretty = " \\\n  ".join(parts)
        self.cmd_text.config(state="normal")
        self.cmd_text.delete("1.0", "end")
        self.cmd_text.insert("end", pretty)
        self.cmd_text.config(state="disabled")

    def _copy_command(self):
        cmd = self._build_docker_cmd()
        self.root.clipboard_clear()
        self.root.clipboard_append(" ".join(cmd))
        self.build_status.config(text="Copied to clipboard!", fg=SUCCESS)
        self.root.after(2500,
                        lambda: self.build_status.config(text="", fg=TEXT_MUTED))

    # ── build image ──────────────────────────────────────────────────────────

    def _build_image(self):
        d = self.dockerfile_dir.get().strip()
        if not d:
            messagebox.showerror("Error",
                                 "Please set the Dockerfile directory first.")
            return
        if not Path(d).is_dir():
            messagebox.showerror("Error", "Directory not found:\n" + d)
            return
        image = self.image_name.get().strip() or "bag-to-vdb"
        cmd = ["docker", "build", "-t", image, "."]
        self.btn_build.config(state="disabled")
        self.build_status.config(text="Building...", fg=WARNING)
        self._log("$ " + " ".join(cmd) + "  (in " + d + ")\n", "info")

        def run():
            try:
                proc = subprocess.Popen(
                    cmd, cwd=d,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1)
                for line in proc.stdout:
                    self._log(line)
                proc.wait()
                if proc.returncode == 0:
                    self.root.after(0, lambda: self.build_status.config(
                        text="Image '" + image + "' built successfully.", fg=SUCCESS))
                    self._log("\nBuild succeeded.\n", "ok")
                else:
                    self.root.after(0, lambda: self.build_status.config(
                        text="Build failed — see console.", fg=ERROR))
                    self._log("\nBuild failed (exit " + str(proc.returncode) + ").\n",
                              "err")
            except FileNotFoundError:
                self.root.after(0, lambda: self.build_status.config(
                    text="'docker' not found.", fg=ERROR))
                self._log("'docker' command not found. Is Docker installed?\n", "err")
            finally:
                self.root.after(0, lambda: self.btn_build.config(state="normal"))

        threading.Thread(target=run, daemon=True).start()

    # ── run / stop ───────────────────────────────────────────────────────────

    def _run(self):
        if not self.bag_path.get().strip():
            messagebox.showerror("Missing input", "Please set bag_path.")
            return
        if not self.output_dir.get().strip():
            messagebox.showerror("Missing input", "Please set output_dir.")
            return
        if not Path(self.bag_path.get().strip()).exists():
            messagebox.showerror("Not found",
                                 "Bag file not found:\n" + self.bag_path.get())
            return

        outd = Path(self.output_dir.get().strip())
        outd.mkdir(parents=True, exist_ok=True)

        cmd = self._build_docker_cmd()
        self._log("\n$ " + " ".join(cmd) + "\n\n", "info")
        self.btn_run.config(state="disabled")
        self.btn_stop.config(state="normal")
        self.status_lbl.config(text="Running...", fg=WARNING)

        def run():
            try:
                self._proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1)
                for line in self._proc.stdout:
                    lo = line.lower()
                    if any(w in lo for w in ("error", "traceback", "exception")):
                        tag = "err"
                    elif "warning" in lo:
                        tag = "warn"
                    else:
                        tag = None
                    self._log(line, tag)
                self._proc.wait()
                rc = self._proc.returncode
                if rc == 0:
                    self._log("\nConversion complete.\n", "ok")
                    self.root.after(0, lambda: self.status_lbl.config(
                        text="Done!", fg=SUCCESS))
                else:
                    self._log("\nProcess exited with code " + str(rc) + ".\n", "err")
                    self.root.after(0, lambda: self.status_lbl.config(
                        text="Exit code " + str(rc), fg=ERROR))
            except FileNotFoundError:
                self._log("'docker' not found. Is Docker running?\n", "err")
                self.root.after(0, lambda: self.status_lbl.config(
                    text="docker not found", fg=ERROR))
            finally:
                self._proc = None
                self.root.after(0, lambda: (
                    self.btn_run.config(state="normal"),
                    self.btn_stop.config(state="disabled")))

        threading.Thread(target=run, daemon=True).start()

    def _stop(self):
        if self._proc:
            try:
                self._proc.terminate()
                self._log("\nProcess terminated by user.\n", "warn")
                self.status_lbl.config(text="Stopped.", fg=WARNING)
            except Exception as exc:
                self._log("Failed to stop process: " + str(exc) + "\n", "err")

    # ── console ───────────────────────────────────────────────────────────────

    def _log(self, text, tag=None):
        def _write():
            self.console.config(state="normal")
            if tag:
                self.console.insert("end", text, tag)
            else:
                self.console.insert("end", text)
            self.console.see("end")
            self.console.config(state="disabled")
        self.root.after(0, _write)

    def _clear_console(self):
        self.console.config(state="normal")
        self.console.delete("1.0", "end")
        self.console.config(state="disabled")


def main():
    root = tk.Tk()
    BagToVdbGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
