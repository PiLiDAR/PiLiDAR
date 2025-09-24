"""Tkinter based GUI for the PiLiDAR project."""

from __future__ import annotations

import os
import tkinter as tk
from tkinter import messagebox, ttk


class PiLiDARApp:
    """Small helper that wraps the Tkinter GUI."""

    def __init__(self, controller) -> None:
        self.controller = controller
        self.controller.register_status_callback(self._handle_status)
        self.controller.register_completion_callback(self._handle_completion)

        self.root = tk.Tk()
        self.root.title("PiLiDAR Steuerung")

        self.scan_id_var = tk.StringVar()
        self.target_res_var = tk.StringVar(value=str(self.controller.config.target_res))
        self.scan_angle_var = tk.StringVar(value=str(self.controller.config.SCAN_ANGLE))
        cfg = self.controller.config
        self.enable_lidar_var = tk.BooleanVar(value=cfg.get("ENABLE_LIDAR"))
        self.enable_cam_var = tk.BooleanVar(value=cfg.get("ENABLE_CAM"))
        self.enable_pano_var = tk.BooleanVar(value=cfg.get("ENABLE_PANO"))
        self.enable_3d_var = tk.BooleanVar(value=cfg.get("ENABLE_3D"))
        self.enable_vertex_var = tk.BooleanVar(value=cfg.get("ENABLE_VERTEXCOLOUR"))
        self.enable_filter_var = tk.BooleanVar(value=cfg.get("ENABLE_FILTERING"))

        self.status_box = None
        self.start_button = None
        self.stop_button = None

        self._build_ui()

    # ------------------------------------------------------------------
    # UI creation
    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        frame = ttk.Frame(self.root, padding=10)
        frame.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        ttk.Label(frame, text="Scan ID (optional):").grid(row=0, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.scan_id_var).grid(row=0, column=1, sticky="ew")

        ttk.Label(frame, text="Horizontale Auflösung (°):").grid(row=1, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.target_res_var).grid(row=1, column=1, sticky="ew")

        ttk.Label(frame, text="Scanwinkel (°):").grid(row=2, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.scan_angle_var).grid(row=2, column=1, sticky="ew")

        ttk.Label(frame, text="Aktive Module:").grid(row=3, column=0, columnspan=2, sticky="w", pady=(10, 0))
        ttk.Checkbutton(frame, text="LiDAR aktivieren", variable=self.enable_lidar_var).grid(row=4, column=0, sticky="w")
        ttk.Checkbutton(frame, text="Kamera aktivieren", variable=self.enable_cam_var).grid(row=4, column=1, sticky="w")
        ttk.Checkbutton(frame, text="Panorama berechnen", variable=self.enable_pano_var).grid(row=5, column=0, sticky="w")
        ttk.Checkbutton(frame, text="3D-Punktwolke speichern", variable=self.enable_3d_var).grid(row=5, column=1, sticky="w")
        ttk.Checkbutton(frame, text="Punktwolke mit Foto-Farben", variable=self.enable_vertex_var).grid(row=6, column=0, sticky="w")
        ttk.Checkbutton(frame, text="Rauschen filtern", variable=self.enable_filter_var).grid(row=6, column=1, sticky="w")

        button_frame = ttk.Frame(frame)
        button_frame.grid(row=7, column=0, columnspan=2, pady=(10, 5))

        self.start_button = ttk.Button(button_frame, text="Scan starten", command=self._start_scan)
        self.start_button.grid(row=0, column=0, padx=5)

        self.stop_button = ttk.Button(button_frame, text="Scan stoppen", command=self._stop_scan, state=tk.DISABLED)
        self.stop_button.grid(row=0, column=1, padx=5)

        ttk.Label(frame, text="Statusmeldungen:").grid(row=8, column=0, columnspan=2, sticky="w")

        self.status_box = tk.Text(frame, height=12, state=tk.DISABLED)
        self.status_box.grid(row=9, column=0, columnspan=2, sticky="nsew")

        frame.columnconfigure(1, weight=1)
        frame.rowconfigure(9, weight=1)

    # ------------------------------------------------------------------
    # event handlers
    # ------------------------------------------------------------------
    def _start_scan(self) -> None:
        try:
            target_res = float(self.target_res_var.get())
            scan_angle = float(self.scan_angle_var.get())
        except ValueError:
            messagebox.showerror("Ungültige Eingabe", "Bitte gültige Zahlen für Auflösung und Scanwinkel eingeben.")
            return

        self.controller.config.set(scan_angle, "STEPPER", "SCAN_ANGLE")
        self.controller.config.SCAN_ANGLE = scan_angle
        self.controller.config.set(target_res, "LIDAR", "TARGET_RES")
        self.controller.config.update_target_res(target_res)

        cfg = self.controller.config
        cfg.set(self.enable_lidar_var.get(), "ENABLE_LIDAR")
        cfg.set(self.enable_cam_var.get(), "ENABLE_CAM")
        cfg.set(self.enable_pano_var.get(), "ENABLE_PANO")
        cfg.set(self.enable_3d_var.get(), "ENABLE_3D")
        cfg.set(self.enable_vertex_var.get(), "ENABLE_VERTEXCOLOUR")
        cfg.set(self.enable_filter_var.get(), "ENABLE_FILTERING")

        scan_id = self.scan_id_var.get().strip() or None
        self.controller.set_scan_id(scan_id)

        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self._append_status("--- Scan gestartet ---")
        self.controller.start_scan_async()

    def _stop_scan(self) -> None:
        self.controller.request_stop()
        self.stop_button.config(state=tk.DISABLED)

    def _handle_status(self, message: str) -> None:
        self.root.after(0, lambda: self._append_status(message))

    def _handle_completion(self, success: bool, error) -> None:
        def finalize():
            self.start_button.config(state=tk.NORMAL)
            self.stop_button.config(state=tk.DISABLED)
            if success:
                messagebox.showinfo("Scan beendet", "Der Scan wurde erfolgreich abgeschlossen.")
                if self.controller.config.scan_dir and os.path.isdir(self.controller.config.scan_dir):
                    self._append_status(f"Dateien gespeichert unter: {self.controller.config.scan_dir}")
            else:
                messagebox.showerror("Scan fehlgeschlagen", f"Fehler: {error}")

        self.root.after(0, finalize)

    def _append_status(self, message: str) -> None:
        self.status_box.config(state=tk.NORMAL)
        self.status_box.insert(tk.END, message + "\n")
        self.status_box.see(tk.END)
        self.status_box.config(state=tk.DISABLED)

    # ------------------------------------------------------------------
    def run(self) -> None:
        self.root.mainloop()
