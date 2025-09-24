"""High level orchestration for a complete PiLiDAR scan."""

from __future__ import annotations

import os
import threading
import time
from typing import Callable, Dict, List, Optional


class ScanController:
    """Manage the LiDAR, stepper motor and camera pipeline."""

    def __init__(
        self,
        config,
        stepper_factory: Optional[Callable] = None,
        lidar_factory: Optional[Callable] = None,
    ) -> None:
        self.config = config
        self._status_callback: Optional[Callable[[str], None]] = None
        self._completion_callback: Optional[Callable[[bool, Optional[Exception]], None]] = None

        self._stop_event = threading.Event()
        self._scan_thread: Optional[threading.Thread] = None
        self.scan_result: Dict[str, Optional[object]] = {}

        self.stepper = None
        self.lidar = None
        self.custom_scan_id: Optional[str] = None

        self.stepper_factory = stepper_factory or self._default_stepper_factory
        self.lidar_factory = lidar_factory or self._default_lidar_factory

    # ------------------------------------------------------------------
    # factory helpers
    # ------------------------------------------------------------------
    def _default_stepper_factory(self, config):  # pragma: no cover - hardware specific
        from lib.a4988_driver import A4988

        pins = config.get("STEPPER", "pins", "MS_PINS")
        use_pwm = config.get("STEPPER", "USE_PWM", default=False)
        return A4988(
            config.get("STEPPER", "pins", "DIR_PIN"),
            config.get("STEPPER", "pins", "STEP_PIN"),
            pins,
            enable_pin=config.get("STEPPER", "pins", "ENABLE_PIN", default=None),
            delay=config.get("STEPPER", "STEP_DELAY"),
            step_angle=config.get("STEPPER", "STEP_ANGLE"),
            microsteps=config.get("STEPPER", "MICROSTEPS"),
            gear_ratio=config.get("STEPPER", "GEAR_RATIO"),
            pwm_frequency=None,
            use_pwm=use_pwm,
        )

    def _default_lidar_factory(self, config):  # pragma: no cover - hardware specific
        from lib.lidar_driver import Lidar

        return Lidar(config)


    # ------------------------------------------------------------------
    # callback registration
    # ------------------------------------------------------------------
    def register_status_callback(self, callback: Callable[[str], None]) -> None:
        self._status_callback = callback

    def register_completion_callback(self, callback: Callable[[bool, Optional[Exception]], None]) -> None:
        self._completion_callback = callback

    def set_scan_id(self, scan_id: Optional[str]) -> None:
        self.custom_scan_id = scan_id or None

    def _notify(self, message: str) -> None:
        print(message)
        if self._status_callback:
            self._status_callback(message)

    def _notify_completion(self, success: bool, error: Optional[Exception]) -> None:
        if self._completion_callback:
            self._completion_callback(success, error)

    # ------------------------------------------------------------------
    # public API
    # ------------------------------------------------------------------
    def start_scan_async(self) -> None:
        if self._scan_thread and self._scan_thread.is_alive():
            self._notify("Ein Scan läuft bereits.")
            return
        self._stop_event.clear()
        self._scan_thread = threading.Thread(target=self._run_with_callbacks, daemon=True)
        self._scan_thread.start()

    def run_scan(self) -> Dict[str, Optional[object]]:
        self._stop_event.clear()
        result = self._execute_scan()
        self.scan_result = result
        return result

    def _run_with_callbacks(self) -> None:
        error = None
        try:
            self.scan_result = self._execute_scan()
            self._notify_completion(True, None)
        except Exception as exc:  # pragma: no cover - debugging aid
            error = exc
            self._notify(f"Fehler: {exc}")
            self._notify_completion(False, exc)
        finally:
            if error:
                self.scan_result = {}

    def request_stop(self) -> None:
        self._notify("Stopp angefordert.")
        self._stop_event.set()
        if self.lidar is not None:
            self.lidar.request_stop()

    def is_running(self) -> bool:
        return self._scan_thread is not None and self._scan_thread.is_alive()

    # ------------------------------------------------------------------
    # scan pipeline
    # ------------------------------------------------------------------
    def _execute_scan(self) -> Dict[str, Optional[object]]:
        self._notify("Bereite neuen Scan vor...")
        self.config.init(scan_id=self.custom_scan_id)
        self.config.relay_on()

        self.stepper = self.stepper_factory(self.config) if self.config.get("ENABLE_LIDAR") else None
        self.lidar = self.lidar_factory(self.config) if self.config.get("ENABLE_LIDAR") else None
        if self.lidar is not None:
            self.lidar.set_status_callback(self._notify)

        try:
            if self._stop_event.is_set():
                return {}

            if self.config.get("ENABLE_LIDAR") and self.lidar is not None:
                self._notify("LiDAR wird hochgefahren...")
                self.lidar.power_on()
                self.lidar.start_motor()
                time.sleep(2)

            if self.config.get("ENABLE_CAM"):
                self._capture_panorama()

            raw_scan = None
            if self.config.get("ENABLE_LIDAR") and self.lidar is not None:
                raw_scan = self._capture_lidar()

            pano_path = None
            if self.config.get("ENABLE_CAM") and self.config.get("ENABLE_PANO"):
                pano_path = self._build_panorama()

            pointclouds: Dict[str, Optional[object]] = {}
            if raw_scan is not None or os.path.exists(self.config.raw_path):
                pointclouds = self._process_pointcloud()

            result = {
                "raw_scan": raw_scan,
                "panorama": pano_path,
                "pointclouds": pointclouds,
                "scan_dir": self.config.scan_dir,
            }
            self._notify("Scan erfolgreich abgeschlossen.")
            return result
        finally:
            self._cleanup()

    # ------------------------------------------------------------------
    # individual pipeline stages
    # ------------------------------------------------------------------
    def _capture_panorama(self) -> None:
        from lib.rpicam_utils import estimate_camera_parameters, take_HDR_photo

        self._notify("Kalibriere Kamera...")
        exposure_time, gain, awbgains = estimate_camera_parameters(self.config)

        IMGCOUNT = self.config.get("PANO", "IMGCOUNT")
        for i in range(IMGCOUNT):
            if self._stop_event.is_set():
                self._notify("Panorama-Aufnahme abgebrochen.")
                return

            current_angle = 0 if self.stepper is None else self.stepper.get_current_angle()
            formatted_angle = format(current_angle, f".{self.config.get('ANGULAR_DIGITS')}f")
            self._notify(f"Foto {i + 1}/{IMGCOUNT} bei {formatted_angle}°")

            imgname = f"image_{formatted_angle}.jpg"
            imgpath = os.path.join(self.config.img_dir, imgname)

            imgpaths = take_HDR_photo(
                AEB=self.config.get("CAM", "AEB"),
                AEB_stops=self.config.get("CAM", "AEB_STOPS"),
                path=imgpath,
                exposure_time=exposure_time,
                gain=gain,
                awbgains=awbgains,
                denoise=self.config.get("CAM", "denoise"),
                sharpness=self.config.get("CAM", "sharpness"),
                saturation=self.config.get("CAM", "saturation"),
                save_raw=self.config.get("CAM", "raw"),
                blocking=True,
            )

            self.config.imglist.extend(imgpaths)

            if self.stepper is not None:
                self.stepper.move_to_angle((360 / IMGCOUNT) * (i + 1))
                time.sleep(0.5)

        if self.stepper is not None and self.lidar is not None:
            self.lidar.z_angle = self.stepper.get_current_angle(mod=False)

        if self.stepper is not None:
            # Für Panorama-Aufnahme normale move_to_angle verwenden (kürzester Weg)
            self.stepper.move_to_angle(0)
            time.sleep(0.5)

    def _capture_lidar(self):
        scan_delay = self.config.get("STEPPER", "SCAN_DELAY")
        stepper_log: List[Dict[str, float]] = []

        def record_stepper_state(angle: Optional[float] = None) -> None:
            if self.stepper is None:
                return

            state_angle = angle
            if state_angle is None:
                state_angle = self.stepper.get_current_angle(mod=False)
            if state_angle is None:
                return

            timestamp = time.time()
            steps_value: Optional[int] = None
            if hasattr(self.stepper, "get_steps_for_angle"):
                try:
                    steps_value = int(self.stepper.get_steps_for_angle(state_angle))
                except Exception:
                    steps_value = getattr(self.stepper, "current_steps", None)
            else:
                steps_value = getattr(self.stepper, "current_steps", None)

            entry: Dict[str, float] = {
                "timestamp": float(timestamp),
                "angle": float(state_angle),
            }
            if steps_value is not None:
                entry["steps"] = float(steps_value)
            stepper_log.append(entry)

        supports_continuous = (
            self.stepper is not None
            and getattr(self.stepper, "use_pwm", False)
            and self.config.get("STEPPER", "SCAN_ANGLE") != 0
            and self.config.steps > 0
        )

        if supports_continuous:
            direction_negative = self.config.SCAN_ANGLE < 0
            total_rotation = abs(self.config.SCAN_ANGLE)
            steps_per_second = abs(self.config.steps * self.config.TARGET_SPEED)
            stepper_active = False
            rotation_done = False
            start_angle = self.stepper.get_current_angle(mod=False)
            self.lidar.z_angle = start_angle
            record_stepper_state(start_angle)

            if steps_per_second > 0 and total_rotation > 0:
                self.stepper.start_continuous(steps_per_second, direction=direction_negative)
                stepper_active = True

            def update_angle_callback():
                nonlocal stepper_active, rotation_done
                if self._stop_event.is_set():
                    if stepper_active:
                        self.stepper.stop_continuous()
                        stepper_active = False
                        record_stepper_state()
                    self.lidar.request_stop()
                    return

                if stepper_active:
                    current_angle = self.stepper.get_current_angle(mod=False)
                    record_stepper_state(current_angle)
                    self.lidar.z_angle = current_angle

                    rotated = (
                        start_angle - current_angle
                        if direction_negative
                        else current_angle - start_angle
                    )

                    if rotated >= total_rotation:
                        rotation_done = True
                        self.stepper.stop_continuous()
                        stepper_active = False
                        record_stepper_state(current_angle)
                        self.lidar.request_stop()
                else:
                    if self.stepper is not None:
                        self.lidar.z_angle = self.stepper.get_current_angle(mod=False)

            self._notify("Starte LiDAR-Aufnahme...")
            self.lidar.read_loop(callback=update_angle_callback, max_packages=self.config.max_packages)

            if stepper_active:
                self.stepper.stop_continuous()
            if self.stepper is not None and total_rotation == 0:
                self.lidar.z_angle = self.stepper.get_current_angle(mod=False)
        else:
            if self.stepper is not None:
                record_stepper_state(self.stepper.get_current_angle(mod=False))

            def move_steps_callback():
                if self._stop_event.is_set():
                    record_stepper_state()
                    self.lidar.request_stop()
                    return
                if self.stepper is None:
                    return
                steps = self.config.steps if self.config.SCAN_ANGLE > 0 else -self.config.steps
                self.stepper.move_steps(steps)
                angle_mod = self.stepper.get_current_angle()
                self.lidar.z_angle = angle_mod
                record_stepper_state(self.stepper.get_current_angle(mod=False))
                time.sleep(scan_delay)

            self._notify("Starte LiDAR-Aufnahme...")
            self.lidar.read_loop(callback=move_steps_callback, max_packages=self.config.max_packages)

        if hasattr(self.lidar, "stop_motor"):
            try:
                self.lidar.stop_motor()
            except Exception as exc:  # pragma: no cover - hardware environment
                self._notify(f"Warnung: LiDAR-Motor konnte nicht gestoppt werden ({exc}).")

        if self.stepper is not None:
            try:
                self.stepper.stop_continuous()
            except Exception:
                pass
            
            # Rückwärts zur Ausgangsposition (0°) zurückkehren
            if hasattr(self.stepper, 'move_to_angle_reverse'):
                self._notify("Stepper kehrt rückwärts zur Ausgangsposition zurück...")
                self.stepper.move_to_angle_reverse(0.0)
            else:
                # Fallback für ältere Implementierungen
                self.stepper.move_to_angle(0)
            
            record_stepper_state(self.stepper.get_current_angle(mod=False))
            if hasattr(self.stepper, "disable"):
                try:
                    self.stepper.disable()
                except Exception:
                    pass

        if hasattr(self.lidar, "stepper_log"):
            self.lidar.stepper_log = list(stepper_log)

        packages = getattr(self.lidar, "package_history", [])
        metadata = {"samples": len(packages)}
        raw_scan = self.lidar.create_raw_scan(metadata=metadata)
        self._notify(f"Rohdaten gespeichert unter {self.config.raw_path}")
        return raw_scan

    def _build_panorama(self):
        from lib.pano_utils import hugin_stitch

        self._notify("Berechne Panorama...")
        pano_path = hugin_stitch(self.config)
        return pano_path

    def _process_pointcloud(self):
        from lib.pointcloud import process_raw

        self._notify("Erzeuge Punktwolken...")
        result = process_raw(self.config, save=True)
        return result

    # ------------------------------------------------------------------
    # cleanup
    # ------------------------------------------------------------------
    def _cleanup(self):
        if self.lidar is not None:
            self.lidar.close()
        if self.stepper is not None:
            self.stepper.close()
        
        # Force stepper to stay disabled after cleanup
        self.config.force_stepper_disabled()
        self.config.relay_off()
