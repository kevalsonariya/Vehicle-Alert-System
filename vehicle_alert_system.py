"""
Vehicle Alert System – Integrated Script
Raspberry Pi 4 (Bookworm) + Pi Camera v1.3 + Ultrasonic (HC-SR04) + ADXL335 via ADS1115 + GPS6MV2 + SIM900A GSM

Features
- Drowsiness detection (simple blink/eye-closed heuristic using Haar cascades)
- Object/obstacle proximity alert (HC-SR04)
- Accident detection via g-force spike (ADXL335 on ADS1115)
- GPS coordinate reading (NMEA)
- SMS alert with Google Maps link via SIM900A (AT commands)
- Voice alerts via pyttsx3 (offline TTS)

Dependencies (install with pip):
  pip install opencv-python==4.9.0.80 pyttsx3 pyserial adafruit-circuitpython-ads1x15 RPi.GPIO pynmea2

Enable interfaces on Pi:
  sudo raspi-config  # enable Camera, I2C, Serial (disable login shell on serial, enable HW port)
Wiring notes are included near each section.

Author: (You)
Last updated: 2025-08-26
"""

import os
import sys
import time
import threading
import queue
import math
import signal
from dataclasses import dataclass
import cv2
import pyttsx3
import RPi.GPIO as GPIO
import board
import busio
from adafruit_ads1x15.ads1115 import ADS1115, Mode
from adafruit_ads1x15.analog_in import AnalogIn

# --- Serial for GPS + GSM ---
import serial
import pynmea2

# =========================
# Global Config karel chhe
# =========================
@dataclass
class Config:
    # Drowsiness
    eye_closed_thresh_frames: int = 15        # frame define karel chhe 
    camera_index: int = 0                     # /dev/video0
    camera_resolution: tuple = (640, 480)
    camera_fps: int = 15
    cascade_face: str = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
    cascade_eye: str = cv2.data.haarcascades + 'haarcascade_eye_tree_eyeglasses.xml'

    # Ultrasonic sensor ni pin niche difine karel chhe 
    trig_pin: int = 23
    echo_pin: int = 24
    obstacle_distance_cm: float = 40.0
    ultrasonic_interval_s: float = 0.15

    # ADS1115 @ 0x48 (chek kari levu fix na hoy ano alag thi code aave)
    ads_address: int = 0x48
    ads_rate: int = 128  # SPS
    ads_mode: Mode = Mode.CONTINUOUS
    g_threshold: float = 2.5  # g te accident mate nu threshold chhe 
    accel_zero_g_offset: tuple = (1.65, 1.65, 1.65)  # default mid-scale; calibrate!
    accel_sensitivity_v_per_g: float = 0.330  # ADXL335 ~330 mV/g at 3.3V
    sample_interval_accel_s: float = 0.01
    spike_window_s: float = 0.2

    # GPS
    gps_port: str = '/dev/serial0'
    gps_baud: int = 9600

    # GSM SIM900A
    gsm_port: str = '/dev/ttyUSB0'  # or '/dev/serial0' if wired to UART
    gsm_baud: int = 9600
    alert_phone_numbers: tuple = ('+91XXXXXXXXXX',)  # fill in numbers

    # General
    voice_rate: int = 175
    voice_volume: float = 1.0

CFG = Config()

# =========================
# Utilities
# =========================
class Voice:
    def __init__(self, rate=175, volume=1.0):
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', rate)
        self.engine.setProperty('volume', volume)
        self.lock = threading.Lock()

    def say(self, text: str):
        # Non-blocking TTS by running each phrase in a separate thread
        def _run():
            with self.lock:
                self.engine.say(text)
                self.engine.runAndWait()
        threading.Thread(target=_run, daemon=True).start()

voice = Voice(rate=CFG.voice_rate, volume=CFG.voice_volume)

stop_event = threading.Event()

# =========================
# Ultrasonic Sensor (HC-SR04)
# =========================
class Ultrasonic:
    def __init__(self, trig_pin: int, echo_pin: int):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(trig_pin, GPIO.OUT)
        GPIO.setup(echo_pin, GPIO.IN)
        self.trig = trig_pin
        self.echo = echo_pin

    def distance_cm(self) -> float:
        # Send 10us pulse
        GPIO.output(self.trig, True)
        time.sleep(0.00001)
        GPIO.output(self.trig, False)

        start = time.time()
        while GPIO.input(self.echo) == 0:
            start = time.time()
            if (start - time.time()) > 0.02:
                break

        while GPIO.input(self.echo) == 1:
            end = time.time()
            if (end - start) > 0.04:
                break
        else:
            end = time.time()

        elapsed = end - start
        # speed of sound ~34300 cm/s
        dist = (elapsed * 34300) / 2.0
        return max(0.0, min(dist, 400.0))

# =========================
# ADS1115 + ADXL335
# =========================
class ADXL335:
    def __init__(self, i2c, address=0x48, zero_offsets=(1.65,1.65,1.65), sensitivity=0.330):
        self.ads = ADS1115(i2c, address=address)
        self.ads.mode = CFG.ads_mode
        self.ads.data_rate = CFG.ads_rate
        self.chx = AnalogIn(self.ads, ADS1115.P0)
        self.chy = AnalogIn(self.ads, ADS1115.P1)
        self.chz = AnalogIn(self.ads, ADS1115.P2)
        self.vref = 3.3  # Pi 3.3V assumed
        self.zero = zero_offsets
        self.sens = sensitivity

    def _volts(self, ch):
        return ch.voltage

    def read_g(self):
        vx = self._volts(self.chx)
        vy = self._volts(self.chy)
        vz = self._volts(self.chz)
        gx = (vx - self.zero[0]) / self.sens
        gy = (vy - self.zero[1]) / self.sens
        gz = (vz - self.zero[2]) / self.sens
        g_total = math.sqrt(gx*gx + gy*gy + gz*gz)
        return gx, gy, gz, g_total

# =========================
# GPS Reader
# =========================
class GPSReader:
    def __init__(self, port: str, baud: int):
        self.ser = serial.Serial(port, baudrate=baud, timeout=1)
        self.lock = threading.Lock()
        self.latest = None  # (lat, lon)

    def run(self):
        while not stop_event.is_set():
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('$GPGGA') or line.startswith('$GPRMC'):
                    try:
                        msg = pynmea2.parse(line)
                        lat = getattr(msg, 'latitude', None)
                        lon = getattr(msg, 'longitude', None)
                        if lat and lon:
                            with self.lock:
                                self.latest = (lat, lon)
                    except Exception:
                        pass
            except Exception:
                time.sleep(0.1)

    def get_latest(self):
        with self.lock:
            return self.latest

# =========================
# GSM (SIM900A) SMS Sender
# =========================
class GSM:
    def __init__(self, port: str, baud: int):
        self.ser = serial.Serial(port, baudrate=baud, timeout=1)
        time.sleep(0.5)

    def _write(self, cmd: str):
        self.ser.write((cmd + "\r").encode())
        time.sleep(0.2)
        return self.ser.read_all().decode(errors='ignore')

    def send_sms(self, number: str, text: str) -> bool:
        try:
            self._write('AT')
            self._write('AT+CMGF=1')  # text mode
            self._write(f'AT+CMGS="{number}"')
            self.ser.write((text + "\x1A").encode())  # CTRL+Z
            # wait for send
            time.sleep(3)
            resp = self.ser.read_all().decode(errors='ignore')
            return "+CMGS" in resp or "OK" in resp
        except Exception:
            return False

# =========================
# Drowsiness via simple eye-closed heuristic
# =========================
class DrowsinessDetector:
    def __init__(self, cam_index: int, res: tuple, fps: int, face_cascade: str, eye_cascade: str):
        self.cap = cv2.VideoCapture(cam_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, res[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res[1])
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.face_cascade = cv2.CascadeClassifier(face_cascade)
        self.eye_cascade = cv2.CascadeClassifier(eye_cascade)
        self.closed_frames = 0
        self.alerted = False

        if not self.face_cascade.load(face_cascade):
            pass

    def run(self):
        while not stop_event.is_set():
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.05)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.2, 5, minSize=(60,60))
            eyes_detected = 0
            for (x,y,w,h) in faces:
                roi = gray[y:y+h, x:x+w]
                eyes = self.eye_cascade.detectMultiScale(roi, 1.1, 3, minSize=(20,20))
                eyes_detected = max(eyes_detected, len(eyes))
                break  # consider first face

            if eyes_detected >= 1:
                self.closed_frames = 0
                self.alerted = False
            else:
                self.closed_frames += 1

            if self.closed_frames >= CFG.eye_closed_thresh_frames and not self.alerted:
                voice.say("Alert! Drowsiness detected. Please pay attention.")
                self.alerted = True

            # Optional preview window (comment out on headless)
            # cv2.imshow('Drowsiness', frame)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     stop_event.set()

        self.cap.release()
        cv2.destroyAllWindows()

# =========================
# Supervisors
# =========================
class ProximitySupervisor:
    def __init__(self, ultrasonic: Ultrasonic, threshold_cm: float):
        self.u = ultrasonic
        self.th = threshold_cm
        self.last_spoken = 0

    def run(self):
        while not stop_event.is_set():
            dist = self.u.distance_cm()
            if dist > 0 and dist < self.th:
                now = time.time()
                if now - self.last_spoken > 1.5:
                    voice.say(f"Obstacle ahead at {int(dist)} centimeters. Slow down.")
                    self.last_spoken = now
            time.sleep(CFG.ultrasonic_interval_s)

class AccidentSupervisor:
    def __init__(self, accel: ADXL335, gps: GPSReader, gsm: GSM, g_threshold: float):
        self.accel = accel
        self.gps = gps
        self.gsm = gsm
        self.th = g_threshold
        self.samples = []  # (timestamp, g_total)

    def run(self):
        while not stop_event.is_set():
            gx,gy,gz,g = self.accel.read_g()
            t = time.time()
            self.samples.append((t,g))
            # keep recent window
            cutoff = t - CFG.spike_window_s
            self.samples = [s for s in self.samples if s[0] >= cutoff]

            # detect spike above threshold
            if any(gv >= self.th for (_,gv) in self.samples):
                # Debounce: wait a moment and confirm persistent spike
                time.sleep(0.05)
                if max(gv for (_,gv) in self.samples) >= self.th:
                    self.handle_accident()
                    # Cooldown
                    time.sleep(5)
                    self.samples.clear()
            time.sleep(CFG.sample_interval_accel_s)

    def handle_accident(self):
        voice.say("Severe impact detected. Sending emergency SMS.")
        loc = self.gps.get_latest()
        if loc:
            lat, lon = loc
            maps = f"https://maps.google.com/?q={lat},{lon}"
            msg = f"EMERGENCY: Possible accident detected. Location: {lat:.6f}, {lon:.6f} {maps}"
        else:
            msg = "EMERGENCY: Possible accident detected. GPS fix unavailable."

        for num in CFG.alert_phone_numbers:
            try:
                ok = self.gsm.send_sms(num, msg)
            except Exception:
                ok = False
            if ok:
                voice.say("Emergency message sent.")
            else:
                voice.say("Failed to send SMS.")

# =========================
# Graceful shutdown
# =========================

def handle_sigterm(sig, frame):
    stop_event.set()

signal.signal(signal.SIGINT, handle_sigterm)
signal.signal(signal.SIGTERM, handle_sigterm)

# =========================
# Main
# =========================

def main():
    print("Starting Vehicle Alert System...")

    # --- Setup Ultrasonic ---
    ultrasonic = Ultrasonic(CFG.trig_pin, CFG.echo_pin)

    # --- Setup I2C + ADXL335 ---
    i2c = busio.I2C(board.SCL, board.SDA)
    accel = ADXL335(i2c, address=CFG.ads_address, zero_offsets=CFG.accel_zero_g_offset,
                    sensitivity=CFG.accel_sensitivity_v_per_g)

    # --- GPS ---
    gps = GPSReader(CFG.gps_port, CFG.gps_baud)

    # --- GSM ---
    try:
        gsm = GSM(CFG.gsm_port, CFG.gsm_baud)
    except Exception as e:
        print(f"Warning: GSM init failed: {e}. SMS will be disabled.")
        gsm = None

    # --- Drowsiness ---
    drowsy = DrowsinessDetector(CFG.camera_index, CFG.camera_resolution, CFG.camera_fps,
                                CFG.cascade_face, CFG.cascade_eye)

    # Threads
    threads = []

    t_gps = threading.Thread(target=gps.run, daemon=True)
    t_d = threading.Thread(target=drowsy.run, daemon=True)
    prox = ProximitySupervisor(ultrasonic, CFG.obstacle_distance_cm)
    t_p = threading.Thread(target=prox.run, daemon=True)

    if gsm:
        accident = AccidentSupervisor(accel, gps, gsm, CFG.g_threshold)
        t_a = threading.Thread(target=accident.run, daemon=True)
        threads.extend([t_a])

    threads.extend([t_gps, t_d, t_p])

    for t in threads:
        t.start()

    try:
        while not stop_event.is_set():
            time.sleep(0.5)
    finally:
        GPIO.cleanup()
        print("Shutting down.")

if __name__ == '__main__':
    main()
