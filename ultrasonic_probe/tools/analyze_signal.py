#!/usr/bin/env python3
# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy>=2", "scipy>=1.13", "matplotlib>=3.9", "pyserial>=3.5"]
# ///
"""Capture and analyze the ~40 kHz ultrasonic signal on GPIO32 of the LilyGO T-Display.

Talks to the ultrasonic_probe firmware over serial and runs, in order:
  1. a connection test: GPIO32 with no pull / internal pull-down / internal pull-up
     (a driven signal barely moves, a floating pin follows the pull),
  2. a sampler sweep: real ADC conversion rate for several I2S rates (the I2S ADC path
     repeats each conversion several times, so the DMA rate is not the true sample rate),
  3. long captures at the best rate, as wired (no pull) and, if that saturates, with pull-down,
  4. a trend run (repeated short captures over ~20 s),
  5. interference controls: GPIO33 (nothing connected, held at mid-rail by both internal pulls)
     and GPIO32, each with the ST7789 display awake and asleep. A line that also shows up on the
     unconnected pin is not the ultrasonic signal; one that vanishes when the display sleeps comes
     from the display's charge pump.
Then writes graphs, raw data, summary.json and REPORT.md into the output folder.

Run from the project folder:  uv run tools/analyze_signal.py
"""

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import serial
from scipy import signal

CHANNEL = 4  # ADC1 channel of GPIO32
CONTROL_CHANNEL = 5  # ADC1 channel of GPIO33
FULL_SCALE_CODE = 4095
TARGET_HZ = 40_000.0
BAND_HZ = (38_000.0, 42_000.0)  # accepted ultrasonic band (typical 40 kHz transducers: +-1 kHz)
SEARCH_HZ = (35_000.0, 45_000.0)  # where the tone is looked for
MIN_VPP_MV = 20.0
MAX_CLIP_FRACTION = 0.001
MIN_TONE_SNR_DB = 15.0
PULL_RESISTOR_OHM = 45_000  # ESP32 datasheet typical internal pull resistor
ASSUMED_RAIL_MV = 3300.0
SWEEP_RATES = [250_000, 500_000, 1_000_000, 2_000_000]
DEFAULT_RATE = 1_000_000
LONG_CAPTURE = 40960
SHORT_CAPTURE = 8192

COLORS = {"none": "tab:red", "down": "tab:blue", "up": "tab:orange"}
LABELS = {"none": "as wired (no pull)", "down": "internal pull-down", "up": "internal pull-up"}
CONTROL_COLOR = "0.45"
CONTROL_LABEL = "control: GPIO33 unconnected, mid-rail"
CONTROL_PLAN = [  # label, gpio, pull, display on?
    ("gpio33_floating", 33, "none", True),
    ("gpio33_display_on", 33, "both", True),
    ("gpio33_display_off", 33, "both", False),
    ("gpio32_display_off", 32, None, False),  # None = same pull as the GPIO32 analysis mode
]


# ---------------------------------------------------------------- serial link


class Probe:
    def __init__(self, port: str):
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = 921600
        self.ser.timeout = 1
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        # Opening the port resets the board (cdc_acm toggles DTR/RTS): wait for the boot to finish.
        deadline = time.time() + 12
        while time.time() < deadline:
            if self.ser.readline().startswith(b"READY"):
                break
        self.info, self.cal_raw, self.cal_mv = self.read_info()

    def command(self, cmd: str, terminator: str, timeout: float = 30) -> list[str]:
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\n").encode())
        lines = []
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self.ser.readline().decode(errors="replace").strip()
            if line:
                lines.append(line)
            if line.startswith(terminator):
                return lines
        raise TimeoutError(f"no {terminator} reply to '{cmd}' (last lines: {lines[-2:]})")

    def read_info(self):
        lines = self.command("info", "CAL")
        info = parse_kv(next(l for l in lines if l.startswith("INFO")))
        pairs = [p.split(":") for p in lines[-1].split()[1:]]
        cal = np.array(pairs, dtype=float)
        return info, cal[:, 0], cal[:, 1]

    def set_pull(self, mode: str):
        self.command(f"pull {mode}", "PULL")
        time.sleep(0.2)

    def set_rate(self, rate: int) -> tuple[bool, float]:
        kv = parse_kv(self.command(f"rate {rate}", "RATE")[-1])
        return kv["ok"] == "1", float(kv["meas"])

    def capture(self, n: int, attempts: int = 3) -> tuple[np.ndarray, float]:
        # The USB/IP serial link occasionally drops a byte; the checksum catches it, then retry.
        for _ in range(attempts):
            lines = self.command(f"cap {n}", "END", timeout=60)
            start = max(i for i, l in enumerate(lines) if l.startswith("CAP"))
            header = parse_kv(lines[start])
            payload = "".join(lines[start + 1 : -1])
            if len(payload) != 4 * int(header["n"]):
                continue
            try:
                words = np.frombuffer(bytes.fromhex(payload), dtype=">u2").astype(np.uint16)
            except ValueError:
                continue
            if int(words.astype(np.uint64).sum()) % 2**32 == int(parse_kv(lines[-1])["sum"]):
                return words, float(header["rate"])
        raise ValueError(f"capture corrupted in transfer {attempts} times in a row")

    def close(self):
        self.ser.close()


def parse_kv(line: str) -> dict[str, str]:
    return dict(tok.split("=", 1) for tok in line.split()[1:] if "=" in tok)


# ---------------------------------------------------------------- signal processing


def decode(words: np.ndarray, channel: int = CHANNEL) -> tuple[np.ndarray, float, bool]:
    """Return (12-bit codes in time order, fraction of words tagged with our channel, swapped?)."""
    valid = float(np.mean((words >> 12) == channel))
    codes = (words & 0x0FFF).astype(np.int32)
    swapped = codes.reshape(-1, 2)[:, ::-1].ravel()  # the I2S FIFO swaps each 16-bit sample pair
    tv = lambda x: np.abs(np.diff(x)).sum()
    use_swapped = bool(tv(swapped) <= tv(codes))
    return (swapped if use_swapped else codes), valid, use_swapped


def repeat_factor(codes: np.ndarray) -> int | None:
    """How many consecutive DMA samples share one ADC conversion (None if the signal never moves)."""
    changes = np.flatnonzero(np.diff(codes) != 0)
    if len(changes) < 100:
        return None
    values, counts = np.unique(np.diff(changes), return_counts=True)
    return int(values[np.argmax(counts)])


def true_samples(codes: np.ndarray, k: int) -> np.ndarray:
    if k <= 1:
        return codes
    changes = np.flatnonzero(np.diff(codes) != 0)
    phase = (changes[0] + 1) % k if len(changes) else 0
    return codes[phase::k]


def amplitude_spectrum(x: np.ndarray, fs: float) -> tuple[np.ndarray, np.ndarray]:
    w = np.hanning(len(x))
    spec = np.abs(np.fft.rfft((x - x.mean()) * w)) * 2 / w.sum()
    return np.fft.rfftfreq(len(x), 1 / fs), spec


def analyze(codes: np.ndarray, mv: np.ndarray, fs: float) -> dict:
    lo, hi = np.percentile(mv, [0.1, 99.9])
    m = {
        "dc_mv": float(mv.mean()),
        "vpp_mv": float(hi - lo),
        "ac_rms_mv": float(mv.std()),
        "clip_fraction": float(np.mean((codes <= 0) | (codes >= FULL_SCALE_CODE))),
        "code_min": int(codes.min()),
        "code_max": int(codes.max()),
        "tone_hz": None,
        "tone_mv": 0.0,
        "tone_snr_db": None,
        "floor_mv": None,
        "strongest_hz": None,
    }
    if np.ptp(codes) == 0:
        return m  # flat line: nothing to analyze
    f, spec = amplitude_spectrum(mv, fs)
    search = np.flatnonzero((f >= SEARCH_HZ[0]) & (f <= SEARCH_HZ[1]))
    k = search[np.argmax(spec[search])]
    a, b, c = np.log(spec[k - 1 : k + 2] + 1e-12)
    p = 0.5 * (a - c) / (a - 2 * b + c) if (a - 2 * b + c) != 0 else 0.0
    local = (f >= 30_000) & (f <= 50_000) & (np.abs(f - f[k]) > 1_000)
    floor = float(np.median(spec[local]))
    above = np.flatnonzero(f > 1_000)
    m.update(
        tone_hz=float((k + p) * fs / len(mv)),
        tone_mv=float(spec[k]),
        tone_snr_db=float(20 * np.log10(spec[k] / floor)),
        floor_mv=floor,
        strongest_hz=float(f[above[np.argmax(spec[above])]]),
    )
    return m


def tone_present(m: dict) -> bool:
    return m["tone_snr_db"] is not None and m["tone_snr_db"] >= MIN_TONE_SNR_DB


# ---------------------------------------------------------------- measurement plan


def measure(probe: Probe, trend_runs: int, long_captures: int) -> dict:
    to_mv = lambda codes: np.interp(codes, probe.cal_raw, probe.cal_mv)
    run = {"info": probe.info, "cal": {"raw": probe.cal_raw.tolist(), "mv": probe.cal_mv.tolist()}}

    print("[1/5] connection test (pull none / down / up)")
    probe.set_rate(DEFAULT_RATE)
    connection = {}
    for mode in ["none", "down", "up"]:
        probe.set_pull(mode)
        words, rate_io = probe.capture(SHORT_CAPTURE)
        codes, valid, _ = decode(words)
        connection[mode] = {"valid": valid, **analyze(codes, to_mv(codes), rate_io)}
        print(f"      {mode:5s} dc={connection[mode]['dc_mv']:.0f} mV vpp={connection[mode]['vpp_mv']:.0f} mV "
              f"clip={connection[mode]['clip_fraction']:.3f}")
    run["connection"] = connection

    # Analyze as wired; if that is pinned to a rail, also analyze with the pull that brings it into range.
    modes = ["none"]
    if connection["none"]["clip_fraction"] > 0.5:
        alt = min(["down", "up"], key=lambda md: connection[md]["clip_fraction"])
        if connection[alt]["clip_fraction"] < 0.5:
            modes.append(alt)
    run["modes"] = modes
    sweep_mode = modes[-1]

    print(f"[2/5] sampler sweep (pull {sweep_mode})")
    probe.set_pull(sweep_mode)
    sweep = []
    for rate in SWEEP_RATES:
        ok, rate_io = probe.set_rate(rate)
        words, _ = probe.capture(16384)
        codes, valid, _ = decode(words)
        k = repeat_factor(codes) if ok else None
        sweep.append({"rate_cfg": rate, "ok": ok, "rate_io": rate_io, "valid": valid, "repeat": k,
                      "rate_true": rate_io / k if k else None})
        print(f"      cfg={rate:>8d} io={rate_io:>10.0f} repeat={k} true={sweep[-1]['rate_true']}")
    run["sweep"] = sweep
    usable = [s for s in sweep if s["repeat"] and s["valid"] > 0.99]
    best = None
    if usable:
        # The true conversion rate tops out; among configs reaching it, the one with the fewest repeated
        # samples gives the longest capture for the same DMA buffer.
        top = max(s["rate_true"] for s in usable)
        best = min((s for s in usable if s["rate_true"] >= 0.99 * top), key=lambda s: s["repeat"])
    main_rate = best["rate_cfg"] if best else DEFAULT_RATE
    k = best["repeat"] if best else 4  # 4 = value observed at the 1 MHz default
    run["main_rate_cfg"], run["repeat"] = main_rate, k

    print(f"[3/5] long captures at cfg {main_rate} Hz (true ~{main_rate / k / 1e3:.0f} kS/s)")
    _, rate_io = probe.set_rate(main_rate)
    fs = rate_io / k
    run["fs_true"] = fs
    captures = {}
    for mode in modes:
        probe.set_pull(mode)
        captures[mode] = []
        for i in range(long_captures):
            words, _ = probe.capture(LONG_CAPTURE)
            codes, valid, swapped = decode(words)
            x = true_samples(codes, k)
            captures[mode].append({"words": words, "codes": x, "mv": to_mv(x), "valid": valid,
                                   "swapped": swapped, **analyze(x, to_mv(x), fs)})
            c = captures[mode][-1]
            print(f"      {mode:5s} #{i} dc={c['dc_mv']:.0f} mV vpp={c['vpp_mv']:.0f} mV "
                  f"tone={c['tone_hz']} Hz snr={c['tone_snr_db']}")
    run["captures"] = captures

    print(f"[4/5] trend ({trend_runs} runs)")
    trend = {mode: [] for mode in modes}
    t0 = time.time()
    for _ in range(trend_runs):
        for mode in modes:
            probe.set_pull(mode)
            words, _ = probe.capture(SHORT_CAPTURE)
            codes, _, _ = decode(words)
            x = true_samples(codes, k)
            trend[mode].append({"t": time.time() - t0, **analyze(x, to_mv(x), fs)})
    run["trend"] = trend

    print("[5/5] interference controls (GPIO33 unconnected, display on/off)")
    controls = {}
    for label, gpio, pull, display in CONTROL_PLAN:
        pull = pull or modes[-1]
        probe.command(f"src {gpio}", "SRC")  # restarts the ADC at the current rate, pull reset to none
        probe.command(f"disp {'on' if display else 'off'}", "DISP")
        probe.set_pull(pull)
        words, _ = probe.capture(LONG_CAPTURE)
        codes, valid, _ = decode(words, CHANNEL if gpio == 32 else CONTROL_CHANNEL)
        x = true_samples(codes, k)
        controls[label] = {"gpio": gpio, "pull": pull, "display": display, "words": words, "codes": x,
                           "mv": to_mv(x), "valid": valid, **analyze(x, to_mv(x), fs)}
        c = controls[label]
        print(f"      {label:20s} dc={c['dc_mv']:.0f} mV vpp={c['vpp_mv']:.0f} mV tone={c['tone_hz']} Hz "
              f"snr={c['tone_snr_db']}")
    run["controls"] = controls
    probe.command("disp on", "DISP")
    probe.set_pull("none")
    probe.command("src 32", "SRC")
    probe.set_rate(DEFAULT_RATE)
    return run


# ---------------------------------------------------------------- graphs


def style_axes(ax):
    ax.grid(True, alpha=0.3)
    ax.spines[["top", "right"]].set_visible(False)


def plot_connection(run, out: Path):
    full_scale = run["cal"]["mv"][-1]
    fig, ax = plt.subplots(figsize=(8, 4.5))
    modes = ["none", "down", "up"]
    for i, mode in enumerate(modes):
        c = run["connection"][mode]
        lo, hi = c["dc_mv"] - c["vpp_mv"] / 2, c["dc_mv"] + c["vpp_mv"] / 2
        ax.bar(i, c["dc_mv"], color=COLORS[mode], alpha=0.8, width=0.55)
        ax.errorbar(i, c["dc_mv"], yerr=[[c["dc_mv"] - lo], [hi - c["dc_mv"]]], color="k", capsize=6)
        ax.text(i, c["dc_mv"] / 2, f"{c['dc_mv']:.0f} mV\nVpp {c['vpp_mv']:.0f} mV\nclipped {c['clip_fraction'] * 100:.0f}%",
                ha="center", va="center", fontsize=10, color="white", fontweight="bold")
    ax.axhline(full_scale, color="k", ls="--", lw=1)
    ax.text(-0.45, full_scale - 30, f"ADC full scale {full_scale:.0f} mV", va="top", fontsize=8)
    ax.axhline(ASSUMED_RAIL_MV, color="grey", ls=":", lw=1)
    ax.text(-0.45, ASSUMED_RAIL_MV + 20, "3.3 V rail", va="bottom", fontsize=8, color="grey")
    ax.set_xlim(-0.5, 2.5)
    ax.set_xticks(range(3), [LABELS[m] for m in modes])
    ax.set_ylabel("GPIO32 voltage (mV)")
    ax.set_ylim(0, 3700)
    ax.set_title("Connection test: GPIO32 level with each internal pull\n(bar = DC, whisker = peak-to-peak)")
    style_axes(ax)
    fig.tight_layout()
    fig.savefig(out / "01_connection_test.png", dpi=120)
    plt.close(fig)


def plot_waveforms(run, out: Path):
    fs = run["fs_true"]
    modes = run["modes"]
    fig, axes = plt.subplots(len(modes), 2, figsize=(13, 3.6 * len(modes)), squeeze=False)
    for row, mode in enumerate(modes):
        c = run["captures"][mode][0]
        t_ms = np.arange(len(c["mv"])) / fs * 1e3
        ax = axes[row, 0]
        zoom = t_ms < 0.5
        ax.plot(t_ms[zoom] * 1e3, c["mv"][zoom], ".-", color=COLORS[mode], lw=0.8, ms=3)
        for t in np.arange(0, 500, 1e6 / TARGET_HZ):
            ax.axvline(t, color="grey", lw=0.5, alpha=0.4)
        ax.set_xlabel("time (us) - grey lines every 25 us = one 40 kHz period")
        ax.set_ylabel("mV")
        ax.set_title(f"{LABELS[mode]}: first 500 us")
        style_axes(ax)
        ax = axes[row, 1]
        ax.plot(t_ms, c["mv"], color=COLORS[mode], lw=0.4)
        ax.set_xlabel("time (ms)")
        ax.set_title(f"{LABELS[mode]}: full capture ({t_ms[-1]:.0f} ms)")
        style_axes(ax)
        for a in axes[row]:
            a.axhline(run["cal"]["mv"][-1], color="k", ls="--", lw=0.8)
            span = max(c["vpp_mv"], 40)
            a.set_ylim(c["dc_mv"] - span, min(c["dc_mv"] + span, run["cal"]["mv"][-1] + span * 0.3))
    fig.suptitle(f"GPIO32 waveform, true sample rate {fs / 1e3:.0f} kS/s (dashed = ADC full scale)")
    fig.tight_layout()
    fig.savefig(out / "02_waveform.png", dpi=120)
    plt.close(fig)


def plot_spectrum(run, out: Path):
    fs = run["fs_true"]
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5))
    for mode in run["modes"]:
        c = run["captures"][mode][0]
        if np.ptp(c["codes"]) == 0:
            axes[0].plot([], [], color=COLORS[mode], label=f"{LABELS[mode]}: flat line (no AC at all)")
            continue
        f, spec = amplitude_spectrum(c["mv"], fs)
        db = 20 * np.log10(spec + 1e-6)
        label = f"{LABELS[mode]}: 35-45 kHz peak {c['tone_mv']:.2f} mV @ {c['tone_hz'] / 1e3:.2f} kHz, SNR {c['tone_snr_db']:.1f} dB"
        for ax in axes:
            ax.plot(f / 1e3, db, color=COLORS[mode], lw=0.5, label=label)
        floor_db = 20 * np.log10(c["floor_mv"])
        axes[1].axhline(floor_db + MIN_TONE_SNR_DB, color=COLORS[mode], ls="--", lw=1,
                        label=f"detection threshold (floor + {MIN_TONE_SNR_DB:.0f} dB)")
    ctl = run["controls"]["gpio33_display_on"]
    if np.ptp(ctl["codes"]) > 0:
        f, spec = amplitude_spectrum(ctl["mv"], fs)
        label = (f"{CONTROL_LABEL}: 35-45 kHz peak {ctl['tone_mv']:.2f} mV @ {ctl['tone_hz'] / 1e3:.2f} kHz, "
                 f"SNR {ctl['tone_snr_db']:.1f} dB")
        for ax in axes:
            ax.plot(f / 1e3, 20 * np.log10(spec + 1e-6), color=CONTROL_COLOR, lw=0.5, alpha=0.8, label=label)
    for ax in axes:
        ax.axvspan(BAND_HZ[0] / 1e3, BAND_HZ[1] / 1e3, color="green", alpha=0.12, label="expected band 38-42 kHz")
        ax.set_xlabel("frequency (kHz)")
        ax.set_ylabel("amplitude (dB re 1 mV)")
        style_axes(ax)
    axes[0].set_xlim(0, fs / 2e3)
    axes[0].set_title("Spectrum, 0 - Nyquist")
    axes[1].set_xlim(30, 50)
    axes[1].set_title("Zoom around 40 kHz")
    handles, labels = axes[1].get_legend_handles_labels()
    uniq = dict(zip(labels, handles))
    axes[1].legend(uniq.values(), uniq.keys(), fontsize=7, loc="lower left", framealpha=0.9)
    h0, l0 = axes[0].get_legend_handles_labels()
    keep = [i for i, l in enumerate(l0) if not l.startswith("expected band")]
    axes[0].legend([h0[i] for i in keep], [l0[i] for i in keep], fontsize=7, loc="lower left", framealpha=0.9)
    for ax in axes:
        ax.set_ylim(-90, 15)
    fig.tight_layout()
    fig.savefig(out / "03_spectrum.png", dpi=120)
    plt.close(fig)


def plot_spectrogram(run, out: Path):
    fs = run["fs_true"]
    mode = run["modes"][-1]
    c = run["captures"][mode][0]
    fig, ax = plt.subplots(figsize=(10, 4.5))
    if np.ptp(c["codes"]) == 0:
        ax.text(0.5, 0.5, "signal is a flat line: no spectrogram", ha="center", transform=ax.transAxes)
    else:
        f, t, sxx = signal.spectrogram(c["mv"] - c["mv"].mean(), fs=fs, nperseg=512, noverlap=384)
        mesh = ax.pcolormesh(t * 1e3, f / 1e3, 10 * np.log10(sxx + 1e-12), shading="auto", cmap="magma")
        fig.colorbar(mesh, ax=ax, label="PSD (dB re 1 mV^2/Hz)")
        ax.axhline(TARGET_HZ / 1e3, color="cyan", ls="--", lw=0.8, label="40 kHz")
        ax.legend(loc="upper right", fontsize=8)
    ax.set_xlabel("time (ms)")
    ax.set_ylabel("frequency (kHz)")
    ax.set_title(f"Spectrogram ({LABELS[mode]})")
    fig.tight_layout()
    fig.savefig(out / "04_spectrogram.png", dpi=120)
    plt.close(fig)


def plot_histogram(run, out: Path):
    fig, ax = plt.subplots(figsize=(9, 4))
    for mode in run["modes"]:
        codes = run["captures"][mode][0]["codes"]
        width = 4 if np.ptp(codes) == 0 else 1
        ax.hist(codes, bins=np.arange(codes.min() - 2 * width, codes.max() + 3 * width, width) - 0.5,
                color=COLORS[mode], alpha=0.8, label=f"{LABELS[mode]} (min {codes.min()}, max {codes.max()})")
        if np.mean(codes >= FULL_SCALE_CODE) > 0.5:
            ax.annotate(f"{LABELS[mode]}: {np.mean(codes >= FULL_SCALE_CODE) * 100:.0f}% of samples at 4095",
                        (FULL_SCALE_CODE, np.sum(codes >= FULL_SCALE_CODE)), xytext=(-260, -10),
                        textcoords="offset points", color=COLORS[mode], fontsize=9,
                        arrowprops={"arrowstyle": "->", "color": COLORS[mode]})
    ax.axvline(FULL_SCALE_CODE, color="k", ls=":", lw=1, label="full scale (4095)")
    ax.set_xlabel("ADC code")
    ax.set_ylabel("samples")
    ax.set_yscale("log")
    ax.set_title("ADC code histogram (a pile-up at 4095 means the input is above the measurable range)")
    ax.legend(fontsize=8)
    style_axes(ax)
    fig.tight_layout()
    fig.savefig(out / "05_histogram.png", dpi=120)
    plt.close(fig)


def plot_trend(run, out: Path):
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for mode in run["modes"]:
        tr = run["trend"][mode]
        t = [p["t"] for p in tr]
        axes[0].plot(t, [p["dc_mv"] for p in tr], "o-", color=COLORS[mode], ms=3, label=LABELS[mode])
        axes[1].plot(t, [p["vpp_mv"] for p in tr], "o-", color=COLORS[mode], ms=3)
        axes[2].plot(t, [p["tone_snr_db"] if p["tone_snr_db"] is not None else np.nan for p in tr], "o-",
                     color=COLORS[mode], ms=3)
    axes[0].axhline(run["cal"]["mv"][-1], color="k", ls="--", lw=0.8)
    axes[0].set_ylabel("DC (mV)")
    axes[0].legend(fontsize=8)
    axes[1].axhline(MIN_VPP_MV, color="k", ls="--", lw=0.8)
    axes[1].set_ylabel("Vpp (mV)")
    axes[2].axhline(MIN_TONE_SNR_DB, color="k", ls="--", lw=0.8)
    axes[2].set_ylabel("35-45 kHz peak SNR (dB)")
    axes[2].set_xlabel("time (s)")
    axes[0].set_title("Trend over repeated captures (dashed = full scale / pass thresholds)")
    for ax in axes:
        style_axes(ax)
    fig.tight_layout()
    fig.savefig(out / "06_trend.png", dpi=120)
    plt.close(fig)


def plot_sampler(run, out: Path):
    sw = run["sweep"]
    fig, ax = plt.subplots(figsize=(8, 4))
    cfg = [s["rate_cfg"] / 1e3 for s in sw]
    ax.plot(cfg, [s["rate_io"] / 1e3 for s in sw], "o-", label="DMA sample rate (measured)")
    ax.plot(cfg, [s["rate_true"] / 1e3 if s["rate_true"] else np.nan for s in sw], "s-",
            label="true ADC conversion rate")
    for s in sw:
        if s["repeat"]:
            ax.annotate(f"x{s['repeat']}", (s["rate_cfg"] / 1e3, s["rate_true"] / 1e3), textcoords="offset points",
                        xytext=(0, 8), ha="center", fontsize=8)
    ax.axhline(2 * TARGET_HZ / 1e3, color="grey", ls=":", label="Nyquist minimum for 40 kHz (80 kS/s)")
    ax.set_xlabel("configured I2S rate (kHz)")
    ax.set_ylabel("rate (kS/s)")
    ax.set_title("ESP32 I2S-ADC sampler: each conversion is repeated 'xN' times in the DMA stream")
    ax.legend(fontsize=8)
    style_axes(ax)
    fig.tight_layout()
    fig.savefig(out / "07_sampler.png", dpi=120)
    plt.close(fig)


def plot_interference(run, out: Path):
    fs = run["fs_true"]
    ctl = run["controls"]
    sig_mode = ctl["gpio32_display_off"]["pull"]
    sig_on = run["captures"][sig_mode][0] if sig_mode in run["captures"] else None
    panels = [
        (f"GPIO32, {LABELS[sig_mode]}", sig_on, ctl["gpio32_display_off"]),
        ("GPIO33, nothing connected (mid-rail)", ctl["gpio33_display_on"], ctl["gpio33_display_off"]),
    ]
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5), sharey=True)
    for ax, (title, on, off) in zip(axes, panels):
        for c, color, state in [(on, "tab:purple", "display on"), (off, "tab:green", "display asleep")]:
            if c is None or np.ptp(c["codes"]) == 0:
                continue
            f, spec = amplitude_spectrum(c["mv"], fs)
            snr = f"{c['tone_snr_db']:.1f} dB" if c["tone_snr_db"] is not None else "n/a"
            ax.plot(f / 1e3, 20 * np.log10(spec + 1e-6), color=color, lw=0.5, alpha=0.85,
                    label=f"{state}: 35-45 kHz peak SNR {snr}")
        ax.axvspan(BAND_HZ[0] / 1e3, BAND_HZ[1] / 1e3, color="green", alpha=0.12)
        ax.set_xlim(20, 100)
        ax.set_ylim(-90, 15)
        ax.set_xlabel("frequency (kHz)")
        ax.set_title(title)
        ax.legend(fontsize=8, loc="lower left", framealpha=0.9)
        style_axes(ax)
    axes[0].set_ylabel("amplitude (dB re 1 mV)")
    fig.suptitle("Where does the ~41 kHz line come from? (green band = expected 38-42 kHz)")
    fig.tight_layout()
    fig.savefig(out / "08_interference.png", dpi=120)
    plt.close(fig)


# ---------------------------------------------------------------- verdict + report


def verdict(run) -> tuple[bool, list[str], list[str]]:
    info = run["info"]
    full_scale = run["cal"]["mv"][-1]
    nat = run["captures"]["none"][0]
    failures, notes = [], []

    if nat["valid"] < 0.99:
        failures.append(f"Only {nat['valid'] * 100:.1f}% of the DMA words come from ADC1 channel 4: sampler problem.")
    if nat["clip_fraction"] > MAX_CLIP_FRACTION:
        failures.append(
            f"As wired, {nat['clip_fraction'] * 100:.1f}% of samples sit at the ADC limit "
            f"(codes {nat['code_min']}-{nat['code_max']}, full scale = {full_scale:.0f} mV). "
            "The input is at or above the measurable range, so no AC component can be seen.")
    if nat["vpp_mv"] < MIN_VPP_MV:
        failures.append(f"As wired, peak-to-peak amplitude is {nat['vpp_mv']:.1f} mV (< {MIN_VPP_MV:.0f} mV).")
    if not tone_present(nat):
        failures.append("As wired, no tone stands out in the 35-45 kHz search band.")
    elif not BAND_HZ[0] <= nat["tone_hz"] <= BAND_HZ[1]:
        failures.append(f"Tone found at {nat['tone_hz']:.0f} Hz, outside the 38-42 kHz band.")

    pcnt_hz = float(info["pcnt_hz"])
    if pcnt_hz == 0:
        notes.append("Hardware edge counter (PCNT, digital input) counted **0 edges in 1 s** at boot: "
                     "the pin never crossed the logic threshold.")
    else:
        notes.append(f"Hardware edge counter (PCNT) measured **{pcnt_hz:.1f} Hz** at boot.")
    oneshot_mv = float(np.interp(float(info["oneshot_mean"]), run["cal"]["raw"], run["cal"]["mv"]))
    notes.append(f"Independent `analogRead()` check at boot: mean code {float(info['oneshot_mean']):.0f} "
                 f"(~{oneshot_mv:.0f} mV), range {info['oneshot_min']}-{info['oneshot_max']}.")

    conn = run["connection"]
    if conn["none"]["clip_fraction"] > 0.5:
        drop = conn["none"]["dc_mv"] - conn["down"]["dc_mv"]
        if conn["down"]["dc_mv"] < 500:
            notes.append("With the internal pull-down the pin falls to ~0 V: GPIO32 is **floating** "
                         "(nothing, or something extremely high-impedance, is connected).")
        else:
            notes.append(
                f"With the internal ~45 kOhm pull-down the pin only drops by ~{drop:.0f} mV to "
                f"{conn['down']['dc_mv']:.0f} mV: GPIO32 is **actively driven high**, not floating.")
            if conn["down"]["dc_mv"] < ASSUMED_RAIL_MV:
                rs = PULL_RESISTOR_OHM * (ASSUMED_RAIL_MV / conn["down"]["dc_mv"] - 1)
                notes.append(f"If that node is tied to 3.3 V, its source resistance is roughly {rs / 1e3:.1f} kOhm "
                             "(rough estimate: the internal pull value varies +-50 %). That is the signature of a "
                             "pull-up resistor (4.7 kOhm is a standard value) with nothing pulling the line down.")
                run["stuck_high_via_resistor"] = True
    if "down" in run["captures"]:
        d = run["captures"]["down"][0]
        if tone_present(d):
            notes.append(f"With the pull-down (signal brought into range) a tone **is** visible at "
                         f"{d['tone_hz']:.0f} Hz, {d['tone_mv']:.2f} mV amplitude, SNR {d['tone_snr_db']:.1f} dB.")
        else:
            notes.append(
                f"With the pull-down the pin is inside the ADC range ({d['dc_mv']:.0f} mV DC, {d['ac_rms_mv']:.1f} mV RMS noise) "
                f"and there is **still no 40 kHz tone**: the strongest 35-45 kHz component is "
                f"{d['tone_mv']:.2f} mV at {d['tone_hz'] / 1e3:.2f} kHz, only {d['tone_snr_db']:.1f} dB above the noise floor "
                f"(threshold {MIN_TONE_SNR_DB:.0f} dB).")
    ctl = run["controls"]
    on, off = ctl["gpio33_display_on"], ctl["gpio33_display_off"]
    sig_off = ctl["gpio32_display_off"]
    if tone_present(on):
        where = (f"also appears on GPIO33, where nothing is connected ({on['tone_mv']:.2f} mV at "
                 f"{on['tone_hz'] / 1e3:.2f} kHz, SNR {on['tone_snr_db']:.1f} dB)")
        if not tone_present(off):
            notes.append(f"The ~41 kHz line {where}, and disappears when the ST7789 display is put to sleep "
                         f"(SNR {off['tone_snr_db']:.1f} dB): it is switching noise from the display's charge pump, "
                         "**not** the ultrasonic signal.")
        else:
            notes.append(f"The ~41 kHz line {where}, even with the display asleep: at least part of it is coupled through "
                         "the shared supply/ground (board or external circuit), not carried by the GPIO32 signal.")
    elif "down" in run["captures"] and tone_present(run["captures"]["down"][0]):
        notes.append("GPIO33 (nothing connected) shows no 35-45 kHz line: the component seen with the pull-down "
                     "is specific to the GPIO32 wire.")
    sig_caps = run["captures"].get(sig_off["pull"], []) + [sig_off]
    if tone_present(on) and sig_caps and all(tone_present(c) for c in sig_caps):
        sig_mv = float(np.mean([c["tone_mv"] for c in sig_caps]))
        ratio = sig_mv / on["tone_mv"]
        if ratio >= 3:
            notes.append(
                f"On GPIO32 that line averages {sig_mv:.2f} mV, {ratio:.1f}x the unconnected pin ({on['tone_mv']:.2f} mV): "
                "part of it probably arrives through the GPIO32 wire, i.e. a ~41 kHz source may be running upstream. "
                "Even so, it is about a millivolt on a line pinned at the top of the range, not a usable receiver signal.")
        else:
            notes.append(
                f"On GPIO32 that line averages {sig_mv:.2f} mV, {ratio:.1f}x the unconnected pin ({on['tone_mv']:.2f} mV). "
                "Given the different DC level and source impedance, shared supply/ground coupling can explain this: "
                "it is no evidence that the ultrasonic signal reaches GPIO32.")
    if sig_off["clip_fraction"] < 0.5:
        state = "still shows" if tone_present(sig_off) else "shows **no**"
        notes.append(f"With the display asleep, GPIO32 ({LABELS[sig_off['pull']]}) {state} 35-45 kHz component "
                     f"(best peak {sig_off['tone_mv']:.2f} mV at {sig_off['tone_hz'] / 1e3:.2f} kHz, "
                     f"SNR {sig_off['tone_snr_db']:.1f} dB).")
    return not failures, failures, notes


HARDWARE_CHECKS = """## What to check on the hardware

1. **Measure GPIO32 with a multimeter** while the ultrasonic circuit runs. A properly biased analog receiver output idles near mid-supply (~1.65 V on 3.3 V). About 3.3 V means the output is stuck at the rail. **Above 3.6 V means the module runs from 5 V: disconnect it**, because it can damage the ESP32 (power it from 3.3 V, or add a resistor divider).
2. **Check what GPIO32 is wired to.** The measurement looks like a ~4.7 kOhm pull-up to 3.3 V with nothing driving the line. That fits a wire landing on the module's VCC/pull-up pin instead of the amplifier output, or an open-collector comparator output (e.g. LM393) that never switches.
3. **If the module outputs a comparator/echo signal** (idle high, pulled low only when an echo is detected), it never toggled here: 0 edges counted in 1 s. Check that the transmitter fires and that something reflects back to the receiver.
4. **If it is meant to be an analog amplifier output,** check its bias: the amplifier needs a mid-supply reference (VCC/2 divider) and the transducer must be AC-coupled through a capacitor. A missing or shorted coupling capacitor, a wrong bias or too much gain drives the output to the positive rail.
5. **Check the 40 kHz source on its own** with an oscilloscope at the transmitter and at the receiver output, before the ESP32. The faint ~41 kHz line seen here also shows up on an unconnected pin, so it cannot confirm the transmitter is running.

"""


def write_report(run, out: Path, timestamp: str):
    ok, failures, notes = verdict(run)
    info = run["info"]
    rows = []
    for mode in run["modes"]:
        for i, c in enumerate(run["captures"][mode]):
            tone = f"{c['tone_hz'] / 1e3:.2f} kHz / {c['tone_mv']:.2f} mV / {c['tone_snr_db']:.1f} dB" \
                if c["tone_hz"] else "n/a (flat line)"
            rows.append(f"| {LABELS[mode]} #{i} | {c['dc_mv']:.0f} | {c['vpp_mv']:.1f} | {c['ac_rms_mv']:.2f} | "
                        f"{c['clip_fraction'] * 100:.1f} % | {c['code_min']}-{c['code_max']} | {tone} |")
    sweep_rows = "\n".join(
        f"| {s['rate_cfg'] / 1e3:.0f} | {s['rate_io'] / 1e3:.1f} | {s['repeat'] or 'n/a'} | "
        f"{(s['rate_true'] / 1e3) if s['rate_true'] else float('nan'):.1f} |" for s in run["sweep"])
    status = "PASS - 40 kHz signal present and measurable" if ok else "FAIL - no usable 40 kHz signal on GPIO32"
    nl = "\n"
    report = f"""# GPIO32 ultrasonic signal check

- **Date:** {timestamp}
- **Board:** LilyGO T-Display v1.1 (ESP32-D0WDQ6-V3), firmware `ultrasonic_probe`
- **Input:** GPIO32 = ADC1 channel 4, 12 dB attenuation (range ~{run['cal']['mv'][0]:.0f}-{run['cal']['mv'][-1]:.0f} mV, eFuse-calibrated)
- **Sampler:** I2S built-in ADC DMA, configured {run['main_rate_cfg'] / 1e3:.0f} kHz, true conversion rate **{run['fs_true'] / 1e3:.1f} kS/s** ({run['fs_true'] / TARGET_HZ:.1f} samples per 40 kHz period)

## Verdict: {status}

{nl.join(f'- {f}' for f in failures) if failures else '- All checks passed.'}

### Supporting evidence

{nl.join(f'- {n}' for n in notes)}

{HARDWARE_CHECKS if run.get("stuck_high_via_resistor") and not ok else ""}## Measurements

| Capture | DC (mV) | Vpp (mV) | AC RMS (mV) | Clipped | Code range | 35-45 kHz peak (freq / amplitude / SNR) |
|---|---|---|---|---|---|---|
{nl.join(rows)}

Pass criteria (as wired): clipped < {MAX_CLIP_FRACTION * 100:.1f} %, Vpp >= {MIN_VPP_MV:.0f} mV, a tone >= {MIN_TONE_SNR_DB:.0f} dB above the local noise floor inside {BAND_HZ[0] / 1e3:.0f}-{BAND_HZ[1] / 1e3:.0f} kHz.

### Connection test

| Pull | DC (mV) | Vpp (mV) | Clipped |
|---|---|---|---|
{nl.join(f"| {LABELS[m]} | {c['dc_mv']:.0f} | {c['vpp_mv']:.1f} | {c['clip_fraction'] * 100:.1f} % |" for m, c in run['connection'].items())}

### Interference controls

GPIO33 has nothing connected; with both internal pulls it sits at mid-rail through ~22 kOhm. The display is put to sleep with `SLPIN`, which also stops its internal charge pumps.

| Capture | Pin | Pull | Display | DC (mV) | Vpp (mV) | 35-45 kHz peak (freq / amplitude / SNR) |
|---|---|---|---|---|---|---|
{nl.join(f"| {lbl} | GPIO{c['gpio']} | {c['pull']} | {'on' if c['display'] else 'asleep'} | {c['dc_mv']:.0f} | {c['vpp_mv']:.1f} | " + (f"{c['tone_hz'] / 1e3:.2f} kHz / {c['tone_mv']:.2f} mV / {c['tone_snr_db']:.1f} dB" if c['tone_hz'] else "n/a (flat line)") + " |" for lbl, c in run['controls'].items())}

### Sampler sweep

The ESP32 I2S-ADC path repeats each conversion several times in the DMA stream. The true rate is the DMA rate divided by that repeat factor.

| Configured (kHz) | DMA rate (kS/s) | Repeat | True rate (kS/s) |
|---|---|---|---|
{sweep_rows}

Boot info: `PCNT counts per 250 ms gate = {info['pcnt_counts']}`, `analogRead mean = {info['oneshot_mean']}`, `ADC cal = {info['cal']} (Vref {info['vref']} mV)`.

## Graphs

![Connection test](01_connection_test.png)
![Waveform](02_waveform.png)
![Spectrum](03_spectrum.png)
![Spectrogram](04_spectrogram.png)
![Histogram](05_histogram.png)
![Trend](06_trend.png)
![Sampler](07_sampler.png)
![Interference controls](08_interference.png)

Raw captures: `data/*.npz` (`words` = raw 16-bit DMA words, `codes`/`mv` = un-swapped true samples, `fs` = true sample rate).
Re-run: `cd ultrasonic_probe && uv run tools/analyze_signal.py`.
"""
    (out / "REPORT.md").write_text(report)
    return ok, failures, notes


def save(run, out: Path, timestamp: str, ok: bool, failures, notes):
    data = out / "data"
    data.mkdir(exist_ok=True)
    for mode, caps in run["captures"].items():
        for i, c in enumerate(caps):
            np.savez_compressed(data / f"capture_{mode}_{i}.npz", words=c["words"], codes=c["codes"], mv=c["mv"],
                                fs=run["fs_true"], pull=mode)
    for label, c in run["controls"].items():
        np.savez_compressed(data / f"control_{label}.npz", words=c["words"], codes=c["codes"], mv=c["mv"],
                            fs=run["fs_true"], pull=c["pull"], gpio=c["gpio"], display=c["display"])
    strip = lambda c: {k: v for k, v in c.items() if k not in ("words", "codes", "mv")}
    summary = {
        "timestamp": timestamp,
        "verdict": "PASS" if ok else "FAIL",
        "failures": failures,
        "notes": notes,
        "info": run["info"],
        "fs_true": run["fs_true"],
        "main_rate_cfg": run["main_rate_cfg"],
        "repeat": run["repeat"],
        "connection": run["connection"],
        "sweep": run["sweep"],
        "modes": {m: [strip(c) for c in caps] for m, caps in run["captures"].items()},
        "trend": run["trend"],
        "controls": {m: strip(c) for m, c in run["controls"].items()},
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent.parent / "results"))
    ap.add_argument("--trend", type=int, default=20, help="number of trend runs")
    ap.add_argument("--captures", type=int, default=2, help="long captures per pull mode")
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().isoformat(timespec="seconds")
    probe = Probe(args.port)
    try:
        run = measure(probe, args.trend, args.captures)
    finally:
        probe.close()

    plot_connection(run, out)
    plot_waveforms(run, out)
    plot_spectrum(run, out)
    plot_spectrogram(run, out)
    plot_histogram(run, out)
    plot_trend(run, out)
    plot_sampler(run, out)
    plot_interference(run, out)
    ok, failures, notes = write_report(run, out, timestamp)
    save(run, out, timestamp, ok, failures, notes)

    print(f"\nVERDICT: {'PASS' if ok else 'FAIL'}")
    for line in failures + notes:
        print(f"  - {line}")
    print(f"Report: {out / 'REPORT.md'}")


if __name__ == "__main__":
    main()
