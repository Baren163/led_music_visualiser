#!/usr/bin/env python3
"""
LED music visualiser - laptop bridge.

Polls Spotify once per second for what is playing, and keeps the ESP32-C3
LED controller in sync over Bluetooth LE:

    iPhone (Spotify) --> Spotify cloud <-- this script --BLE--> ESP32-C3 --> LEDs

* Sends a STATE message (track ID, playing flag, position) every poll.
* If the ESP32 does not have the current track cached, packs the two
  generator outputs in songs/<spotify_track_id>/ into one song container
  and sends it over BLE.
* Reconnects automatically when the BLE link drops.

While running, type into the console:
    +N / -N     shift the latency offset by N ms (e.g. +20)
    o N         set the latency offset to N ms
    s           show ESP32 status
    q           quit

The message formats must match main/protocol.h in the firmware.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import re
import shutil
import struct
import sys
import threading
import time
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import requests
import spotipy
from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError
from spotipy.oauth2 import SpotifyOAuth


log = logging.getLogger("visualiser")


# ============================================================
# PROTOCOL (keep in sync with main/protocol.h)
# ============================================================

SERVICE_UUID = "a3c87500-8ed3-4bdf-8a39-a01bebede295"
STATE_UUID = "a3c87501-8ed3-4bdf-8a39-a01bebede295"
FILE_UUID = "a3c87502-8ed3-4bdf-8a39-a01bebede295"
STATUS_UUID = "a3c87503-8ed3-4bdf-8a39-a01bebede295"

MSG_STATE = 0x01
MSG_SET_OFFSET = 0x02
MSG_FILE_START = 0x10
MSG_FILE_CHUNK = 0x11
MSG_FILE_END = 0x12

FLAG_PLAYING = 0x01
FLAG_FILE_AVAILABLE = 0x02

TRACK_ID_LEN = 22

# Song container: header + frames + notes (see protocol.h)
SONG_MAGIC = b"LVS2"
SONG_FORMAT_VERSION = 1
SONG_HEADER_FORMAT = "<4sBBHI"      # magic, version, reserved, note_count, frame_count (12 bytes)
SONG_FRAME_SIZE = 6
SONG_NOTE_SIZE = 7
SONG_MAX_NOTES = 9000

# Files written by generate_song_vis.py, kept in songs/<track_id>/
FRAMES_FILENAME = "led_visualisation_data.bin"
NOTES_FILENAME = "led_other_notes.bin"

STATE_FORMAT = "<B22sBIH"           # 30 bytes
SET_OFFSET_FORMAT = "<Bh"           # 3 bytes
FILE_START_FORMAT = "<B22sII"       # 31 bytes
FILE_CHUNK_HEADER = "<BI"           # 5 bytes + data
STATUS_FORMAT = "<22sBBIHh"         # 32 bytes

FILE_CHUNK_HEADER_LEN = struct.calcsize(FILE_CHUNK_HEADER)
PROTO_MAX_WRITE_LEN = 256
ATT_HEADER_LEN = 3

TRANSFER_IDLE, TRANSFER_RECEIVING, TRANSFER_OK, TRANSFER_ERROR = range(4)
TRANSFER_NAMES = {0: "idle", 1: "receiving", 2: "ok", 3: "error"}

ERROR_NAMES = {
    0: "none",
    1: "bad message",
    2: "not receiving",
    3: "bad chunk offset",
    4: "bad file size",
    5: "file too large for storage",
    6: "no space",
    7: "flash write failed",
    8: "size mismatch",
    9: "CRC mismatch",
    10: "rename failed",
    11: "timeout",
    12: "aborted",
    13: "invalid song file",
}

TRACK_ID_RE = re.compile(r"^[0-9A-Za-z]{22}$")

MAX_TRANSFER_ATTEMPTS = 3


# ============================================================
# DATA TYPES
# ============================================================

@dataclass(frozen=True)
class Playback:
    """One Spotify playback-state sample."""

    track_id: Optional[str]     # None for nothing / podcasts / local files
    title: str
    playing: bool
    progress_ms: int
    duration_ms: int
    measured_at: float          # time.monotonic() at the request midpoint

    def position_ms(self, now: Optional[float] = None) -> int:
        """Estimated playback position at `now`."""
        if not self.playing:
            return self.progress_ms

        now = time.monotonic() if now is None else now
        position = self.progress_ms + int((now - self.measured_at) * 1000)

        if self.duration_ms:
            position = min(position, self.duration_ms)

        return max(position, 0)


@dataclass(frozen=True)
class EspStatus:
    track_id: Optional[str]
    transfer_state: int
    error_code: int
    bytes_received: int
    ack_seq: int
    offset_ms: int

    @classmethod
    def from_bytes(cls, data: bytes) -> "EspStatus":
        track, state, error, received, ack, offset = struct.unpack(
            STATUS_FORMAT, bytes(data[: struct.calcsize(STATUS_FORMAT)])
        )
        track_id = track.rstrip(b"\0").decode("ascii", "replace") or None
        return cls(track_id, state, error, received, ack, offset)

    def describe(self) -> str:
        return (
            f"loaded={self.track_id or '-'} "
            f"transfer={TRANSFER_NAMES.get(self.transfer_state, '?')} "
            f"last_error={ERROR_NAMES.get(self.error_code, self.error_code)} "
            f"bytes={self.bytes_received} offset={self.offset_ms} ms"
        )


def encode_track_id(track_id: Optional[str]) -> bytes:
    if not track_id:
        return b"\0" * TRACK_ID_LEN
    return track_id.encode("ascii")[:TRACK_ID_LEN].ljust(TRACK_ID_LEN, b"\0")


def check_song_folder(folder: Path) -> Optional[str]:
    """Returns why the generator output in `folder` is unusable, or None if it is fine."""
    frames = folder / FRAMES_FILENAME
    notes = folder / NOTES_FILENAME

    for path in (frames, notes):
        if not path.is_file():
            return f"{path.name} is missing"

    frames_size = frames.stat().st_size
    notes_size = notes.stat().st_size

    if frames_size == 0 or frames_size % SONG_FRAME_SIZE != 0:
        return f"{frames.name} is {frames_size} bytes, not a multiple of {SONG_FRAME_SIZE}"

    if notes_size % SONG_NOTE_SIZE != 0:
        return f"{notes.name} is {notes_size} bytes, not a multiple of {SONG_NOTE_SIZE}"

    if notes_size // SONG_NOTE_SIZE > SONG_MAX_NOTES:
        return f"{notes.name} has more than {SONG_MAX_NOTES} notes"

    return None


def pack_song(folder: Path) -> bytes:
    """Packs the two generator binaries into one song container."""
    frames = (folder / FRAMES_FILENAME).read_bytes()
    notes = (folder / NOTES_FILENAME).read_bytes()

    header = struct.pack(
        SONG_HEADER_FORMAT,
        SONG_MAGIC,
        SONG_FORMAT_VERSION,
        0,
        len(notes) // SONG_NOTE_SIZE,
        len(frames) // SONG_FRAME_SIZE,
    )

    return header + frames + notes


# ============================================================
# SPOTIFY
# ============================================================

class SpotifyPoller:
    """Blocking Spotify client; called from a worker thread."""

    SCOPE = "user-read-playback-state"

    def __init__(self, cfg: dict, cache_path: Path):
        auth = SpotifyOAuth(
            client_id=cfg.get("spotify_client_id"),
            client_secret=cfg.get("spotify_client_secret"),
            redirect_uri=cfg.get("spotify_redirect_uri", "http://127.0.0.1:8888/callback"),
            scope=self.SCOPE,
            cache_path=str(cache_path),
            open_browser=True,
        )

        # No automatic retries: a slow or rate-limited request should
        # not stall the 1 s loop. Errors are handled by the caller.
        self.sp = spotipy.Spotify(
            auth_manager=auth,
            requests_timeout=5,
            retries=0,
            status_retries=0,
        )

    def login(self) -> None:
        """Forces the OAuth flow (browser) on first run; token is cached."""
        self.sp.current_playback()

    def poll(self) -> Playback:
        start = time.monotonic()
        pb = self.sp.current_playback()
        end = time.monotonic()

        # progress_ms is Spotify's position at some point during the
        # request; the midpoint is the best estimate of when.
        midpoint = (start + end) / 2

        if not pb or not pb.get("item"):
            return Playback(None, "", False, 0, 0, midpoint)

        item = pb["item"]
        is_track = item.get("type") == "track" and not item.get("is_local")
        track_id = item.get("id") if is_track else None

        artists = ", ".join(a.get("name", "?") for a in item.get("artists", []))
        title = f"{item.get('name', '?')} - {artists}" if artists else item.get("name", "?")

        return Playback(
            track_id=track_id,
            title=title,
            playing=bool(pb.get("is_playing")),
            progress_ms=int(pb.get("progress_ms") or 0),
            duration_ms=int(item.get("duration_ms") or 0),
            measured_at=midpoint,
        )


# ============================================================
# BLE LINK
# ============================================================

class EspLink:
    """Owns the BLE connection to the ESP32 and keeps it alive."""

    def __init__(self, device_name: str, on_connected: Callable[[], "asyncio.Future"]):
        self.device_name = device_name
        self.on_connected = on_connected

        self.client: Optional[BleakClient] = None
        self.status: Optional[EspStatus] = None
        self.chunk_size = 20
        self.seq = 0

        self._status_version = 0
        self._status_event = asyncio.Event()
        self._write_lock = asyncio.Lock()

    @property
    def connected(self) -> bool:
        return self.client is not None and self.client.is_connected

    # --------------------------------------------------------
    # Connection management
    # --------------------------------------------------------

    def _matches(self, device, adv) -> bool:
        uuids = [u.lower() for u in (adv.service_uuids or [])]
        return adv.local_name == self.device_name or SERVICE_UUID in uuids

    async def run(self) -> None:
        """Scan, connect, and reconnect forever."""
        loop = asyncio.get_running_loop()

        while True:
            client = None

            try:
                log.info("Scanning for %s ...", self.device_name)
                device = await BleakScanner.find_device_by_filter(self._matches, timeout=10.0)

                if device is None:
                    continue

                disconnected = asyncio.Event()

                def on_disconnect(_client):
                    loop.call_soon_threadsafe(disconnected.set)

                kwargs = {}
                if sys.platform == "win32":
                    # Avoid stale GATT tables after reflashing the firmware.
                    kwargs["winrt"] = {"use_cached_services": False}

                client = BleakClient(
                    device,
                    disconnected_callback=on_disconnect,
                    timeout=20.0,
                    **kwargs,
                )

                log.info("Connecting to %s (%s)", device.name, device.address)
                await client.connect()

                await self._setup(client)
                self.client = client

                log.info("Connected. %s", self.status.describe() if self.status else "")
                await self.on_connected()

                await disconnected.wait()
                log.warning("BLE connection lost")

            except (BleakError, asyncio.TimeoutError, OSError) as e:
                log.warning("BLE error: %s", e)

            finally:
                self.client = None

                if client is not None:
                    try:
                        await client.disconnect()
                    except Exception:
                        pass

            await asyncio.sleep(2.0)

    async def _setup(self, client: BleakClient) -> None:
        self.chunk_size = await self._negotiate_mtu(client)
        self.seq = 0    # ESP32 resets its seq tracking on every connection

        await client.start_notify(STATUS_UUID, self._on_status)

        raw = await client.read_gatt_char(STATUS_UUID)
        self._set_status(EspStatus.from_bytes(raw))

    async def _negotiate_mtu(self, client: BleakClient) -> int:
        # BlueZ needs the MTU exchange requested explicitly. Windows does
        # the exchange itself straight after connecting, so just wait for
        # the negotiated value to appear.
        backend = getattr(client, "_backend", None)

        if hasattr(backend, "_acquire_mtu"):
            try:
                await backend._acquire_mtu()
            except Exception:
                pass

        deadline = time.monotonic() + 3.0
        mtu = client.mtu_size

        while mtu <= 23 and time.monotonic() < deadline:
            await asyncio.sleep(0.1)
            mtu = client.mtu_size

        chunk = min(mtu - ATT_HEADER_LEN, PROTO_MAX_WRITE_LEN) - FILE_CHUNK_HEADER_LEN

        if mtu <= 23:
            log.warning("MTU stayed at %d; file transfers will be slow", mtu)
        else:
            log.info("MTU %d -> %d bytes per file chunk", mtu, chunk)

        return chunk

    # --------------------------------------------------------
    # STATUS notifications
    # --------------------------------------------------------

    def _set_status(self, status: EspStatus) -> None:
        self.status = status
        self._status_version += 1
        self._status_event.set()

    def _on_status(self, _sender, data: bytearray) -> None:
        try:
            self._set_status(EspStatus.from_bytes(data))
        except struct.error:
            log.warning("Malformed STATUS notification (%d bytes)", len(data))

    async def wait_status(
        self,
        predicate: Callable[[EspStatus], bool],
        timeout: float,
        since_version: int = -1,
    ) -> Optional[EspStatus]:
        """Waits for a STATUS newer than `since_version` matching `predicate`."""
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout

        while True:
            self._status_event.clear()

            if (self.status is not None
                    and self._status_version > since_version
                    and predicate(self.status)):
                return self.status

            remaining = deadline - loop.time()

            if remaining <= 0 or not self.connected:
                return None

            try:
                await asyncio.wait_for(self._status_event.wait(), remaining)
            except asyncio.TimeoutError:
                pass

    # --------------------------------------------------------
    # Messages
    # --------------------------------------------------------

    async def _write(self, uuid: str, data: bytes) -> None:
        async with self._write_lock:
            client = self.client

            if client is None or not client.is_connected:
                raise BleakError("not connected")

            # Write-with-response: Windows waits for the ESP32 to accept
            # each write, which is our flow control.
            await client.write_gatt_char(uuid, data, response=True)

    async def send_state(
        self,
        track_id: Optional[str],
        playing: bool,
        file_available: bool,
        position_ms: int,
    ) -> int:
        self.seq = (self.seq + 1) & 0xFFFF

        flags = (FLAG_PLAYING if playing else 0) | (FLAG_FILE_AVAILABLE if file_available else 0)
        position_ms = max(0, min(position_ms, 0xFFFFFFFF))

        await self._write(
            STATE_UUID,
            struct.pack(STATE_FORMAT, MSG_STATE, encode_track_id(track_id), flags, position_ms, self.seq),
        )

        return self.seq

    async def send_offset(self, offset_ms: int) -> None:
        offset_ms = max(-32768, min(offset_ms, 32767))
        await self._write(STATE_UUID, struct.pack(SET_OFFSET_FORMAT, MSG_SET_OFFSET, offset_ms))

    async def transfer_file(
        self,
        track_id: str,
        data: bytes,
        still_wanted: Callable[[], bool],
    ) -> bool:
        """Sends one song file. Returns True once the ESP32 has committed it."""
        crc = zlib.crc32(data) & 0xFFFFFFFF
        size = len(data)

        # --- FILE_START --------------------------------------------------
        version = self._status_version
        await self._write(
            FILE_UUID,
            struct.pack(FILE_START_FORMAT, MSG_FILE_START, encode_track_id(track_id), size, crc),
        )

        # The ESP32 may need to evict old songs first, which takes a moment.
        status = await self.wait_status(
            lambda s: s.transfer_state in (TRANSFER_RECEIVING, TRANSFER_ERROR),
            timeout=10.0,
            since_version=version,
        )

        if status is None or status.transfer_state != TRANSFER_RECEIVING:
            self._log_transfer_error(status)
            return False

        start_version = self._status_version

        # --- FILE_CHUNK --------------------------------------------------
        started = time.monotonic()
        next_report = 0.25

        for offset in range(0, size, self.chunk_size):
            if not still_wanted():
                log.info("Track changed, abandoning transfer")
                return False

            if self._status_version > start_version and self.status.transfer_state == TRANSFER_ERROR:
                self._log_transfer_error(self.status)
                return False

            chunk = data[offset: offset + self.chunk_size]
            await self._write(FILE_UUID, struct.pack(FILE_CHUNK_HEADER, MSG_FILE_CHUNK, offset) + chunk)

            done = (offset + len(chunk)) / size
            if done >= next_report:
                log.info("  %3d%%", int(done * 100))
                next_report += 0.25

        # --- FILE_END ----------------------------------------------------
        version = self._status_version
        await self._write(FILE_UUID, bytes([MSG_FILE_END]))

        status = await self.wait_status(
            lambda s: s.transfer_state in (TRANSFER_OK, TRANSFER_ERROR),
            timeout=15.0,
            since_version=version,
        )

        if status is None or status.transfer_state != TRANSFER_OK:
            self._log_transfer_error(status)
            return False

        elapsed = max(time.monotonic() - started, 1e-3)
        log.info("Transfer complete: %d bytes in %.1f s (%.1f kB/s)", size, elapsed, size / elapsed / 1000)
        return True

    @staticmethod
    def _log_transfer_error(status: Optional[EspStatus]) -> None:
        if status is None:
            log.error("Transfer failed: no response from ESP32")
        else:
            log.error("Transfer failed: %s", ERROR_NAMES.get(status.error_code, status.error_code))


# ============================================================
# BRIDGE (poll loop + sync loop + console)
# ============================================================

class Bridge:
    def __init__(self, cfg: dict, args: argparse.Namespace, base_dir: Path):
        self.songs_dir = (base_dir / cfg.get("songs_dir", "songs")).resolve()
        self.poll_interval = float(cfg.get("poll_interval_s", 1.0))
        self.initial_offset = args.offset

        self.poller = SpotifyPoller(cfg, base_dir / ".spotify_cache")
        self.link = EspLink(cfg.get("ble_device_name", "LED-Visualiser"), self._on_connected)

        self.playback: Optional[Playback] = None
        self._poll_event = asyncio.Event()
        self._stop = asyncio.Event()
        self._failures: dict[str, int] = {}
        self._warned_bad_files: set[str] = set()

    # --------------------------------------------------------

    async def run(self) -> None:
        log.info("Songs folder: %s", self.songs_dir)

        tasks = [
            asyncio.create_task(self.link.run()),
            asyncio.create_task(self._poll_loop()),
            asyncio.create_task(self._sync_loop()),
            asyncio.create_task(self._console_loop()),
        ]

        try:
            await self._stop.wait()
        finally:
            for task in tasks:
                task.cancel()

            await self._shutdown()

    async def _shutdown(self) -> None:
        if not self.link.connected:
            return

        try:
            # Turn the strip off on exit.
            await self.link.send_state(None, False, False, 0)
            await self.link.client.disconnect()
        except Exception:
            pass

    async def _on_connected(self) -> None:
        if self.initial_offset is not None:
            log.info("Setting latency offset to %d ms", self.initial_offset)
            await self.link.send_offset(self.initial_offset)
            self.initial_offset = None

        self._poll_event.set()      # sync straight away

    # --------------------------------------------------------
    # Spotify polling
    # --------------------------------------------------------

    async def _poll_loop(self) -> None:
        loop = asyncio.get_running_loop()

        while True:
            started = loop.time()
            delay = self.poll_interval

            try:
                playback = await asyncio.to_thread(self.poller.poll)
                self._log_track_change(playback)
                self.playback = playback
                self._poll_event.set()

            except spotipy.SpotifyException as e:
                if e.http_status == 429:
                    retry_after = int((e.headers or {}).get("Retry-After", 5))
                    log.warning("Spotify rate limit, waiting %d s", retry_after)
                    delay = max(delay, retry_after)
                else:
                    log.warning("Spotify error: %s", e)

            except (requests.RequestException, OSError) as e:
                log.warning("Spotify request failed: %s", e)

            await asyncio.sleep(max(0.0, started + delay - loop.time()))

    def _log_track_change(self, playback: Playback) -> None:
        previous = self.playback

        if previous is not None and previous.track_id == playback.track_id and previous.title == playback.title:
            return

        if playback.track_id is None:
            log.info("Now playing: %s", playback.title or "nothing")
        else:
            has_file = self._song_folder(playback.track_id) is not None
            log.info(
                "Now playing: %s [%s] %s",
                playback.title,
                playback.track_id,
                "" if has_file else "(no visualisation file)",
            )

    # --------------------------------------------------------
    # ESP32 sync
    # --------------------------------------------------------

    def _song_folder(self, track_id: Optional[str]) -> Optional[Path]:
        """songs/<track_id>/ if it holds valid generator output."""
        if not track_id:
            return None

        folder = self.songs_dir / track_id

        if not folder.is_dir():
            return None

        problem = check_song_folder(folder)

        if problem is not None:
            if track_id not in self._warned_bad_files:
                log.warning("Ignoring songs/%s: %s", track_id, problem)
                self._warned_bad_files.add(track_id)
            return None

        return folder

    async def _sync_loop(self) -> None:
        while True:
            await self._poll_event.wait()
            self._poll_event.clear()

            if not self.link.connected or self.playback is None:
                continue

            try:
                await self._sync_once()
            except (BleakError, asyncio.TimeoutError, OSError) as e:
                log.warning("Sync failed: %s", e)

    async def _send_current_state(self) -> int:
        pb = self.playback
        folder = self._song_folder(pb.track_id)

        return await self.link.send_state(
            pb.track_id,
            pb.playing,
            folder is not None,
            pb.position_ms(),
        )

    async def _sync_once(self) -> None:
        track_id = self.playback.track_id
        folder = self._song_folder(track_id)

        seq = await self._send_current_state()
        status = await self.link.wait_status(lambda s: s.ack_seq == seq, timeout=1.0)

        if status is None:
            log.debug("No STATUS ack for seq %d", seq)
            return

        # Does the ESP32 need the file?
        if folder is None or status.track_id == track_id:
            return

        if self._failures.get(track_id, 0) >= MAX_TRANSFER_ATTEMPTS:
            return

        data = pack_song(folder)
        log.info("Sending %s (%d bytes, %d byte chunks)", track_id, len(data), self.link.chunk_size)

        ok = await self.link.transfer_file(
            track_id,
            data,
            still_wanted=lambda: self.playback is not None and self.playback.track_id == track_id,
        )

        if ok:
            self._failures.pop(track_id, None)
        else:
            self._failures[track_id] = self._failures.get(track_id, 0) + 1

            if self._failures[track_id] >= MAX_TRANSFER_ATTEMPTS:
                log.error("Giving up on %s after %d attempts", track_id, MAX_TRANSFER_ATTEMPTS)

        # Send state immediately so the ESP32 opens the new file at the
        # right position instead of waiting for the next poll.
        if self.link.connected and self.playback is not None:
            await self._send_current_state()

    # --------------------------------------------------------
    # Console commands
    # --------------------------------------------------------

    async def _console_loop(self) -> None:
        loop = asyncio.get_running_loop()
        lines: asyncio.Queue[str] = asyncio.Queue()

        def reader() -> None:
            for line in sys.stdin:
                loop.call_soon_threadsafe(lines.put_nowait, line.strip())

        threading.Thread(target=reader, daemon=True).start()

        while True:
            line = (await lines.get()).lower()

            if not line:
                continue

            try:
                await self._handle_command(line)
            except (BleakError, asyncio.TimeoutError, OSError) as e:
                log.warning("Command failed: %s", e)

    async def _handle_command(self, line: str) -> None:
        if line in ("q", "quit", "exit"):
            self._stop.set()
            return

        if line in ("h", "help", "?"):
            print("  +N / -N  shift offset by N ms   o N  set offset   s  status   q  quit")
            return

        if line == "s":
            status = self.link.status
            print("  " + (status.describe() if status and self.link.connected else "not connected"))
            return

        if not self.link.connected or self.link.status is None:
            print("  not connected")
            return

        current = self.link.status.offset_ms

        if re.fullmatch(r"[+-]\d+", line):
            new_offset = current + int(line)
        elif m := re.fullmatch(r"o(?:ffset)?\s*(-?\d+)", line):
            new_offset = int(m.group(1))
        else:
            print("  unknown command, type h for help")
            return

        await self.link.send_offset(new_offset)
        print(f"  offset {current} -> {new_offset} ms")


# ============================================================
# ENTRY POINT
# ============================================================

def parse_track_id(text: str) -> str:
    """Accepts a bare ID, a spotify:track: URI or an open.spotify.com link."""
    m = re.search(r"(?:track[/:])([0-9A-Za-z]{22})", text)
    track_id = m.group(1) if m else text.strip()

    if not TRACK_ID_RE.match(track_id):
        raise ValueError(f"not a Spotify track ID or link: {text}")

    return track_id


def add_song(cfg: dict, base_dir: Path, source: Path, track: str) -> None:
    """Copies a generator song folder's two binaries into songs/<track_id>/."""
    track_id = parse_track_id(track)

    problem = check_song_folder(source)

    if problem is not None:
        raise SystemExit(f"{source}: {problem}")

    dest = (base_dir / cfg.get("songs_dir", "songs")).resolve() / track_id
    dest.mkdir(parents=True, exist_ok=True)

    for name in (FRAMES_FILENAME, NOTES_FILENAME):
        shutil.copyfile(source / name, dest / name)

    print(f"Copied {source} -> {dest}")


def load_config(path: Path) -> dict:
    if not path.is_file():
        raise SystemExit(
            f"Config file {path} not found. Copy config.example.json to config.json "
            "and fill in your Spotify app credentials."
        )

    with path.open(encoding="utf-8") as f:
        return json.load(f)


def main() -> None:
    base_dir = Path(__file__).resolve().parent

    parser = argparse.ArgumentParser(description="Sync the LED visualiser with Spotify over BLE.")
    parser.add_argument("--config", type=Path, default=base_dir / "config.json")
    parser.add_argument("--offset", type=int, help="set the latency offset (ms) on the ESP32 when connecting")
    parser.add_argument("--add", nargs=2, metavar=("SONG_FOLDER", "SPOTIFY_TRACK"),
                        help="copy a generate_song_vis.py output folder into songs/<track_id>/ and exit")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",
    )

    # Bleak and spotipy are chatty at DEBUG.
    logging.getLogger("bleak").setLevel(logging.INFO)
    logging.getLogger("spotipy").setLevel(logging.WARNING)
    logging.getLogger("urllib3").setLevel(logging.WARNING)

    cfg = load_config(args.config)

    if args.add:
        add_song(cfg, base_dir, Path(args.add[0]), args.add[1])
        return

    async def run() -> None:
        bridge = Bridge(cfg, args, base_dir)

        # Opens the browser for Spotify login the first time.
        await asyncio.to_thread(bridge.poller.login)
        log.info("Spotify login OK. Type h for commands.")

        await bridge.run()

    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
