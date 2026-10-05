# LED visualiser: wireless sync (phase 2)

```
iPhone (Spotify) ──plays──► Spotify cloud ◄──polls 1/s── laptop (visualiser_link.py) ──BLE──► ESP32-C3 ──► LEDs
```

## 1. Flash the firmware

The firmware now needs Bluetooth (NimBLE), 1 ms FreeRTOS ticks, LittleFS and a
larger storage partition. These are set in `sdkconfig.defaults`, but your
existing `sdkconfig` overrides them. Regenerate it once:

```bash
idf.py set-target esp32c3
```

(`set-target` backs up the old `sdkconfig` to `sdkconfig.old` and rebuilds it from
the defaults.) Then build and flash as usual:

```bash
idf.py build flash monitor
```

On first boot the storage partition still holds the old SPIFFS image, so LittleFS
formats it automatically. The old `song.bin` is erased and songs now arrive over
BLE. Re-flashing the firmware does **not** wipe the song cache.

## 2. Spotify developer app

1. Go to <https://developer.spotify.com/dashboard> and create an app.
2. Add the redirect URI `http://127.0.0.1:8888/callback` (Spotify no longer
   accepts `localhost`).
3. Select **Web API**.
4. Copy the client ID and secret into `config.json` (copy it from
   `config.example.json`).

On the first run a browser window opens for you to log in. The token is cached in
`.spotify_cache` and refreshed automatically. The only permission requested is
`user-read-playback-state`.

## 3. Install and run

```bash
pip install -r requirements.txt
```

```bash
python visualiser_link.py
```

Options:

| Option | Purpose |
|---|---|
| `--offset N` | Set the latency offset (ms) on the ESP32 when it connects |
| `--add BIN TRACK` | Copy a generated `.bin` into `songs/`, named by track ID |
| `-v` | Debug logging |

## 4. Song files

Files go in `songs/` and are named by Spotify track ID:
`songs/4uLU6hMCjMI75M1A2tKUQC.bin`. To get the ID, choose **Share → Copy song link**
in Spotify. The ID is the part after `/track/`. `--add` accepts the whole link:

```bash
python visualiser_link.py --add ../path/to/song.bin "https://open.spotify.com/track/4uLU6hMCjMI75M1A2tKUQC?si=..."
```

## 5. Latency tuning

While the script runs you can type commands:

| Command | Effect |
|---|---|
| `+20` / `-20` | Shift the offset by 20 ms |
| `o 150` | Set the offset to 150 ms |
| `s` | Show the ESP32 status |
| `q` | Quit (turns the strip off) |

The ESP32 shows frame `(position_ms + offset_ms) / 40`.

- **Lights late?** Increase the offset.
- **Lights early?** Decrease it. Bluetooth headphones/speakers on the phone
  usually need a negative offset (≈ −150 to −250 ms).

The offset is saved in the ESP32's NVS and survives reboots.

## What the strip shows

| Situation | Strip |
|---|---|
| Playing, file cached | Visualisation |
| Playing, no file / podcast / transfer pending | Idle wave |
| File transfer in progress | Progress bar filling from both ends |
| Paused, or nothing playing | Off |
| No message from the laptop for 15 s | Off |

## Protocol summary

See `main/protocol.h` (firmware) and the top of `visualiser_link.py`. Both must
stay in sync.

| Characteristic | UUID | Messages |
|---|---|---|
| STATE (write) | `a3c87501-8ed3-4bdf-8a39-a01bebede295` | `0x01` STATE (30 B), `0x02` SET_OFFSET (3 B) |
| FILE (write) | `a3c87502-…` | `0x10` FILE_START (31 B), `0x11` FILE_CHUNK (5 B + data), `0x12` FILE_END |
| STATUS (read/notify) | `a3c87503-…` | track_id[22], transfer_state, error_code, bytes_received u32, ack_seq u16, offset_ms i16 (32 B) |

## Troubleshooting

- **Windows can't find the device:** make sure it isn't paired in Windows
  Bluetooth settings (pairing isn't needed), and that nothing else (e.g. nRF
  Connect) is connected to it.
- **Transfers are slow:** the log line `MTU … bytes per file chunk` should show
  about 239 bytes. If it shows 20, the MTU exchange failed.
