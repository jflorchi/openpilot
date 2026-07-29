# logserver

A self-hosted stand-in for the comma/retropilot API — just enough of it for your
device to register and upload its logs to a machine you own. Ships as a compose
stack with a Cloudflare tunnel, so the device can reach it from LTE without you
opening a port.

## Quick start

1. **Create a tunnel.** In the Cloudflare Zero Trust dashboard: *Networks →
   Tunnels → Create a tunnel → Cloudflared*. Copy the connector token. Then add a
   public hostname route on that tunnel pointing at the compose service:

   | field | value |
   |---|---|
   | Subdomain / domain | e.g. `openpilot.example.com` |
   | Service type | `HTTP` |
   | URL | `logserver:8080` |

2. **Configure and run:**

   ```bash
   cp .env.example .env
   # set BASE_URL=https://openpilot.example.com, TUNNEL_TOKEN=..., REGISTRATION_TOKEN=...
   docker compose up -d
   ```

3. **Point the device at it.** Add to `/data/openpilot/launch_env.sh` on the device:

   ```bash
   export API_HOST="https://openpilot.example.com"
   ```

   `API_HOST` is read by `common/api/__init__.py`, `tools/lib/api.py`, and
   `selfdrive/ui/qt/api.h`, so the uploader, the offroad UI, and the replay tools
   all follow it.

4. **Force re-registration** if the device already registered elsewhere:

   ```bash
   python3 -c "from common.params import Params; Params().delete('DongleId')"
   ```

   (This fork's Cython `Params` uses `delete`, not upstream's `remove`. `rm
   /data/params/d/DongleId` works too.)

Logs land in `./data/<dongle_id>/...` on the host. Check `docker compose logs -f`
and `curl https://openpilot.example.com/health` to confirm.

### LAN-only, no tunnel

Drop the `cloudflared` service, change the port mapping to `"8080:8080"`, and set
`BASE_URL` to your host's LAN IP (`http://192.168.1.50:8080`). It must be the
address the *device* can reach — `localhost` will not work.

### Without Docker

```bash
pip install -r requirements.txt
BASE_URL=http://192.168.1.50:8080 DATA_DIR=./data ./server.py
```

## What it implements

| Endpoint | Caller |
|---|---|
| `POST /v2/pilotauth/` | `selfdrive/athena/registration.py` |
| `GET /v1.4/<dongle_id>/upload_url/` | `selfdrive/loggerd/uploader.py` |
| `PUT /upload/<dongle_id>/<path>` | the URL handed back above |
| `GET /v1/route/<dongle_id>\|<route>/files` | `tools/lib/route.py`, replay |
| `GET /v1/devices/<dongle_id>/bootlogs` | `tools/lib/bootlog.py` |
| `GET /v1.1/devices/<dongle_id>/` | offroad UI pairing/prime widget |
| `GET /v1/devices/<dongle_id>/owner` | offroad UI points widget |
| `GET /v1.1/devices/<dongle_id>/stats` | offroad UI drive stats |

Drive files land in `DATA_DIR/<dongle_id>/<YYYY-MM-DD--HH-MM-SS>/<segment>/<filename>`,
which is the layout `tools/lib/route.py` expects, so `tools/replay` and
`tools/plotjuggler` work against your own routes. Bootlogs and crash logs use the
flat `DATA_DIR/<dongle_id>/boot/` and `crash/` folders, matching the `boot/` and
`crash/` keys the uploader sends for those.

## How auth works

At registration the device sends its RSA public key (from `/persist/comma/id_rsa.pub`)
along with a self-signed `register_token`; the server verifies that token against the
submitted key to confirm the device holds the matching private key, then stores the
public key. Every later request carries a short-lived RS256 JWT the device signs with
that key, and the server verifies it and checks the `identity` claim matches the
dongle id in the URL.

Dongle ids are derived deterministically from the hardware serial, so a device that
re-registers after a reflash keeps its existing route history.

Upload and download URLs are HMAC-signed with a one-hour expiry, using a secret
generated on first run and kept at `DATA_DIR/secret.key`.

## Testing

```bash
docker compose run --rm -e BASE_URL=http://test logserver python test_server.py
```

Generates a keypair, registers a fake device, and walks the same call sequence the
uploader makes — segment uploads, bootlogs, dedupe, forged tokens, path traversal,
and tampered URLs.

## Things to know

- **Cloudflare caps proxied request bodies at 100 MB** on Free/Pro plans. Most
  segment files are well under that (qlog ~1 MB, qcamera ~5 MB, rlog.bz2 ~10 MB),
  but a minute of `fcamera.hevc` can get close. If you see uploads failing with 413,
  either disable full-resolution camera uploads or expose the server another way —
  the size limit is Cloudflare's, not this server's.
- **Set `REGISTRATION_TOKEN`.** Without it, anything that can reach the tunnel can
  register and claim a dongle id. The device sends it when `REGISTRATION_TOKEN` is
  set in its environment (`selfdrive/athena/registration.py`); `launch_env.sh` reads
  it from `/data/registration_token` so the secret stays out of git. Put that file on
  the device *before* enabling the check on the server, or registration will 403 and
  the device falls back to `UnregisteredDevice`.
- **Nothing deletes old logs.** A comma three fills a disk with rlogs quickly; add a
  cron job to prune `./data` or configure the device to upload qlogs only.
- **SQLite + one volume.** Fine for a handful of devices. It is not built for
  concurrent writers across multiple hosts.
- **No athena.** Athena is the persistent websocket connect uses for live remote
  control — SSH, screen viewing, on-demand uploads. This server implements only the
  REST API (the `api.comma.ai` role), so leave `ATHENA_HOST` unset; pointing it here
  just produces endless failed-connection retries in the logs.
