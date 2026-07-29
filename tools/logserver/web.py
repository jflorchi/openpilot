"""Authenticated web interface for browsing, playing, and downloading logs.

Mounted at /ui by server.py. Session-cookie auth, entirely separate from the
device's JWT auth -- a browser session never grants access to the device API and
vice versa.
"""

import functools
import mimetypes
import shutil
import subprocess
import threading
import time
from datetime import datetime, timedelta

from flask import (Blueprint, abort, flash, redirect, render_template, request,
                   send_file, session, url_for)
from werkzeug.security import check_password_hash, generate_password_hash

import store
from store import DATA_DIR, FILE_LABELS, route_dir, safe_segment_path, segment_dirs

bp = Blueprint("web", __name__, url_prefix="/ui",
               template_folder="templates", static_folder="static")

# Hash once at import so the comparison is constant-time and the plaintext isn't
# repeatedly handled. The password itself still lives in the environment.
_PW_HASH = generate_password_hash(store.WEB_PASSWORD) if store.WEB_PASSWORD else None

SESSION_MAX_AGE = 14 * 24 * 3600

# Crude in-memory login throttle. Resets on restart, which is fine for a
# single-user server -- it exists to make guessing slow, not to be a real WAF.
_attempts: dict[str, list[float]] = {}
_attempts_lock = threading.Lock()
MAX_ATTEMPTS, ATTEMPT_WINDOW = 8, 300

# One lock per route so two viewers don't transcode the same video twice.
_video_locks: dict[str, threading.Lock] = {}
_video_locks_guard = threading.Lock()


def _throttled(ip: str) -> bool:
  now = time.time()
  with _attempts_lock:
    hits = [t for t in _attempts.get(ip, []) if now - t < ATTEMPT_WINDOW]
    _attempts[ip] = hits
    return len(hits) >= MAX_ATTEMPTS


def _record_attempt(ip: str):
  with _attempts_lock:
    _attempts.setdefault(ip, []).append(time.time())


def login_required(f):
  @functools.wraps(f)
  def wrapper(*args, **kwargs):
    if _PW_HASH is None:
      abort(503, "web interface disabled: set WEB_PASSWORD")
    if not session.get("auth"):
      return redirect(url_for("web.login", next=request.path))
    if time.time() - session.get("at", 0) > SESSION_MAX_AGE:
      session.clear()
      return redirect(url_for("web.login"))
    return f(*args, **kwargs)
  return wrapper


# ---------------------------------------------------------------- formatting

def fmt_size(n):
  for unit in ("B", "KB", "MB", "GB", "TB"):
    if n < 1024:
      return f"{n:.0f} {unit}" if unit == "B" else f"{n:.1f} {unit}"
    n /= 1024
  return f"{n:.1f} PB"


def fmt_duration(minutes):
  if minutes < 60:
    return f"{minutes} min"
  return f"{minutes // 60}h {minutes % 60:02d}m"


def parse_time(time_str):
  try:
    return datetime.strptime(time_str, "%Y-%m-%d--%H-%M-%S")
  except ValueError:
    return None


def fmt_when(dt):
  if dt is None:
    return "unknown"
  today = datetime.now().date()
  if dt.date() == today:
    return f"Today, {dt:%H:%M}"
  if dt.date() == today - timedelta(days=1):
    return f"Yesterday, {dt:%H:%M}"
  if dt.year == today.year:
    return f"{dt:%a %-d %b, %H:%M}"
  return f"{dt:%-d %b %Y, %H:%M}"


@bp.app_context_processor
def inject_globals():
  return {"base_url": store.BASE_URL}


bp.add_app_template_filter(fmt_size, "filesize")
bp.add_app_template_filter(fmt_duration, "duration")
bp.add_app_template_filter(fmt_when, "when")


# ---------------------------------------------------------------- auth views

@bp.route("/login", methods=["GET", "POST"])
def login():
  if _PW_HASH is None:
    abort(503, "web interface disabled: set WEB_PASSWORD")
  if session.get("auth"):
    return redirect(url_for("web.devices"))

  if request.method == "POST":
    ip = request.headers.get("CF-Connecting-IP") or request.remote_addr or "?"
    if _throttled(ip):
      flash("Too many attempts. Wait a few minutes.")
      return render_template("login.html"), 429

    if check_password_hash(_PW_HASH, request.form.get("password", "")):
      session.clear()
      session["auth"] = True
      session["at"] = time.time()
      session.permanent = True
      nxt = request.args.get("next", "")
      # only ever redirect within this app, never to an attacker-supplied host
      return redirect(nxt if nxt.startswith("/ui/") else url_for("web.devices"))

    _record_attempt(ip)
    flash("Wrong password.")
    return render_template("login.html"), 401

  return render_template("login.html")


@bp.route("/logout", methods=["POST"])
def logout():
  session.clear()
  return redirect(url_for("web.login"))


# ---------------------------------------------------------------- browsing

@bp.route("/")
@login_required
def devices():
  rows = []
  for d in store.all_devices():
    routes = store.routes_for(d["dongle_id"])
    rows.append({
      "dongle_id": d["dongle_id"],
      "serial": d["serial"],
      "last_seen": datetime.fromtimestamp(d["last_seen"]),
      "routes": len(routes),
      "minutes": sum(r["count"] for r in routes),
      "size": sum(r["size"] for r in routes),
      "latest": parse_time(routes[0]["time_str"]) if routes else None,
    })
  return render_template("devices.html", devices=rows)


@bp.route("/d/<dongle_id>")
@login_required
def device(dongle_id):
  if store.get_device(dongle_id) is None:
    abort(404, "unknown device")
  routes = store.routes_for(dongle_id)
  for r in routes:
    r["when"] = parse_time(r["time_str"])
  return render_template("device.html", dongle_id=dongle_id, routes=routes,
                         total_size=sum(r["size"] for r in routes))


@bp.route("/d/<dongle_id>/r/<time_str>")
@login_required
def route(dongle_id, time_str):
  rdir = route_dir(dongle_id, time_str)
  if not rdir.is_dir():
    abort(404, "unknown route")

  segments = []
  for t, seg, seg_dir in segment_dirs(dongle_id):
    if t != time_str:
      continue
    files = [{
      "name": f.name,
      "label": FILE_LABELS.get(f.name, f.name),
      "size": f.stat().st_size,
    } for f in sorted(seg_dir.iterdir()) if f.is_file() and not f.name.endswith(".part")]
    segments.append({"num": int(seg), "files": files})

  summary = store.route_summary(dongle_id, time_str)
  return render_template("route.html", dongle_id=dongle_id, time_str=time_str,
                         when=parse_time(time_str), segments=segments, summary=summary,
                         has_ffmpeg=shutil.which("ffmpeg") is not None)


# ---------------------------------------------------------------- media

@bp.route("/dl/<dongle_id>/<time_str>/<seg>/<fn>")
@login_required
def download(dongle_id, time_str, seg, fn):
  path = safe_segment_path(dongle_id, f"{time_str}--{seg}/{fn}")
  if not path.is_file():
    abort(404)
  return send_file(path, as_attachment=True,
                   download_name=f"{dongle_id}_{time_str}--{seg}_{fn}", conditional=True)


def _lock_for(key):
  with _video_locks_guard:
    return _video_locks.setdefault(key, threading.Lock())


def build_route_video(dongle_id, time_str):
  """Concatenate a route's qcamera.ts segments into one seekable mp4.

  qcamera is already H.264 in MPEG-TS, so this is a stream copy -- no re-encode,
  a few seconds for a long route. Cached next to the segments and rebuilt only
  when a newer segment shows up.
  """
  rdir = route_dir(dongle_id, time_str)
  out = rdir / "route.mp4"

  parts = []
  for t, seg, seg_dir in segment_dirs(dongle_id):
    if t == time_str and (seg_dir / "qcamera.ts").is_file():
      parts.append(seg_dir / "qcamera.ts")
  if not parts:
    return None

  newest = max(p.stat().st_mtime for p in parts)
  if out.is_file() and out.stat().st_mtime >= newest:
    return out

  with _lock_for(f"{dongle_id}/{time_str}"):
    # another request may have finished the work while we waited
    if out.is_file() and out.stat().st_mtime >= newest:
      return out

    listing = rdir / ".concat.txt"
    listing.write_text("".join(f"file '{p.resolve()}'\n" for p in parts))
    tmp = rdir / ".route.mp4.tmp"
    try:
      subprocess.run(
        ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
         "-f", "concat", "-safe", "0", "-i", str(listing),
         "-c", "copy", "-movflags", "+faststart", "-f", "mp4", str(tmp)],
        check=True, capture_output=True, timeout=600)
      tmp.replace(out)
    except subprocess.CalledProcessError as e:
      raise RuntimeError(e.stderr.decode()[-400:] or "ffmpeg failed") from e
    except subprocess.TimeoutExpired as e:
      raise RuntimeError("ffmpeg timed out") from e
    finally:
      listing.unlink(missing_ok=True)
      tmp.unlink(missing_ok=True)
  return out


@bp.route("/video/<dongle_id>/<time_str>.mp4")
@login_required
def route_video(dongle_id, time_str):
  if not shutil.which("ffmpeg"):
    abort(503, "ffmpeg not installed")
  try:
    path = build_route_video(dongle_id, time_str)
  except RuntimeError as e:
    abort(500, str(e))
  if path is None:
    abort(404, "no qcamera footage for this route")
  return send_file(path, mimetype="video/mp4", conditional=True)


mimetypes.add_type("video/mp4", ".mp4")
