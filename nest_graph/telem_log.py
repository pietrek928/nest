"""Append-only telem log (keeps stdout free for tqdm progress)."""

import os
import time
from datetime import datetime
from pathlib import Path

_DEFAULT = "artifacts/tune/telem.log"
_path: Path | None = None
_t0 = time.perf_counter()


def telem_log_path() -> Path:
    env = str(os.environ.get("NEST_TELEM_LOG", "") or "").strip()
    if env:
        return Path(env)
    if _path is not None:
        return _path
    return Path(_DEFAULT)


def configure_telem_log(
    path: str | Path | None = None,
    *,
    reset: bool = False,
) -> Path:
    """Set log path; optionally truncate for a new session."""
    global _path, _t0
    if path is not None and str(path).strip():
        _path = Path(str(path))
        os.environ["NEST_TELEM_LOG"] = str(_path)
    out = telem_log_path()
    out.parent.mkdir(parents=True, exist_ok=True)
    if reset:
        out.write_text("", encoding="utf-8")
        _t0 = time.perf_counter()
    return out


def write_telem(msg: str = "", *parts: object) -> None:
    """Append one telem line with wall + session-elapsed timing. No stdout."""
    if parts:
        body = " ".join(str(x) for x in (msg, *parts))
    else:
        body = str(msg)
    body = body.rstrip("\n")
    wall = datetime.now().isoformat(timespec="milliseconds")
    elapsed = time.perf_counter() - _t0
    line = f"[{wall} +{elapsed:.3f}s] {body}\n"
    path = telem_log_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as fh:
        fh.write(line)
