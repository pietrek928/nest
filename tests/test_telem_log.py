"""Telem log writes to file, not stdout."""

import re

from nest_graph.telem_log import configure_telem_log, write_telem


def test_write_telem_appends_to_file(tmp_path, monkeypatch, capsys):
    path = tmp_path / "telem.log"
    monkeypatch.delenv("NEST_TELEM_LOG", raising=False)
    configure_telem_log(path, reset=True)
    write_telem("hello")
    write_telem("a", 1, "b")
    text = path.read_text(encoding="utf-8")
    stamp = re.compile(
        r"^\[\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{3} \+\d+\.\d{3}s\] "
    )
    lines = [ln for ln in text.splitlines() if ln]
    assert len(lines) == 2
    assert stamp.match(lines[0]) and lines[0].endswith(" hello")
    assert stamp.match(lines[1]) and lines[1].endswith(" a 1 b")
    captured = capsys.readouterr()
    assert captured.out == ""
    assert captured.err == ""
