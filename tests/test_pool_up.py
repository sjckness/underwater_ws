"""deploy/hil/windows/pool/pool_up.py (M12): the Work's one-command pool. Its Windows actions cannot run here; its
decisions are plain functions over the text Windows' tools print, and those are tested with that text."""
import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("pool_up", ROOT / "deploy/hil/windows/pool/pool_up.py")
pu = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pu)


def test_the_repo_root_and_the_files_it_drives_exist():
    assert pu.REPO == ROOT
    for f in ("run_pool.bat", "setup_blueos_wsl.sh", "stack.sh", "wslconfig.example"):
        assert (pu.HERE / f).is_file(), f
    assert (ROOT / "pool.bat").is_file() and (ROOT / "vehicles/bluerov2_heavy/fetch_bluerov2.py").is_file()
    assert (ROOT / "sim/tools/fetch_props.py").is_file()


def test_wsl_output_is_utf16_and_its_own_parsing():
    raw = "WSL version: 2.3.26.0\r\nKernel version: 5.15.167.4-1\r\n".encode("utf-16-le")
    text = pu.decode(raw)
    assert pu.wsl_major_version(text) == 2
    assert pu.wsl_major_version(pu.decode("Versione WSL: 2.0.14.0".encode("utf-16-le"))) == 2   # localised label
    assert pu.wsl_major_version("") is None
    names = pu.decode("Ubuntu-24.04\r\ndocker-desktop\r\n".encode("utf-16-le"))
    assert pu.distros(names) == ["Ubuntu-24.04", "docker-desktop"]
    assert pu.decode(b"DOCKER_OK\nMODE=mirrored\n") == "DOCKER_OK\nMODE=mirrored\n"     # a Linux command: UTF-8


def test_the_wslconfig_gets_only_what_it_lacks():
    example = (pu.HERE / "wslconfig.example").read_text()
    text, added = pu.merge_wslconfig(None, example)
    assert text == example and added
    mine = "[wsl2]\nmemory=16GB\nnetworkingMode=NAT\n\n[interop]\nenabled=true\n"
    text, added = pu.merge_wslconfig(mine, example)
    assert "memory=16GB" in text and "networkingMode=NAT" in text, "the user's values stay"
    assert "wsl2.networkingMode" not in added and "wsl2.dnsTunneling" in added
    assert pu.networking_mode(text) == "NAT", "a different mode is the user's call (check_wsl stops on it)"
    assert "[experimental]\nhostAddressLoopback=true" in text
    assert text.index("dnsTunneling=true") < text.index("[interop]"), "added inside its own section"
    assert pu.networking_mode(example) == "mirrored" and pu.networking_mode("[wsl2]\nmemory=8GB\n") is None
    again, added2 = pu.merge_wslconfig(text, example)
    assert added2 == [] and again.rstrip() == text.rstrip()


def test_crlf_files_and_ports_from_the_tools_text():
    eol = ("i/lf    w/crlf  attr/text eol=lf      \tdeploy/docker/pool/compose.yaml\n"
           "i/lf    w/lf    attr/text eol=lf      \tdeploy/hil/windows/pool/stack.sh\n"
           "i/crlf  w/crlf  attr/text eol=crlf    \tdeploy/hil/windows/pool/run_pool.bat\n")
    assert pu.crlf_files(eol) == ["deploy/docker/pool/compose.yaml"]
    netstat = ("  Proto  Local Address          Foreign Address        State           PID\n"
               "  TCP    0.0.0.0:80             0.0.0.0:0              LISTENING       4\n"
               "  TCP    127.0.0.1:16171        0.0.0.0:0              LISTENING       9876\n"
               "  TCP    192.168.1.5:8000       0.0.0.0:0              LISTENING       55\n"
               "  TCP    127.0.0.1:9020         127.0.0.1:50000        ESTABLISHED     11\n"
               "  UDP    127.0.0.1:9002         *:*                                    4321\n")
    assert pu.listening_ports(netstat) == {80: "4", 16171: "9876", 9002: "4321"}


def test_missing_props(tmp_path):
    (tmp_path / "chair").mkdir()
    (tmp_path / "chair" / "chair.usd").write_text("#usda 1.0\n")
    assert pu.missing_props(["ball", "chair"], tmp_path) == ["ball"]


def test_the_placed_objects_are_the_pool_configs():
    ids = pu.placed_object_ids()
    assert {"diver", "chair", "ball"} <= set(ids)


def test_it_refuses_to_run_off_windows(monkeypatch):
    monkeypatch.setattr(pu.os, "name", "posix")
    with pytest.raises(SystemExit, match="Work machine"):
        pu.main(["--check"])
