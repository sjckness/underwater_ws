"""`python -m uw` (host and container): command line, docker command, native runs and exit codes."""
import os
import subprocess
import sys
from pathlib import Path

from uw import launcher


def _uw(*args, **env):
    """Run `python -m uw ...` in native mode with this test's Python as the target."""
    e = {**os.environ, "UW_RUNTIME": "native", "UW_PYTHON": sys.executable, **env}
    e["PYTHONPATH"] = os.pathsep.join([str(launcher.WS), *filter(None, [os.environ.get("PYTHONPATH")])])
    return subprocess.run([sys.executable, "-m", "uw", *args], env=e, cwd=launcher.WS, capture_output=True, text=True,
                          timeout=120)


def test_docker_command():
    rt = launcher.runtime()
    argv = launcher.docker_argv(rt, "py", ["/ws/tools/x.py", "--set", "a=1"], tty=False, workdir=Path("/ws/tools"),
                                name="uw-py-1")
    assert argv[:4] == ["docker", "compose", "-f", str(launcher.WS / rt["docker"]["compose_file"])]
    assert "-T" in argv and argv[argv.index("--name") + 1] == "uw-py-1" and argv[argv.index("-w") + 1] == "/ws/tools"
    assert f"UW_WARP_PATH={rt['docker']['isaac_root']}/extscache/omni.warp.core-1.8.2+lx64" in argv
    tail = argv[argv.index(rt["docker"]["service"]):]
    assert tail[1:3] == ["-c", launcher.BOOT] and tail[3:] == [rt["docker"]["isaac_root"], "py", "/ws/tools/x.py",
                                                               "--set", "a=1"]
    assert "-T" not in launcher.docker_argv(rt, "test", [], tty=True, workdir=launcher.WS, name="n")


def test_native_py_code_gets_its_arguments_and_exit_code():
    r = _uw("py", "-c", "import sys; print('args', sys.argv[1:]); raise SystemExit(3)", "x", "--set", "a.b=1")
    assert r.returncode == 3, r.stderr
    assert "args ['x', '--set', 'a.b=1']" in r.stdout


def test_native_py_script(tmp_path):
    script = tmp_path / "s.py"
    script.write_text("import sys, os\nprint('script', os.path.basename(sys.argv[0]), sys.argv[1:], sys.path[0])\n")
    r = _uw("py", str(script), "--profile", "workstation")
    assert r.returncode == 0, r.stderr
    assert f"script s.py ['--profile', 'workstation'] {tmp_path}" in r.stdout


def test_unknown_command():
    r = _uw("bogus")
    assert r.returncode == 2 and "unknown command" in r.stderr


def test_gui_gets_the_x_display(monkeypatch, tmp_path):
    xauth = tmp_path / "xauth"
    xauth.write_text("cookie")
    monkeypatch.setenv("DISPLAY", ":7")
    monkeypatch.setenv("XAUTHORITY", str(xauth))
    extra = launcher.x11_args()
    assert "DISPLAY=:7" in extra and f"{xauth}:/uw_xauth:ro" in extra and "XAUTHORITY=/uw_xauth" in extra
    argv = launcher.docker_argv(launcher.runtime(), "sim", ["--gui"], tty=True, workdir=launcher.WS, name="n",
                                extra=extra)
    assert argv.index("DISPLAY=:7") < argv.index(launcher.runtime()["docker"]["service"])