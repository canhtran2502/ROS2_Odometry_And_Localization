"""Process helpers for the safety supervisor tests: start an executable, signals, logs."""
import os
import shutil
import signal
import subprocess
import tempfile
import time

import yaml
from ament_index_python.packages import get_package_prefix

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, ".."))
DATA = os.path.join(HERE, "data")
SHIPPED_CONFIG = os.path.join(PKG, "config", "safety_supervisor.yaml")
IMPLS = ("cpp", "py")


def executable(impl):
    name = "safety_supervisor" if impl == "cpp" else "safety_supervisor.py"
    path = os.path.join(get_package_prefix("bumperbot_utils"), "lib", "bumperbot_utils", name)
    assert os.path.exists(path), "executable not installed: " + path
    return path


def shipped_params():
    with open(SHIPPED_CONFIG) as f:
        return yaml.safe_load(f)["safety_supervisor"]["ros__parameters"]


class Workdir:
    """Temporary directory for parameter files and process logs."""

    def __init__(self):
        self.path = tempfile.mkdtemp(prefix="safety_test_")

    def write_params(self, overrides, name="params.yaml", base=None, node_key="safety_supervisor"):
        params = dict(shipped_params() if base is None else base)
        params.update(overrides or {})
        path = os.path.join(self.path, name)
        with open(path, "w") as f:
            yaml.safe_dump({node_key: {"ros__parameters": params}}, f)
        return path

    def cleanup(self):
        shutil.rmtree(self.path, ignore_errors=True)


class SupervisorProcess:
    """One safety_supervisor process in its own process group."""

    def __init__(self, impl, params_file, workdir, extra_args=()):
        self.impl = impl
        stamp = int(time.time() * 1e3)
        self.out_path = os.path.join(workdir.path, "{}_{}.log".format(impl, stamp))
        self.log = open(self.out_path, "w")
        cmd = [executable(impl), "--ros-args", "--params-file", params_file] + list(extra_args)
        self.proc = subprocess.Popen(cmd, stdout=self.log, stderr=subprocess.STDOUT,
                                     start_new_session=True, env=dict(os.environ))
        self.pid = self.proc.pid
        self.started = time.monotonic()

    def output(self):
        self.log.flush()
        with open(self.out_path) as f:
            return f.read()

    def wait_for_output(self, text, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if text in self.output():
                return True
            time.sleep(0.05)
        return False

    def exe_path(self):
        """Return the resolved /proc/<pid>/exe and the command line (proves the implementation)."""
        exe = os.path.realpath("/proc/{}/exe".format(self.pid))
        with open("/proc/{}/cmdline".format(self.pid), "rb") as f:
            cmdline = f.read().replace(b"\0", b" ").decode(errors="replace")
        return exe, cmdline

    def alive(self):
        return self.proc.poll() is None

    def signal(self, sig):
        """Signal the process itself (own session, one process)."""
        try:
            os.kill(self.pid, sig)
            return True
        except ProcessLookupError:
            return False

    def wait(self, timeout):
        try:
            return self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            return None

    def stop(self, timeout=8.0):
        """SIGINT, then SIGKILL of the whole group if it does not exit."""
        if self.alive():
            try:
                os.kill(self.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            if self.wait(timeout) is None:
                try:
                    os.killpg(self.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.proc.wait(timeout=5.0)
        self.log.close()
        return self.proc.returncode
