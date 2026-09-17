"""Sample only the dedicated profiling driver's processes on macOS."""

import argparse
from pathlib import Path
import selectors
import shutil
import subprocess
import sys


def stop(process):
    if process is not None and process.poll() is None:
        process.terminate()
        try:
            process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate()


def profile(executable, sampler, output, scene):
    broadphase = "sap" if scene.startswith("mixed") else "tree"
    sample_path = output / f"cpu-{broadphase}-{scene}.sample.txt"
    log_path = output / f"cpu-{broadphase}-{scene}.run.txt"
    if sample_path.exists() or log_path.exists():
        raise FileExistsError(f"Choose a fresh output directory; {scene} results already exist")
    process = subprocess.Popen(
        [str(executable), scene, "20", "--wait"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1,
    )
    sampling = None
    try:
        with selectors.DefaultSelector() as ready:
            ready.register(process.stdout, selectors.EVENT_READ)
            if not ready.select(timeout=120):
                raise TimeoutError(f"{scene} did not finish setup/warmup")
            message = process.stdout.readline()
        if not message.startswith("PROFILE_READY") or process.poll() is not None:
            raise RuntimeError(f"{scene} setup failed: {message}")
        print(f"{scene}: sampling dedicated PID {process.pid}; {message.strip()}", flush=True)
        sampling = subprocess.Popen(
            [sampler, str(process.pid), "15", "1", "-mayDie", "-file", str(sample_path)],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        process.stdin.write("\n")
        process.stdin.flush()
        process.stdin.close()
        process.stdin = None
        sampling_output, _ = sampling.communicate(timeout=90)
        run_output, _ = process.communicate(timeout=30)
        log_path.write_text(
            f"pid: {process.pid}\nsample_seconds: 15\nsample_interval_ms: 1\n"
            + message + run_output + "\nSampler output:\n" + sampling_output
        )
        if sampling.returncode != 0 or process.returncode != 0:
            raise RuntimeError(f"{scene} profiling failed; see {log_path}")
        if not sample_path.exists() or "Call graph:" not in sample_path.read_text():
            raise RuntimeError(f"{scene} did not produce a usable sample report")
        print(f"{scene}: wrote {sample_path} and {log_path}", flush=True)
    finally:
        stop(sampling)
        stop(process)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path)
    parser.add_argument("output_directory", type=Path)
    parser.add_argument(
        "--scenes",
        nargs="+",
        choices=("spheres", "boxes", "mixed5k", "mixed10k"),
        default=("spheres", "boxes"),
    )
    args = parser.parse_args()
    if sys.platform != "darwin":
        parser.error("This launcher requires macOS sample")
    executable = args.executable.resolve(strict=True)
    sampler = shutil.which("sample")
    if sampler is None:
        parser.error("macOS sample is required")
    output = args.output_directory.resolve()
    output.mkdir(parents=True, exist_ok=True)
    for scene in args.scenes:
        profile(executable, sampler, output, scene)


if __name__ == "__main__":
    main()
