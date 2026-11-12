"""Build LD19 firmware without opening a probe or serial port (Python 3.9+)."""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

root = Path(__file__).resolve().parents[1]
firmware = root / "firmware"
output = root / "build" / "Release"
output.mkdir(parents=True, exist_ok=True)


def tool(variable, executable):
    value = os.environ.get(variable) or shutil.which(executable)
    if not value:
        raise SystemExit(f"Set {variable} or add {executable} to PATH.")
    return str(Path(value).resolve())


gcc = tool("ARM_GCC", "arm-none-eabi-gcc")
cmake = tool("CMAKE", "cmake")
ninja = tool("NINJA", "ninja")
subprocess.run([
    cmake, "-S", str(firmware), "-B", str(output), "-G", "Ninja",
    "-DCMAKE_TOOLCHAIN_FILE=" + str(firmware / "cmake/arm-none-eabi.cmake"),
    "-DARM_GCC=" + gcc, "-DCMAKE_MAKE_PROGRAM=" + ninja,
    "-DCMAKE_BUILD_TYPE=Release",
], check=True)
subprocess.run([cmake, "--build", str(output)], check=True)

config = (firmware / "CMakeLists.txt").read_text(encoding="utf-8")
version = re.search(r'set\(APP_VERSION "([^"]+)"', config).group(1)
artifacts = {}
for extension in ("bin", "hex"):
    p = output / ("LD19Bench." + extension)
    data = p.read_bytes()
    artifacts[extension] = {"file": p.name, "bytes": len(data),
                            "sha256": hashlib.sha256(data).hexdigest()}
dependencies = [p for p in firmware.rglob("*") if p.is_file()
                and p.suffix.lower() in {".c", ".h", ".s", ".ld", ".cmake", ".txt"}
                and "current" not in p.relative_to(firmware).parts]
metadata = {
    "firmware_version": version, "protocol_version": 2,
    "hse_mhz": 25, "sysclk_mhz": 168, "config": "Release",
    "toolchain": subprocess.check_output([gcc, "--version"], text=True).splitlines()[0],
    "artifacts": artifacts,
    "source_sha256": {p.relative_to(root).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in sorted(dependencies)},
    "flashed": False, "hardware_access": False,
    "validation": "build only; this rebuilt image was NOT flashed",
}
(output / "build.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
print(json.dumps({k: v for k, v in metadata.items() if k != "source_sha256"}, indent=2))
