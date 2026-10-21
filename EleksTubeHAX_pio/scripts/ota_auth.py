# PlatformIO pre-script: take OTA_PASSWORD from src/_USER_DEFINES.h
# and pass it to espota as --auth. The value is never printed.
import os
import re

Import("env")  # noqa: F821

defines = os.path.join(env.subst("$PROJECT_DIR"), "src", "_USER_DEFINES.h")  # noqa: F821
try:
    with open(defines, encoding="utf-8") as f:
        match = re.search(r'^\s*#define\s+OTA_PASSWORD\s+"([^"]+)"', f.read(), re.M)
except OSError:
    match = None

if not match:
    print("ota_auth.py: OTA_PASSWORD is missing from src/_USER_DEFINES.h")
    env.Exit(1)  # noqa: F821

env.Append(UPLOAD_FLAGS=["--auth=" + match.group(1)])  # noqa: F821
