from datetime import datetime
from pathlib import Path
import re

Import("env")


def parse_macro_from_cppdefines(macro_name):
    cppdefines = env.get("CPPDEFINES", [])
    for item in cppdefines:
        key = None
        value = None
        if isinstance(item, tuple) and len(item) >= 2:
            key = item[0]
            value = item[1]
        elif isinstance(item, str):
            if item == macro_name:
                return ""
            if item.startswith(macro_name + "="):
                key = macro_name
                value = item.split("=", 1)[1]
        if key != macro_name:
            continue
        if value is None:
            return ""
        if not isinstance(value, str):
            return str(value)
        return value.replace('\\"', '"').strip('"')
    return None


def parse_macro_from_config(config_path, macro_name):
    if not config_path.exists():
        return None

    source_text = config_path.read_text(encoding="utf-8", errors="ignore")
    pattern = rf'^\s*#define\s+{macro_name}\s+"([^"]*)"'
    match = re.search(pattern, source_text, flags=re.MULTILINE)
    if not match:
        return None
    return match.group(1)


existing_release = parse_macro_from_cppdefines("APP_RELEASE_VERSION")
if existing_release is not None:
    print(f"build_version: APP_RELEASE_VERSION already set to '{existing_release}', leaving it unchanged")
else:
    project_dir = Path(env["PROJECT_DIR"])
    config_path = project_dir / "include" / "Configuration.h"

    base_version = parse_macro_from_cppdefines("APP_VERSION")
    if base_version is None:
        base_version = parse_macro_from_config(config_path, "APP_VERSION")
    if not base_version:
        base_version = "0.0"

    build_date = datetime.now().strftime("%Y%m%d")
    separator = "." if "+" in base_version else "+"
    release_version = f"{base_version}{separator}{build_date}"

    env.Append(CPPDEFINES=[f'APP_RELEASE_VERSION=\\"{release_version}\\"'])
    print(f"build_version: APP_RELEASE_VERSION={release_version}")
