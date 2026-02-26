from pathlib import Path
import re
from typing import Optional

Import("env")


CDC0_MACRO = "USB_CDC0_IFACE_NAME"
CDC1_MACRO = "USB_CDC1_IFACE_NAME"
CDC0_DEFAULT = "Console"
CDC1_DEFAULT = "NMEA"

CDC_PATCH_BLOCK = """{indent}const char* default_cdc_name = "TinyUSB Serial";
{indent}#ifdef USB_CDC_IFACE_NAME_0
{indent}if (_instance == 0) {{
{indent}  default_cdc_name = USB_CDC_IFACE_NAME_0;
{indent}}}
{indent}#endif
{indent}#ifdef USB_CDC_IFACE_NAME_1
{indent}if (_instance == 1) {{
{indent}  default_cdc_name = USB_CDC_IFACE_NAME_1;
{indent}}}
{indent}#endif
{indent}this->setStringDescriptor(default_cdc_name);"""

CDC_PATCH_BLOCK_OLD = """  const char* default_cdc_name = "TinyUSB Serial";
#ifdef USB_CDC_IFACE_NAME_0
  if (_instance_count == 0) {
    default_cdc_name = USB_CDC_IFACE_NAME_0;
  }
#endif
#ifdef USB_CDC_IFACE_NAME_1
  if (_instance_count == 1) {
    default_cdc_name = USB_CDC_IFACE_NAME_1;
  }
#endif
  this->setStringDescriptor(default_cdc_name);"""


def parse_macro_define(source_text: str, macro_name: str) -> Optional[str]:
    pattern = rf'^\s*#define\s+{macro_name}\s+"([^"]*)"'
    m = re.search(pattern, source_text, flags=re.MULTILINE)
    if not m:
        return None
    return m.group(1)


def parse_macro_from_cppdefines(macro_name: str) -> Optional[str]:
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
                value = item.split("=", 1)[1]
                key = macro_name
        if key != macro_name:
            continue
        if value is None:
            return ""
        if not isinstance(value, str):
            return str(value)
        cleaned = value.replace('\\"', '"').strip('"')
        return cleaned
    return None


def parse_macro_from_project_files(project_dir: Path, macro_name: str) -> Optional[str]:
    # Prefer the known controller source, then search src/include for portability.
    candidates = [
        project_dir / "src" / "controller_main.cpp",
        project_dir / "src" / "pico_master.cpp",  # legacy filename fallback
    ]
    for root in ("src", "include"):
        base = project_dir / root
        if not base.exists():
            continue
        for ext in ("*.h", "*.hpp", "*.c", "*.cpp", "*.ino"):
            for path in sorted(base.rglob(ext)):
                if path not in candidates:
                    candidates.append(path)

    for path in candidates:
        if not path.exists():
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        value = parse_macro_define(text, macro_name)
        if value is not None:
            return value
    return None


def resolve_cdc_name(project_dir: Path, macro_name: str, default_value: str) -> str:
    value = parse_macro_from_cppdefines(macro_name)
    if value is not None:
        return value or default_value

    value = parse_macro_from_project_files(project_dir, macro_name)
    if value is not None:
        return value

    return default_value


def patch_framework_cdc_cpp(framework_dir: Path) -> None:
    cdc_cpp = framework_dir / "libraries" / "Adafruit_TinyUSB_Arduino" / "src" / "arduino" / "Adafruit_USBD_CDC.cpp"
    if not cdc_cpp.exists():
        raise RuntimeError(f"patch_tinyusb_cdc: file not found: {cdc_cpp}")

    text = cdc_cpp.read_text(encoding="utf-8")

    if "this->setStringDescriptor(default_cdc_name);" in text and "USB_CDC_IFACE_NAME_0" in text:
        if "_instance_count == 0" in text or "_instance_count == 1" in text:
            text = text.replace(CDC_PATCH_BLOCK_OLD, CDC_PATCH_BLOCK.format(indent="  "), 1)
            cdc_cpp.write_text(text, encoding="utf-8")
            print("patch_tinyusb_cdc: corrected existing framework patch")
        else:
            print("patch_tinyusb_cdc: framework already patched")
        return

    marker = re.compile(r'^(?P<indent>\s*)this->setStringDescriptor\("TinyUSB Serial"\);\s*$', re.MULTILINE)
    m = marker.search(text)
    if not m:
        raise RuntimeError("patch_tinyusb_cdc: expected TinyUSB CDC marker not found")

    indent = m.group("indent")
    patched = CDC_PATCH_BLOCK.format(indent=indent)
    updated = text[:m.start()] + patched + text[m.end():]
    cdc_cpp.write_text(updated, encoding="utf-8")
    print("patch_tinyusb_cdc: patched Adafruit_USBD_CDC.cpp")


project_dir = Path(env["PROJECT_DIR"])
cdc0_name = resolve_cdc_name(project_dir, CDC0_MACRO, CDC0_DEFAULT)
cdc1_name = resolve_cdc_name(project_dir, CDC1_MACRO, CDC1_DEFAULT)

env.Append(
    CPPDEFINES=[
        f'USB_CDC_IFACE_NAME_0=\\"{cdc0_name}\\"',
        f'USB_CDC_IFACE_NAME_1=\\"{cdc1_name}\\"',
    ]
)
print(f"patch_tinyusb_cdc: CDC names -> 0='{cdc0_name}', 1='{cdc1_name}'")

framework_dir = Path(env.PioPlatform().get_package_dir("framework-arduinopico"))
patch_framework_cdc_cpp(framework_dir)
