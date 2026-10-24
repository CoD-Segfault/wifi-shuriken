"""Replace only the framework's MTP transport, without editing the installed SDK."""
from pathlib import Path

Import("env")

framework = Path(env.PioPlatform().get_package_dir("framework-arduinopico"))
upstream_mtp = (framework / "libraries/Adafruit_TinyUSB_Arduino/src/class/mtp/mtp_device.c").resolve()


def skip_upstream_mtp(build_env, node):
    if Path(node.srcnode().get_abspath()).resolve() == upstream_mtp:
        print("use_local_mtp: replacing framework MTP transport")
        return None
    return node


env.AddBuildMiddleware(skip_upstream_mtp)
