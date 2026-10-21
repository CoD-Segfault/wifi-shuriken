from pathlib import Path

Import("env")

root = Path(env.subst("$PROJECT_DIR")) / "third_party" / "fatfs"
libraries = env.get("LIBS", [])
if not any(str(lib) in ("fatfs", "-lfatfs") for lib in libraries):
    raise RuntimeError("SDK FatFs library not found; check the exFAT build integration")

# Rebuild the entire component, including VFS, against the exFAT structure layouts.
env.Replace(LIBS=[lib for lib in libraries if str(lib) not in ("fatfs", "-lfatfs")])
replacement = env.BuildLibrary(
    "$BUILD_DIR/fatfs",
    str(root),
    src_filter=[
        "+<src/ff.c>",
        "+<src/ffunicode.c>",
        "+<port/freertos/ffsystem.c>",
        "+<diskio/*.c>",
        "+<vfs/*.c>",
    ],
)
env.Prepend(LIBS=[replacement])
