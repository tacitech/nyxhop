#!/usr/bin/env python3
"""Package the Android app without gradle: only aapt, zipalign and apksigner from the
Android SDK. There is no Java in the app (a NativeActivity around the Rust .so).

    python build_apk.py                # build arm64, package, sign
    python build_apk.py --skip-cargo   # package again without rebuilding

Result: android/out/nyxhop.apk. Copy it to the phone and install it (allow unknown sources).
Needs the Android SDK and NDK, cargo-ndk and the aarch64-linux-android Rust target.
"""
import os, shutil, subprocess, sys, zipfile

def workspace_root():
    """Nearest ancestor holding the cargo workspace (this script moves between trees)."""
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        p = os.path.join(d, "Cargo.toml")
        if os.path.isfile(p) and "[workspace]" in open(p, encoding="utf-8").read():
            return d
        up = os.path.dirname(d)
        if up == d:
            sys.exit("no cargo workspace above " + __file__)
        d = up


ROOT = workspace_root()
# android/ sits next to this script in the public tree, at the repo root here.
ANDROID = os.path.join(os.path.dirname(os.path.abspath(__file__)), "android")
if not os.path.isdir(ANDROID):
    ANDROID = os.path.join(ROOT, "android")
SDK = os.environ.get(
    "ANDROID_HOME",
    os.path.expanduser("~/AppData/Local/Android/Sdk").replace("\\", "/"))
NDK = os.environ.get(
    "ANDROID_NDK_HOME", f"{SDK}/ndk/23.1.7779620")
BT = os.environ.get("ANDROID_BUILD_TOOLS", f"{SDK}/build-tools/34.0.0")
JAR = f"{SDK}/platforms/android-34/android.jar"
KEYTOOL = os.environ.get(
    "KEYTOOL", "C:/Program Files/Android/Android Studio/jbr/bin/keytool.exe")
OUT = os.path.join(ANDROID, "out")
KS = os.path.join(OUT, "debug.keystore")
SO = os.path.join(ROOT, "target", "aarch64-linux-android", "release",
                  "libnyx_mobile.so")
TC = f"{NDK}/toolchains/llvm/prebuilt/windows-x86_64"
# libc++_shared.so must ship inside the APK: openh264 is C++, so libnyx_mobile.so
# has a DT_NEEDED on it, and Android does not carry that library in the system.
# Without it the APK installs fine and dies on the first launch with:
#   UnsatisfiedLinkError: dlopen failed: library "libc++_shared.so" not found
STL = f"{TC}/sysroot/usr/lib/aarch64-linux-android/libc++_shared.so"
STRIP = f"{TC}/bin/llvm-strip.exe"
READELF = f"{TC}/bin/llvm-readelf.exe"

# Libraries Android provides itself; these need not be packaged.
SYSLIBS = {
    "libc.so", "libm.so", "libdl.so", "liblog.so", "libandroid.so",
    "libz.so", "libEGL.so", "libGLESv1_CM.so", "libGLESv2.so", "libGLESv3.so",
    "libOpenSLES.so", "libaaudio.so", "libmediandk.so", "libjnigraphics.so",
    "libnativewindow.so", "libvulkan.so", "libcamera2ndk.so", "ld-android.so",
}


def needed_libs(path):
    """The DT_NEEDED entries of a .so."""
    r = subprocess.run([READELF, "-d", path], capture_output=True, text=True,
                       env=ENV)
    out = []
    for line in r.stdout.splitlines():
        if "(NEEDED)" in line and "[" in line:
            out.append(line.split("[")[1].split("]")[0])
    return out


# apksigner.bat needs JAVA_HOME (Android Studio's bundled JBR does): a Windows
# machine without java on the PATH is the normal case.
JBR = os.environ.get("JAVA_HOME") or os.path.dirname(os.path.dirname(KEYTOOL))
ENV = dict(os.environ, JAVA_HOME=JBR,
           PATH=os.path.join(JBR, "bin") + os.pathsep + os.environ.get("PATH", ""))


def run(cmd, **kw):
    kw.setdefault("env", ENV)
    print("+", " ".join(str(c) for c in cmd[:4]), "…", flush=True)
    r = subprocess.run(cmd, capture_output=True, text=True, **kw)
    if r.returncode != 0:
        print(r.stdout[-2000:])
        print(r.stderr[-2000:])
        sys.exit(f"FAIL: {cmd[0]}")
    return r


def main():
    os.makedirs(OUT, exist_ok=True)
    for p, what in [(SDK, "SDK"), (NDK, "NDK"), (BT, "build-tools"),
                    (JAR, "android.jar")]:
        if not os.path.exists(p):
            sys.exit(f"{what} not found: {p}")

    # 1) build the aarch64 .so with cargo-ndk. Note --platform, not -p: cargo-ndk
    #    reads "-p 31" as a package name and panics.
    if "--skip-cargo" not in sys.argv:
        env = dict(os.environ, ANDROID_NDK_HOME=NDK, ANDROID_HOME=SDK)
        run(["cargo", "ndk", "-t", "arm64-v8a", "--platform", "31",
             "build", "--release", "-p", "nyx-mobile", "--lib"],
            cwd=ROOT, env=env)
    if not os.path.exists(SO):
        sys.exit(f".so not found: {SO}")

    # 2) a debug keystore, made once
    if not os.path.exists(KS):
        run([KEYTOOL, "-genkeypair", "-v", "-keystore", KS,
             "-alias", "nyx", "-keyalg", "RSA", "-keysize", "2048",
             "-validity", "10000", "-storepass", "nyxhop",
             "-keypass", "nyxhop",
             "-dname", "CN=NyxHop, OU=Dev, O=NyxHop, C=VN"])

    # 3) the bare APK: manifest only; hasCode=false, so no classes.dex
    base = os.path.join(OUT, "base.apk")
    run([f"{BT}/aapt.exe", "package", "-f",
         "-M", os.path.join(ANDROID, "AndroidManifest.xml"),
         "-I", JAR, "-F", base])

    # 4) the .so files into lib/arm64-v8a/ (STORED: Android maps them in place, no unpacking)
    stl = os.path.join(OUT, "libc++_shared.so")
    shutil.copy(STL, stl)
    run([STRIP, stl])  # 6.9 MB -> ~1 MB, nothing lost
    libs = {"libnyx_mobile.so": SO, "libc++_shared.so": stl}
    with zipfile.ZipFile(base, "a", zipfile.ZIP_STORED) as z:
        for name, src in libs.items():
            z.write(src, f"lib/arm64-v8a/{name}")

    # 4b) dependency check: every DT_NEEDED that is not a system library must be in
    #     the APK. A missing one installs with "Success" and dies on launch; better to
    #     stop here than to carry a broken APK to the field.
    missing = set()
    for name, src in libs.items():
        for dep in needed_libs(src):
            if dep not in SYSLIBS and dep not in libs:
                missing.add(f"{dep} (needed by {name})")
    if missing:
        sys.exit("missing .so in the APK: " + ", ".join(sorted(missing)))
    print(f"  deps OK: {', '.join(sorted(libs))}")

    # 5) zipalign and sign
    aligned = os.path.join(OUT, "aligned.apk")
    final = os.path.join(OUT, "nyxhop.apk")
    if os.path.exists(aligned):
        os.remove(aligned)
    run([f"{BT}/zipalign.exe", "-p", "-f", "4", base, aligned])
    if os.path.exists(final):
        os.remove(final)
    shutil.copy(aligned, final)
    run([f"{BT}/apksigner.bat", "sign", "--ks", KS,
         "--ks-pass", "pass:nyxhop", "--key-pass", "pass:nyxhop",
         "--v1-signing-enabled", "true", "--v2-signing-enabled", "true",
         final])
    run([f"{BT}/apksigner.bat", "verify", final])

    mb = os.path.getsize(final) / 1e6
    print(f"\nOK -> {final}  ({mb:.1f} MB)")
    print("Copy it to the phone and install it (allow unknown sources). On the first "
          "start, tap the screen and enter the receiving board's address, e.g. 192.168.0.12:7011.")


if __name__ == "__main__":
    main()
