#!/usr/bin/env python3
"""
Compile Lua source files into 32-bit ArduPilot-compatible bytecode.

This tool builds a host-native luac compiler configured for 32-bit target
architecture (LUA_32BITS=1, sizeof(size_t)=4), ensuring bytecode compatibility
with ArduPilot's embedded Lua VM (e.g. ESP32, STM32) regardless of whether the host
is a 64-bit macOS, Linux, or Windows system.

AP_FLAKE8_CLEAN
"""

import argparse
import hashlib
import os
import shutil
import subprocess
import sys
import tempfile


def get_repo_root():
    """Return absolute path to repository root."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(script_dir, "..", ".."))


def find_host_compiler():
    """Find a C compiler on the host system."""
    cc = os.environ.get("CC")
    if cc and shutil.which(cc):
        return cc
    for candidate in ["clang", "gcc", "cc"]:
        if shutil.which(candidate):
            return candidate
    raise RuntimeError("No suitable C compiler (clang/gcc/cc) found on host system")


def get_cached_luac_path():
    """Get path to the cached 32-bit luac binary."""
    repo_root = get_repo_root()
    cache_dir = os.path.join(repo_root, "build", "Tools")
    os.makedirs(cache_dir, exist_ok=True)
    bin_name = "luac_32bit"
    if sys.platform == "win32":
        bin_name += ".exe"
    return os.path.join(cache_dir, bin_name)


def build_luac_32bit():
    """Build the host 32-bit target-compatible luac compiler."""
    repo_root = get_repo_root()
    lua_src_dir = os.path.join(repo_root, "libraries", "AP_Scripting", "lua", "src")
    luac_bin = get_cached_luac_path()

    # Calculate hash of source files to determine if rebuild is needed
    c_files = [
        "lapi.c", "lcode.c", "lctype.c", "ldebug.c", "ldo.c", "ldump.c",
        "lfunc.c", "lgc.c", "llex.c", "lmem.c", "lobject.c", "lopcodes.c",
        "lparser.c", "lstate.c", "lstring.c", "ltable.c", "ltm.c",
        "lundump.c", "lvm.c", "lzio.c", "lauxlib.c", "lbaselib.c", "luac.c"
    ]
    h_files = [
        "lapi.h", "lauxlib.h", "lcode.h", "lctype.h", "ldebug.h", "ldo.h",
        "lfunc.h", "lgc.h", "llex.h", "llimits.h", "lmem.h", "lobject.h",
        "lopcodes.h", "lparser.h", "lprefix.h", "lstate.h", "lstring.h",
        "ltable.h", "ltm.h", "lua.h", "luaconf.h", "lualib.h", "lundump.h",
        "lvm.h", "lzio.h"
    ]

    hasher = hashlib.sha256()
    for fname in sorted(c_files + h_files):
        fpath = os.path.join(lua_src_dir, fname)
        if os.path.exists(fpath):
            with open(fpath, "rb") as f:
                hasher.update(f.read())
    # Include tool version in hash
    hasher.update(b"v1_32bit_compat")
    src_hash = hasher.hexdigest()

    hash_file = luac_bin + ".sha256"
    if os.path.exists(luac_bin) and os.path.exists(hash_file):
        with open(hash_file, "r") as f:
            if f.read().strip() == src_hash:
                return luac_bin

    cc = find_host_compiler()

    with tempfile.TemporaryDirectory() as tmpdir:
        # Copy source and header files
        for fname in c_files + h_files:
            src_path = os.path.join(lua_src_dir, fname)
            dst_path = os.path.join(tmpdir, fname)
            if os.path.exists(src_path):
                shutil.copyfile(src_path, dst_path)

        # Apply transformations for host standalone compilation with 32-bit bytecode target:

        # 1. lprefix.h: Remove AP_Filesystem/posix_compat.h and fix lua_writestringerror
        lprefix_path = os.path.join(tmpdir, "lprefix.h")
        with open(lprefix_path, "r") as f:
            content = f.read()
        content = content.replace("#include <AP_Filesystem/posix_compat.h>", "/* no posix_compat */")
        content = content.replace(
            "#define lua_writestringerror(s,l) lua_writestring(s,l)",
            "#define lua_writestringerror(s,p) (fprintf(stderr, (s), (p)), fflush(stderr))"
        )
        with open(lprefix_path, "w") as f:
            f.write(content)

        # 2. luaconf.h: Remove AP_Scripting/lua_common_defs.h
        luaconf_path = os.path.join(tmpdir, "luaconf.h")
        with open(luaconf_path, "r") as f:
            content = f.read()
        content = content.replace("#include <AP_Scripting/lua_common_defs.h>", "/* no lua_common_defs */")
        with open(luaconf_path, "w") as f:
            f.write(content)

        # 3. ldo.c: Replace ap_setjmp and lua_abort with standard POSIX equivalents
        ldo_path = os.path.join(tmpdir, "ldo.c")
        with open(ldo_path, "r") as f:
            content = f.read()
        content = content.replace("#include <AP_HAL/ap_setjmp.h>", "#include <setjmp.h>")
        content = content.replace("ap_setjmp", "setjmp")
        content = content.replace("ap_longjmp", "longjmp")
        content = content.replace("ap_jmp_buf", "jmp_buf")
        content = content.replace("lua_abort()", "abort()")
        with open(ldo_path, "w") as f:
            f.write(content)

        # 4. lauxlib.c: Uncomment l_alloc, panic, and luaL_newstate
        lauxlib_path = os.path.join(tmpdir, "lauxlib.c")
        with open(lauxlib_path, "r") as f:
            lines = f.readlines()
        new_lines = []
        for line in lines:
            if line.startswith("// static void *l_alloc") or \
               line.startswith("//   (void)ud;") or \
               line.startswith("//   if (nsize ==") or \
               line.startswith("//     free(ptr);") or \
               line.startswith("//     return NULL;") or \
               line.startswith("//   }") or \
               line.startswith("//   else {") or \
               line.startswith("//     void *newptr =") or \
               line.startswith("//     if (newptr ==") or \
               line.startswith("//       return ptr;") or \
               line.startswith("//     else") or \
               line.startswith("//       return newptr;") or \
               line.startswith("// static int panic") or \
               line.startswith("//   lua_writestringerror") or \
               line.startswith("//                         lua_tostring") or \
               line.startswith("//   return 0;") or \
               line.startswith("// LUALIB_API lua_State *luaL_newstate") or \
               line.startswith("//   lua_State *L =") or \
               line.startswith("//   if (L) lua_atpanic") or \
               line.startswith("//   return L;"):
                new_lines.append(line[3:])
            elif line.strip() == "// }":
                new_lines.append("}\n")
            else:
                new_lines.append(line)
        with open(lauxlib_path, "w") as f:
            f.writelines(new_lines)

        # 5. ldump.c: Force 32-bit bytecode output (sizeof(size_t) = 4, size_t dump = uint32_t)
        ldump_path = os.path.join(tmpdir, "ldump.c")
        with open(ldump_path, "r") as f:
            content = f.read()
        content = content.replace("DumpByte(sizeof(size_t), D);", "DumpByte(4, D);")
        content = content.replace(
            "DumpVar(size, D);",
            "{ uint32_t s32 = (uint32_t)size; DumpVector(&s32, 1, D); }"
        )
        with open(ldump_path, "w") as f:
            f.write(content)

        # Compile command
        cmd = [
            cc, "-O2", "-DLUA_32BITS=1",
            "-o", luac_bin
        ]
        if sys.platform == "darwin":
            cmd.append("-DLUA_USE_MACOSX")
        elif sys.platform.startswith("linux"):
            cmd.append("-DLUA_USE_LINUX")
        else:
            cmd.append("-DLUA_USE_POSIX")

        for c_file in c_files:
            cmd.append(os.path.join(tmpdir, c_file))
        cmd.append("-lm")

        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if res.returncode != 0:
            raise RuntimeError(f"Failed to build luac_32bit:\n{res.stderr}")

    with open(hash_file, "w") as f:
        f.write(src_hash + "\n")

    return luac_bin


def compile_lua_file(input_path, output_path=None, strip=True):
    """
    Compile a single Lua file to 32-bit bytecode.

    If output_path is not specified, returns the bytecode as bytes.
    If the file is already compiled bytecode, copies/returns it directly.
    """
    with open(input_path, "rb") as f:
        header = f.read(4)

    if header == b"\x1bLua":
        # Already compiled bytecode
        if output_path:
            shutil.copyfile(input_path, output_path)
            return output_path
        with open(input_path, "rb") as f:
            return f.read()

    luac_bin = build_luac_32bit()

    dest = output_path
    if not dest:
        temp_out = tempfile.NamedTemporaryFile(delete=False, suffix=".luac")
        dest = temp_out.name
        temp_out.close()

    try:
        cmd = [luac_bin]
        if strip:
            cmd.append("-s")
        cmd.extend(["-o", dest, input_path])
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if res.returncode != 0:
            raise RuntimeError(f"Lua compilation failed for {input_path}:\n{res.stderr}")

        if not output_path:
            with open(dest, "rb") as f:
                data = f.read()
            return data
        return dest
    finally:
        if not output_path and os.path.exists(dest):
            os.unlink(dest)


def main():
    parser = argparse.ArgumentParser(
        description="Compile Lua scripts to 32-bit ArduPilot-compatible bytecode"
    )
    parser.add_argument("input", help="Path to input Lua script (.lua)")
    parser.add_argument("-o", "--output", help="Path to output compiled bytecode file")
    parser.add_argument("--no-strip", action="store_true", help="Do not strip debug information")

    args = parser.parse_args()

    out_path = args.output
    if not out_path:
        out_path = args.input

    try:
        compile_lua_file(args.input, out_path, strip=not args.no_strip)
        print(f"Successfully compiled {args.input} -> {out_path}")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
