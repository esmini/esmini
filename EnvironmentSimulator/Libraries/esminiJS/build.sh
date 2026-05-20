#!/usr/bin/env bash

set -euo pipefail

if ! command -v emcmake >/dev/null 2>&1; then
    echo "emcmake not found. Activate emsdk before building esminiJS." >&2
    exit 1
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/../../.." && pwd)"
build_dir="${script_dir}/build"
example_dir="${script_dir}/example"
patched_fmt_root="${build_dir}/patched-externals/fmt"
patched_fmt_include_dir="${patched_fmt_root}/include"
fmt_patch="${repo_root}/scripts/patches/fmt-11.2.0-emscripten-cstdlib.patch"
user_em_config="${HOME}/.emscripten"
parallel_jobs=1

if [[ -f "$user_em_config" ]]; then
    export EM_CONFIG="$user_em_config"
fi

if command -v nproc >/dev/null 2>&1; then
    parallel_jobs="$(nproc)"
elif command -v sysctl >/dev/null 2>&1; then
    parallel_jobs="$(sysctl -n hw.logicalcpu 2>/dev/null || printf '1')"
elif command -v getconf >/dev/null 2>&1; then
    parallel_jobs="$(getconf _NPROCESSORS_ONLN 2>/dev/null || printf '1')"
fi

if [[ ! -f "$fmt_patch" ]]; then
    echo "Expected fmt compatibility patch not found: $fmt_patch" >&2
    exit 1
fi

if ! command -v patch >/dev/null 2>&1; then
    echo "patch command not found. Install patch to prepare the WASM build includes." >&2
    exit 1
fi

if [[ -f "$build_dir/CMakeCache.txt" ]]; then
    cache_source_dir="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$build_dir/CMakeCache.txt" | head -n 1)"
    if [[ -n "$cache_source_dir" && "$cache_source_dir" != "$script_dir" ]]; then
        echo "Clearing stale CMake cache from $cache_source_dir"
        rm -rf "$build_dir"
    fi
fi

mkdir -p "$build_dir"
rm -rf "$patched_fmt_root"
mkdir -p "$patched_fmt_root"
cp -R "$repo_root/externals/fmt/include" "$patched_fmt_root/"

patch -s -N -p1 -d "$patched_fmt_root" < "$fmt_patch"

cd "$build_dir"
emcmake cmake -DCMAKE_BUILD_TYPE=Release -DFMT_OVERRIDE_INCLUDE_DIR="$patched_fmt_include_dir" ..
cmake --build . --parallel "$parallel_jobs"

cp -f esmini.js "$example_dir/"
if [[ -f esmini.wasm ]]; then
    cp -f esmini.wasm "$example_dir/"
fi

echo "Built esminiJS artifacts in $build_dir and copied runtime files to $example_dir"