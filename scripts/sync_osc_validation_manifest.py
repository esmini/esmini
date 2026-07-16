#!/usr/bin/env python3
"""
Sync osc_validation_manifest.yml with the tests currently discovered by
pytest's --collect-only.

For every test currently discovered by pytest:
  - if it already has an entry in osc_validation_manifest.yml, its rule and reason are left
    untouched
  - if it is new, it is appended to the NOT_REVIEWED_YET rule

Tests listed in osc_validation_manifest.yml that are no longer discovered by pytest are not
removed automatically, they are only reported as a warning.

Requirements:
  - --cwd must point at an existing local checkout of PMSFIT/osc-validation
    with its dependencies installed (e.g. `pip install -e .` in a venv, with
    that venv active or its pytest otherwise on PATH).
  - --collect-root is a path *inside* that checkout, e.g. "osc_validation/validation"
    (note the underscore - it is a Python package name, not a directory to guess).

Example, run from the esmini repo root, assuming a sibling checkout of
osc-validation with its venv already active:
    python3 scripts/sync_osc_validation_manifest.py \
        --manifest .github/actions/run_osc_validation/osc_validation_manifest.yml \
        --collect-root osc_validation/validation \
        --cwd ../osc-validation
"""
import argparse
import re
import subprocess
import sys
from pathlib import Path

import yaml

VALID_KEYWORDS = (
    "INCLUDE",
    "INTENDED_DEVIATION",
    "DEVIATION",
    "REJECTED",
    "NOT_SUPPORTED",
    "NOT_REVIEWED_YET",
)
# Matches pytest -q collect-only summary lines, e.g. "28 tests collected in 1.76s"
# or "no tests collected", which should not be treated as test ids.
SUMMARY_LINE_RE = re.compile(r"^(\d+ tests? collected|no tests collected)", re.IGNORECASE)


def collect_tests(cwd: Path, collect_root: str) -> list[str]:
    """Run pytest --collect-only and return the collected test ids, prefixed
    with collect_root so they match the format used in osc_validation_manifest.yml."""
    result = subprocess.run(
        ["pytest", collect_root, "--collect-only", "-q"],
        cwd=cwd,
        capture_output=True,
        text=True,
    )
    # pytest exits with 5 when no tests are collected, which is not an error here.
    if result.returncode not in (0, 5):
        print(result.stdout)
        print(result.stderr, file=sys.stderr)
        raise SystemExit(f"pytest --collect-only failed with exit code {result.returncode}")

    prefix = collect_root.rstrip("/") + "/"
    tests = []
    for line in result.stdout.splitlines():
        line = line.strip()
        if not line or line.startswith("=") or SUMMARY_LINE_RE.match(line):
            continue
        tests.append(prefix + line)
    return tests


def parse_manifest(path: Path) -> tuple[dict, dict[str, str], str]:
    """Return the YAML document, test id rules, and leading comments."""
    text = path.read_text(encoding="utf-8") if path.exists() else ""
    try:
        document = yaml.safe_load(text) or {"rules": {}}
    except yaml.YAMLError as error:
        raise SystemExit(f"Unable to parse manifest '{path}': {error}") from error

    if not isinstance(document, dict) or not isinstance(document.get("rules"), dict):
        raise SystemExit(f"Manifest '{path}' must contain a 'rules' mapping")

    rules = document["rules"]
    unknown_rules = set(rules) - set(VALID_KEYWORDS)
    if unknown_rules:
        raise SystemExit(
            f"Unknown manifest rule(s): {', '.join(sorted(unknown_rules))}"
        )

    known: dict[str, str] = {}
    for keyword in VALID_KEYWORDS:
        entries = rules.setdefault(keyword, [])
        if not isinstance(entries, list):
            raise SystemExit(f"'rules.{keyword}' in '{path}' must be a list")
        for entry in entries:
            if not isinstance(entry, dict) or not isinstance(entry.get("id"), str):
                raise SystemExit(
                    f"Every 'rules.{keyword}' entry in '{path}' must contain a string 'id'"
                )
            known[entry["id"]] = keyword

    leading_comments = []
    for line in text.splitlines():
        if line.startswith("#") or not line.strip():
            leading_comments.append(line)
        else:
            break
    comment_block = "\n".join(leading_comments).rstrip()
    return document, known, comment_block


def sync(manifest_path: Path, collected: list[str]) -> None:
    document, known, leading_comments = parse_manifest(manifest_path)

    new_tests = [t for t in collected if t not in known]
    stale_tests = sorted(set(known) - set(collected))

    if new_tests:
        document["rules"]["NOT_REVIEWED_YET"].extend(
            {"id": test} for test in new_tests
        )
        yaml_text = yaml.safe_dump(
            document, sort_keys=False, allow_unicode=True, width=4096
        )
        if leading_comments:
            yaml_text = f"{leading_comments}\n\n{yaml_text}"
        manifest_path.write_text(yaml_text, encoding="utf-8")

    included = sum(1 for keyword in known.values() if keyword == "INCLUDE")
    intended_deviations = sum(
        1 for keyword in known.values() if keyword == "INTENDED_DEVIATION"
    )
    deviations = sum(1 for keyword in known.values() if keyword == "DEVIATION")
    rejected = sum(1 for keyword in known.values() if keyword == "REJECTED")
    not_supported = sum(1 for keyword in known.values() if keyword == "NOT_SUPPORTED")
    not_reviewed_yet = sum(
        1 for keyword in known.values() if keyword == "NOT_REVIEWED_YET"
    )

    print(f"Collected tests:     {len(collected)}")
    print(f"New tests added:     {len(new_tests)} (as NOT_REVIEWED_YET)")
    print(f"Existing included:   {included}")
    print(f"Existing intended deviations: {intended_deviations}")
    print(f"Existing deviations: {deviations}")
    print(f"Existing not supported: {not_supported}")
    print(f"Existing rejected:   {rejected}")
    print(f"Existing not reviewed yet: {not_reviewed_yet}")
    print(f"Existing total:      {len(known)}")

    if stale_tests:
        print(f"\nWarning: {len(stale_tests)} manifest entries were not collected by pytest (stale?):")
        for test in stale_tests:
            print(f"  - {test}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--manifest",
        required=True,
        type=Path,
        help="Path to osc_validation_manifest.yml",
    )
    parser.add_argument(
        "-cwd",
        default=Path("."),
        type=Path,
        help="Directory to run pytest from, i.e. the osc-validation checkout (default: %(default)s)",
    )
    parser.add_argument(
        "-cr",
        "--collect-root",
        default="osc_validation/validation",
        help="pytest collection root, relative to -cwd (default: %(default)s)",
    )
    args = parser.parse_args()

    if not args.cwd.is_dir():
        raise SystemExit(f"--cwd '{args.cwd}' is not a directory (expected the osc-validation checkout)")

    collect_path = args.cwd / args.collect_root
    if not collect_path.exists():
        raise SystemExit(
            f"--collect-root '{args.collect_root}' does not exist under --cwd '{args.cwd}' "
            f"(looked for '{collect_path}'). Did you mean 'osc_validation/validation' (with an underscore)?"
        )

    collected = collect_tests(args.cwd, args.collect_root)
    sync(args.manifest, collected)


if __name__ == "__main__":
    main()
