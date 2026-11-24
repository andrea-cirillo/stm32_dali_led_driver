#!/usr/bin/env python3
"""
Creates the GitHub issues listed in bench_tests_github.csv with the gh CLI.

Usage (from the repository root, gh authenticated):
    python3 Docs/BenchTests/import_github.py [--repo owner/name] [--dry-run]

Missing labels are created first. Run it once: it does not check for
issues that already exist.
"""
import argparse
import csv
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CSV_FILE = os.path.join(HERE, "bench_tests_github.csv")


def run(cmd, dry):
    print("+", " ".join(cmd[:4]), "...")
    if not dry:
        subprocess.run(cmd, check=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", help="owner/name (default: current repository)")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    repo = ["--repo", args.repo] if args.repo else []

    with open(CSV_FILE, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    labels = sorted({l for r in rows for l in r["labels"].split(",") if l})
    for label in labels:
        run(["gh", "label", "create", label, "--force"] + repo, args.dry_run)

    for r in rows:
        cmd = ["gh", "issue", "create", "--title", r["title"], "--body", r["body"]] + repo
        for label in r["labels"].split(","):
            cmd += ["--label", label]
        run(cmd, args.dry_run)
    return 0


if __name__ == "__main__":
    sys.exit(main())
