#!/usr/bin/env python3
"""
Creates the GitHub issues listed in bench_tests_github.csv with the gh CLI.

Requirements:
    - GitHub CLI installed and in PATH (https://cli.github.com,
      Windows: "winget install --id GitHub.cli", then open a new terminal).
    - Authenticated: "gh auth login".

Usage (from any directory):
    python3 Docs/BenchTests/import_github.py --repo owner/name [--dry-run]

Missing labels are created first. Issues whose title already exists in the
repository (open or closed) are skipped, so the script can be run again.
"""
import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
CSV_FILE = os.path.join(HERE, "bench_tests_github.csv")


def find_gh():
    """Returns the gh executable path or exits with an explanation."""
    gh = shutil.which("gh")
    if gh is None:
        sys.exit("ERROR: GitHub CLI 'gh' not found in PATH.\n"
                 "Install it (Windows: winget install --id GitHub.cli) and open a\n"
                 "new terminal, or add its folder to PATH. Then run 'gh auth login'.")
    if subprocess.run([gh, "auth", "status"], capture_output=True).returncode != 0:
        sys.exit("ERROR: gh is not authenticated. Run 'gh auth login' first.")
    return gh


def run(cmd, dry):
    """Prints and (unless dry) executes a command, stopping on error."""
    print("+", " ".join(cmd[1:4]), "...")
    if not dry:
        subprocess.run(cmd, check=True)


def existing_titles(gh, repo):
    """Titles of all issues already in the repository."""
    out = subprocess.run([gh, "issue", "list", "--state", "all", "--limit", "1000",
                          "--json", "title"] + repo,
                         check=True, capture_output=True, text=True, encoding="utf-8").stdout
    return {i["title"] for i in json.loads(out)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", help="owner/name (default: repository of the current directory)")
    ap.add_argument("--dry-run", action="store_true", help="print the commands only")
    args = ap.parse_args()
    repo = ["--repo", args.repo] if args.repo else []

    with open(CSV_FILE, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    gh = "gh" if args.dry_run else find_gh()
    done = set() if args.dry_run else existing_titles(gh, repo)

    labels = sorted({l for r in rows for l in r["labels"].split(",") if l})
    for label in labels:
        run([gh, "label", "create", label, "--force"] + repo, args.dry_run)

    for r in rows:
        if r["title"] in done:
            print("= skipped (already exists):", r["title"])
            continue
        # body through a file: no command line length / quoting issues on Windows
        with tempfile.NamedTemporaryFile("w", suffix=".md", delete=False,
                                         encoding="utf-8") as tmp:
            tmp.write(r["body"])
        try:
            cmd = [gh, "issue", "create", "--title", r["title"], "--body-file", tmp.name] + repo
            for label in r["labels"].split(","):
                cmd += ["--label", label]
            run(cmd, args.dry_run)
        finally:
            os.remove(tmp.name)
    return 0


if __name__ == "__main__":
    sys.exit(main())
