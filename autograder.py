"""Run each public test in a fresh subprocess with a hard timeout."""
import argparse
import ast
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--test", help="One PublicTests method, e.g. test_one_box")
    parser.add_argument("--json", type=Path, help="Optional JSON results file")
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    # Discover test names without importing potentially broken student code.
    tree = ast.parse((root / "public_tests.py").read_text(encoding="utf-8"))
    names = sorted(node.name for cls in tree.body if isinstance(cls, ast.ClassDef)
                   and cls.name == "PublicTests" for node in cls.body
                   if isinstance(node, ast.FunctionDef) and node.name.startswith("test_"))
    if args.test:
        if args.test not in names:
            parser.error("Unknown public test: " + args.test)
        names = [args.test]
    results = []
    for name in names:
        try:
            run = subprocess.run(
                [sys.executable, "-m", "unittest", "public_tests.PublicTests." + name],
                cwd=root, capture_output=True, text=True, errors="replace", timeout=120)
            status = "PASS" if run.returncode == 0 else "FAIL"
            detail = run.stdout + run.stderr
        except subprocess.TimeoutExpired:
            status, detail = "TIMEOUT", "Exceeded public test safety limit (120 seconds)."
        results.append(dict(test=name, status=status, detail=detail))
        print("{} {}".format(status, name))
        if status != "PASS":
            print(detail.strip())
    passed = sum(item["status"] == "PASS" for item in results)
    payload = dict(kind="public_feedback_only", passed=passed, total=len(results), results=results)
    if args.json:
        args.json.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    print("\nPublic feedback: {}/{}. This is NOT the official grade.".format(passed, len(results)))
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
