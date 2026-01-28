#!/usr/bin/env python3
import sys
from pathlib import Path


def main() -> int:
    repo_root = Path(__file__).resolve().parent
    scripts_dir = repo_root / "scripts"
    sys.path.insert(0, str(scripts_dir))
    try:
        from vlm_debug import main as debug_main
    except Exception as exc:
        raise SystemExit(f"Failed to import scripts/vlm_debug.py: {exc}")
    return debug_main()


if __name__ == "__main__":
    sys.exit(main())
