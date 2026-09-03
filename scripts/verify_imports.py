#!/usr/bin/env python3
"""Import every project module; exit 1 on failure."""

import importlib
import pkgutil
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]

# Directories that sit alongside the project's own packages but aren't part
# of it: this checker's own home, vendored/generated code, and scratch space.
_EXCLUDED_DIRS = {"scripts", "generated", "modules", "z3"}


def _discover_top_level_packages() -> list[str]:
    """Every root-level directory that's an actual Python package."""
    return sorted(
        p.name
        for p in PROJECT_ROOT.iterdir()
        if p.is_dir() and p.name not in _EXCLUDED_DIRS and (p / "__init__.py").is_file()
    )


def _discover_root_modules() -> list[str]:
    """Every standalone .py file living directly at the project root."""
    return sorted(p.stem for p in PROJECT_ROOT.glob("*.py") if p.name != "__init__.py")


def main() -> int:
    sys.path.insert(0, str(PROJECT_ROOT))
    failures = []

    def walk_package(package_name: str) -> None:
        package = importlib.import_module(package_name)
        if not hasattr(package, "__path__"):
            return
        for module_info in pkgutil.walk_packages(package.__path__, package.__name__ + "."):
            try:
                importlib.import_module(module_info.name)
            except Exception as exc:
                failures.append((module_info.name, exc))

    for top in _discover_top_level_packages():
        walk_package(top)

    for name in _discover_root_modules():
        try:
            importlib.import_module(name)
        except Exception as exc:
            failures.append((name, exc))

    if failures:
        print(f"FAILED {len(failures)} module(s):", file=sys.stderr)
        for name, exc in failures:
            print(f"  {name}: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1

    print("All modules imported successfully")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
