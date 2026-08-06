"""Compare the numeric output produced by legacy_probe and current_probe."""

from __future__ import annotations

import math
import sys
from pathlib import Path


def load(path: Path) -> dict[str, list[float] | str]:
    result: dict[str, list[float] | str] = {}
    for raw_line in path.read_text(encoding="utf-8-sig").splitlines():
        parts = raw_line.split()
        if not parts:
            continue
        key = parts[0]
        values = parts[1:]
        if key in {"atmosphere", "atmosphere.scalar"}:
            key = f"{key}.{values.pop(0)}"
        try:
            result[key] = [float(value) for value in values]
        except ValueError:
            result[key] = " ".join(values)
    return result


def close(a: float, b: float, abs_tol: float, rel_tol: float = 1e-12) -> bool:
    return math.isclose(a, b, abs_tol=abs_tol, rel_tol=rel_tol)


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: compare_outputs.py LEGACY_OUTPUT CURRENT_OUTPUT")
        return 2

    legacy = load(Path(sys.argv[1]))
    current = load(Path(sys.argv[2]))
    failures: list[str] = []
    compared = 0

    intentionally_changed = {
        # 旧版单项大气函数使用不同模型。
        key for key in legacy if key.startswith("atmosphere.scalar.")
    }
    intentionally_changed.update({"geodesy.inverse", "geodesy.direct"})

    for key, old_value in legacy.items():
        if key in intentionally_changed:
            continue
        if key not in current:
            failures.append(f"missing current output: {key}")
            continue
        new_value = current[key]
        if isinstance(old_value, str) or isinstance(new_value, str):
            compared += 1
            if old_value != new_value:
                failures.append(f"{key}: {old_value!r} != {new_value!r}")
            continue

        if len(old_value) != len(new_value):
            failures.append(f"{key}: output lengths differ")
            continue

        abs_tol = 1e-9
        if key.startswith("coordinate.ecef.") or key == "coordinate.velocity_ecef_north":
            abs_tol = 1e-3
        elif key.startswith("coordinate.gps_back."):
            abs_tol = 1e-3
        elif key.startswith("atmosphere."):
            abs_tol = 2e-2
        elif key == "geodesy.site_distance":
            abs_tol = 1e-2
        elif key == "geodesy.site_azimuth_signed":
            abs_tol = 1e-6

        for index, (old_number, new_number) in enumerate(zip(old_value, new_value)):
            compared += 1
            if not close(old_number, new_number, abs_tol):
                failures.append(
                    f"{key}[{index}]: old={old_number:.17g}, "
                    f"new={new_number:.17g}, tolerance={abs_tol:g}"
                )

    # A21 定义变化单独排除。
    for key, indices, tolerance in (
        ("geodesy.inverse", (0, 1), 1e-3),
        ("geodesy.direct", (0, 1), 1e-8),
    ):
        old_values = legacy[key]
        new_values = current[key]
        assert isinstance(old_values, list) and isinstance(new_values, list)
        for index in indices:
            compared += 1
            if not close(old_values[index], new_values[index], tolerance):
                failures.append(
                    f"{key}[{index}]: old={old_values[index]:.17g}, "
                    f"new={new_values[index]:.17g}, tolerance={tolerance:g}"
                )

    if failures:
        print(f"FAILED: {len(failures)} differences across {compared} comparisons")
        for failure in failures:
            print(f"  {failure}")
        return 1

    print(f"PASSED: {compared} legacy/current numeric comparisons")
    print("Intentional exclusions: legacy scalar-atmosphere inconsistencies and A21 convention")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
