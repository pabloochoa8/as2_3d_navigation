#!/usr/bin/env python3
"""
Standalone test for ArtificialMap query logic (no ROS, no C++ bindings).

Replicates the exact same logic as ArtificialMap::getVoxelState() / isOccupied()
from artificial_map.hpp and validates the expected results for the corridor map:
  Volume  : x=[0,10], y=[0,5], z=[0,3] m
  Obstacle: x=[4.5,5.5], y=[0,2], z=[0,3]  (wall; passage via y=[2,5])
"""

from enum import IntEnum


class VoxelState(IntEnum):
    FREE = 0
    OCCUPIED = 1
    UNKNOWN = 2
    OUT_OF_BOUNDS = 3


class ArtificialMap:
    X_MIN, X_MAX = 0.0, 10.0
    Y_MIN, Y_MAX = 0.0, 5.0
    Z_MIN, Z_MAX = 0.0, 3.0
    RESOLUTION = 0.05
    FRAME_ID = "earth"

    def is_in_bounds(self, x: float, y: float, z: float) -> bool:
        return (self.X_MIN <= x <= self.X_MAX and
                self.Y_MIN <= y <= self.Y_MAX and
                self.Z_MIN <= z <= self.Z_MAX)

    def get_voxel_state(self, x: float, y: float, z: float) -> VoxelState:
        if not self.is_in_bounds(x, y, z):
            return VoxelState.OUT_OF_BOUNDS
        if 4.5 <= x <= 5.5 and 0.0 <= y <= 2.0 and 0.0 <= z <= 3.0:
            return VoxelState.OCCUPIED
        return VoxelState.FREE

    def is_occupied(self, x: float, y: float, z: float) -> bool:
        return self.get_voxel_state(x, y, z) != VoxelState.FREE


def run_tests():
    m = ArtificialMap()
    tests = [
        # (description, actual, expected, comparison)
        (
            "test 1: (2.0, 1.0, 1.0) → FREE",
            m.get_voxel_state(2.0, 1.0, 1.0),
            VoxelState.FREE,
        ),
        (
            "test 2: (5.0, 1.0, 1.5) → OCCUPIED (inside wall)",
            m.get_voxel_state(5.0, 1.0, 1.5),
            VoxelState.OCCUPIED,
        ),
        (
            "test 3: (5.0, 3.0, 1.5) → FREE (passage side)",
            m.get_voxel_state(5.0, 3.0, 1.5),
            VoxelState.FREE,
        ),
        (
            "test 4: (15.0, 0.0, 0.0) → OUT_OF_BOUNDS",
            m.get_voxel_state(15.0, 0.0, 0.0),
            VoxelState.OUT_OF_BOUNDS,
        ),
        (
            "test 5: isOccupied(5.0, 1.0, 1.5) → True",
            m.is_occupied(5.0, 1.0, 1.5),
            True,
        ),
        (
            "test 6: isOccupied(2.0, 1.0, 1.0) → False",
            m.is_occupied(2.0, 1.0, 1.0),
            False,
        ),
    ]

    all_passed = True
    for desc, actual, expected in tests:
        passed = actual == expected
        status = "PASS" if passed else "FAIL"
        if not passed:
            all_passed = False
            print(f"[{status}] {desc}  (got {actual!r}, expected {expected!r})")
        else:
            print(f"[{status}] {desc}")

    print()
    if all_passed:
        print("All tests PASSED.")
    else:
        print("Some tests FAILED.")
        raise SystemExit(1)


if __name__ == "__main__":
    run_tests()
