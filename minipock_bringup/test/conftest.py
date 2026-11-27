"""Make the simulation tests (marker `integration`) opt-in.

They start Gazebo and Nav2 and take minutes: a plain `colcon test` or `pytest` skips them. They
run when the marker expression names them (`-m integration`) or when `MINIPOCK_INTEGRATION=1`
(local option: CI selects them with `-m integration`).
"""

import os

import pytest

ENABLE_VARIABLE = "MINIPOCK_INTEGRATION"


def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    """Skip the `integration` items unless they were asked for."""
    if os.environ.get(ENABLE_VARIABLE) == "1":
        return
    if "integration" in (config.getoption("markexpr") or ""):
        return
    skip = pytest.mark.skip(
        reason=f"simulation test: select it with -m integration or set {ENABLE_VARIABLE}=1"
    )
    for item in items:
        if item.get_closest_marker("integration") is not None:
            item.add_marker(skip)
