"""Unit tests for the hardware-independent unit conversions and the TCP geometry model."""

import math

import pytest

from robotiq2f import Robotiq2F


@pytest.fixture
def gripper() -> Robotiq2F:
    """A driver instance that is never connected, so only pure computations may be exercised."""
    return object.__new__(Robotiq2F)


def test_count_to_opening_spans_the_full_stroke(gripper: Robotiq2F) -> None:
    assert gripper.count_to_opening(230) == pytest.approx(0.0)
    assert gripper.count_to_opening(0) == pytest.approx(85.0)


def test_count_to_opening_clamps_out_of_range_counts(gripper: Robotiq2F) -> None:
    assert gripper.count_to_opening(-10) == pytest.approx(85.0)
    assert gripper.count_to_opening(1000) == pytest.approx(0.0)


def test_opening_and_count_round_trip(gripper: Robotiq2F) -> None:
    for opening in (0.0, 10.0, 42.5, 85.0):
        count = gripper.opening_to_count(opening)
        assert gripper.count_to_opening(count) == pytest.approx(opening, abs=0.39)


@pytest.mark.parametrize("speed", [20.0, 85.0, 150.0])
def test_speed_and_count_round_trip(gripper: Robotiq2F, speed: float) -> None:
    count = gripper.speed_to_count(speed)
    assert gripper.count_to_speed(count) == pytest.approx(speed, abs=0.6)


@pytest.mark.parametrize("force", [20.0, 127.5, 235.0])
def test_force_and_count_round_trip(gripper: Robotiq2F, force: float) -> None:
    count = gripper.force_to_count(force)
    assert gripper.count_to_force(count) == pytest.approx(force, abs=1.0)


def test_speed_and_force_counts_stay_in_register_range(gripper: Robotiq2F) -> None:
    assert gripper.speed_to_count(-100) == 0
    assert gripper.speed_to_count(1000) == 255
    assert gripper.force_to_count(-100) == 0
    assert gripper.force_to_count(1000) == 255


def test_current_conversion(gripper: Robotiq2F) -> None:
    assert gripper.count_to_current(0) == pytest.approx(0.0)
    assert gripper.count_to_current(100) == pytest.approx(10.0)


def test_tcp_z_matches_the_closed_form_model(gripper: Robotiq2F) -> None:
    # d = 85/2 + 7.8 = 50.3, which is above the 12.7mm offset.
    expected = 87.308 + 57.15 * math.cos(math.asin((50.3 - 12.7) / 57.15))
    assert gripper.tcp_Z_from_opening(85.0) == pytest.approx(expected)


def test_tcp_z_is_maximal_where_the_fingers_are_vertical(gripper: Robotiq2F) -> None:
    # The fingertip reaches its farthest point when d == 12.7, i.e. opening == 2 * (12.7 - 7.8).
    peak = gripper.tcp_Z_from_opening(2 * (12.7 - 7.8))
    assert peak == pytest.approx(87.308 + 57.15)
    assert gripper.tcp_Z_from_opening(0.0) < peak
    assert gripper.tcp_Z_from_opening(85.0) < peak
