"""Tests for cover switch cluster in momentary mode."""
import pytest

from client import StubProc
from conftest import Device, DEBOUNCE_MS
from zcl_consts import (
    ZCL_ATTR_COVER_SWITCH_SWITCH_TYPE,
    ZCL_ATTR_COVER_SWITCH_LONG_PRESS_DURATION,
    ZCL_CLUSTER_COVER_SWITCH_CONFIG,
    ZCL_ONOFF_CONFIGURATION_SWITCH_TYPE_MOMENTARY,
)


RELEASED = "0"
OPEN = "1"
CLOSE = "2"
STOP = "3"
LONG_OPEN = "4"
LONG_CLOSE = "5"


@pytest.fixture
def cover_switch_device() -> Device:
    """Initialize a device with cover switch."""
    p = StubProc(device_config="Mfr;Model;XA0A1u;WB0B1;").start()
    try:
        d = Device(p)
        yield d
    finally:
        p.stop()


@pytest.fixture
def momentary_cover_switch(cover_switch_device: Device) -> Device:
    """Set cover switch to momentary mode (this is the default)."""
    cover_switch_device.write_zigbee_attr(
        1,  # cover switch endpoint
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_SWITCH_TYPE,
        ZCL_ONOFF_CONFIGURATION_SWITCH_TYPE_MOMENTARY,
    )
    return cover_switch_device


def test_momentary_open_button_short_press(momentary_cover_switch: Device):
    """Test that short pressing open button sets open state (value 1)."""
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == RELEASED
    momentary_cover_switch.press_button("A0")
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == OPEN
    momentary_cover_switch.release_button("A0")
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == RELEASED



def test_momentary_close_button_short_press(momentary_cover_switch: Device):
    """Test that short pressing close button sets close state (value 2)."""
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == RELEASED
    momentary_cover_switch.press_button("A1")
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == CLOSE
    momentary_cover_switch.release_button("A1")
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == RELEASED


def test_momentary_both_buttons_pressed(momentary_cover_switch: Device):
    """Test that pressing both buttons sets stop state (value 3)."""
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == RELEASED
    momentary_cover_switch.press_button("A0")
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == OPEN
    momentary_cover_switch.press_button("A1")
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == STOP
    momentary_cover_switch.release_button("A1")
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == STOP
    momentary_cover_switch.release_button("A0")
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == RELEASED


def test_momentary_open_button_long_press(momentary_cover_switch: Device):
    """Test that long pressing open button sets long open state (value 4)."""
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == RELEASED
    momentary_cover_switch.press_button("A0")
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == OPEN
    momentary_cover_switch.step_time(1000)
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == LONG_OPEN
    momentary_cover_switch.release_button("A0")
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == RELEASED


def test_momentary_close_button_long_press(momentary_cover_switch: Device):
    """Test that long pressing close button sets long close state (value 5)."""
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == RELEASED
    momentary_cover_switch.press_button("A1")
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == CLOSE
    momentary_cover_switch.step_time(1000)
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == LONG_CLOSE
    momentary_cover_switch.release_button("A1")
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == RELEASED
