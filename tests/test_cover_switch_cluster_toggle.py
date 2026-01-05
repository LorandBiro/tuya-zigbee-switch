"""Tests for cover switch cluster in toggle mode."""
import pytest

from client import StubProc
from conftest import Device, DEBOUNCE_MS
from zcl_consts import (
    ZCL_ATTR_COVER_SWITCH_SWITCH_TYPE,
    ZCL_CLUSTER_COVER_SWITCH_CONFIG,
    ZCL_ONOFF_CONFIGURATION_SWITCH_TYPE_TOGGLE,
)


OPEN = "1"
CLOSE = "2"
STOP = "3"


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
def toggle_cover_switch(cover_switch_device: Device) -> Device:
    """Set cover switch to toggle mode."""
    cover_switch_device.write_zigbee_attr(
        1,  # cover switch endpoint
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_SWITCH_TYPE,
        ZCL_ONOFF_CONFIGURATION_SWITCH_TYPE_TOGGLE,
    )
    return cover_switch_device


def test_toggle_open_button_press(toggle_cover_switch: Device):
    """Test that pressing open button sets open state (value 1)."""
    assert toggle_cover_switch.zcl_switch_get_multistate_value(1) == STOP
    toggle_cover_switch.press_button("A0")
    assert toggle_cover_switch.zcl_switch_get_multistate_value(1) == OPEN
    toggle_cover_switch.release_button("A0")
    assert toggle_cover_switch.zcl_switch_get_multistate_value(1) == STOP


def test_toggle_close_button_press(toggle_cover_switch: Device):
    """Test that pressing close button sets close state (value 2)."""
    assert toggle_cover_switch.zcl_switch_get_multistate_value(1) == STOP
    toggle_cover_switch.press_button("A1")
    assert toggle_cover_switch.zcl_switch_get_multistate_value(1) == CLOSE
    toggle_cover_switch.release_button("A1")
    assert toggle_cover_switch.zcl_switch_get_multistate_value(1) == STOP


def test_toggle_both_buttons_pressed(toggle_cover_switch: Device):
    """Test that pressing both buttons sets stop state (value 3)."""
    assert toggle_cover_switch.zcl_switch_get_multistate_value(1) == STOP
    toggle_cover_switch.press_button("A0")
    assert toggle_cover_switch.zcl_switch_get_multistate_value(1) == OPEN
    toggle_cover_switch.press_button("A1")
    assert toggle_cover_switch.zcl_switch_get_multistate_value(1) == STOP
    toggle_cover_switch.release_button("A1")
    assert toggle_cover_switch.zcl_switch_get_multistate_value(1) == OPEN
    toggle_cover_switch.release_button("A0")
    assert toggle_cover_switch.zcl_switch_get_multistate_value(1) == STOP
