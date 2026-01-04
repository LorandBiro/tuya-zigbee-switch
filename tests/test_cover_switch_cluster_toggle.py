"""Tests for cover switch cluster in toggle mode."""
import pytest

from client import StubProc
from conftest import Device, DEBOUNCE_MS
from zcl_consts import (
    ZCL_ATTR_COVER_SWITCH_SWITCH_TYPE,
    ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE,
    ZCL_CLUSTER_COVER_SWITCH_CONFIG,
    ZCL_CLUSTER_MULTISTATE_INPUT_BASIC,
    ZCL_COVER_SWITCH_ACTION_POSITION_STOP,
    ZCL_COVER_SWITCH_ACTION_POSITION_OPEN,
    ZCL_COVER_SWITCH_ACTION_POSITION_CLOSE,
    ZCL_ONOFF_CONFIGURATION_SWITCH_TYPE_TOGGLE,
)


@pytest.fixture
def cover_switch_device_config() -> str:
    """Device with one cover switch (X) and one cover output (W)."""
    return "Mfr;Model;XA0A1u;WB0B1;"


@pytest.fixture
def cover_switch_device(cover_switch_device_config: str) -> Device:
    """Initialize a device with cover switch."""
    p = StubProc(device_config=cover_switch_device_config).start()
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
    cover_switch_device.step_time(10)  # Allow time for attribute change to propagate
    return cover_switch_device


def test_toggle_initial_position_stop(toggle_cover_switch: Device):
    """Test that toggle switch starts in POSITION_STOP state."""
    action = toggle_cover_switch.read_zigbee_attr(
        1, ZCL_CLUSTER_MULTISTATE_INPUT_BASIC, ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE
    )
    assert int(action) == ZCL_COVER_SWITCH_ACTION_POSITION_STOP


def test_toggle_open_button_press(toggle_cover_switch: Device):
    """Test that pressing open button sets POSITION_OPEN."""
    # Press open button (A0)
    toggle_cover_switch.press_button("A0")
    
    action = toggle_cover_switch.read_zigbee_attr(
        1, ZCL_CLUSTER_MULTISTATE_INPUT_BASIC, ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE
    )
    assert int(action) == ZCL_COVER_SWITCH_ACTION_POSITION_OPEN


def test_toggle_open_button_release(toggle_cover_switch: Device):
    """Test that releasing open button returns to POSITION_STOP."""
    # Press and release open button
    toggle_cover_switch.press_button("A0")
    toggle_cover_switch.release_button("A0")
    
    action = toggle_cover_switch.read_zigbee_attr(
        1, ZCL_CLUSTER_MULTISTATE_INPUT_BASIC, ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE
    )
    assert int(action) == ZCL_COVER_SWITCH_ACTION_POSITION_STOP


def test_toggle_close_button_press(toggle_cover_switch: Device):
    """Test that pressing close button sets POSITION_CLOSE."""
    # Press close button (A1)
    toggle_cover_switch.press_button("A1")
    
    action = toggle_cover_switch.read_zigbee_attr(
        1, ZCL_CLUSTER_MULTISTATE_INPUT_BASIC, ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE
    )
    assert int(action) == ZCL_COVER_SWITCH_ACTION_POSITION_CLOSE


def test_toggle_close_button_release(toggle_cover_switch: Device):
    """Test that releasing close button returns to POSITION_STOP."""
    # Press and release close button
    toggle_cover_switch.press_button("A1")
    toggle_cover_switch.release_button("A1")
    
    action = toggle_cover_switch.read_zigbee_attr(
        1, ZCL_CLUSTER_MULTISTATE_INPUT_BASIC, ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE
    )
    assert int(action) == ZCL_COVER_SWITCH_ACTION_POSITION_STOP


def test_toggle_both_buttons_pressed(toggle_cover_switch: Device):
    """Test that pressing both buttons sets POSITION_STOP."""
    # Press open button first
    toggle_cover_switch.press_button("A0")
    
    # Now press close button while open is still pressed
    toggle_cover_switch.set_gpio("A1", 0)  # Press close (low is pressed)
    toggle_cover_switch.step_time(DEBOUNCE_MS + 10)
    
    action = toggle_cover_switch.read_zigbee_attr(
        1, ZCL_CLUSTER_MULTISTATE_INPUT_BASIC, ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE
    )
    assert int(action) == ZCL_COVER_SWITCH_ACTION_POSITION_STOP


def test_toggle_no_long_press_action(toggle_cover_switch: Device):
    """Test that toggle mode doesn't trigger long press."""
    # Hold button for long duration
    toggle_cover_switch.press_button("A0")
    toggle_cover_switch.step_time(1500)  # Hold for 1.5 seconds
    
    # Should still be at POSITION_OPEN, not long press
    action = toggle_cover_switch.read_zigbee_attr(
        1, ZCL_CLUSTER_MULTISTATE_INPUT_BASIC, ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE
    )
    assert int(action) == ZCL_COVER_SWITCH_ACTION_POSITION_OPEN
    
    # Release should go to STOP
    toggle_cover_switch.release_button("A0")
    action = toggle_cover_switch.read_zigbee_attr(
        1, ZCL_CLUSTER_MULTISTATE_INPUT_BASIC, ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE
    )
    assert int(action) == ZCL_COVER_SWITCH_ACTION_POSITION_STOP
