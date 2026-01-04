"""Tests for cover switch cluster in momentary mode."""
import pytest

from client import StubProc
from conftest import Device, DEBOUNCE_MS
from zcl_consts import (
    ZCL_ATTR_COVER_SWITCH_SWITCH_TYPE,
    ZCL_ATTR_COVER_SWITCH_LONG_PRESS_DURATION,
    ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE,
    ZCL_CLUSTER_COVER_SWITCH_CONFIG,
    ZCL_CLUSTER_MULTISTATE_INPUT_BASIC,
    ZCL_COVER_SWITCH_ACTION_RELEASED,
    ZCL_COVER_SWITCH_ACTION_OPEN_PRESS,
    ZCL_COVER_SWITCH_ACTION_CLOSE_PRESS,
    ZCL_COVER_SWITCH_ACTION_STOP_PRESS,
    ZCL_COVER_SWITCH_ACTION_OPEN_LONG_PRESS,
    ZCL_COVER_SWITCH_ACTION_CLOSE_LONG_PRESS,
    ZCL_ONOFF_CONFIGURATION_SWITCH_TYPE_MOMENTARY,
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
def momentary_cover_switch(cover_switch_device: Device) -> Device:
    """Set cover switch to momentary mode (this is the default)."""
    # Momentary is the default, but set it explicitly for clarity
    cover_switch_device.write_zigbee_attr(
        1,  # cover switch endpoint
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_SWITCH_TYPE,
        ZCL_ONOFF_CONFIGURATION_SWITCH_TYPE_MOMENTARY,
    )
    return cover_switch_device


def test_momentary_initial_released(momentary_cover_switch: Device):
    """Test that momentary switch starts in RELEASED state."""
    action = momentary_cover_switch.read_zigbee_attr(
        1, ZCL_CLUSTER_MULTISTATE_INPUT_BASIC, ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE
    )
    assert int(action) == ZCL_COVER_SWITCH_ACTION_RELEASED


def test_momentary_open_button_short_press(momentary_cover_switch: Device):
    """Test that short pressing open button sets OPEN_PRESS."""
    # Press open button (A0)
    momentary_cover_switch.press_button("A0")
    
    action = momentary_cover_switch.read_zigbee_attr(
        1, ZCL_CLUSTER_MULTISTATE_INPUT_BASIC, ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE
    )
    assert int(action) == ZCL_COVER_SWITCH_ACTION_OPEN_PRESS


def test_momentary_open_button_release(momentary_cover_switch: Device):
    """Test that releasing open button returns to RELEASED."""
    # Press and release open button
    momentary_cover_switch.press_button("A0")
    momentary_cover_switch.release_button("A0")
    
    action = momentary_cover_switch.read_zigbee_attr(
        1, ZCL_CLUSTER_MULTISTATE_INPUT_BASIC, ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE
    )
    assert int(action) == ZCL_COVER_SWITCH_ACTION_RELEASED


def test_momentary_close_button_short_press(momentary_cover_switch: Device):
    """Test that short pressing close button sets CLOSE_PRESS."""
    # Press close button (A1)
    momentary_cover_switch.press_button("A1")
    
    action = momentary_cover_switch.read_zigbee_attr(
        1, ZCL_CLUSTER_MULTISTATE_INPUT_BASIC, ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE
    )
    assert int(action) == ZCL_COVER_SWITCH_ACTION_CLOSE_PRESS


def test_momentary_close_button_release(momentary_cover_switch: Device):
    """Test that releasing close button returns to RELEASED."""
    # Press and release close button
    momentary_cover_switch.press_button("A1")
    momentary_cover_switch.release_button("A1")
    
    action = momentary_cover_switch.read_zigbee_attr(
        1, ZCL_CLUSTER_MULTISTATE_INPUT_BASIC, ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE
    )
    assert int(action) == ZCL_COVER_SWITCH_ACTION_RELEASED


def test_momentary_both_buttons_pressed(momentary_cover_switch: Device):
    """Test that pressing both buttons sets STOP_PRESS."""
    # Press open button first
    momentary_cover_switch.press_button("A0")
    
    # Now press close button while open is still pressed
    momentary_cover_switch.set_gpio("A1", 0)  # Press close (low is pressed)
    momentary_cover_switch.step_time(DEBOUNCE_MS + 10)
    
    action = momentary_cover_switch.read_zigbee_attr(
        1, ZCL_CLUSTER_MULTISTATE_INPUT_BASIC, ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE
    )
    assert int(action) == ZCL_COVER_SWITCH_ACTION_STOP_PRESS


def test_momentary_open_button_long_press(momentary_cover_switch: Device):
    """Test that long pressing open button sets OPEN_LONG_PRESS."""
    # Set long press duration to 1000ms
    momentary_cover_switch.write_zigbee_attr(
        1,
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_LONG_PRESS_DURATION,
        1000,
    )
    
    # Press and hold open button
    momentary_cover_switch.press_button("A0")
    
    # Initially should be OPEN_PRESS
    action = momentary_cover_switch.read_zigbee_attr(
        1, ZCL_CLUSTER_MULTISTATE_INPUT_BASIC, ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE
    )
    assert int(action) == ZCL_COVER_SWITCH_ACTION_OPEN_PRESS
    
    # Hold for long press duration
    momentary_cover_switch.step_time(1100)
    
    # Should now be OPEN_LONG_PRESS
    action = momentary_cover_switch.read_zigbee_attr(
        1, ZCL_CLUSTER_MULTISTATE_INPUT_BASIC, ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE
    )
    assert int(action) == ZCL_COVER_SWITCH_ACTION_OPEN_LONG_PRESS
    
    # Release should return to RELEASED
    momentary_cover_switch.release_button("A0")
    action = momentary_cover_switch.read_zigbee_attr(
        1, ZCL_CLUSTER_MULTISTATE_INPUT_BASIC, ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE
    )
    assert int(action) == ZCL_COVER_SWITCH_ACTION_RELEASED


def test_momentary_close_button_long_press(momentary_cover_switch: Device):
    """Test that long pressing close button sets CLOSE_LONG_PRESS."""
    # Set long press duration to 1000ms
    momentary_cover_switch.write_zigbee_attr(
        1,
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_LONG_PRESS_DURATION,
        1000,
    )
    
    # Press and hold close button
    momentary_cover_switch.press_button("A1")
    
    # Initially should be CLOSE_PRESS
    action = momentary_cover_switch.read_zigbee_attr(
        1, ZCL_CLUSTER_MULTISTATE_INPUT_BASIC, ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE
    )
    assert int(action) == ZCL_COVER_SWITCH_ACTION_CLOSE_PRESS
    
    # Hold for long press duration
    momentary_cover_switch.step_time(1100)
    
    # Should now be CLOSE_LONG_PRESS
    action = momentary_cover_switch.read_zigbee_attr(
        1, ZCL_CLUSTER_MULTISTATE_INPUT_BASIC, ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE
    )
    assert int(action) == ZCL_COVER_SWITCH_ACTION_CLOSE_LONG_PRESS
    
    # Release should return to RELEASED
    momentary_cover_switch.release_button("A1")
    action = momentary_cover_switch.read_zigbee_attr(
        1, ZCL_CLUSTER_MULTISTATE_INPUT_BASIC, ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE
    )
    assert int(action) == ZCL_COVER_SWITCH_ACTION_RELEASED
