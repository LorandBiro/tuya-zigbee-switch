"""Tests for cover switch cluster in momentary mode."""
import pytest

from client import StubProc
from conftest import Device, DEBOUNCE_MS
from zcl_consts import (
    ZCL_ATTR_COVER_SWITCH_SWITCH_TYPE,
    ZCL_ATTR_COVER_SWITCH_LONG_PRESS_DURATION,
    ZCL_ATTR_COVER_SWITCH_LOCAL_MODE,
    ZCL_ATTR_COVER_SWITCH_COVER_INDEX,
    ZCL_CLUSTER_COVER_SWITCH_CONFIG,
    ZCL_ONOFF_CONFIGURATION_SWITCH_TYPE_TOGGLE,
    ZCL_ONOFF_CONFIGURATION_SWITCH_TYPE_MOMENTARY,
    ZCL_COVER_SWITCH_MODE_IMMEDIATE,
    ZCL_COVER_SWITCH_MODE_LONG_PRESS,
    ZCL_COVER_SWITCH_MODE_SHORT_PRESS,
    ZCL_COVER_SWITCH_MODE_HYBRID,
    ZCL_CLUSTER_WINDOW_COVERING,
    ZCL_ATTR_WINDOW_COVERING_MOVING,
    ZCL_WINDOW_COVERING_MOVING_STOPPED,
    ZCL_WINDOW_COVERING_MOVING_OPENING,
    ZCL_WINDOW_COVERING_MOVING_CLOSING,
)


RELEASED = "0"
OPEN = "1"
CLOSE = "2"
STOP = "3"
LONG_OPEN = "4"
LONG_CLOSE = "5"


# ============================================================================
# Fixtures
# ============================================================================


@pytest.fixture
def cover_switch_device() -> Device:
    p = StubProc(device_config="Mfr;Model;XA0A1u;WB0B1;").start()
    try:
        d = Device(p)
        yield d
    finally:
        p.stop()


@pytest.fixture
def toggle_cover_switch(cover_switch_device: Device) -> Device:
    cover_switch_device.write_zigbee_attr(
        1,  # cover switch endpoint
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_SWITCH_TYPE,
        ZCL_ONOFF_CONFIGURATION_SWITCH_TYPE_TOGGLE,
    )
    return cover_switch_device


@pytest.fixture
def dual_cover_switch_device() -> Device:
    p = StubProc(device_config="Mfr;Model;XA0A1u;WB0B1;XC0C1u;WD0D1;").start()
    try:
        d = Device(p)
        yield d
    finally:
        p.stop()


@pytest.fixture
def dual_toggle_cover_switch(dual_cover_switch_device: Device) -> Device:
    dual_cover_switch_device.write_zigbee_attr(
        1,  # cover switch 1 endpoint
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_SWITCH_TYPE,
        ZCL_ONOFF_CONFIGURATION_SWITCH_TYPE_TOGGLE,
    )
    dual_cover_switch_device.write_zigbee_attr(
        2,  # cover switch 2 endpoint
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_SWITCH_TYPE,
        ZCL_ONOFF_CONFIGURATION_SWITCH_TYPE_TOGGLE,
    )
    return dual_cover_switch_device


# ============================================================================
# Tests for toggle mode multistate value
# ============================================================================


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


# ============================================================================
# Tests for momentary mode multistate value
# ============================================================================


def test_momentary_open_button_short_press(cover_switch_device: Device):
    assert cover_switch_device.zcl_switch_get_multistate_value(1) == RELEASED
    cover_switch_device.press_button("A0")
    assert cover_switch_device.zcl_switch_get_multistate_value(1) == OPEN
    cover_switch_device.release_button("A0")
    assert cover_switch_device.zcl_switch_get_multistate_value(1) == RELEASED



def test_momentary_close_button_short_press(cover_switch_device: Device):
    assert cover_switch_device.zcl_switch_get_multistate_value(1) == RELEASED
    cover_switch_device.press_button("A1")
    assert cover_switch_device.zcl_switch_get_multistate_value(1) == CLOSE
    cover_switch_device.release_button("A1")
    assert cover_switch_device.zcl_switch_get_multistate_value(1) == RELEASED


def test_momentary_both_buttons_pressed(cover_switch_device: Device):
    assert cover_switch_device.zcl_switch_get_multistate_value(1) == RELEASED
    cover_switch_device.press_button("A0")
    assert cover_switch_device.zcl_switch_get_multistate_value(1) == OPEN
    cover_switch_device.press_button("A1")
    assert cover_switch_device.zcl_switch_get_multistate_value(1) == STOP
    cover_switch_device.release_button("A1")
    assert cover_switch_device.zcl_switch_get_multistate_value(1) == STOP
    cover_switch_device.release_button("A0")
    assert cover_switch_device.zcl_switch_get_multistate_value(1) == RELEASED


def test_momentary_open_button_long_press(cover_switch_device: Device):
    assert cover_switch_device.zcl_switch_get_multistate_value(1) == RELEASED
    cover_switch_device.press_button("A0")
    assert cover_switch_device.zcl_switch_get_multistate_value(1) == OPEN
    cover_switch_device.step_time(1000)
    assert cover_switch_device.zcl_switch_get_multistate_value(1) == LONG_OPEN
    cover_switch_device.release_button("A0")
    assert cover_switch_device.zcl_switch_get_multistate_value(1) == RELEASED


def test_momentary_close_button_long_press(cover_switch_device: Device):
    assert cover_switch_device.zcl_switch_get_multistate_value(1) == RELEASED
    cover_switch_device.press_button("A1")
    assert cover_switch_device.zcl_switch_get_multistate_value(1) == CLOSE
    cover_switch_device.step_time(1000)
    assert cover_switch_device.zcl_switch_get_multistate_value(1) == LONG_CLOSE
    cover_switch_device.release_button("A1")
    assert cover_switch_device.zcl_switch_get_multistate_value(1) == RELEASED


# ============================================================================
# Tests for stop-on-repeat behavior across different local modes
# ============================================================================


def test_immediate_open_button_stops_on_repeat(cover_switch_device: Device):
    cover_switch_device.write_zigbee_attr(
        1,
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_LOCAL_MODE,
        ZCL_COVER_SWITCH_MODE_IMMEDIATE,
    )
    
    # Press should start opening
    cover_switch_device.press_button("A0")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_OPENING
    cover_switch_device.step_time(200) # Minimum relay on time
    cover_switch_device.release_button("A0")

    # Repeated press should stop the cover
    cover_switch_device.press_button("A0")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_STOPPED


def test_immediate_close_button_stops_on_repeat(cover_switch_device: Device):
    cover_switch_device.write_zigbee_attr(
        1,
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_LOCAL_MODE,
        ZCL_COVER_SWITCH_MODE_IMMEDIATE,
    )
    
    # Press should start closing
    cover_switch_device.press_button("A1")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_CLOSING
    cover_switch_device.step_time(200) # Minimum relay on time
    cover_switch_device.release_button("A1")

    # Repeated press should stop the cover
    cover_switch_device.press_button("A1")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_STOPPED


def test_long_press_open_stops_on_repeat(cover_switch_device: Device):
    cover_switch_device.write_zigbee_attr(
        1,
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_LOCAL_MODE,
        ZCL_COVER_SWITCH_MODE_LONG_PRESS,
    )

    # Long press should start opening
    cover_switch_device.press_button("A0")
    cover_switch_device.step_time(800)
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_OPENING
    cover_switch_device.release_button("A0")

    # Repeated long press should stop the cover
    cover_switch_device.press_button("A0")
    cover_switch_device.step_time(800)
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_STOPPED


def test_long_press_close_stops_on_repeat(cover_switch_device: Device):
    cover_switch_device.write_zigbee_attr(
        1,
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_LOCAL_MODE,
        ZCL_COVER_SWITCH_MODE_LONG_PRESS,
    )

    # Long press should start closing
    cover_switch_device.press_button("A1")
    cover_switch_device.step_time(800)
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_CLOSING
    cover_switch_device.release_button("A1")

    # Repeated long press should stop the cover
    cover_switch_device.press_button("A1")
    cover_switch_device.step_time(800)
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_STOPPED


def test_short_press_open_stops_on_repeat(cover_switch_device: Device):
    cover_switch_device.write_zigbee_attr(
        1,
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_LOCAL_MODE,
        ZCL_COVER_SWITCH_MODE_SHORT_PRESS,
    )
    
    # Press and release should start opening
    cover_switch_device.press_button("A0")
    cover_switch_device.release_button("A0")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_OPENING
    cover_switch_device.step_time(200) # Minimum relay on time

    # Repeated press and release should stop the cover
    cover_switch_device.press_button("A0")
    cover_switch_device.release_button("A0")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_STOPPED


def test_short_press_close_stops_on_repeat(cover_switch_device: Device):
    cover_switch_device.write_zigbee_attr(
        1,
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_LOCAL_MODE,
        ZCL_COVER_SWITCH_MODE_SHORT_PRESS,
    )
    
    # Press and release should start closing
    cover_switch_device.press_button("A1")
    cover_switch_device.release_button("A1")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_CLOSING
    cover_switch_device.step_time(200) # Minimum relay on time

    # Repeated press and release should stop the cover
    cover_switch_device.press_button("A1")
    cover_switch_device.release_button("A1")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_STOPPED


def test_hybrid_short_open_stops_on_repeat(cover_switch_device: Device):
    cover_switch_device.write_zigbee_attr(
        1,
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_LOCAL_MODE,
        ZCL_COVER_SWITCH_MODE_HYBRID,
    )
    
    # Press and release should start opening
    cover_switch_device.press_button("A0")
    cover_switch_device.release_button("A0")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_OPENING
    cover_switch_device.step_time(200) # Minimum relay on time

    # Repeated press and release should stop the cover
    cover_switch_device.press_button("A0")
    cover_switch_device.release_button("A0")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_STOPPED


def test_hybrid_short_close_stops_on_repeat(cover_switch_device: Device):
    cover_switch_device.write_zigbee_attr(
        1,
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_LOCAL_MODE,
        ZCL_COVER_SWITCH_MODE_HYBRID,
    )
    
    # Press and release should start closing
    cover_switch_device.press_button("A1")
    cover_switch_device.release_button("A1")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_CLOSING
    cover_switch_device.step_time(200) # Minimum relay on time

    # Repeated press and release should stop the cover
    cover_switch_device.press_button("A1")
    cover_switch_device.release_button("A1")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_STOPPED


def test_hybrid_long_keeps_moving(cover_switch_device: Device):
    cover_switch_device.write_zigbee_attr(
        1,
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_LOCAL_MODE,
        ZCL_COVER_SWITCH_MODE_HYBRID,
    )
    
    # Press and release should start opening
    cover_switch_device.press_button("A0")
    cover_switch_device.release_button("A0")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_OPENING

    # Long press in the same direction should not stop the cover
    cover_switch_device.press_button("A0")
    cover_switch_device.step_time(800)
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_OPENING

    # Release should stop the cover
    cover_switch_device.release_button("A0")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_STOPPED


def test_hybrid_long_opposite_works(cover_switch_device: Device):
    cover_switch_device.write_zigbee_attr(
        1,
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_LOCAL_MODE,
        ZCL_COVER_SWITCH_MODE_HYBRID,
    )
    
    # Press and release should start opening
    cover_switch_device.press_button("A0")
    cover_switch_device.release_button("A0")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_OPENING

    # Long press in the opposite direction should stop the cover and start closing
    cover_switch_device.press_button("A1")
    cover_switch_device.step_time(800)
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_STOPPED

    # Wait for safety delay before changing direction
    cover_switch_device.step_time(500)
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_CLOSING
    cover_switch_device.step_time(200) # Minimum relay on time

    # Release should stop the cover
    cover_switch_device.release_button("A1")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_STOPPED


# ============================================================================
# Tests for cover index
# ============================================================================


def test_cover_switch_detached_mode(toggle_cover_switch: Device):
    # Pressing open button should trigger local cover
    toggle_cover_switch.press_button("A0")
    assert toggle_cover_switch.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_OPENING
    toggle_cover_switch.step_time(200) # Minimum relay on time

    # Release button should stop the cover
    toggle_cover_switch.release_button("A0")
    assert toggle_cover_switch.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_STOPPED
    
    # Set cover index to 0 (detached mode)
    toggle_cover_switch.write_zigbee_attr(
        1,  # cover switch endpoint
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_COVER_INDEX,
        0,
    )
    
    # Pressing open button should no longer trigger local cover
    toggle_cover_switch.press_button("A0")
    assert toggle_cover_switch.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_STOPPED


def test_cover_switch_index_switching(dual_toggle_cover_switch: Device):
    # Pressing open button should trigger the first cover endpoint
    dual_toggle_cover_switch.press_button("A0")
    assert dual_toggle_cover_switch.zcl_cover_get_moving(3) == ZCL_WINDOW_COVERING_MOVING_OPENING
    dual_toggle_cover_switch.step_time(200) # Minimum relay on time

    # Release button should stop the cover
    dual_toggle_cover_switch.release_button("A0")
    assert dual_toggle_cover_switch.zcl_cover_get_moving(3) == ZCL_WINDOW_COVERING_MOVING_STOPPED
    
    # Set cover index to 2
    dual_toggle_cover_switch.write_zigbee_attr(
        1,  # cover switch 1 endpoint
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_COVER_INDEX,
        2,
    )

    # Pressing open button should trigger the second cover endpoint
    dual_toggle_cover_switch.press_button("A0")
    assert dual_toggle_cover_switch.zcl_cover_get_moving(4) == ZCL_WINDOW_COVERING_MOVING_OPENING
    dual_toggle_cover_switch.step_time(200) # Minimum relay on time

    # Release button should stop the cover
    dual_toggle_cover_switch.release_button("A0")
    assert dual_toggle_cover_switch.zcl_cover_get_moving(4) == ZCL_WINDOW_COVERING_MOVING_STOPPED
