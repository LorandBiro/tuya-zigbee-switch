"""Tests for cover switch cluster in momentary mode."""
import pytest

from client import StubProc
from conftest import Device, DEBOUNCE_MS
from zcl_consts import (
    ZCL_ATTR_COVER_SWITCH_SWITCH_TYPE,
    ZCL_ATTR_COVER_SWITCH_LONG_PRESS_DURATION,
    ZCL_ATTR_COVER_SWITCH_LOCAL_MODE,
    ZCL_CLUSTER_COVER_SWITCH_CONFIG,
    ZCL_ONOFF_CONFIGURATION_SWITCH_TYPE_MOMENTARY,
    ZCL_COVER_SWITCH_LOCAL_MODE_PRESS_START,
    ZCL_COVER_SWITCH_LOCAL_MODE_LONG_PRESS,
    ZCL_COVER_SWITCH_LOCAL_MODE_SHORT_PRESS,
    ZCL_COVER_SWITCH_LOCAL_MODE_SHORT_AND_LONG_PRESS,
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


# ============================================================================
# Tests for multistate value
# ============================================================================


def test_momentary_open_button_short_press(momentary_cover_switch: Device):
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == RELEASED
    momentary_cover_switch.press_button("A0")
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == OPEN
    momentary_cover_switch.release_button("A0")
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == RELEASED



def test_momentary_close_button_short_press(momentary_cover_switch: Device):
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == RELEASED
    momentary_cover_switch.press_button("A1")
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == CLOSE
    momentary_cover_switch.release_button("A1")
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == RELEASED


def test_momentary_both_buttons_pressed(momentary_cover_switch: Device):
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
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == RELEASED
    momentary_cover_switch.press_button("A0")
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == OPEN
    momentary_cover_switch.step_time(1000)
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == LONG_OPEN
    momentary_cover_switch.release_button("A0")
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == RELEASED


def test_momentary_close_button_long_press(momentary_cover_switch: Device):
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == RELEASED
    momentary_cover_switch.press_button("A1")
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == CLOSE
    momentary_cover_switch.step_time(1000)
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == LONG_CLOSE
    momentary_cover_switch.release_button("A1")
    assert momentary_cover_switch.zcl_switch_get_multistate_value(1) == RELEASED


# ============================================================================
# Tests for stop-on-repeat behavior across different local modes
# ============================================================================


def test_press_start_open_button_stops_on_repeat(cover_switch_device: Device):
    cover_switch_device.write_zigbee_attr(
        1,
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_LOCAL_MODE,
        ZCL_COVER_SWITCH_LOCAL_MODE_PRESS_START,
    )
    
    # Press should start opening
    cover_switch_device.press_button("A0")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_OPENING
    cover_switch_device.release_button("A0")

    # Repeated press should stop the cover
    cover_switch_device.press_button("A0")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_STOPPED


def test_press_start_close_button_stops_on_repeat(cover_switch_device: Device):
    cover_switch_device.write_zigbee_attr(
        1,
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_LOCAL_MODE,
        ZCL_COVER_SWITCH_LOCAL_MODE_PRESS_START,
    )
    
    # Press should start closing
    cover_switch_device.press_button("A1")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_CLOSING
    cover_switch_device.release_button("A1")

    # Repeated press should stop the cover
    cover_switch_device.press_button("A1")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_STOPPED


def test_long_press_open_stops_on_repeat(cover_switch_device: Device):
    cover_switch_device.write_zigbee_attr(
        1,
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_LOCAL_MODE,
        ZCL_COVER_SWITCH_LOCAL_MODE_LONG_PRESS,
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
        ZCL_COVER_SWITCH_LOCAL_MODE_LONG_PRESS,
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
        ZCL_COVER_SWITCH_LOCAL_MODE_SHORT_PRESS,
    )
    
    # Press and release should start opening
    cover_switch_device.press_button("A0")
    cover_switch_device.release_button("A0")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_OPENING

    # Repeated press and release should stop the cover
    cover_switch_device.press_button("A0")
    cover_switch_device.release_button("A0")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_STOPPED


def test_short_press_close_stops_on_repeat(cover_switch_device: Device):
    cover_switch_device.write_zigbee_attr(
        1,
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_LOCAL_MODE,
        ZCL_COVER_SWITCH_LOCAL_MODE_SHORT_PRESS,
    )
    
    # Press and release should start closing
    cover_switch_device.press_button("A1")
    cover_switch_device.release_button("A1")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_CLOSING

    # Repeated press and release should stop the cover
    cover_switch_device.press_button("A1")
    cover_switch_device.release_button("A1")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_STOPPED


def test_short_and_long_press_short_open_stops_on_repeat(cover_switch_device: Device):
    cover_switch_device.write_zigbee_attr(
        1,
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_LOCAL_MODE,
        ZCL_COVER_SWITCH_LOCAL_MODE_SHORT_AND_LONG_PRESS,
    )
    
    # Press and release should start opening
    cover_switch_device.press_button("A0")
    cover_switch_device.release_button("A0")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_OPENING

    # Repeated press and release should stop the cover
    cover_switch_device.press_button("A0")
    cover_switch_device.release_button("A0")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_STOPPED


def test_short_and_long_press_short_close_stops_on_repeat(cover_switch_device: Device):
    cover_switch_device.write_zigbee_attr(
        1,
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_LOCAL_MODE,
        ZCL_COVER_SWITCH_LOCAL_MODE_SHORT_AND_LONG_PRESS,
    )
    
    # Press and release should start closing
    cover_switch_device.press_button("A1")
    cover_switch_device.release_button("A1")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_CLOSING

    # Repeated press and release should stop the cover
    cover_switch_device.press_button("A1")
    cover_switch_device.release_button("A1")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_STOPPED


def test_short_and_long_press_long_keeps_moving(cover_switch_device: Device):
    cover_switch_device.write_zigbee_attr(
        1,
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_LOCAL_MODE,
        ZCL_COVER_SWITCH_LOCAL_MODE_SHORT_AND_LONG_PRESS,
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


def test_short_and_long_press_long_opposite_works(cover_switch_device: Device):
    cover_switch_device.write_zigbee_attr(
        1,
        ZCL_CLUSTER_COVER_SWITCH_CONFIG,
        ZCL_ATTR_COVER_SWITCH_LOCAL_MODE,
        ZCL_COVER_SWITCH_LOCAL_MODE_SHORT_AND_LONG_PRESS,
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

    # Release should stop the cover
    cover_switch_device.release_button("A1")
    assert cover_switch_device.zcl_cover_get_moving(2) == ZCL_WINDOW_COVERING_MOVING_STOPPED
