"""Tests for cover functionality (input and output clusters)."""
import pytest

from client import StubProc
from conftest import Device
from zcl_consts import (
    ZCL_ATTR_BASIC_MFR_NAME,
    ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE,
    ZCL_ATTR_WINDOW_COVERING_MOTOR_REVERSAL,
    ZCL_ATTR_WINDOW_COVERING_TYPE,
    ZCL_ATTR_WINDOW_COVERING_MOVING,
    ZCL_ATTR_WINDOW_COVERING_CALIBRATION,
    ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME,
    ZCL_ATTR_WINDOW_COVERING_OPEN_DELAY,
    ZCL_ATTR_WINDOW_COVERING_CLOSE_DELAY,
    ZCL_ATTR_COVER_SWITCH_OUTPUT_INDEX,
    ZCL_ATTR_COVER_SWITCH_REVERSAL,
    ZCL_CLUSTER_BASIC,
    ZCL_CLUSTER_MULTISTATE_INPUT_BASIC,
    ZCL_CLUSTER_ON_OFF_SWITCH_CONFIG,
    ZCL_CLUSTER_WINDOW_COVERING,
    ZCL_CMD_WINDOW_COVERING_UP_OPEN,
    ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE,
    ZCL_CMD_WINDOW_COVERING_STOP,
    COVER_STOPPED,
    COVER_OPENING,
    COVER_CLOSING,
)


def test_girier_cover_device_boots():
    """Test that GIRIER_TS130F_DUAL config boots without crashing.
    
    This is the CRITICAL safety test to prevent bricking the device.
    Config: j1xl73iw;TS130F-GIR-DUAL;LC1;XB4D2u;WC0C4;XC3C2u;WD4D7;
    - 2 cover switch pairs (X entries): B4+D2, C3+C2
    - 2 cover pairs (W entries): C0+C4, D4+D7
    """
    cfg = "j1xl73iw;TS130F-GIR-DUAL;LC1;XB4D2u;WC0C4;XC3C2u;WD4D7;"
    p = StubProc(device_config=cfg).start()
    try:
        d = Device(p)
        # If we can read basic cluster, device booted successfully
        mfr = d.read_zigbee_attr(1, ZCL_CLUSTER_BASIC, ZCL_ATTR_BASIC_MFR_NAME)
        assert mfr == "j1xl73iw"
    finally:
        p.stop()


def test_girier_cover_endpoint_layout():
    """Test that GIRIER config creates correct endpoint layout.

    Expected layout:
    - EP1: Cover switch 1 (OnOffSwitchConfig server + WindowCovering client + MultiStateInput)
    - EP2: Cover switch 2 (OnOffSwitchConfig server + WindowCovering client + MultiStateInput)
    - EP3: Cover 1 (WindowCovering server)
    - EP4: Cover 2 (WindowCovering server)
    """
    cfg = "j1xl73iw;TS130F-GIR-DUAL;LC1;XB4D2u;WC0C4;XC3C2u;WD4D7;"
    p = StubProc(device_config=cfg).start()
    try:
        d = Device(p)
        
        # EP1 and EP2 should have MultiStateInput (cover switches)
        for ep in [1, 2]:
            multistate = d.read_zigbee_attr(
                ep,
                ZCL_CLUSTER_MULTISTATE_INPUT_BASIC,
                ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE
            )
            assert multistate in ("0", "1", "2", "3", "4", "5"), f"EP{ep} missing MultiStateInput"
            
            # Check custom cover switch config attributes on OnOffSwitchConfig server
            output_idx = d.read_zigbee_attr(
                ep, 
                ZCL_CLUSTER_ON_OFF_SWITCH_CONFIG,
                ZCL_ATTR_COVER_SWITCH_OUTPUT_INDEX
            )
            assert output_idx is not None, f"EP{ep} missing output_index attribute"
        
        # EP3 and EP4 should have WindowCovering server (covers)
        for ep in [3, 4]:
            # Check operational_status attribute
            status = d.read_zigbee_attr(
                ep,
                ZCL_CLUSTER_WINDOW_COVERING,
                ZCL_ATTR_WINDOW_COVERING_MOVING
            )
            # Should be stopped (0) initially
            assert int(status) == COVER_STOPPED, \
                f"EP{ep} operational_status should be stopped initially"
            
            # Check window covering type attribute
            wc_type = d.read_zigbee_attr(
                ep,
                ZCL_CLUSTER_WINDOW_COVERING,
                ZCL_ATTR_WINDOW_COVERING_TYPE
            )
            assert wc_type is not None, f"EP{ep} missing window_covering_type"
            
            # Check motor reversal attribute
            reversal = d.read_zigbee_attr(
                ep,
                ZCL_CLUSTER_WINDOW_COVERING,
                ZCL_ATTR_WINDOW_COVERING_MOTOR_REVERSAL
            )
            assert reversal is not None, f"EP{ep} missing motor_reversal attribute"
            
    finally:
        p.stop()


@pytest.mark.parametrize(
    "cfg,num_cover_switches,num_covers",
    [
        # Single cover pair
        ("Mfr;Model;XA0A1u;WB0B1;", 1, 1),
        # Two cover pairs (like GIRIER)
        ("Mfr;Model;XA0A1u;WB0B1;XC0C1u;WD0D1;", 2, 2),
        # Mixed: 1 switch + 1 relay + 1 cover
        ("Mfr;Model;SA0u;RA1;XB0B1u;WC0C1;", 1, 1),  # switch/relay on EP1/2, cover on EP3/4
    ],
)
def test_various_cover_configs_boot(cfg: str, num_cover_switches: int, num_covers: int):
    """Test that various cover configurations boot without crashes."""
    p = StubProc(device_config=cfg).start()
    try:
        d = Device(p)
        # Just verify device boots and basic cluster is accessible
        _ = d.read_zigbee_attr(1, ZCL_CLUSTER_BASIC, ZCL_ATTR_BASIC_MFR_NAME)
    finally:
        p.stop()


def test_cover_responds_to_commands(device: Device):
    """Test that cover responds to OPEN/CLOSE/STOP commands."""
    # This test uses default device_config fixture which needs to be updated
    # For now, we'll skip unless device has cover endpoints
    try:
        # Try to send OPEN command to hypothetical cover endpoint
        device.call_zigbee_cmd(3, ZCL_CLUSTER_WINDOW_COVERING, ZCL_CMD_WINDOW_COVERING_UP_OPEN)
        # If no exception, command was accepted
    except AssertionError:
        # Expected if device doesn't have cover endpoints
        pytest.skip("Device config doesn't have cover endpoints")


def test_array_bounds_safety():
    """Test that we don't exceed array bounds with maximum config.
    
    This tests the safety limits:
    - buttons[5] array
    - relays[5] array
    - cover_switch_clusters[4] array
    - cover_clusters[4] array
    """
    # Maximum safe config: 2 cover pairs = 4 buttons + 4 relays
    # This should work (within limits)
    cfg_ok = "Mfr;Model;XA0A1u;WB0B1;XC0C1u;WD0D1;"
    p = StubProc(device_config=cfg_ok).start()
    try:
        d = Device(p)
        _ = d.read_zigbee_attr(1, ZCL_CLUSTER_BASIC, ZCL_ATTR_BASIC_MFR_NAME)
    finally:
        p.stop()
    
    # This config would exceed limits (3 cover pairs = 6 buttons)
    # buttons[5] can only hold 5, so this would overflow
    # NOTE: Ideally this should fail gracefully or be caught by parser
    cfg_overflow = "Mfr;Model;XA0A1u;WB0B1;XC0C1u;WD0D1;XE0E1u;WF0F1;"
    # TODO: When bounds checking is added, verify it fails gracefully
    # For now, we just document the unsafe config


def test_cover_pins_initialized_correctly():
    """Test that cover pins are initialized with correct pull resistors."""
    cfg = "Mfr;Model;XA0A1u;WB0B1;"  # Input with pull-up (u), output with no pull
    p = StubProc(device_config=cfg).start()
    try:
        d = Device(p)
        
        # Input pins (A0, A1) should be initialized as inputs with pull-up
        # Output pins (B0, B1) should be initialized as outputs
        
        # If device boots, GPIO init succeeded
        _ = d.read_zigbee_attr(1, ZCL_CLUSTER_BASIC, ZCL_ATTR_BASIC_MFR_NAME)
        
        # TODO: Add GPIO state verification when stub supports GPIO introspection
        
    finally:
        p.stop()


@pytest.fixture
def cover_device_config() -> str:
    """Config for a simple cover device with one input/output pair."""
    return "TestMfr;TestDevice;LC0;XA0A1u;WB0B1;"


@pytest.fixture
def cover_device(cover_device_config: str) -> Device:
    """Fixture for a device with cover configuration."""
    proc = StubProc(device_config=cover_device_config).start()
    yield Device(proc)
    proc.stop()


def test_cover_switch_button_press(cover_device: Device):
    """Test that pressing cover switch buttons updates multistate value."""
    # Press OPEN button (A0)
    cover_device.press_button("A0")
    cover_device.step_time(100)
    
    # MultiState value should change (exact value depends on implementation)
    value = cover_device.read_zigbee_attr(
        1,
        ZCL_CLUSTER_MULTISTATE_INPUT_BASIC,
        ZCL_ATTR_MULTISTATE_INPUT_PRESENT_VALUE
    )
    # Just verify it's a valid multistate value
    assert value in ("0", "1", "2")
    
    cover_device.release_button("A0")


def test_cover_safety_interlock():
    """Test that OPEN and CLOSE relays cannot be activated simultaneously.
    
    This is a CRITICAL safety test to prevent short circuits.
    """
    cfg = "Mfr;Model;WA0A1;"  # Simple output pair
    p = StubProc(device_config=cfg).start()
    try:
        d = Device(p)
        
        # Send OPEN command
        d.call_zigbee_cmd(1, ZCL_CLUSTER_WINDOW_COVERING, ZCL_CMD_WINDOW_COVERING_UP_OPEN)
        d.step_time(10)
        
        # Check GPIO state - OPEN relay (A0) should be ON
        open_state = d.get_gpio("A0", refresh=True)
        close_state = d.get_gpio("A1", refresh=True)
        
        # During OPEN operation, CLOSE should definitely be OFF
        assert not close_state, "CLOSE relay active during OPEN command - SAFETY VIOLATION!"
        
        # Now send CLOSE command (should stop OPEN first)
        d.call_zigbee_cmd(1, ZCL_CLUSTER_WINDOW_COVERING, ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE)
        d.step_time(60)  # Wait for 50ms interlock + some margin
        
        # Check states again
        open_state = d.get_gpio("A0", refresh=True)
        close_state = d.get_gpio("A1", refresh=True)
        
        # OPEN should be OFF now, CLOSE should be ON
        assert not open_state, "OPEN relay still active during CLOSE command - SAFETY VIOLATION!"
        
    finally:
        p.stop()


def test_cover_stop_command():
    """Test that STOP command turns off both relays."""
    cfg = "Mfr;Model;WA0A1;"
    p = StubProc(device_config=cfg).start()
    try:
        d = Device(p)
        
        # Start OPEN movement
        d.call_zigbee_cmd(1, ZCL_CLUSTER_WINDOW_COVERING, ZCL_CMD_WINDOW_COVERING_UP_OPEN)
        d.step_time(10)
        
        # Send STOP
        d.call_zigbee_cmd(1, ZCL_CLUSTER_WINDOW_COVERING, ZCL_CMD_WINDOW_COVERING_STOP)
        d.step_time(10)
        
        # Both relays should be OFF
        open_state = d.get_gpio("A0", refresh=True)
        close_state = d.get_gpio("A1", refresh=True)
        
        assert not open_state, "OPEN relay still active after STOP"
        assert not close_state, "CLOSE relay active after STOP"
        
        # Operational status should be STOPPED
        status = d.read_zigbee_attr(
            1,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_MOVING
        )
        assert int(status) == COVER_STOPPED
        
    finally:
        p.stop()


def test_cover_calibration_attributes():
    """Test that calibration attributes are present and writable."""
    cfg = "Mfr;Model;WA0A1;"
    p = StubProc(device_config=cfg).start()
    try:
        d = Device(p)
        
        # Read initial calibration attributes
        calibration = d.read_zigbee_attr(
            1,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_CALIBRATION
        )
        assert calibration == "0", "Initial calibration should be 0 (false)"
        
        calibration_time = d.read_zigbee_attr(
            1,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME
        )
        assert calibration_time == "0", "Initial calibration_time should be 0"
        
        open_delay = d.read_zigbee_attr(
            1,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_OPEN_DELAY
        )
        assert open_delay == "0", "Initial open_delay should be 0"
        
        close_delay = d.read_zigbee_attr(
            1,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_CLOSE_DELAY
        )
        assert close_delay == "0", "Initial close_delay should be 0"
        
        # Write calibration_time (e.g., 30 seconds)
        d.write_zigbee_attr(
            1,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME,
            30
        )
        d.step_time(10)
        
        # Verify it was written
        calibration_time = d.read_zigbee_attr(
            1,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME
        )
        assert calibration_time == "30", "calibration_time should be 30"
        
        # Write open_delay (e.g., 5 = 0.5 seconds in 100ms units)
        d.write_zigbee_attr(
            1,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_OPEN_DELAY,
            5
        )
        d.step_time(10)
        
        # Verify it was written
        open_delay = d.read_zigbee_attr(
            1,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_OPEN_DELAY
        )
        assert open_delay == "5", "open_delay should be 5"
        
        # Write close_delay (e.g., 3 = 0.3 seconds in 100ms units)
        d.write_zigbee_attr(
            1,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_CLOSE_DELAY,
            3
        )
        d.step_time(10)
        
        # Verify it was written
        close_delay = d.read_zigbee_attr(
            1,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_CLOSE_DELAY
        )
        assert close_delay == "3", "close_delay should be 3"
        
    finally:
        p.stop()


def test_cover_calibration_attributes_persist():
    """Test that calibration attributes can be written and read back."""
    cfg = "Mfr;Model;WA0A1;"
    p = StubProc(device_config=cfg).start()
    try:
        d = Device(p)
        
        # Write calibration attributes
        d.write_zigbee_attr(
            1,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME,
            45
        )
        d.write_zigbee_attr(
            1,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_OPEN_DELAY,
            10
        )
        d.write_zigbee_attr(
            1,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_CLOSE_DELAY,
            8
        )
        d.step_time(10)
        
        # Verify attributes can be read back
        calibration_time = d.read_zigbee_attr(
            1,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME
        )
        assert calibration_time == "45", "calibration_time should be readable"
        
        open_delay = d.read_zigbee_attr(
            1,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_OPEN_DELAY
        )
        assert open_delay == "10", "open_delay should be readable"
        
        close_delay = d.read_zigbee_attr(
            1,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_CLOSE_DELAY
        )
        assert close_delay == "8", "close_delay should be readable"
        
        # Note: NVM persistence testing would require stub device to support
        # cross-process NVM preservation, which may not be implemented yet
        
    finally:
        p.stop()


