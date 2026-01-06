"""Tests for cover functionality (input and output clusters)."""
import pytest

from client import StubProc
from conftest import Device
from zcl_consts import (
    ZCL_ATTR_WINDOW_COVERING_MOTOR_REVERSAL,
    ZCL_ATTR_WINDOW_COVERING_MOVING,
    ZCL_ATTR_WINDOW_COVERING_CALIBRATION,
    ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME,
    ZCL_ATTR_WINDOW_COVERING_CLOSED_SLACK,
    ZCL_ATTR_WINDOW_COVERING_OPEN_SLACK,
    ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE,
    ZCL_CLUSTER_WINDOW_COVERING,
    ZCL_CMD_WINDOW_COVERING_UP_OPEN,
    ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE,
    ZCL_CMD_WINDOW_COVERING_STOP,
    ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE,
    ZCL_WINDOW_COVERING_MOVING_STOPPED,
    ZCL_WINDOW_COVERING_MOVING_OPENING,
    ZCL_WINDOW_COVERING_MOVING_CLOSING,
)


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
        assert int(status) == ZCL_WINDOW_COVERING_MOVING_STOPPED
        
    finally:
        p.stop()


def test_cover_config_preserved_via_nvm():
    """Test that cover config attributes persist across device restarts via NVM."""
    device_config = "Mfr;Model;WA0A1;"
    endpoint = 1
    
    test_values = {
        ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME: "450",
        ZCL_ATTR_WINDOW_COVERING_CLOSED_SLACK: "20",
        ZCL_ATTR_WINDOW_COVERING_OPEN_SLACK: "15",
        ZCL_ATTR_WINDOW_COVERING_MOTOR_REVERSAL: "1",
    }
    
    # Session 1: Write config values
    with StubProc(device_config=device_config) as proc:
        device = Device(proc)
        
        for attr_id, value in test_values.items():
            device.write_zigbee_attr(
                endpoint, ZCL_CLUSTER_WINDOW_COVERING, attr_id, int(value)
            )
    
    # Session 2: Restart device and verify values are preserved
    with StubProc(device_config=device_config) as proc:
        device = Device(proc)
        
        for attr_id, expected_value in test_values.items():
            actual_value = device.read_zigbee_attr(
                endpoint, ZCL_CLUSTER_WINDOW_COVERING, attr_id
            )
            assert actual_value == expected_value, (
                f"Attribute {attr_id:04x} not preserved via NVM: "
                f"expected {expected_value}, got {actual_value}"
            )


def test_cover_open_command():
    """Test OPEN command activates correct relay and updates state."""
    cfg = "Mfr;Model;WA0A1;"
    p = StubProc(device_config=cfg).start()
    try:
        d = Device(p)
        endpoint = 1
        
        # Set calibration time first
        d.write_zigbee_attr(
            endpoint,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME,
            100  # 10 seconds
        )
        d.step_time(10)
        
        # Send OPEN command
        d.call_zigbee_cmd(endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_CMD_WINDOW_COVERING_UP_OPEN)
        d.step_time(50)
        
        # Verify OPEN relay is ON, CLOSE relay is OFF (integrated safety check)
        assert d.get_gpio("A0", refresh=True), "OPEN relay should be active"
        assert not d.get_gpio("A1", refresh=True), "CLOSE relay should be OFF (safety)"
        
        # Verify moving state
        moving = d.read_zigbee_attr(
            endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_ATTR_WINDOW_COVERING_MOVING
        )
        assert int(moving) == ZCL_WINDOW_COVERING_MOVING_OPENING
        
    finally:
        p.stop()


def test_cover_close_command():
    """Test CLOSE command activates correct relay and updates state."""
    cfg = "Mfr;Model;WA0A1;"
    p = StubProc(device_config=cfg).start()
    try:
        d = Device(p)
        endpoint = 1
        
        # Set calibration time first
        d.write_zigbee_attr(
            endpoint,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME,
            100  # 10 seconds
        )
        d.step_time(10)
        
        # Send CLOSE command
        d.call_zigbee_cmd(endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE)
        d.step_time(50)
        
        # Verify CLOSE relay is ON, OPEN relay is OFF (integrated safety check)
        assert d.get_gpio("A1", refresh=True), "CLOSE relay should be active"
        assert not d.get_gpio("A0", refresh=True), "OPEN relay should be OFF (safety)"
        
        # Verify moving state
        moving = d.read_zigbee_attr(
            endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_ATTR_WINDOW_COVERING_MOVING
        )
        assert int(moving) == ZCL_WINDOW_COVERING_MOVING_CLOSING
        
    finally:
        p.stop()


def test_cover_motor_reversal():
    """Test that motor reversal swaps which relay activates for OPEN/CLOSE."""
    cfg = "Mfr;Model;WA0A1;"
    p = StubProc(device_config=cfg).start()
    try:
        d = Device(p)
        endpoint = 1
        
        # Set calibration time
        d.write_zigbee_attr(
            endpoint,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME,
            100
        )
        d.step_time(10)
        
        # Normal operation (reversal = 0)
        d.write_zigbee_attr(
            endpoint,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_MOTOR_REVERSAL,
            0
        )
        d.step_time(10)
        
        d.call_zigbee_cmd(endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_CMD_WINDOW_COVERING_UP_OPEN)
        d.step_time(200) # Minimum relay on time
        assert d.get_gpio("A0", refresh=True), "Normal: OPEN command uses A0 relay"
        assert not d.get_gpio("A1", refresh=True), "Normal: A1 should be OFF"
        
        d.call_zigbee_cmd(endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_CMD_WINDOW_COVERING_STOP)
        d.step_time(550)  # Wait for safety delay after stop
        
        # Reversed operation (reversal = 1)
        d.write_zigbee_attr(
            endpoint,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_MOTOR_REVERSAL,
            1
        )
        d.step_time(10)
        
        d.call_zigbee_cmd(endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_CMD_WINDOW_COVERING_UP_OPEN)
        d.step_time(50)
        assert d.get_gpio("A1", refresh=True), "Reversed: OPEN command uses A1 relay"
        assert not d.get_gpio("A0", refresh=True), "Reversed: A0 should be OFF"
        
    finally:
        p.stop()


def test_cover_goto_position():
    """Test go-to-percentage command moves cover to target position."""
    cfg = "Mfr;Model;WA0A1;"
    p = StubProc(device_config=cfg).start()
    try:
        d = Device(p)
        endpoint = 1
        
        # Set calibration time (10 seconds)
        d.write_zigbee_attr(
            endpoint,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME,
            100
        )
        d.step_time(10)
        
        # Start at 0% (fully closed)
        d.call_zigbee_cmd(endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE)
        d.step_time(50)
        # Wait for it to fully close
        d.step_time(11000)  # Wait for full calibration time to complete
        
        # Verify stopped at 0%
        position = int(d.read_zigbee_attr(
            endpoint,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE
        ))
        assert position == 0, f"Should be at 0% to start, but at {position}%"
        
        # Wait for safety delay after auto-stop
        d.step_time(500)
        
        # Go to 50%
        d.call_zigbee_cmd(
            endpoint,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE,
            payload=bytes([50])
        )
        d.step_time(50)
        
        # Should be moving
        moving = int(d.read_zigbee_attr(
            endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_ATTR_WINDOW_COVERING_MOVING
        ))
        assert moving != ZCL_WINDOW_COVERING_MOVING_STOPPED, "Should be moving to position"
        
        # Wait for auto-stop (half of calibration time + margin)
        d.step_time(6000)  # If calib is 10s, halfway is ~5s
        
        # Should have stopped
        moving = int(d.read_zigbee_attr(
            endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_ATTR_WINDOW_COVERING_MOVING
        ))
        assert moving == ZCL_WINDOW_COVERING_MOVING_STOPPED, "Should have auto-stopped"
        
        # Position should be near target (minimal timing test - just verify it's not 0 or 100)
        position = int(d.read_zigbee_attr(
            endpoint,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE
        ))
        assert 20 < position < 80, f"Position {position} should be somewhere in middle range"
        
        # Both relays should be OFF after auto-stop
        assert not d.get_gpio("A0", refresh=True), "OPEN relay should be OFF after auto-stop"
        assert not d.get_gpio("A1", refresh=True), "CLOSE relay should be OFF after auto-stop"
        
    finally:
        p.stop()


def test_cover_moving_state_attribute():
    """Test that moving attribute accurately reflects current state."""
    cfg = "Mfr;Model;WA0A1;"
    p = StubProc(device_config=cfg).start()
    try:
        d = Device(p)
        endpoint = 1
        
        # Set calibration time
        d.write_zigbee_attr(
            endpoint,
            ZCL_CLUSTER_WINDOW_COVERING,
            ZCL_ATTR_WINDOW_COVERING_CALIBRATION_TIME,
            100
        )
        d.step_time(10)
        
        # Initially stopped
        moving = int(d.read_zigbee_attr(
            endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_ATTR_WINDOW_COVERING_MOVING
        ))
        assert moving == ZCL_WINDOW_COVERING_MOVING_STOPPED, "Should be initially stopped"
        
        # Start opening
        d.call_zigbee_cmd(endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_CMD_WINDOW_COVERING_UP_OPEN)
        d.step_time(50)
        moving = int(d.read_zigbee_attr(
            endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_ATTR_WINDOW_COVERING_MOVING
        ))
        assert moving == ZCL_WINDOW_COVERING_MOVING_OPENING, "Should be opening"
        d.step_time(200) # Minimum relay on time
        
        # Stop
        d.call_zigbee_cmd(endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_CMD_WINDOW_COVERING_STOP)
        d.step_time(50)
        moving = int(d.read_zigbee_attr(
            endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_ATTR_WINDOW_COVERING_MOVING
        ))
        assert moving == ZCL_WINDOW_COVERING_MOVING_STOPPED, "Should be stopped after STOP command"
        
        # Wait for safety delay before starting new movement
        d.step_time(500)
        
        # Start closing
        d.call_zigbee_cmd(endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE)
        d.step_time(50)
        moving = int(d.read_zigbee_attr(
            endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_ATTR_WINDOW_COVERING_MOVING
        ))
        assert moving == ZCL_WINDOW_COVERING_MOVING_CLOSING, "Should be closing"
        
    finally:
        p.stop()


def test_cover_direction_change():
    p = StubProc(device_config="Mfr;Model;WA0A1;").start()
    try:
        d = Device(p)
        endpoint = 1
        
        # Open command should work immediately
        d.call_zigbee_cmd(endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_CMD_WINDOW_COVERING_UP_OPEN)
        assert d.get_gpio("A0", refresh=True)
        assert not d.get_gpio("A1", refresh=True)
        assert d.zcl_cover_get_moving(endpoint) == ZCL_WINDOW_COVERING_MOVING_OPENING
        
        # Close command should not have an effect right after open
        d.call_zigbee_cmd(endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE)
        assert d.get_gpio("A0", refresh=True)
        assert not d.get_gpio("A1", refresh=True)
        assert d.zcl_cover_get_moving(endpoint) == ZCL_WINDOW_COVERING_MOVING_OPENING

        # Relay should open after minimum ON-time
        d.step_time(200)
        assert not d.get_gpio("A0", refresh=True)
        assert not d.get_gpio("A1", refresh=True)
        assert d.zcl_cover_get_moving(endpoint) == ZCL_WINDOW_COVERING_MOVING_STOPPED

        # Opposite relay should close after minimum OFF-time
        d.step_time(500)
        assert not d.get_gpio("A0", refresh=True)
        assert d.get_gpio("A1", refresh=True)
        assert d.zcl_cover_get_moving(endpoint) == ZCL_WINDOW_COVERING_MOVING_CLOSING
    finally:
        p.stop()


def test_cover_restart_after_stop():
    p = StubProc(device_config="Mfr;Model;WA0A1;").start()
    try:
        d = Device(p)
        endpoint = 1

        # Open command should work immediately
        d.call_zigbee_cmd(endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_CMD_WINDOW_COVERING_UP_OPEN)
        assert d.get_gpio("A0", refresh=True)
        assert not d.get_gpio("A1", refresh=True)
        assert d.zcl_cover_get_moving(endpoint) == ZCL_WINDOW_COVERING_MOVING_OPENING
        
        # Stop command should work after minimum ON-time
        d.step_time(200)
        d.call_zigbee_cmd(endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_CMD_WINDOW_COVERING_STOP)
        assert not d.get_gpio("A0", refresh=True)
        assert not d.get_gpio("A1", refresh=True)
        assert d.zcl_cover_get_moving(endpoint) == ZCL_WINDOW_COVERING_MOVING_STOPPED
        
        # Open command should not have an effect right after stop
        d.call_zigbee_cmd(endpoint, ZCL_CLUSTER_WINDOW_COVERING, ZCL_CMD_WINDOW_COVERING_UP_OPEN)
        assert not d.get_gpio("A0", refresh=True)
        assert not d.get_gpio("A1", refresh=True)
        assert d.zcl_cover_get_moving(endpoint) == ZCL_WINDOW_COVERING_MOVING_STOPPED
        
        # Relay should close after minimum OFF-time
        d.step_time(500)
        assert d.get_gpio("A0", refresh=True)
        assert not d.get_gpio("A1", refresh=True)
        assert d.zcl_cover_get_moving(endpoint) == ZCL_WINDOW_COVERING_MOVING_OPENING
    finally:
        p.stop()
