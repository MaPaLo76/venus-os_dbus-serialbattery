# -*- coding: utf-8 -*-
"""Tests for Powerbootz_Ble BMS frame decoding and parsing."""

import os
import sys
import types

# ---------------------------------------------------------------------------
# Path setup – must happen before any dbus-serialbattery imports
# ---------------------------------------------------------------------------
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "dbus-serialbattery"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "dbus-serialbattery", "ext", "velib_python"))

# Stub out bleak so the import works without BLE hardware
sys.modules.setdefault("bleak", types.SimpleNamespace(BleakClient=None, BleakScanner=None))
sys.modules.setdefault("bleak.exc", types.SimpleNamespace(BleakDBusError=Exception))

from battery import History  # noqa: E402
from bms.powerbootz_ble import Powerbootz_Ble  # noqa: E402

# ---------------------------------------------------------------------------
# Example frame from the Powerbootz protocol documentation (section 6),
# with the CRC byte corrected to 0x36 so that sum(frame) & 0xFF == 0xFF
# (emulator algorithm: CRC = sum(frame[0..115]) ^ 0xFF).
# (The original doc used CRC=0xD2 which is illustrative only.)
# ---------------------------------------------------------------------------
_EXAMPLE_FRAME_ASCII = (
    b":01543100E0000000000000000C900C9A0C9C0C99000000000000000000000000000000000000000000000"
    b"000F00200003F42424200E5800013EC80000F003170000C0400020000000A8C0000000000000A23006401"
    b"0107060001000000000000000000000000000000000000000000000000000036~"
)


def _make_bms() -> Powerbootz_Ble:
    bms = Powerbootz_Ble.__new__(Powerbootz_Ble)
    bms.address = "04:7F:0E:4F:16:AC"
    bms.cells = []
    bms.history = History()
    bms.history.exclude_values_to_calculate = ["charge_cycles"]
    # initialise protection stub so _parse_frame can write to it
    from battery import Protection

    bms.protection = Protection()
    return bms


def test_decode_frame_returns_117_bytes():
    frame = Powerbootz_Ble._decode_frame(_EXAMPLE_FRAME_ASCII)
    assert frame is not None, "_decode_frame must succeed on the example frame"
    assert len(frame) == 117


def test_decode_frame_checksum():
    """The checksum of the decoded binary frame must pass: sum(all) & 0xFF == 0xFF."""
    frame = Powerbootz_Ble._decode_frame(_EXAMPLE_FRAME_ASCII)
    assert frame is not None
    assert (sum(frame) & 0xFF) == 0xFF


def test_decode_frame_cmd_byte():
    frame = Powerbootz_Ble._decode_frame(_EXAMPLE_FRAME_ASCII)
    assert frame is not None
    assert frame[1] == 0x54


def test_decode_frame_rejects_wrong_length():
    """Truncated frame must be rejected."""
    bad = b":DEADBEEF~"
    frame = Powerbootz_Ble._decode_frame(bad)
    assert frame is None


def test_decode_frame_rejects_missing_delimiters():
    frame = Powerbootz_Ble._decode_frame(b"no delimiters here")
    assert frame is None


def test_parse_frame_cell_count():
    bms = _make_bms()
    frame = Powerbootz_Ble._decode_frame(_EXAMPLE_FRAME_ASCII)
    assert frame is not None
    assert bms._parse_frame(frame) is True
    assert bms.cell_count == 4


def test_parse_frame_cell_voltages():
    """Expected values from the example frame: 3216, 3226, 3228, 3225 mV."""
    bms = _make_bms()
    frame = Powerbootz_Ble._decode_frame(_EXAMPLE_FRAME_ASCII)
    assert frame is not None
    bms._parse_frame(frame)
    assert len(bms.cells) == 4
    assert abs(bms.cells[0].voltage - 3.216) < 0.001
    assert abs(bms.cells[1].voltage - 3.226) < 0.001
    assert abs(bms.cells[2].voltage - 3.228) < 0.001
    assert abs(bms.cells[3].voltage - 3.225) < 0.001


def test_parse_frame_total_voltage():
    """Total voltage from the example frame: 0x3170 = 12656 mV = 12.656 V."""
    bms = _make_bms()
    frame = Powerbootz_Ble._decode_frame(_EXAMPLE_FRAME_ASCII)
    assert frame is not None
    bms._parse_frame(frame)
    assert abs(bms.voltage - 12.656) < 0.001


def test_parse_frame_current_discharge():
    """
    WorkState F002 → DING=1 (discharging in BMS convention).
    Current 0x800013EC, bit31=1 → charge in emulator convention, magnitude 0x13EC=5100 mA.
    Victron convention: positive = charging → current = +5.1 A.
    """
    bms = _make_bms()
    frame = Powerbootz_Ble._decode_frame(_EXAMPLE_FRAME_ASCII)
    assert frame is not None
    bms._parse_frame(frame)
    assert abs(bms.current - 5.1) < 0.01


def test_parse_frame_temperature():
    """Temp1 = 0x3F = 63 → 63 - 40 = 23 °C."""
    bms = _make_bms()
    frame = Powerbootz_Ble._decode_frame(_EXAMPLE_FRAME_ASCII)
    assert frame is not None
    bms._parse_frame(frame)
    # to_temperature stores into temperature_1 via the Battery base class
    assert abs(bms.temperature_1 - 23.0) < 0.5


def test_parse_frame_soc():
    """SOC byte at offset 69 = 0x00 → 0 %."""
    bms = _make_bms()
    frame = Powerbootz_Ble._decode_frame(_EXAMPLE_FRAME_ASCII)
    assert frame is not None
    bms._parse_frame(frame)
    assert bms.soc == 0.0


def test_parse_frame_capacity():
    """Full-charge capacity: 0x00000A23 = 2595 mAh → 2.595 Ah."""
    bms = _make_bms()
    frame = Powerbootz_Ble._decode_frame(_EXAMPLE_FRAME_ASCII)
    assert frame is not None
    bms._parse_frame(frame)
    assert abs(bms.capacity - 2.595) < 0.001


def test_parse_frame_soh():
    """SOH: 0x0064 = 100 %."""
    bms = _make_bms()
    frame = Powerbootz_Ble._decode_frame(_EXAMPLE_FRAME_ASCII)
    assert frame is not None
    bms._parse_frame(frame)
    assert bms.soh == 100.0


def test_parse_frame_discharge_fet_enabled():
    """WorkState 0xF002: DFET=1, CFET=1 (bits 12+13 set)."""
    bms = _make_bms()
    frame = Powerbootz_Ble._decode_frame(_EXAMPLE_FRAME_ASCII)
    assert frame is not None
    bms._parse_frame(frame)
    assert bms.charge_fet is True
    assert bms.discharge_fet is True


def test_parse_frame_no_protection_alarms():
    """WorkState 0xF002 has no alarm bits set (bits 2..11 are 0)."""
    bms = _make_bms()
    frame = Powerbootz_Ble._decode_frame(_EXAMPLE_FRAME_ASCII)
    assert frame is not None
    bms._parse_frame(frame)
    assert bms.protection.voltage_high == 0
    assert bms.protection.voltage_low == 0
    assert bms.protection.current_over == 0


def test_parse_frame_cycle_count():
    """Cycle count: 0x0002 = 2."""
    bms = _make_bms()
    frame = Powerbootz_Ble._decode_frame(_EXAMPLE_FRAME_ASCII)
    assert frame is not None
    bms._parse_frame(frame)
    assert bms.history.charge_cycles == 2


def test_parse_frame_repeated_calls_do_not_grow_cells():
    """Calling _parse_frame multiple times must not grow the cells list."""
    bms = _make_bms()
    frame = Powerbootz_Ble._decode_frame(_EXAMPLE_FRAME_ASCII)
    assert frame is not None
    for _ in range(10):
        bms._parse_frame(frame)
    assert bms.cell_count == 4
    assert len(bms.cells) == 4


def test_unique_identifier():
    bms = _make_bms()
    assert bms.unique_identifier() == "047f0e4f16ac"
