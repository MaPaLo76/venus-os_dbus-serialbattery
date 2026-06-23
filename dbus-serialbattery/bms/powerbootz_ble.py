# -*- coding: utf-8 -*-

# NOTES
# Added by lotzm - Powerbootz Bluetooth BMS driver
# Protocol: ASCII-hex framing over BLE notifications
# BLE Service UUID:     0000fff0-0000-1000-8000-00805f9b34fb
# BLE Characteristic:   0000fff6-0000-1000-8000-00805f9b34fb (write + notify)
# Request command:      :015150000EFE~   (ASCII)
# Response command:     :0154...~        (ASCII-hex encoded, 117 decoded bytes)

import asyncio
import sys
from struct import unpack_from
from typing import Optional

from battery import Battery, Cell
from utils import logger

# ── BLE constants ─────────────────────────────────────────────────────────────
BLE_SERVICE_UUID = "0000fff0-0000-1000-8000-00805f9b34fb"
BLE_CHAR_UUID = "0000fff6-0000-1000-8000-00805f9b34fb"

# Request command sent to BMS (ASCII bytes)
REQUEST_CMD = b":015150000EFE~"

# Expected decoded binary frame length (bytes, incl. CRC)
FRAME_BINARY_LEN = 117

# Expected Cmd byte in response frame (at binary offset 1)
EXPECTED_CMD_BYTE = 0x54

# ── WorkState bit masks (uint16 BE at binary offset 44) ──────────────────────
_BIT_CING = 0x0001  # charging in progress
_BIT_DING = 0x0002  # discharging in progress
_BIT_VOLT_H = 0x0004  # overvoltage protection
_BIT_VOLT_L = 0x0008  # undervoltage protection
_BIT_CURR_C = 0x0010  # charge overcurrent
_BIT_CURR_S = 0x0020  # short circuit
_BIT_CURR_D1 = 0x0040  # discharge overcurrent 1
_BIT_CURR_D2 = 0x0080  # discharge overcurrent 2
_BIT_TEMP_CH = 0x0100  # charge over-temperature
_BIT_TEMP_CL = 0x0200  # charge under-temperature
_BIT_TEMP_DH = 0x0400  # discharge over-temperature
_BIT_TEMP_DL = 0x0800  # discharge under-temperature
_BIT_DFET = 0x1000  # discharge FET open (enabled)
_BIT_CFET = 0x2000  # charge FET open (enabled)


class Powerbootz_Ble(Battery):
    """
    dbus-serialbattery driver for Powerbootz Bluetooth BMS.

    Configure in config.ini:
        BLUETOOTH_BMS = Powerbootz_Ble AA:BB:CC:DD:EE:FF
    """

    BATTERYTYPE = "Powerbootz_Ble"
    poll_interval = 5000  # ms – BLE polling interval

    def __init__(self, port: str, baud: int, address: Optional[str] = None):
        super(Powerbootz_Ble, self).__init__(port, baud, address)
        self.type = self.BATTERYTYPE
        self.address = address
        # Persistent event-loop reused across calls (avoids bleak scanner overhead)
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        # Active BleakClient (reconnected on drop)
        self._client = None
        # Notification accumulation buffer
        self._buf = bytearray()
        # asyncio.Event signalled when a complete frame has arrived
        self._event: Optional[asyncio.Event] = None
        self.history.exclude_values_to_calculate = ["charge_cycles"]

    # ── dbus-serialbattery interface ──────────────────────────────────────────

    def test_connection(self) -> bool:
        """Connect to BMS, send one query and validate the response."""
        result = False
        try:
            result = self.get_settings()
            result = result and self.refresh_data()
        except Exception:
            exception_type, exception_object, exception_traceback = sys.exc_info()
            file = exception_traceback.tb_frame.f_code.co_filename
            line = exception_traceback.tb_lineno
            logger.error(f"Powerbootz_Ble: test_connection exception: {repr(exception_object)} " f"of type {exception_type} in {file} line #{line}")
        return result

    def unique_identifier(self) -> str:
        return self.address.replace(":", "").lower() if self.address else ""

    def connection_name(self) -> str:
        return "BLE " + (self.address or "")

    def custom_name(self) -> str:
        return "Powerbootz " + (self.address[-5:] if self.address else "")

    def get_settings(self) -> bool:
        """Static settings are populated from the first data frame in refresh_data."""
        return True

    def refresh_data(self) -> bool:
        """Query BMS and update all Battery fields."""
        try:
            raw = self._run(self._query())
            if raw is None:
                return False
            frame = self._decode_frame(raw)
            if frame is None:
                return False
            return self._parse_frame(frame)
        except Exception:
            exception_type, exception_object, exception_traceback = sys.exc_info()
            file = exception_traceback.tb_frame.f_code.co_filename
            line = exception_traceback.tb_lineno
            logger.error(f"Powerbootz_Ble: refresh_data exception: {repr(exception_object)} " f"of type {exception_type} in {file} line #{line}")
            return False

    # ── Async helpers ─────────────────────────────────────────────────────────

    def _run(self, coro):
        """Run an async coroutine synchronously using a persistent event loop."""
        if self._loop is None or self._loop.is_closed():
            self._loop = asyncio.new_event_loop()
        return self._loop.run_until_complete(coro)

    async def _connect(self) -> bool:
        """Ensure an active BLE connection with notifications registered."""
        from bleak import BleakClient, BleakScanner

        try:
            if self._client and self._client.is_connected:
                return True

            logger.info(f"Powerbootz_Ble: connecting to {self.address}")
            device = await BleakScanner.find_device_by_address(self.address, timeout=10.0)
            if device is None:
                logger.warning(f"Powerbootz_Ble: device {self.address} not found during scan")
                self._client = None
                return False

            self._client = BleakClient(device, timeout=15.0)
            await self._client.connect()
            self._buf.clear()
            self._event = asyncio.Event()
            await self._client.start_notify(BLE_CHAR_UUID, self._on_notify)
            logger.info(f"Powerbootz_Ble: connected to {self.address}")
            return True

        except Exception as e:
            logger.error(f"Powerbootz_Ble: connect error: {e}")
            self._client = None
            return False

    async def _disconnect(self):
        """Cleanly disconnect from BLE device."""
        if self._client:
            try:
                if self._client.is_connected:
                    await self._client.stop_notify(BLE_CHAR_UUID)
                    await self._client.disconnect()
            except Exception:
                pass
            self._client = None

    def _on_notify(self, _sender, data: bytearray):
        """BLE notification callback – accumulates data and signals when frame complete."""
        self._buf.extend(data)
        # The ASCII-hex frame ends with the '~' character (0x7E)
        if b"~" in self._buf and self._event is not None:
            self._loop.call_soon_threadsafe(self._event.set)

    async def _query(self, timeout: float = 8.0) -> Optional[bytes]:
        """Send the status-request command and return the raw ASCII-hex response."""
        if not await self._connect():
            return None

        try:
            self._buf.clear()
            self._event.clear()

            await self._client.write_gatt_char(BLE_CHAR_UUID, REQUEST_CMD, response=True)

            await asyncio.wait_for(self._event.wait(), timeout=timeout)
            return bytes(self._buf)

        except asyncio.TimeoutError:
            logger.warning("Powerbootz_Ble: timeout waiting for BMS response")
            await self._disconnect()
            return None
        except Exception as e:
            logger.error(f"Powerbootz_Ble: query error: {e}")
            await self._disconnect()
            return None

    # ── Frame decode / parse ──────────────────────────────────────────────────

    @staticmethod
    def _decode_frame(raw: bytes) -> Optional[bytes]:
        """
        Convert the ASCII-hex BLE response to a binary frame.

        The BLE response is: :XXYYZZ...CC~
        where each pair of ASCII characters is one hex byte.
        After stripping the delimiters and hex-decoding, the result is
        FRAME_BINARY_LEN (117) bytes.

        Returns the binary frame or None on error.
        """
        try:
            start = raw.find(b":")
            end = raw.find(b"~", start if start != -1 else 0)
            if start == -1 or end == -1 or end <= start:
                logger.error("Powerbootz_Ble: missing frame delimiters ':' / '~'")
                return None
            hex_str = raw[start + 1 : end].decode("ascii", errors="ignore").strip()
            frame = bytes.fromhex(hex_str)
        except Exception as e:
            logger.error(f"Powerbootz_Ble: frame hex-decode error: {e}")
            return None

        if len(frame) != FRAME_BINARY_LEN:
            logger.error(f"Powerbootz_Ble: unexpected frame length {len(frame)}, " f"expected {FRAME_BINARY_LEN}")
            return None

        # Checksum: CRC = sum(frame[0..115]) ^ 0xFF
        # Validation: sum(all 117 bytes) & 0xFF == 0xFF
        if (sum(frame) & 0xFF) != 0xFF:
            logger.warning(f"Powerbootz_Ble: checksum failed " f"(sum mod 256 = 0x{sum(frame) & 0xFF:02X}, expected 0xFF)")
            return None

        if frame[1] != EXPECTED_CMD_BYTE:
            logger.error(f"Powerbootz_Ble: unexpected cmd byte 0x{frame[1]:02X}, " f"expected 0x{EXPECTED_CMD_BYTE:02X}")
            return None

        return frame

    def _parse_frame(self, frame: bytes) -> bool:
        """
        Parse the 117-byte binary frame and populate all Battery fields.

        Binary layout (see bms-docs/POWERBOOTZ-BMS-Bluetooth.md):
          [0]      Addr
          [1]      Cmd  = 0x54
          [2]      Ver
          [3-4]    Len
          [5-11]   RTC  (reserved, 7 bytes)
          [12-43]  Cell voltages 1..16 (uint16 BE, mV)
          [44-45]  WorkState (uint16 BE)
          [46-47]  BalanceState (uint16 BE, bit per cell)
          [48-51]  Temp 1..4 (uint8, value – 40 = °C)
          [52-53]  BQ-Temp (uint16 BE, ÷10 = °C)
          [54-57]  Current actual (uint32 BE, mA; bit31=1 → discharge)
          [58-61]  Current mean   (uint32 BE, mA; bit31=1 → discharge)
          [62-63]  Total voltage  (uint16 BE, mV)
          [64-65]  Max cell diff  (uint16 BE, mV)
          [66]     Cell count     (uint8)
          [67-68]  Cycle count    (uint16 BE)
          [69]     SOC            (uint8, %)
          [70-73]  Design capacity   (uint32 BE, mAh)
          [74-77]  Remaining capacity(uint32 BE, mAh)
          [78-81]  Full-charge capacity (uint32 BE, mAh)
          [82-83]  SOH            (uint16 BE, %)
          [84]     SOC max error  (uint8)
          [85-86]  Flags          (uint16 BE)
          [87]     Learned Status (uint8)
          [88-89]  Serial number  (uint16 BE)
          [90-93]  Mfr date       (uint32 BE)
          [94-103] Manufacturer   (10 bytes ASCII)
          [104-115] System reserved
          [116]    CRC
        """
        try:
            # --- cell count (offset 66) ---
            cell_count = unpack_from(">B", frame, 66)[0]
            if cell_count < 1 or cell_count > 16:
                logger.error(f"Powerbootz_Ble: invalid cell count {cell_count}")
                return False

            self.cell_count = cell_count

            # Resize cells array only when necessary
            if len(self.cells) != self.cell_count:
                self.cells = [Cell(False) for _ in range(self.cell_count)]

            # --- cell voltages (offsets 12..43, uint16 BE, mV) ---
            for i in range(self.cell_count):
                mv = unpack_from(">H", frame, 12 + i * 2)[0]
                self.cells[i].voltage = mv / 1000.0

            # --- balance state (offset 46, uint16 BE, bit per cell) ---
            balance_state = unpack_from(">H", frame, 46)[0]
            for i, cell in enumerate(self.cells):
                cell.balance = bool(balance_state & (1 << i))

            # --- WorkState (offset 44, uint16 BE) ---
            work_state = unpack_from(">H", frame, 44)[0]

            self.charge_fet = bool(work_state & _BIT_CFET)
            self.discharge_fet = bool(work_state & _BIT_DFET)

            # --- temperatures (offsets 48..51, uint8, value – 40 = °C) ---
            for sensor_idx in range(4):
                raw_t = unpack_from(">B", frame, 48 + sensor_idx)[0]
                temp_c = raw_t - 40
                self.to_temperature(sensor_idx + 1, temp_c)

            # --- current (offset 54, uint32 BE, mA) ---
            # Emulator/App convention (confirmed from ESP32 emulator source):
            #   bit31 = 0 → discharge (positive mA in BMS convention)
            #   bit31 = 1 → charge (negative mA in BMS convention)
            # Victron dbus-serialbattery convention:
            #   positive = charging, negative = discharging
            curr_raw = unpack_from(">I", frame, 54)[0]
            if curr_raw & 0x80000000:
                # charge – positive in Victron convention
                self.current = (curr_raw & 0x7FFFFFFF) / 1000.0
            else:
                # discharge – negative in Victron convention
                self.current = -(curr_raw / 1000.0)

            # --- total voltage (offset 62, uint16 BE, mV) ---
            total_mv = unpack_from(">H", frame, 62)[0]
            if total_mv > 0:
                self.voltage = total_mv / 1000.0
            else:
                # Fallback: sum cell voltages (protocol note: always sum all 16 slots)
                self.voltage = sum(c.voltage for c in self.cells)

            # --- SOC (offset 69, uint8, %) ---
            self.soc = float(unpack_from(">B", frame, 69)[0])

            # --- capacities (mAh → Ah) ---
            design_mah = unpack_from(">I", frame, 70)[0]
            remain_mah = unpack_from(">I", frame, 74)[0]
            full_mah = unpack_from(">I", frame, 78)[0]

            if full_mah > 0:
                self.capacity = full_mah / 1000.0
            elif design_mah > 0:
                self.capacity = design_mah / 1000.0

            if remain_mah > 0:
                self.capacity_remain = remain_mah / 1000.0

            # --- SOH (offset 82, uint16 BE, %) ---
            self.soh = float(unpack_from(">H", frame, 82)[0])

            # --- cycle count (offset 67, uint16 BE) ---
            self.history.charge_cycles = unpack_from(">H", frame, 67)[0]

            # --- serial number (offset 88, uint16 BE) ---
            self.serial_number = str(unpack_from(">H", frame, 88)[0])

            # --- protection alarms ---
            self.protection.voltage_high = 2 if (work_state & _BIT_VOLT_H) else 0
            self.protection.voltage_low = 2 if (work_state & _BIT_VOLT_L) else 0
            self.protection.current_over = 2 if (work_state & (_BIT_CURR_C | _BIT_CURR_D1 | _BIT_CURR_D2 | _BIT_CURR_S)) else 0
            self.protection.temp_high_charge = 2 if (work_state & _BIT_TEMP_CH) else 0
            self.protection.temp_low_charge = 2 if (work_state & _BIT_TEMP_CL) else 0
            self.protection.temp_high_discharge = 2 if (work_state & _BIT_TEMP_DH) else 0
            self.protection.temp_low_discharge = 2 if (work_state & _BIT_TEMP_DL) else 0

            return True

        except Exception:
            exception_type, exception_object, exception_traceback = sys.exc_info()
            file = exception_traceback.tb_frame.f_code.co_filename
            line = exception_traceback.tb_lineno
            logger.error(f"Powerbootz_Ble: _parse_frame exception: {repr(exception_object)} " f"of type {exception_type} in {file} line #{line}")
            return False
