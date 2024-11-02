"""Voegtlin Pressure Controller control."""
import asyncio
import aioserial
import struct
from flowchem import ureg

from loguru import logger
from dataclasses import dataclass
from flowchem.components.device_info import DeviceInfo
from flowchem.devices.flowchem_device import FlowchemDevice
from flowchem.devices.voegtlin.voegtlin_pressure_controller_component import VoegtlinPressureControl
from flowchem.utils.exceptions import InvalidConfigurationError
from flowchem.utils.people import miguel, jakob


@dataclass
class ModBusCommand:
    """Class representing a ModBus command for the pressure controller."""

    address: int = 247  # Address of the device (1-247)
    function_code: int = 3  # Function code (3 for read, 6 for write, 16 to write to multiple registers)
    register_address: int = 0x0002  # Register address to read/write from
    data: bytes = b"\x00\x01"  # Data to be sent or processed, could be empty for read commands
    response_format: str = ""
    crc: bytes = b""  # Checksum (CRC, calculated when needed)

    def calculate_crc(self):
        """Calculate and set CRC for the command."""
        # Build the message (device address, function code, register address, data)
        message = self.address.to_bytes(1, 'big') + \
                  self.function_code.to_bytes(1, 'big') + \
                  self.register_address.to_bytes(2, 'big') + \
                  self.data

        # Calculate CRC-16-IBM (ModBus) and store it
        self.crc = self._calculate_crc_for_modbus(message)

    def _calculate_crc_for_modbus(self, message: bytes) -> bytes:
        """Private method to calculate the ModBus CRC-16."""
        crc = 0xFFFF
        for pos in message:
            crc ^= pos  # XOR byte into least significant byte of crc
            for _ in range(8):  # Loop over each bit
                if (crc & 0x0001) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        # Return the CRC in little-endian order (as ModBus expects it)
        return crc.to_bytes(2, byteorder='little')

    def parse_command(self) -> str:
        """Return the entire ModBus command as a byte sequence."""
        # Ensure CRC is calculated before forming the command
        if not self.crc:
            self.calculate_crc()

        # Build the complete command as bytes
        command = (
                self.address.to_bytes(1, 'big') +  # 1 byte for device address
                self.function_code.to_bytes(1, 'big') +  # 1 byte for function code
                self.register_address.to_bytes(2, 'big') +  # 2 bytes for register address
                self.data +  # Data bytes
                self.crc  # 2 bytes for CRC
        )

        # Return the byte sequence representing the full ModBus command
        return command.hex()

class VoegtlinIO:
    """Setup with serial parameters, low-level IO for Voegtlin devices."""

    DEFAULT_CONFIG = {
        "baudrate": 9600,  # Default baudrate, ModBus supports several other rates (2400 - 19200)
        "parity": aioserial.PARITY_NONE,  # No parity, but can be configured if needed
        "stopbits": aioserial.STOPBITS_TWO,  # ModBus often uses two stop bits
        "bytesize": aioserial.EIGHTBITS,
    }

    def __init__(self, aio_port: aioserial.AioSerial) -> None:
        """Initialize serial port for SV-06 valve."""
        self._serial = aio_port

    @classmethod
    def from_config(cls, config):
        """Create VoegtlinIO from config."""
        # Combine the default configuration with the user provided configuration
        configuration = VoegtlinIO.DEFAULT_CONFIG | config

        try:
            serial_object = aioserial.AioSerial(**configuration)
        except aioserial.SerialException as serial_exception:
            raise InvalidConfigurationError(
                f"Cannot connect to the valve on the port <{configuration.get('port')}>"
            ) from serial_exception

        return cls(serial_object)

    async def _write_async(self, command: bytes):
        """Write a command to the valve."""
        await self._serial.write_async(command)

    async def _read_reply_async(self) -> str:
        """Read the valve reply from serial communication."""
        reply_string = await self._serial.readline_async()
        return reply_string.hex()

    async def write_and_read_reply_async(self, command: ModBusCommand) -> dict:
        """Send a command to the valve, read the replies and returns it, optionally parsed."""
        self._serial.reset_input_buffer()
        await self._write_async(bytes.fromhex(f"{command.parse_command()}"))
        response = await self._read_reply_async()
        if not response:
            raise InvalidConfigurationError(
                f"No response received from valve! "
                f"Maybe wrong valve address? (Set to {command.address})"
            )
        return self.parse_response(response=response, response_format=command.response_format)

    @staticmethod
    def parse_response(response: str, response_format: str) -> dict:
        """
        Parses the Vögtlin device response into address, function code, and data components.

        Args:
            response (str): Hexadecimal response string from the Vögtlin device.
            format (str): Expected data format (e.g., "uint8", "float32", "string8") for interpreting the data.

        Returns:
            dict: Parsed components including 'address', 'function_code', 'data', and 'crc'.
        """
        # Extract address (first byte), function code (second byte), and data (remaining bytes excluding CRC)
        address = response[:2]
        function_code = response[2:4]
        byte_count = response[4:6]
        data = response[6:-4]  # Exclude CRC at the end
        crc = response[-4:]

        # Interpret data based on the provided format
        parsed_data = {}
        if response_format == "u8" and len(data) == 2:
            parsed_data["u8"] = int(data, 16)

        elif response_format == "u16" and len(data) == 4:
            parsed_data["u16"] = int(data, 16)

        elif response_format == "u32" and len(data) == 8:
            parsed_data["u32"] = int(data, 16)

        elif response_format == "f32" and len(data) == 8:
            hex_int = int(data, 16)
            parsed_data["f32"] = struct.unpack('!f', hex_int.to_bytes(4, 'big'))[0]

        elif response_format == "s8" and len(data) == 16:
            parsed_data["s8"] = bytes.fromhex(data).decode('latin-1').strip('\x00')

        elif response_format == "s50" and len(data) == 100:
            parsed_data["s50"] = bytes.fromhex(data).decode('latin-1').strip('\x00')

        else:
            parsed_data["error"] = f"Data length mismatch for format '{response_format}'"

        if response_format == "":
            return {"response": response} #for now
        else:
            return {
                "address": address,
                "function_code": function_code,
                "byte_count": byte_count,
                "data": parsed_data,
                "crc": crc
            }

class VoegtlinPressureController(FlowchemDevice):
    """Control class for Voegtlin Pressure Controller."""

    DEFAULT_CONFIG = {

    }

    _io_instances: set[VoegtlinIO] = set()

    def __init__(
        self,
        voegtlin_io: VoegtlinIO,
        name: str = "",
        address: int = 1,
        **config,
    ) -> None:
        super().__init__(name)

        self.voegtlin_io = voegtlin_io
        VoegtlinPressureController._io_instances.add(self.voegtlin_io)
        self.config = VoegtlinPressureController.DEFAULT_CONFIG | config
        self.address = int(address)
        self.address = address
        self.device_info = DeviceInfo(
            authors=[miguel, jakob],
            manufacturer="Voegtlin Instruments AG",
            model="red-y smart pressure controller GSP/GSB",
        )

    @classmethod
    def from_config(cls, **config):
        """Create instances via config file."""
        voegtlin_io = None
        for obj in VoegtlinPressureController._io_instances:
            # noinspection PyProtectedMember
            if obj._serial.port == config.get("port"):
                voegtlin_io = obj
                break

        # If not existing serial object are available for the port provided, create a new one
        if voegtlin_io is None:
            # Remove RunzeValve-specific keys to only have RunzeeIO's configs
            config_for_voegtlin_io = {
                k: v
                for k, v in config.items()
                if k not in ("address", "name")
            }
            voegtlin_io = VoegtlinIO.from_config(config_for_voegtlin_io)

        return cls(
            voegtlin_io,
            address=config.get("address", 1),
            name=config.get("name", ""),
        )

    async def initialize(self):
        """Ensure the connection w/ device is working."""
        self.device_info.version = await self.version()
        if not self.device_info.version:
            raise InvalidConfigurationError("No reply received from Voegtlin PC!")

        await self.turn_off_control()
        logger.debug(f"Connected with version {self.device_info.version}")

        self.components.append(VoegtlinPressureControl("pressure-control", self))

    async def version(self):
        """Get version."""
        modbus_command = ModBusCommand(
            address=self.address,  # Device address 1
            function_code=3,  # Read holding registers (ModBus function code 3)
            register_address=0x0021,  # first register address
            data=b"\x00\x01",
            response_format="u16"
        )
        response = await self.voegtlin_io.write_and_read_reply_async(modbus_command)
        return response['data'][modbus_command.response_format]

    async def set_pressure(self, pressure: pint.Quantity):
        """Set current pressure in mbar."""
        # mbar = int(pressure.m_as("mbar"))
        # await self._send_command_and_read_reply(f"OUT_SP_1 {mbar}")

    async def get_pressure(self):
        """Return current pressure in mbar."""
        modbus_command = ModBusCommand(
            address=self.address,  # Device address 1
            function_code=3,  # Read holding registers (ModBus function code 3)
            register_address=0x5f00,  # first register address
            data=b"\x00\x02",
            response_format="f32"
        )
        response = await pc.voegtlin_io.write_and_read_reply_async(modbus_command)
        return response['data'][modbus_command.response_format]

    # async def get_pressure_unit(self):
    #     """Return current pressure in mbar."""
    #     modbus_command = ModBusCommand(
    #         address=self.address,  # Device address 1
    #         function_code=3,  # Read holding registers (ModBus function code 3)
    #         register_address=0x5f00,  # first register address
    #         data=b"\x00\x02",
    #         response_format="s8"
    #     )
    #     response = await pc.voegtlin_io.write_and_read_reply_async(modbus_command)
    #     return response['data'][modbus_command.response_format]

    async def motor_speed(self, speed):
        """Set motor speed to target % value."""
        # return await self._send_command_and_read_reply(f"OUT_SP_2 {speed}")

    async def status(self):
        """Get process status reply."""
        # raw_status = await self._send_command_and_read_reply("IN_STAT")
        # # Sometimes fails on first call
        # if not raw_status:
        #     raw_status = await self._send_command_and_read_reply("IN_STAT")
        # return ProcessStatus.from_reply(raw_status)


if __name__ == "__main__":
    import asyncio

    conf = {
        "port": "COM8",
        "address": 7,
        "name": "voegtlin_test",
    }
    pc = VoegtlinPressureController.from_config(**conf)


    async def main(pc):
        """Test function."""
        version = await pc.version()
        print(version)
        pressure = await pc.get_pressure()
        print(pressure)
        pressure_unit = await pc.get_pressure_unit()
        print(pressure_unit)

    asyncio.run(main(pc))