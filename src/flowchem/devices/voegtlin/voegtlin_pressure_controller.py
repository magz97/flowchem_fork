"""Vacuubrand CVC3000 control."""
import asyncio

import aioserial
import pint
from loguru import logger
from dataclasses import dataclass

from flowchem.components.device_info import DeviceInfo
from flowchem.devices.flowchem_device import FlowchemDevice
from flowchem.devices.voegtlin.voegtlin_pressure_controller_component import VoegtlinPressureControl
# from flowchem.devices.voegtlin.constants import ProcessStatus
from flowchem.utils.exceptions import InvalidConfigurationError
from flowchem.utils.people import miguel, jakob


@dataclass
class ModBusCommand:
    """Class representing a ModBus command for the pressure controller."""

    address: int  # Address of the device (1-247)
    function_code: int  # Function code (3 for read, 6 for write, 16 to write to multiple registers)
    register_address: int  # Register address to read/write from
    data: bytes = b""  # Data to be sent or processed, could be empty for read commands
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

    def parse_command(self) -> bytes:
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
        "timeout": 0.1,  # test
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

    async def write_and_read_reply_async(self, command: ModBusCommand, raise_errors: bool = True) -> tuple[str,str]:
        """Send a command to the valve, read the replies and returns it, optionally parsed."""
        self._serial.reset_input_buffer()
        print(command.parse_command())
        print(bytes.fromhex(f"{command.parse_command()}\r"))
        await self._write_async(bytes.fromhex(f"{command.parse_command()}\r"))
        response = await self._read_reply_async()
        if not response and raise_errors:
            raise InvalidConfigurationError(
                f"No response received from valve! "
                f"Maybe wrong valve address? (Set to {command.address})"
            )
        return self.parse_response(response=response, raise_errors=raise_errors)

    #TODO parse response
    @staticmethod
    def parse_response(response: str, raise_errors: bool = True) -> tuple[str, str]:
        """Split a received line in its components: status, reply."""
        status, parameters = response[4:6], response[6:10]
        parameters = parameters[2:] + parameters[:2]  # The bytes are swapped in the reply
        status_strings = {
            ...
        }

        status_string = status_strings.get(status, "Unknown status code")
        # Check if the status indicates an error
        if status in ("01", "02", "03", "04", "05", "06", "fe", "ff"):
            if raise_errors:
                logger.error(f"{status_string} (Status code: {status})")
                raise DeviceError(
                    f"{status_string} - Check command syntax or device status!"
                )
        return status_string, parameters

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
        # self.device_info.version = await self.version()
        # if not self.device_info.version:
        #     raise InvalidConfigurationError("No reply received from Voegtlin PC!")

        # # Set to CVC3000 mode and save
        # await self._send_command_and_read_reply("CVC 3")
        # await self._send_command_and_read_reply("STORE")
        # # Get reply to set commands
        # await self._send_command_and_read_reply("ECHO 1")
        # # Remote control
        # await self._send_command_and_read_reply("REMOTE 1")
        # # mbar, no autostart, no beep, venting auto
        # await self._send_command_and_read_reply("OUT_CFG 00001")
        # await self.motor_speed(100)

        logger.debug(f"Connected with version {self.device_info.version}")

        self.components.append(VoegtlinPressureControl("pressure-control", self))

    async def _send_command_and_read_reply(
            self,
            function_code: int,
            register_address: int,
            data: bytes,
    ):
        modbus_command = ModBusCommand(
            address=self.address,
            function_code=function_code,
            register_address=register_address,
            data=data
        )
        status, parameters = await self.voegtlin_io.write_and_read_reply_async(command, raise_errors)
        return status, parameters

    async def version(self):
        """Get version."""
        # raw_version = await self._send_command_and_read_reply("IN_VER")
        # # raw_version = CVC 3000 VX.YY
        # try:
        #     return raw_version.split()[-1]
        # except IndexError:
        #     return None

    async def set_pressure(self, pressure: pint.Quantity):
        """Set current pressure in mbar."""
        # mbar = int(pressure.m_as("mbar"))
        # await self._send_command_and_read_reply(f"OUT_SP_1 {mbar}")

    async def get_pressure(self):
        """Return current pressure in mbar."""
        # pressure_text = await self._send_command_and_read_reply("IN_PV_1")
        # return float(pressure_text.split()[0])

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
    # Assuming ModBusCommand class is defined as before
    modbus_command = ModBusCommand(
        address=1,  # Device address 1
        function_code=3,  # Read holding registers (ModBus function code 3)
        register_address=0x0021,  # Software version register (address 33 in decimal)
        data=b'\x00\x01'  # Read 1 register
    )

    # Get the complete command in bytes
    command_bytes = modbus_command.parse_command()

    # Print the command in hex format to verify
    print(f"ModBus command to read software version: {command_bytes}")

    import asyncio

    conf = {
        "port": "COM8",
        "address": 1,
        "name": "voegtlin_test",
    }
    pc = VoegtlinPressureController.from_config(**conf)


    async def main(pc):
        """Test function."""
        pc.voegtlin_io._serial.reset_input_buffer()
        s = await pc.voegtlin_io.write_and_read_reply_async(modbus_command)
        print(s)

    asyncio.run(main(pc))