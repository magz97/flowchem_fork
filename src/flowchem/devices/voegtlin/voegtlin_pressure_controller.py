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

    device_address: int  # Address of the device (1-247)
    function_code: int  # Function code (3 for read, 6 for write, 16 to write to multiple registers)
    register_address: int  # Register address to read/write from
    data: bytes  # Data to be sent or processed, could be empty for read commands
    crc: bytes = b""  # Checksum (CRC, calculated when needed)

    def calculate_crc(self):
        """Calculate and set CRC for the command."""
        # Build the message (device address, function code, register address, data)
        message = self.device_address.to_bytes(1, 'big') + \
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
                self.device_address.to_bytes(1, 'big') +  # 1 byte for device address
                self.function_code.to_bytes(1, 'big') +  # 1 byte for function code
                self.register_address.to_bytes(2, 'big') +  # 2 bytes for register address
                self.data +  # Data bytes
                self.crc  # 2 bytes for CRC
        )

        # Return the byte sequence representing the full ModBus command
        return command


class VoegtlinPressureController(FlowchemDevice):
    """Control class for Voegtlin Pressure Controller."""

    DEFAULT_CONFIG = {
        "timeout": 0.1,  # test
        "baudrate": 9600,  # Default baudrate, ModBus supports several other rates (2400 - 19200)
        "parity": aioserial.PARITY_NONE,  # No parity, but can be configured if needed
        "stopbits": aioserial.STOPBITS_TWO, # ModBus often uses two stop bits
        "bytesize": aioserial.EIGHTBITS,
    }

    def __init__(
        self,
        aio: aioserial.AioSerial,
        name="",
    ) -> None:
        super().__init__(name)
        self._serial = aio
        self._device_address: int = None  # Device ModBus address, set after initialization

        self.device_info = DeviceInfo(
            authors=[miguel, jakob],
            manufacturer="Voegtlin Instruments AG",
            model="red-y smart pressure controller GSP/GSB",
        )

    @classmethod
    def from_config(cls, port, name=None, **serial_kwargs):
        """Create instance from config dict. Used by server to initialize obj from config.

        Only required parameter is 'port'. Optional 'loop' + others (see AioSerial())
        """
        # Merge default settings, including serial, with provided ones.
        configuration = VoegtlinPressureController.DEFAULT_CONFIG | serial_kwargs

        try:
            serial_object = aioserial.AioSerial(port, **configuration)
        except (OSError, aioserial.SerialException) as serial_exception:
            raise InvalidConfigurationError(
                f"Cannot connect to the CVC3000 on the port <{port}>"
            ) from serial_exception

        return cls(serial_object, name)

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

    async def _send_command_and_read_reply(self, command: bytes) -> bytes:
        """Send ModBus command and read the binary reply.

        Args:
        ----
            command (bytes): Binary data to be transmitted (ModBus command)

        Returns:
        -------
            bytes: Binary reply received
        """
        # Send the binary ModBus command
        await self._serial.write_async(command)
        logger.debug(f"Command `{command.hex()}` sent!")

        # Receive reply as binary data
        try:
            reply = await asyncio.wait_for(self._serial.read_async(8), 2)  # Example: Read 8-byte response
        except asyncio.TimeoutError:
            logger.error("No reply received! Unsupported command?")
            return b""

        await asyncio.sleep(0.1)  # Respect the ModBus timing of 10 commands/second

        logger.debug(f"Reply received: {reply.hex()}")
        return reply

    async def version(self):
        """Get version."""
        # raw_version = await self._send_command_and_read_reply("IN_VER")
        # # raw_version = CVC 3000 VX.YY
        # try:
        #     return raw_version.split()[-1]
        # except IndexError:
        #     return None

    async def set_pressure(self, pressure: pint.Quantity):
        # mbar = int(pressure.m_as("mbar"))
        # await self._send_command_and_read_reply(f"OUT_SP_1 {mbar}")

    async def get_pressure(self):
        """Return current pressure in mbar."""
        # pressure_text = await self._send_command_and_read_reply("IN_PV_1")
        # return float(pressure_text.split()[0])

    async def motor_speed(self, speed):
        """Set motor speed to target % value."""
        # return await self._send_command_and_read_reply(f"OUT_SP_2 {speed}")

    async def status(self) -> ProcessStatus:
        """Get process status reply."""
        # raw_status = await self._send_command_and_read_reply("IN_STAT")
        # # Sometimes fails on first call
        # if not raw_status:
        #     raw_status = await self._send_command_and_read_reply("IN_STAT")
        # return ProcessStatus.from_reply(raw_status)


if __name__ == "__main__":
    # Assuming ModBusCommand class is defined as before
    modbus_command = ModBusCommand(
        device_address=1,  # Device address 1
        function_code=3,  # Read holding registers (ModBus function code 3)
        register_address=0x0021,  # Software version register (address 33 in decimal)
        data=b'\x00\x01'  # Read 1 register
    )

    # Get the complete command in bytes
    command_bytes = modbus_command.parse_command()

    # Print the command in hex format to verify
    print(f"ModBus command to read software version: {command_bytes.hex()}")

    # async def main():
    #     """Test function."""
    #
    # asyncio.run(main())