"""Voegtlin Pressure Control component."""
from __future__ import annotations

from typing import TYPE_CHECKING

from flowchem.components.technical.pressure import PressureControl
from flowchem.devices.flowchem_device import FlowchemDevice

if TYPE_CHECKING:
    from flowchem.devices.voegtlin.voegtlin_pressure_controller import VoegtlinPressureController


class VoegtlinPressureControl(PressureControl):
    hw_device: VoegtlinPressureController  # for typing's sake

    def __init__(self, name: str, hw_device: FlowchemDevice) -> None:
        """Create a PressureControl object."""
        super().__init__(name, hw_device)

    async def set_pressure(self, pressure: str):
        """Set the target pressure to the given string in natural language."""
        set_p = await super().set_pressure(pressure)
        return await self.hw_device.set_pressure(set_p)

    async def get_pressure(self) -> float:
        """Return pressure in mbar."""
        return await self.hw_device.get_pressure()

    async def is_target_reached(self) -> bool:
        """Return True if the set temperature target has been reached."""
        tolerance = 0.001

        current_pressure = await self.hw_device.get_pressure()
        set_point_pressure = await self.hw_device.set_pressure()
        return abs(current_pressure - set_point_pressure) <= tolerance

    async def power_on(self):
        """Turn on pressure control."""
        return await self.hw_device.turn_on_control()

    async def power_off(self):
        """Turn off pressure control."""
        return await self.hw_device.turn_off_control()
