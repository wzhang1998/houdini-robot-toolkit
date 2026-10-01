"""STEP -> USD with Isaac's HOOPS converter (omni.kit.converter.hoops_core): python.bat step_to_usd.py IN OUT"""
import asyncio
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("omni.kit.converter.hoops_core")
for _ in range(10):
    app.update()
from omni.kit.converter.hoops_core import HoopsConverterHelper  # noqa: E402

src, dst = sys.argv[1], sys.argv[2]
helper = HoopsConverterHelper()
task = asyncio.ensure_future(helper.create_import_task(src, dst, {}))
while not task.done():
    app.update()
print("RESULT", task.result())
app.close()
