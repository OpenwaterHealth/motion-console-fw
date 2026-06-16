"""HIL test for the console's boot-time I2C health scan.

At startup the console firmware passively pings every expected I2C device
across its two TCA9548 muxes (I2C1 + I2C2) and the fan bus (I2C4), and
caches the result. It is queryable over OW_CTRL_I2C_STATUS
(SDK: MotionConsole.get_i2c_health()).

Expected devices: both muxes (0x70), the Seed config FPGA (0x40, mux0 ch0),
the PCA9535 GPIO expander (0x20) + two ADS7828 PDU ADCs (0x48/0x4B) on mux1
ch0, three MAX31875 temp sensors (0x49/0x4A/0x4B) on mux1 ch1, the ADS7924
TEC ADC (0x49) on mux1 ch3, the four application FPGAs (TA/Seed/EE/OPT at
0x41) on mux1 ch4-7, and the MAX6663 fan (0x2C) on I2C4.

The scan is strictly passive (no power/reset/activation of the laser-driver
or safety FPGAs).

Run on a bench with a console attached:

    OPENMOTION_HIL=1 pytest tests/test_i2c_health_hil.py
    (PowerShell: $env:OPENMOTION_HIL = "1")
"""
import os
import time

import pytest

requires_bench = pytest.mark.skipif(
    os.environ.get("OPENMOTION_HIL") != "1",
    reason="hardware-in-the-loop test; set OPENMOTION_HIL=1 on the bench",
)

CONNECT_TIMEOUT_S = 15.0


@pytest.fixture
def console():
    from omotion import MotionInterface

    interface = MotionInterface()
    interface.start(wait=False)
    handle = interface.console
    deadline = time.monotonic() + CONNECT_TIMEOUT_S
    while time.monotonic() < deadline and not handle.is_connected():
        time.sleep(0.2)
    if not handle.is_connected():
        interface.stop()
        pytest.fail(f"console not reachable in {CONNECT_TIMEOUT_S:.0f}s")
    yield handle
    interface.stop()


def _assert_all_present(h):
    assert h is not None, "get_i2c_health() returned None"
    assert h["version"] == 1
    singles = ["mux0", "mux1", "seed_cfg_fpga", "gpio_exp",
               "pdu_adc0", "pdu_adc1", "tec_adc", "fan"]
    missing = [k for k in singles if not h[k]]
    assert not missing, f"devices missing: {missing}"
    missing_temps = [i for i, ok in enumerate(h["temps"]) if not ok]
    assert not missing_temps, f"temp sensors missing (mux1 ch1): {missing_temps}"
    missing_fpgas = [k for k, ok in h["fpgas"].items() if not ok]
    assert not missing_fpgas, f"application FPGAs missing (mux1 ch4-7): {missing_fpgas}"
    assert h["all_present"], "all_present should be True with a fully populated console"


@requires_bench
def test_boot_snapshot_all_present(console):
    """The cached boot-time snapshot reports every device present."""
    _assert_all_present(console.get_i2c_health())


@requires_bench
def test_live_rescan_all_present(console):
    """A live rescan (reserved==1) re-runs the passive scan and agrees."""
    _assert_all_present(console.get_i2c_health(rescan=True))
