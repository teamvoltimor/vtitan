import re

with open("tests/hardware/pi_zero/test_motors.py", "r") as f:
    content = f.read()

# Add an autouse fixture at the top to skip if no hardware
fixture_code = """
@pytest.fixture(autouse=True)
def check_hardware(driver):
    try:
        driver.connect()
    except Exception as e:
        pytest.skip(f"Hardware not available: {e}")
"""

content = content.replace(
    "logger = logging.getLogger(__name__)\n", "logger = logging.getLogger(__name__)\n" + fixture_code
)

with open("tests/hardware/pi_zero/test_motors.py", "w") as f:
    f.write(content)
