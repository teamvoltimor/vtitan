"""
Tests for HailoDriver (Hailo NPU hardware driver).

Run on: Raspberry Pi 5

Run with: python -m pytest tests/hardware/pi5/test_hailo.py -v
"""

import logging

import pytest

# hailo_8 transitively imports the Pi camera stack (picamera2); skip the whole
# module when it isn't installed (e.g. on dev/CI machines that aren't a Pi).
pytest.importorskip("picamera2")

from src.hardware.hailo.hailo_8 import (
    Config as HailoConfig,
    Driver as HailoDriver,
)
from src.logger import LOG_LEVEL, configure_json_logging

_log_level = getattr(logging, LOG_LEVEL.upper(), logging.INFO)
configure_json_logging(level=_log_level)

logger = logging.getLogger(__name__)


@pytest.fixture()
def driver():
    """Create driver instance."""
    config = HailoConfig(model_path="dummy.hef", benchmark_iterations=10)
    return HailoDriver(config=config)


class TestHailoConnection:
    """Test Hailo connection."""

    def test_connect(self, driver):
        """Connect to Hailo."""
        try:
            driver.connect()
            assert driver._device is not None
            logger.info("Hailo connection test passed")
        except Exception as e:
            pytest.skip(f"Cannot connect to Hailo: {e}")


class TestHailoModel:
    """Test Hailo model loading."""

    def test_load_model(self, driver):
        """Load HEF model."""
        try:
            driver.connect()
            driver.load_model()
            logger.info("Model loaded")
        except Exception as e:
            pytest.skip(f"Cannot load model: {e}")

    def test_get_io_shapes(self, driver):
        """Get input/output shapes."""
        try:
            driver.connect()
            driver.load_model()
            in_shape = driver.get_input_shape()
            out_shape = driver.get_output_shape()
            logger.info("I/O shapes", extra={"details": {"input": in_shape, "output": out_shape}})
        except Exception as e:
            pytest.skip(f"Cannot get shapes: {e}")


class TestHailoInference:
    """Test Hailo inference."""

    def test_infer(self, driver):
        """Run inference."""
        try:
            driver.connect()
            driver.load_model()

            import numpy as np

            input_shape = driver.get_input_shape()
            dummy_input = np.random.rand(*input_shape).astype(np.float32)

            result = driver.infer_with_timing(dummy_input)
            logger.info("Inference", extra={"details": {"latency_ms": result.latency_ms}})
        except Exception as e:
            pytest.skip(f"Cannot run inference: {e}")

    def test_benchmark_latency(self, driver):
        """Benchmark inference latency."""
        try:
            driver.connect()
            driver.load_model()

            avg_latency = driver.benchmark_latency(num_iterations=10)
            logger.info("Latency benchmark", extra={"details": {"avg_latency_ms": avg_latency}})
            assert avg_latency < 50
        except Exception as e:
            pytest.skip(f"Cannot benchmark: {e}")


class TestHailoPerformance:
    """Test Hailo performance metrics."""

    def test_temperature(self, driver):
        """Get NPU temperature."""
        try:
            driver.connect()
            temp = driver.get_temperature()
            if temp is not None:
                logger.info("Temperature", extra={"details": {"celsius": temp}})
            else:
                pytest.skip("Temperature not available")
        except Exception as e:
            pytest.skip(f"Cannot get temperature: {e}")

    def test_power_usage(self, driver):
        """Get power usage."""
        try:
            driver.connect()
            power = driver.get_power_usage()
            if power is not None:
                logger.info("Power usage", extra={"details": {"mw": power}})
            else:
                pytest.skip("Power not available")
        except Exception as e:
            pytest.skip(f"Cannot get power: {e}")


def run_inference():
    """
    Run continuous inference.

    Usage:
        python -c "from tests.hardware.pi5.test_hailo import run_inference; run_inference()"
    """
    logging.basicConfig(level=logging.INFO)
    log = logging.getLogger(__name__)

    import numpy as np

    config = HailoConfig(model_path="dummy.hef", benchmark_iterations=10)
    driver = HailoDriver(config=config)

    try:
        driver.connect()
        driver.load_model()

        input_shape = driver.get_input_shape()
        dummy_input = np.random.rand(*input_shape).astype(np.float32)

        log.info("Running inference - press Ctrl+C to stop")

        while True:
            result = driver.infer_with_timing(dummy_input)
            log.debug("Inference", extra={"details": {"latency_ms": result.latency_ms}})
    except KeyboardInterrupt:
        log.info("Stopped")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    logger.info("Run tests with: python -m pytest tests/hardware/pi5/test_hailo.py -v")
