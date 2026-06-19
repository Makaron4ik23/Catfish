import os
import subprocess
from .exceptions import LEDError


LED_TOPIC = '/model/x500_mono_cam_{}/led_cmd'
MSG_TYPE = 'gz.msgs.StringMsg'
GZ_CMD = 'gz'


def set_led_mask(drone_id: int, mask: str) -> None:
    if not isinstance(mask, str) or not all(c in '01' for c in mask):
        raise LEDError(f'Invalid LED mask: {mask!r}, expected binary string (e.g. "1010")')
    _publish(drone_id, mask)


def led_on(drone_id: int) -> None:
    _publish(drone_id, 'ON')


def led_off(drone_id: int) -> None:
    _publish(drone_id, 'OFF')


def led_blink(drone_id: int) -> None:
    _publish(drone_id, 'BLINK')


def _publish(drone_id: int, value: str) -> None:
    topic = LED_TOPIC.format(drone_id)
    payload = f'data:"{value}"'
    env = os.environ.copy()
    try:
        result = subprocess.run(
            [GZ_CMD, 'topic', '-t', topic, '-m', MSG_TYPE, '-p', payload],
            capture_output=True, text=True, timeout=5, env=env,
        )
        if result.returncode != 0:
            raise LEDError(
                f'gz topic -p failed (exit={result.returncode}): '
                f'{result.stderr.strip() or result.stdout.strip()}'
            )
    except subprocess.TimeoutExpired:
        raise LEDError(f'LED publish to drone {drone_id} timed out')
    except FileNotFoundError:
        raise LEDError(f'{GZ_CMD} not found — is Gazebo installed?')
    except LEDError:
        raise
    except Exception as e:
        raise LEDError(f'Failed to publish LED command to drone {drone_id}: {e}')
