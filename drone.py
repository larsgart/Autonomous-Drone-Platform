import sys
import logging
from datetime import datetime
from pathlib import Path

from models.flight_controller import FlightController


def _setup_logging():
    Path("Logs").mkdir(exist_ok=True)
    logging.basicConfig(
        filename=f"Logs/drone_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log",
        level=logging.DEBUG,
        format='%(asctime)s %(levelname)s %(name)s — %(message)s',
    )


if __name__ == '__main__':
    _setup_logging()
    test_mode = sys.argv[1] if len(sys.argv) > 1 else False
    fc = FlightController(test_mode=test_mode)
    try:
        fc.run()
    except KeyboardInterrupt:
        fc.close()
