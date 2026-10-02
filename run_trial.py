import argparse
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).parent / 'src' / 'tabletop_robot'))
from tabletop_robot.commands import parse_command
from tabletop_robot.config import TARGETS
from tabletop_robot.vision import detect_objects
from tabletop_robot.world import TabletopWorld


def main():
    parser = argparse.ArgumentParser(description='Run reproducible camera-guided trials without ROS')
    parser.add_argument('--command', default='red left')
    parser.add_argument('--trials', type=int, default=1)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--output', type=Path, default=Path('benchmark.jsonl'))
    args = parser.parse_args()
    if args.trials < 1:
        parser.error('--trials must be positive')
    command = parse_command(args.command)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    failures = 0
    for trial in range(args.trials):
        started = time.monotonic()
        world = TabletopWorld(gui=args.gui, seed=args.seed + trial)
        success = False
        message = ''
        try:
            rgb, depth, focal = world.camera()
            matches = [position for color, position in detect_objects(rgb, depth, focal)
                       if color == command.color]
            if len(matches) != 1:
                raise ValueError('Expected exactly one matching object')
            success = world.pick_place(matches[0], TARGETS[command.destination],
                                       stop=lambda: time.monotonic() - started > 45,
                                       realtime=args.gui)
            message = 'Placed successfully' if success else 'Verification failed'
        except Exception as error:
            message = str(error)
        finally:
            world.close()
        record = {'seed': args.seed + trial, 'command': args.command,
                  'success': success, 'message': message,
                  'elapsed_seconds': round(time.monotonic() - started, 3)}
        with args.output.open('a') as stream:
            stream.write(json.dumps(record) + '\n')
        print(json.dumps(record))
        failures += not success
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
