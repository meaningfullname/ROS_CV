import math
import pytest
from tabletop_robot.commands import parse_command
from tabletop_robot.vision import detect_objects
from tabletop_robot.world import TabletopWorld, TARGETS


@pytest.mark.parametrize('text,color,destination', [
    ('red left', 'red', 'left'),
    ('Положи синий кубик в правую коробку', 'blue', 'right'),
])
def test_commands(text, color, destination):
    command = parse_command(text)
    assert (command.color, command.destination) == (color, destination)


@pytest.mark.parametrize('text', ['red blue left', 'red', 'red left right', 'unknown'])
def test_ambiguous_commands(text):
    with pytest.raises(ValueError):
        parse_command(text)


@pytest.mark.parametrize('seed', [7, 42, 91])
@pytest.mark.parametrize('color,destination', [('red', 'left'), ('green', 'left'), ('blue', 'right')])
def test_camera_to_manipulation(seed, color, destination):
    world = TabletopWorld(seed=seed)
    try:
        rgb, depth, focal = world.camera()
        objects = detect_objects(rgb, depth, focal)
        assert {color for color, position in objects} == {'red', 'green', 'blue'}
        for detected_color, position in objects:
            nearest = min(math.dist(position, __import__('pybullet').getBasePositionAndOrientation(body)[0])
                          for body in world.blocks)
            assert nearest < 0.015
        pick = next(position for detected_color, position in objects if detected_color == color)
        assert world.pick_place(pick, TARGETS[destination])
        # The parked arm must leave the placed cube visible to the actual camera.
        placed = dict(detect_objects(*world.camera()))
        assert set(placed) == {'red', 'green', 'blue'}
        assert math.dist(placed[color][:2], TARGETS[destination][:2]) < 0.055
        world.seed += 1
        world.reset()
        assert {c for c, pos in detect_objects(*world.camera())} == {'red', 'green', 'blue'}
    finally:
        world.close()


def test_invalid_motion_and_cancel():
    world = TabletopWorld()
    try:
        with pytest.raises(ValueError):
            world.pick_place((float('nan'), 0, 0.025), TARGETS['left'])
        with pytest.raises(Exception, match='Canceled'):
            rgb, depth, focal = world.camera()
            position = detect_objects(rgb, depth, focal)[0][1]
            world.pick_place(position, TARGETS['left'], stop=lambda: True)
        assert world.constraint is None
    finally:
        world.close()
