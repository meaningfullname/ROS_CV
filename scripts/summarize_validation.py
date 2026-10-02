"""Collect saved run evidence; does not run tests or infer missing successes."""
from datetime import datetime
from importlib.metadata import version
import json
from pathlib import Path
import platform
import shutil
import xml.etree.ElementTree as ET
import cv2


def main():
    root = Path(__file__).resolve().parents[1]
    artifacts = root / 'artifacts'
    headless = json.loads((artifacts / 'headless/report.json').read_text())
    gui = json.loads((artifacts / 'gui/report.json').read_text())
    tests = ET.parse(artifacts / 'pytest.xml').getroot().find('testsuite')
    trials = []
    for name in ('benchmark-final-red.jsonl', 'benchmark-final-blue.jsonl'):
        trials.extend(json.loads(line) for line in (artifacts / name).read_text().splitlines())
    assert tests is not None and int(tests.attrib['failures']) == 0 and int(tests.attrib['errors']) == 0
    assert headless['passed'] and gui['passed']
    assert len(trials) == 40 and all(t['success'] for t in trials)
    clean_shutdown = {}
    for mode in ('headless', 'gui'):
        log = (artifacts / mode / 'launch.log').read_text()
        assert 'Traceback' not in log and '[ERROR]' not in log, mode
        assert log.count('process has finished cleanly') == 3, mode
        clean_shutdown[mode] = True
    gui_log = (artifacts / 'gui/launch.log').read_text()
    assert 'Created GL 3.3 context' in gui_log and 'GL_RENDERER=' in gui_log
    report = {
        'verified_at': datetime.now().astimezone().isoformat(),
        'system': {'os': platform.freedesktop_os_release()['PRETTY_NAME'],
                   'architecture': platform.machine(), 'kernel': platform.release(),
                   'system_python': '3.12.3', 'environment_python': platform.python_version(),
                   'ros': 'Jazzy (RoboStack, isolated .ros-env)',
                   'initial_available_memory_gib_approx': 9.5,
                   'initial_free_disk_gib_approx': 51,
                   'graphics': 'Wayland / XWayland, AMD Radeon / Mesa 25.2.8'},
        'dependencies': {**{name: version(name) for name in
                         ('numpy', 'pybullet', 'pytest', 'setuptools', 'colcon-core')},
                         'opencv': cv2.__version__},
        'unit_tests_passed': int(tests.attrib['tests']),
        'simulation_trials': trials,
        'ros_build_packages': ['tabletop_interfaces (ament_cmake)', 'tabletop_robot (ament_python)'],
        'ros_integration_tested': True,
        'gui_tested': True,
        'ros_headless': headless,
        'ros_gui': gui,
        'clean_shutdown': clean_shutdown,
        'not_tested': ['Real robot', 'Other OS/architectures/ROS distributions',
                       'ROS networking between computers', 'Long continuous operation',
                       '45-second deadline', 'Cancellation before goal acceptance response',
                       'System ROS apt installation'],
        'remaining_limitations': ['Fixed-constraint grasp', 'No collision-aware planning / MoveIt',
                                  'Fixed camera calibration, no TF', 'Camera paused during motion',
                                  'Flat target zones, no physical boxes'],
        'remaining_failures_in_checked_scenarios': [],
        'note': 'DIRECT timings are accelerated simulation timings. Expected cancellation is not a failed check.',
    }
    previous = root / 'validation.previous.json'
    if (root / 'validation.json').exists() and not previous.exists():
        shutil.copy2(root / 'validation.json', previous)
    (root / 'validation.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print(f'Validated: {report["unit_tests_passed"]} tests, {len(trials)} DIRECT trials, headless ROS and GUI ROS.')


if __name__ == '__main__':
    main()
