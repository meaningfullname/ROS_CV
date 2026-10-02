# Tabletop Robot

ROS 2-проект: Franka Panda на рабочей поверхности распознаёт цветные кубики
по RGB-D изображению и переносит выбранный кубик в левую или правую зону.

Команды на русском и английском разбирает детерминированный парсер.
Цвета и координаты объектов определяются по RGB-D с помощью HSV/OpenCV.
Управление движением реализовано через ROS 2 action с обратной связью,
отменой и проверкой результата переноса.

## Архитектура

| Модуль | Ответственность |
|---|---|
| `commands.py` | Проверка текстовой команды, без ROS-зависимостей |
| `vision.py` | RGB-D → цвет и мировые координаты кубиков |
| `world.py` | PyBullet, Panda, IK, движение, упрощённый захват |
| `simulator_node.py` | Камера, action управления, сброс сцены |
| `perception_node.py` | Синхронизация RGB, depth и CameraInfo |
| `task_node.py` | Выбор предмета, отправка action, отмена, журнал |
| `tabletop_interfaces` | Типизированные сообщения и action |

`/camera/rgb`, `/camera/depth`, `/camera/info` → perception →
`/perception/objects` → task manager → `/arm/pick_place` action.

Координаты в метрах, frame `world`. Камера фиксирована вертикально над
рабочей областью: eye=(0.45, 0, 1.2), target=(0.45, 0, 0).
Преобразование в `vision.py` рассчитано именно на эту калибровку.

## Проверенная среда и установка

Фактическая проверка на этом компьютере: **3 октября 2026, Ubuntu 24.04.5
LTS, x86_64**. Системный Python — 3.12.3; RAM — около 14 ГиБ, в начале
проверки доступно 9.5 ГиБ; swap — 4 ГиБ; свободно на диске — 51 ГиБ.
ROS 2, colcon, NumPy, OpenCV и PyBullet изначально отсутствовали.
Графическая сессия — Wayland с доступным XWayland (`DISPLAY=:0`).

Установлено отдельное окружение **`.ros-env` внутри проекта**, без sudo,
изменения системного Python, apt, разделов диска и других проектов:

| Компонент | Проверенная версия |
|---|---|
| ROS 2 | Jazzy, RoboStack |
| Python окружения | 3.12.14 |
| NumPy / OpenCV | 1.26.4 / 4.13.0 |
| PyBullet | 3.2.7, собран из исходников |
| setuptools / pytest | 79.0.1 / 8.4.2 |
| colcon-core | 0.21.3 |
| GUI | OpenGL через AMD Radeon / Mesa 25.2.8 |

Способ установки основан на [инструкции RoboStack](https://robostack.github.io/micromamba.html).
Все зависимости описаны в `environment.yml`; точные conda-пакеты проверенного
окружения сохранены в `environment-linux-64.lock.txt`, pip-пакеты — в
`requirements-verified.txt`. Эти файлы относятся к Linux x86_64.
Окружение вместе с кэшем занимает примерно 5 ГиБ; во время сборки нужно
дополнительное место. Первичная установка требует сети и нескольких минут.

На **этом компьютере окружение уже установлено**. Для новой установки:

```bash
cd ~/tabletop_robot
bash scripts/setup_env.sh
```

Скрипт скачивает micromamba в `.tools`, складывает кэш в `.mamba` и создаёт
`.ros-env`. `~/.bashrc` не меняется. При повторном вызове скрипт обновляет
окружение согласно `environment.yml`; для обычного запуска он не нужен.
Не смешивай это окружение с `/opt/ros/.../setup.bash` или другой venv.

Версии ограничены намеренно: setuptools 84 вызвал при сборке ошибку
`option --editable not recognized`, pytest 9.1.1 — ошибку ROS-плагина
`launch_testing`. Ограничение setuptools соответствует
[описанной разработчиками colcon несовместимости](https://github.com/colcon/colcon-core/issues/696).
NumPy/OpenCV/cv_bridge берутся совместно из conda; не добавляй сюда
`opencv-python` или другую версию NumPy через pip.

## Сборка и запуск

```bash
cd ~/tabletop_robot
source scripts/activate.sh
colcon build --base-paths src --symlink-install --executor sequential
source install/setup.bash
ros2 launch tabletop_robot demo.launch.py
```

`--base-paths src` ограничивает поиск двумя пакетами проекта, чтобы colcon
не обходил окружение и сохранённые артефакты. Сборка обоих пакетов проверена.
`scripts/activate.sh` активирует локальный ROS и, если сборка существует,
автоматически подключает workspace. Выполняй его в каждом новом bash-терминале.

Для компьютера без графической сессии:

```bash
ros2 launch tabletop_robot demo.launch.py gui:=false seed:=7
```

По умолчанию headless-движение ускорено. Для удобной проверки отмены и
занятости включи темп симуляции с ожиданием между физическими шагами:

```bash
ros2 launch tabletop_robot demo.launch.py gui:=false realtime:=true
```

GUI всегда использует этот темп. По завершении переноса рука уходит из
обзора камеры; `/scene/reset` восстанавливает её начальную позу и меняет seed.

## Команды

Во втором терминале активируй ту же среду:

```bash
cd ~/tabletop_robot
source scripts/activate.sh
ros2 topic pub --once /task/command std_msgs/msg/String "{data: 'red left'}"
```

Русский вариант:

```bash
ros2 topic pub --once /task/command std_msgs/msg/String \
  "{data: 'Положи синий кубик в правую коробку'}"
```

Доступны red/green/blue и left/right. Цвет и сторона должны быть однозначными.
Отправляй следующую команду после завершения предыдущей.

```bash
ros2 topic echo /task/status
ros2 topic echo /perception/objects
ros2 service call /scene/reset std_srvs/srv/Trigger '{}'
ros2 topic pub --once /task/cancel std_msgs/msg/String "{data: 'cancel'}"
```

Сброс меняет seed и расположение кубиков. Во время движения сброс отклоняется.
Камера публикуется, когда рука не выполняет задачу: это дискретная демонстрация,
а не визуальное управление в замкнутом цикле.

Headless:

```bash
ros2 launch tabletop_robot demo.launch.py gui:=false seed:=7
```

## Результаты и турнирный режим

Каждая выполненная задача пишет одну запись в `results.jsonl`: команда,
успех, причина и время. Успех проверяется по фактическому положению кубика
в симуляторе после отпускания. Ground truth используется только для захвата
и проверки результата, не для распознавания.

Для серии испытаний: сбрось сцену, дождись свежего распознавания, отправь
команду, дождись результата; повтори 20 раз. Интеграционный runner описан ниже;
это функциональная проверка, а не оценка по правилам конкретного турнира.

## Проверки

```bash
source scripts/activate.sh
python -m pytest -q --junitxml=artifacts/pytest.xml
python -m compileall -q src scripts run_trial.py
python -m pip check
python scripts/verify_ros.py --output-dir artifacts/headless
python scripts/verify_ros.py --gui --domain-id 84 --output-dir artifacts/gui
```

Пересчитать сводный отчёт по уже сохранённым прогонам:

```bash
python scripts/summarize_validation.py
```

Сводный скрипт ожидает ровно 40 записей в `benchmark-final-red.jsonl` и
`benchmark-final-blue.jsonl` и проверяет сохранённые логи; сам проверки не
запускает. Для повторения этой серии сначала архивируй оба файла, затем выполни:

```bash
python run_trial.py --command 'red left' --trials 20 --seed 42 --output artifacts/benchmark-final-red.jsonl
python run_trial.py --command 'blue right' --trials 20 --seed 42 --output artifacts/benchmark-final-blue.jsonl
```

`run_trial.py` дописывает JSONL и возвращает код 1, если хотя бы один перенос
не удался. Для дополнительных серий можно выбрать другой выходной файл.

Тесты проверяют русский/английский парсер, неоднозначные команды, изображение
и координаты, перенос трёх цветов при разных seed, видимость кубика после
переноса, восстановление сцены, неверные координаты и отмену. Получено
**16 passed**; тесты используют PyBullet DIRECT.

`verify_ros.py` сам запускает `ros2 launch`, ждёт готовности узлов и выполняет
полный сценарий через настоящие ROS-интерфейсы. Его не нужно запускать поверх
другого demo в том же ROS domain. Используется отдельный `ROS_DOMAIN_ID` (83
для headless, 84 в GUI-команде выше) и локальное обнаружение узлов.
Прогоны сохраняют `report.json`, `launch.log`, ROS-логи и журнал задач в
указанную папку. Код возврата 1 означает провал проверки. Узлы останавливаются
после прогона; обычный demo работает до Ctrl+C.

Фактически пройдены **headless и GUI**:

| Проверка | Результат |
|---|---|
| Сборка `tabletop_interfaces` и `tabletop_robot` | Оба собраны через colcon |
| Launch | simulator, perception, task_manager работают отдельными процессами |
| RGB / depth / CameraInfo | 320×320, rgb8 / 32FC1, общая временная метка, положительная конечная глубина |
| Распознавание | red, green, blue, frame world |
| `/task/command`: red left | Успех; положение подтверждено свежим RGB-D распознаванием |
| Прямой `/arm/pick_place`: blue right | STATUS_SUCCEEDED; положение подтверждено распознаванием |
| Неверная команда | Отклонена |
| Занятая рука | Отклонены вторая команда, второй action и reset |
| Отмена через `/task/cancel` после захвата | Задача завершена с success=false; сцена сбрасывается |
| Прямая отмена action | STATUS_CANCELED |
| NaN в action | STATUS_ABORTED, Non-finite coordinate |
| Восстановление | После отмены/ошибки выполнена русская команда blue right |
| Журнал задач | Два успеха и ожидаемая отмена, по одной записи на задачу |
| Выход | Все три процесса корректно завершаются |
| PyBullet DIRECT без ROS | 40/40 успешных переносов, seed 42–61 |
| GUI | Создан настоящий GLX/OpenGL-контекст AMD; весь ROS-сценарий пройден |

Итоги: `validation.json`, `artifacts/headless/report.json`,
`artifacts/gui/report.json`, `artifacts/pytest.xml`,
`artifacts/benchmark-final-red.jsonl`, `artifacts/benchmark-final-blue.jsonl`.
Предыдущий отчёт сохранён как `validation.previous.json`; неудачный первый
ROS-прогон — в `artifacts/headless-initial`.
Каталог `artifacts/` и предыдущий отчёт относятся к локальным результатам
запуска и исключены из Git. Итоговый `validation.json` включён в репозиторий.

Не проверены: реальный робот, сетевое взаимодействие между компьютерами,
другие ОС/архитектуры/дистрибутивы ROS, длительная непрерывная работа,
срабатывание таймаута 45 секунд и отмена до получения ответа о принятии goal.
Системная установка ROS через apt на этом компьютере не выполнялась.
В перечисленных проверках оставшихся ошибок нет; ограничения модели ниже.

## Ограничения

- Захват — временный fixed constraint после подхода руки; пальцы не реализуют
  физический силовой захват. Целевые зоны плоские, без стенок коробок.
- IK задаёт положения руки; полноценного планирования с обходом препятствий
  и MoveIt 2 пока нет. Сцена рассчитана на свободный стол.
- Confidence=1.0 означает принятую цветовую компоненту, не вероятность модели.
- Координаты камеры фиксированы; TF и автоматическая калибровка пока отсутствуют.
- Таймаут симулятора 45 секунд; отмена проверяется на каждом шаге движения.
- Это учебная симуляция. Для реального робота требуется другой контроллер,
  проверка коллизий, аварийная остановка и калибровка.

## Следующее расширение

Заменять части независимо: HSV → обученный детектор, парсер → LLM с проверяемой
схемой команды, PyBullet action → MoveIt 2/SDK реального робота. VLA потребуется
отдельный интерфейс policy для последовательностей действий и наблюдений;
одного вызова существующего pick/place action для неё недостаточно.

## Быстрая проверка без ROS

После установки Python-зависимостей можно проверить алгоритм отдельно:

```bash
python run_trial.py --command 'red left' --trials 20 --seed 42
python run_trial.py --command 'blue right' --gui
```

Это тот же `world.py` и `vision.py`, но без ROS-транспорта. Результаты пишутся
в `benchmark.jsonl`. Такой запуск не подтверждает исправность ROS action/launch.
