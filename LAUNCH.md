# Команди запуску тестового сценарію (Mission/mission_01.json)

Перевірено для портативності: усі шляхи відносні до структури репозиторію
`Catfish/` (`falcon_gaze-release/` + `Mission/` як сусідні теки). Працює на
будь-якому ПК з встановленим стеком ROS 2 Humble + Gazebo Harmonic + PX4
v1.15.4 (`README_setup.md`).

## 0. Одноразово на новій машині

```sh
cd ~/Catfish/falcon_gaze-release
./project_setup.sh                # копіює world/model/LED-плагін у PX4-Autopilot
```

## 1. Термінал №1 — Gazebo + PX4 SITL (рій лідер+3 ведені)

```sh
cd ~/Catfish/falcon_gaze-release
source /opt/ros/humble/setup.bash
source resources/scripts/px4_gz_setup.sh
ros2 launch resources/scripts/swarn_launch.py
```

Дочекайтесь, поки Gazebo завантажить світ і всі 4 PX4 інстанси (`-i 0..3`)
вийдуть на стабільний стан (лог `INFO  [px4] startup script returned successfully`).

## 2. Термінал №2 — лідер + ведені (тестовий сценарій)

```sh
cd ~/Catfish/falcon_gaze-release
source /opt/ros/humble/setup.bash
./run_all.sh
```

`run_all.sh` запускає:
1. `examples/follower_mission.py` — дрони 1,2,3 у гуськ: 1→лідер(0), 2→1, 3→2.
2. `../Mission/mission_launch.py ../Mission/mission_01.json --speed 3` —
   лідер летить за конвертованими px4_ned вейпоінтами з тестової сцени і
   транслює LED FOLLOW (ON) під час руху, LED OFF (FINISH/LOST) на фініші.

Зупинка: `Ctrl+C` у терміналі №2 (трап коректно завершує обидва процеси).

## 3. Запуск лідера/сценарію окремо (без followers)

```sh
cd ~/Catfish/falcon_gaze-release
python3 ../Mission/mission_launch.py ../Mission/mission_01.json --speed 3 --yaw-mode path
```

Параметри:
- `--speed <m/s>` — швидкість між вейпоінтами (за замовчуванням з `mission_01.json`, якщо є `speed_to_next_mps`).
- `--yaw-mode file|current|path` — джерело курсу (за замовч. `current` = висота взльоту).
- `--connection udpin://0.0.0.0:14540` — адреса лідера (drone_id=0), змінювати не потрібно для стандартного стенду.

## 4. Перевірка камер/LED (опційно, debug)

```sh
SHOW_CAMERA=1 python3 examples/follower_mission.py
```

## Примітки щодо переносимості

- Усі скрипти використовують відносні шляхи (`sys.path.insert` від `__file__`),
  тож структуру `Catfish/falcon_gaze-release` + `Catfish/Mission` можна
  переносити як єдине дерево без правок коду.
- `WORLD_NAME` (за замовч. `baylands_custom`) можна перевизначити через env,
  якщо тестова сцена має іншу назву world.
- Якщо PX4-Autopilot встановлено не в `~/PX4-Autopilot`, відредагуйте
  `PX4_DIR` на початку `resources/scripts/px4_gz_setup.sh` і `project_setup.sh`.
