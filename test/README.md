# Тестова підсистема falcon_gaze

## Призначення

Тестовий фреймворк для симуляції рою дронів (4× x500_mono_cam) у Gazebo + PX4 SITL: польотна місія, логування позицій та LED-станів, інтерактивний аналіз траєкторій, DTW-вирівнювання та перевірка проходження шляхових точок.

## Структура

```
test/
├── helper_drones.py           # Місія ведених (drones 1-3) — приклад, замініть на власний
├── logger_node.py             # ROS2-вузол логування (Gazebo pose + LED)
├── run_logged_test.sh         # Скрипт запуску повного тесту
│
├── analysis/
│   ├── analyze_flight.py      # Інтерактивний 3D-плеєр + DTW + метрики
│   ├── convert_mission_to_target.py  # Конвертація mission_XX.json у target waypoints
│   └── config/
│       └── target_waypoints_mission_01.json  # Еталонні WP з mission_01.json
│
├── logs/                      # Логи польотів (JSON)
│
└── results/                   # Збережені метрики (JSON)
```

## Як це працює

1. **`run_logged_test.sh`** запускає `logger_node.py` і польотні скрипти. Лідер (drone 0) виконує місію через `resources/scripts/Mission/mission_launch.py` з файлом `mission_01.json`. `resources/scripts/Mission/` — це **приклад** директорії з місіями, замініть на власну.
2. `helper_drones.py` — це **приклад** скрипта для ведених дронів. Замініть його на власний, який керує drones 1-3 через `drone_sdk` або MAVSDK.
3. `logger_node.py` читає позиції з Gazebo через `gz topic -e /world/…/pose/info` та LED-стани через ROS2-топіки; зберігає JSON при завершенні. Логування починається автоматично, коли drone 0 залишає стартову платформу.
4. **`analyze_flight.py`** завантажує лог і будує інтерактивний 3D-графік (+ слайдер часу), виконує DTW-аналіз для ведених дронів, перевіряє проходження шляхових точок і зберігає звіт метрик у `test/results/`.

## Еталонні файли

- `target_waypoints_mission_01.json` — абсолютні ENU-координати 9 WP з `mission_01.json`.
- `convert_mission_to_target.py` — конвертує `mission_XX.json` (Gazebo world coordinates) у формат цільових waypoints для аналізу.

## Використання

```bash
# Повний тест (замініть helper_drones.py на власний скрипт)
./test/run_logged_test.sh

# Аналіз останнього логу
python3 test/analysis/analyze_flight.py --latest

# Аналіз з перевіркою WP
python3 test/analysis/analyze_flight.py --latest --target test/analysis/config/target_waypoints_mission_01.json

# Конвертація mission JSON у target waypoints
python3 test/analysis/convert_mission_to_target.py resources/scripts/Mission/mission_01.json -o test/analysis/config/my_targets.json

# Запуск місії лідера окремо
python3 resources/scripts/Mission/mission_launch.py resources/scripts/Mission/mission_01.json
```
