## Drone SDK Та Приклади

Цей документ описує поточну реалізацію `drone_sdk/` і приклади скриптів у `examples/`.

SDK побудований навколо двох підсистем:

- MAVSDK для керування польотом, телеметрії, армінгу, зльоту, посадки та offboard-команд.
- ROS 2 + Gazebo bridges для відеопотоку з камери та керування LED.

## Архітектура Під Час Виконання

Для кожного екземпляра дрона:

1. `Drone.connect()` підключається до PX4 MAVSDK UDP endpoint відповідного дрона.
2. Доступ до камери та LED створюється ліниво, коли викликаються відповідні методи.
3. `BridgeManager` запускає:
   - `ros_gz_image image_bridge` для Gazebo camera topic
   - `ros_gz_bridge parameter_bridge` для topic керування LED
4. `DroneROSNode` підписується на ROS camera topic і публікує LED-команди.
5. Користувач повинен регулярно викликати `drone.spin()`, якщо потрібні кадри з камери.

## Порти Та Топіки

### MAVSDK порти

- Дрон 0: UDP `14540`, gRPC `50051`
- Дрон 1: UDP `14541`, gRPC `50052`
- Дрон 2: UDP `14542`, gRPC `50053`
- Дрон 3: UDP `14543`, gRPC `50054`

SDK обчислює порти так:

- UDP: `14540 + drone_id`
- gRPC: `50051 + drone_id`

### Gazebo / ROS топіки

- Камера: `/world/baylands_custom/model/x500_mono_cam_{id}/link/mono_cam/base_link/sensor/camera_sensor/image`
- LED: `/model/x500_mono_cam_{id}/led_cmd`

## Необхідне Оточення

Типовий запуск:

```bash
source /opt/ros/humble/setup.bash
python3 examples/demo.py
```

Також потрібно:

- Запущений Gazebo world з PX4 SITL дронами
- Доступний ROS 2 Humble
- Встановлений MAVSDK Python для інтерпретатора, який ви використовуєте
- Встановлені `cv_bridge`, OpenCV та пакети `ros_gz_*`

### Встановлення MAVSDK

Встановіть MAVSDK у те саме Python-оточення, з якого ви запускаєте приклади:

```bash
python3 -m pip install mavsdk
```

Перевірка, що пакет доступний:

```bash
python3 -c "import mavsdk; print(mavsdk.__file__)"
```

Якщо ви використовуєте virtual environment або нестандартний інтерпретатор, переконайтесь, що одна й та сама команда `python3` використовується і для встановлення, і для запуску скриптів.

## Файли SDK

### `drone_sdk/__init__.py`

Публічні експорти пакета.

Експортує:

- `Drone`
- `PositionNED`
- `DroneSDKError`
- `ConnectionError`
- `TimeoutError`
- `MAVSDKError`
- `GazeboError`
- `CameraError`
- `LEDError`

Використовуйте цей файл для імпорту з кореня пакета:

```python
from drone_sdk import Drone, MAVSDKError
```

### `drone_sdk/exceptions.py`

Містить невелику ієрархію типізованих винятків.

Класи:

- `DroneSDKError`: базовий виняток SDK
- `ConnectionError`: помилки стану підключення або використання без підключення
- `TimeoutError`: помилки таймауту підключення або health check
- `MAVSDKError`: помилки MAVSDK дій або offboard-команд
- `GazeboError`: зарезервований тип для Gazebo-помилок
- `CameraError`: зарезервований тип для помилок камери
- `LEDError`: зарезервований тип для помилок LED

Поточна поведінка:

- `Drone` активно піднімає `ConnectionError`, `TimeoutError` і `MAVSDKError`.
- Інші типи винятків експортуються для узгодженості, але зараз майже не використовуються.

### `drone_sdk/bridges.py`

Відповідає за запуск і зупинку ROS/Gazebo bridge subprocesses, потрібних для одного дрона.

Основні константи:

- `CAMERA_TOPIC`
- `LED_TOPIC`

Клас:

- `BridgeManager(drone_id)`

Методи:

- `start_camera_bridge()`
  - Запускає `ros2 run ros_gz_image image_bridge <camera_topic>`
  - Нічого не робить, якщо bridge вже запущений
- `start_led_bridge()`
  - Запускає `ros2 run ros_gz_bridge parameter_bridge ...` для LED-повідомлень
  - Нічого не робить, якщо bridge вже запущений
- `stop_all()`
  - Завершує обидва subprocesses, якщо вони існують
  - Чекає до 5 секунд, після чого примусово завершує процес

Примітки реалізації:

- Вивід приглушений через `stdout=subprocess.DEVNULL` і `stderr=subprocess.DEVNULL`.
- Запуск bridge відбувається ліниво через `Drone._ensure_ros()`.

### `drone_sdk/ros_node.py`

Реалізує ROS 2 node, який SDK використовує для отримання зображень з камери та публікації LED-команд.

Клас:

- `DroneROSNode(Node)`

Відповідальність:

- Підписка на bridged Gazebo camera topic
- Перетворення `sensor_msgs/Image` в OpenCV BGR кадри через `CvBridge`
- Збереження останнього кадру в thread-safe буфері
- Публікація LED-команд як `std_msgs/String`

Методи:

- `frame()`
  - Повертає останній кешований кадр `numpy.ndarray` або `None`
- `publish_led(value)`
  - Публікує сирі LED-команди на кшталт `1000`, `OFF` або `BLINK`
- `spin_once()`
  - Виконує один цикл ROS callback з `timeout_sec=0.001`
- `start_spin()` / `stop_spin()`
  - Додаткові helper-методи для фонового spin

Поточне використання:

- Поточні приклади використовують inline spin через `drone.spin()`, а не `start_spin()`.

### `drone_sdk/drone.py`

Це головний інтерфейс SDK.

#### Створення

```python
drone = Drone(drone_id=0)
```

Внутрішній стан:

- MAVSDK `System`
- прапорець стану підключення
- `BridgeManager`
- `DroneROSNode`
- локальний прапорець ініціалізації ROS

#### Внутрішня ініціалізація ROS

`_ensure_ros()`:

- Ініціалізує `rclpy`, якщо потрібно
- Запускає bridge-процеси для камери та LED
- Створює `DroneROSNode`

Вона викликається автоматично методами камери й LED. Напряму її викликати не потрібно.

#### Методи підключення

##### `await connect(timeout=20.0)`

Що робить:

- Створює MAVSDK `System` з gRPC портом відповідного дрона
- Підключається до UDP endpoint відповідного дрона
- Чекає на:
  - стан підключення MAVSDK
  - готовність PX4 global position
  - готовність PX4 home position

Можливі помилки:

- `TimeoutError`, якщо очікування підключення або health перевищує timeout
- `ConnectionError`, якщо дрон не повідомив про підключення або готовність

##### `connected`

Read-only властивість, що повертає поточний прапорець підключення на стороні SDK.

#### Польотні дії

##### `await arm()`

Армить дрон через MAVSDK action API.

##### `await disarm()`

Розармлює дрон.

##### `await takeoff(altitude_m=10.0)`

Встановлює висоту зльоту PX4, після чого запускає зліт.

##### `await land()`

Запускає автономну посадку.

##### `await set_takeoff_altitude(altitude_m)`

Встановлює висоту зльоту PX4 без самого зльоту.

#### Offboard керування

##### `await start_offboard()`

Готує offboard mode, кілька разів надсилаючи setpoint поточної позиції з інтервалом 50 мс, а потім запускає MAVSDK offboard mode.

Навіщо це потрібно:

- PX4 вимагає наявності setpoint перед увімкненням offboard.
- Повторення setpoint дозволяє уникнути помилки `NO_SETPOINT_SET`.

Важлива примітка:

- Поточна реалізація читає поточний heading, але стартовий setpoint відправляє з yaw `0.0`.

##### `await stop_offboard()`

Зупиняє MAVSDK offboard mode.

#### Команди позиції та швидкості

##### `await go_to(north, east, down, yaw_deg=0.0, body_frame=False)`

Надсилає position setpoint через `set_position_ned()`.

Є два режими:

- `body_frame=False`
  - `north`, `east` і `down` є абсолютними NED координатами
- `body_frame=True`
  - `north`, `east` і `down` є зсувами відносно поточної позиції та орієнтації дрона
  - forward/sideways зсуви обертаються відповідно до поточного heading
  - цільова позиція обчислюється як `current_position + rotated_offset`
  - `yaw_deg` перезаписується поточним heading, щоб дрон зберігав поточний yaw

Важливі деталі:

- З `body_frame=True` параметр `down` теж трактується як відносний зсув.
- `go_to()` лише відправляє setpoint; він не чекає досягнення точки.
- У прикладах після кожного `go_to()` зазвичай є `await asyncio.sleep(...)`.

Приклад:

```python
await drone.go_to(10.0, -10.0, 0.0, body_frame=True)
```

Це означає: переміститись на 10 м вперед, 10 м вліво, зберігаючи поточну висоту.

##### `await move(forward, right, down, speed_m_s=5.0, yaw_deg=None)`

Надсилає velocity setpoint через `set_velocity_ned()`.

Поведінка:

- Вхідний вектор інтерпретується у body frame
- Вектор обертається в глобальний NED відповідно до heading дрона
- Вектор нормалізується і масштабується до `speed_m_s`
- Якщо довжина вектора майже нульова, команда стає нульовою швидкістю

Використовуйте `move()`, коли потрібне безперервне керування рухом, особливо у повторюваних control loop.

Приклад:

```python
await drone.move(1.0, 0.0, 0.0, speed_m_s=2.0)
```

Це означає: летіти вперед зі швидкістю 2 м/с.

##### `await set_velocity(north_m_s, east_m_s, down_m_s, yaw_deg=None)`

Напряму надсилає глобальну NED velocity-команду без body-frame обертання.

Використовуйте це, якщо ваш контролер уже працює у світовій системі координат.

#### Телеметрія

##### `await position_ned()`

Повертає tuple `PositionNED(north_m, east_m, down_m)` з наступного зразка MAVSDK `position_velocity_ned()`.

##### `await heading()`

Повертає наступне значення heading у градусах.

#### Керування LED

Ці методи ліниво ініціалізують ROS і bridge-процеси, якщо потрібно.

Методи:

- `set_leds(mask)`
- `led_on()`
- `led_off()`
- `led_blink()`

Відомі формати команд, що використовуються у прикладах:

- Бінарні маски на кшталт `1000`, `0100`, `0010`, `0001`
- `ON`
- `OFF`
- `BLINK`

#### Робота з камерою

##### `start_camera()`

Запускає ROS bridge і node, потрібні для отримання кадрів камери.

##### `spin()`

Обробляє один цикл ROS callback. Це потрібно, якщо ви хочете отримувати нові кадри й не використовуєте окремий background spin thread.

##### `camera_frame()`

Повертає останній кешований кадр або `None`.

##### `stop_camera()`

Зупиняє bridge subprocesses і зупиняє node spin.

Типовий шаблон:

```python
drone.start_camera()
while True:
    drone.spin()
    frame = drone.camera_frame()
```

#### Очищення ресурсів

##### `await close()`

- Зупиняє ресурси камери/bridge
- Скидає стан підключення і MAVSDK handle

Цей метод не викликає `rclpy.shutdown()` для всього процесу. У прикладах це робиться явно у верхньорівневому cleanup.

## Приклади Скриптів

## `examples/demo.py`

Базовий однодроновий приклад, який показує:

- підключення
- показ камери в окремому thread
- фонову задачу анімації LED
- зліт і запуск offboard
- body-relative переміщення через `go_to()`
- посадку, disarm і cleanup

Поточна послідовність руху:

1. Зліт на 10 м
2. Запуск offboard
3. Переміщення на 10 м вперед
4. Переміщення на 10 м вправо
5. Повернення до початку через `go_to(-10, -10, 0, body_frame=True)`

Поведінка LED:

- Працює та сама анімована маска, що і в swarm-скриптах:
  - `1000`
  - `0100`
  - `0010`
  - `0001`
  - `BLINK`

Поведінка камери:

- Використовується окремий thread, який циклічно викликає `drone.spin()` і показує останній кадр через OpenCV.

Запуск:

```bash
source /opt/ros/humble/setup.bash
python3 examples/demo.py
```

## `examples/swarm_velocities.py`

Приклад рою з 4 дронів на основі безперервного керування швидкістю.

Що робить:

- Підключає всі 4 дрони
- Відкриває по одному OpenCV-вікну камери для кожного дрона
- Запускає LED-анімацію на дроні 0
- Армить і підіймає всі дрони без затримки між зльотами
- Для кожного дрона запускає offboard
- Багаторазово надсилає команду летіти вперед через `move()`
- Садить усі дрони після `FLIGHT_DURATION`

Основні константи:

- `SWARM_SIZE = 4`
- `ALTITUDE = 10.0`
- `SPEED = 2.0`
- `FLIGHT_DURATION = 40`

Шаблон керування:

```python
while not stop_event.is_set() and not shutdown.is_set():
    await drone.move(1.0, 0.0, 0.0, speed_m_s=SPEED)
    await asyncio.sleep(0.5)
```

Це постійно повторно надсилає команду forward velocity, доки місія не завершиться.

Поведінка при завершенні:

- зупинка руху через `move(0, 0, 0, speed_m_s=0)`
- зупинка offboard
- посадка
- disarm

Запуск:

```bash
source /opt/ros/humble/setup.bash
python3 examples/swarm_velocities.py
```

## `examples/swarm_waypoints.py`

Приклад рою з 4 дронів на основі відносних waypoint-команд через `go_to()`.

Що робить:

- Підключає всі 4 дрони
- Відкриває по одному OpenCV-вікну камери для кожного дрона
- Запускає LED-анімацію на дроні 0
- Армить і підіймає кожен дрон
- Для кожного дрона запускає offboard
- Надсилає коротку послідовність body-relative кроків по позиції
- Садить дрони після завершення таймера місії або після скасування

Поточна послідовність waypoint для кожного дрона:

1. `go_to(10.0, 0.0, 0.0, body_frame=True)`
2. `go_to(10.0, -10.0, 0.0, body_frame=True)`
3. `go_to(10.0, 10.0, 0.0, body_frame=True)`
4. `go_to(10.0, 0.0, 0.0, body_frame=True)`

У body-relative трактуванні це означає:

1. 10 м вперед
2. 10 м вперед і 10 м вліво
3. 10 м вперед і 10 м вправо
4. 10 м вперед

Деталі керування місією:

- Після кожного `go_to()` є пауза 5 секунд, щоб дрони встигали летіти до нового setpoint.
- Після `FLIGHT_DURATION` головна задача виставляє `stop_event` і скасовує mission tasks.
- Логіка посадки все одно виконується всередині кожної mission coroutine після обробки cancellation.

Запуск:

```bash
source /opt/ros/humble/setup.bash
python3 examples/swarm_waypoints.py
```

## Повторно Використовувані Шаблони З Прикладів

### Один дрон з камерою

Використовуйте це, якщо хочете напряму обробляти кадри:

```python
drone.start_camera()
while True:
    drone.spin()
    frame = drone.camera_frame()
    if frame is not None:
        ...
```

### Безпечний вхід в offboard mode

Рекомендована послідовність:

```python
await drone.arm()
await drone.takeoff(altitude_m=10.0)
await asyncio.sleep(12)
await drone.start_offboard()
```

### Безперервний політ вперед

```python
while running:
    await drone.move(1.0, 0.0, 0.0, speed_m_s=2.0)
    await asyncio.sleep(0.5)
```

### Відносний рух по waypoint

```python
await drone.go_to(10.0, 0.0, 0.0, body_frame=True)
await asyncio.sleep(5)
await drone.go_to(10.0, -10.0, 0.0, body_frame=True)
```

## Поточні Обмеження

- `go_to()` надсилає position setpoint, але не чекає фактичного досягнення точки.
- `body_frame=True` використовує поточний heading у момент команди; якщо yaw дрона зміниться, наступний відносний крок буде базуватись на новому heading.
- `camera_frame()` повертає тільки останній кадр; черги кадрів немає.
- `start_camera()` сам по собі не запускає background spin loop.
- У деяких docstring всередині скриптів ще можуть залишатися старі імена файлів; орієнтуйтесь на актуальні назви з цього документа.

## Рекомендовані Точки Старту

- Використовуйте `examples/demo.py`, щоб перевірити один дрон, одну камеру і один LED stream.
- Використовуйте `examples/swarm_velocities.py`, якщо потрібен простий багатодроновий політ вперед зі стабільним velocity control.
- Використовуйте `examples/swarm_waypoints.py`, якщо потрібні покрокові відносні маршрути через `go_to()`.
