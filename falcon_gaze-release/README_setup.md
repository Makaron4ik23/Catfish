## Нижче приведено інструкцію по розгортанню середовища.

  

### Все нижчеприведене протестовано на Windows 11, WSL2, Ubuntu 22.04

  
  

 - Gazebo Harmonic — фізично коректний симулятор. Використовується в робототехніці та в нашому проекті відповідає за фізику та картинку

 - PX4 Autopilot — автопілот для безпілотних літальних\наземних апаратів. PX4 SITL ми використовуємо для стабілізації дронів та їх керування

 - ROS2 — ситема для програмування та керування роботів. Ця система — зручна надбудова, яку використовуємо для керування всім проектом.

 - MAVSDk — це набір бібліотек що представляють собою API для використання протоколу MAVlink. Ми використовуємо його для безпосереднього керування дронами.

  
  
-------------
## Інсталляція Gazebo Harmonic

*Пайплайн прописаний нижче працює під Windows 11, WSL2, Ubuntu 22.04*

  

#### Встановлення *curl*:

```sh
sudo apt-get update

sudo apt-get install curl lsb-release gnupg
```

#### Встановлення *Gazebo Harmonic*:

```
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update

sudo apt-get install gz-harmonic
```

#### Тестовий запуск, в терміналі треба прописати команду:

```sh
gz sim
```

 Можливі проблеми описані тут:

 https://gazebosim.org/docs/harmonic/troubleshooting/#ubuntu

У WSL часто потрібно вказати дисплей графічного виводу:
```sh
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir
export PULSE_SERVER=/mnt/wslg/PulseServer
```

#### Основна проблема — конфлікт з драйверами Nvidia під WSL2

Були введені команди

```sh
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
```
Допомогла команда

```sh
export GZ_RENDER_ENGINE=vulkan
```
#### Повний гайд по встановленню прописаний в оф. документації:

https://gazebosim.org/docs/harmonic/install_ubuntu/

----------------------------

## Інсталяція PX4

#### Для інсталяції необхідно встановити WSL2 під Windows 11 та Ubuntu 22.04, або треба встановити Ubuntu 22.04 як ОС

Також для початку роботи з PX4 Autopilot треба встановити Gazebo Harmonic

  

#### Нижче описаний процес інсталяції PX4 на Windows 11 під WSL2 на Ubuntu 22.04

У терміналі необхідно ввести команди для клонування гілки з ==github.com== v1.15.4(наразі існує v1.18 але вона працює нестабільно):

```sh
cd ~

git clone --recursive https://github.com/PX4/PX4-Autopilot.git -b v1.15.4
```

```sh
cd PX4-Autopilot/

git fetch --all --tags

  
git checkout v1.15.4

git submodule sync --recursive

git submodule update --init --recursive
```

#### Запуск тестового дрона:

```sh
make px4_sitl gz_x500
```

Детальний гайд по інсталяції можна знайти за посиланням:

https://docs.px4.io/main/en/dev_setup/building_px4

Також можливо:

```bash
sudo apt install python3-pip

pip install --user empy==3.3.4 pyros-genmsg setuptools

pip3 install --user future symforce kconfiglib jinja2 jsonschema
```
-----------------------------------

## Інсталяція ROS2:

*Пайплайн прописаний нижче працює під Windows 11, WSL2, Ubuntu 22.04*

  

#### **Встановлення локалі**

(*встановлення UTF-8 для уникнення конфліктів при використанні ROS2* )(необов'язковий етап*)

locale  # check for UTF-8

```sh
sudo apt update && sudo apt install locales

sudo locale-gen en_US en_US.UTF-8

sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

export LANG=en_US.UTF-8

locale  # verify settings
```

Для коректної роботи треба переконатися, що репозиторій Ubuntu Universe підключено:

```sh
sudo apt install software-properties-common

sudo add-apt-repository universe
```

  

Встановлення пакетів ros-apt-source:

```sh
sudo apt update && sudo apt install curl -y

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')

curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb
```
Оновлення системних  файлів:

```sh
sudo apt update

sudo apt upgrade
```

#### Встановлення ROS2 Humble десктопна версія + інструментарій:

```sh
sudo apt install ros-humble-desktop

sudo apt install ros-dev-tools
```

  

#### Для роботи з ROS2 треба налаштувати середовище, прописавши команду в терміналі:

```sh
source /opt/ros/humble/setup.bash
```

#### Це треба робити при запуску кожного нового термінала, в якому буде запускатися ROS2.

Або ж можна зробити середовище постійним, ввівши команду

```sh
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

#### Переконуємося, що все встановлено коректно запустивши два нових термінали. В одному викличемо C++ talker, ввівши

```sh
source /opt/ros/humble/setup.bash

ros2 run demo_nodes_cpp talker
```

а в другому терміналі викличемо Python listener:

```sh
source /opt/ros/humble/setup.bash

ros2 run demo_nodes_py listener
```

  

#### Очікуваний результат, вікно talker:
```
INFO 1778523022.552205386 talker: Publishing: 'Hello World: 31'

INFO 1778523023.635581916 talker: Publishing: 'Hello World: 32'

INFO 1778523024.718779088 talker: Publishing: 'Hello World: 33'

INFO 1778523025.802341880 talker: Publishing: 'Hello World: 34'

INFO 1778523026.885715043 talker: Publishing: 'Hello World: 35'

INFO 1778523027.969009042 talker: Publishing: 'Hello World: 36'
```

  

#### Вікно listener:
```
INFO 1778523022.554040713 listener: I heard: Hello World: 31

INFO 1778523023.637261358 listener: I heard: Hello World: 32

INFO 1778523024.720011405 listener: I heard: Hello World: 33

INFO 1778523025.804212212 listener: I heard: Hello World: 34

INFO 1778523026.887557605 listener: I heard: Hello World: 35

INFO 1778523027.970950494 listener: I heard: Hello World: 36
```
  

#### В майбутній роботі може знадобитися розширений інструментарій, ознайомитися з ним можна по посиланню:

https://docs.ros.org/en/humble/Installation.html

  
---------------------

## Встановлення MAVSDK

#### Це — бібліотека Python 3 тож для роботи з нею необхідно встановити Python 3:

```sh
sudo apt-get update

sudo apt-get install python3
```
#### Після цього:

```sh
pip3 install mavsdk
```

Посилання де Ви можете знайти опис та приклади використання:

https://mavsdk.mavlink.io/main/en/index.html

та

https://github.com/mavlink/MAVSDK-Python