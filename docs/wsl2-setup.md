# Запуск проекта на Windows через WSL2

Эта инструкция повторяет шаги, которые автоматизированы в `scripts/setup-wsl.sh`,
и нужна для понимания того, что и зачем устанавливается, а также для решения
типичных проблем (сеть, USB, GUI Mujoco).

Целевое окружение:

- Windows 10/11 + WSL2 (≥ 2.0)
- Гость: **Ubuntu 24.04 LTS**
- ROS2 **Kilted** через `pixi-robostack`

---

## 1. Установка WSL2 и Ubuntu 24.04

В PowerShell с правами администратора:

```powershell
wsl --install -d Ubuntu-24.04
```

После установки перезайди и создай пользователя (или используй
автоматизированный путь — см. ниже).

Проверь:

```powershell
wsl -l -v
# должен появиться  Ubuntu-24.04   Running   2
```

### Сеть в WSL не работает (Network is unreachable)

Если внутри Ubuntu `apt update` падает с `Network is unreachable`, чаще всего
виновата активная VPN или Docker Desktop. Лечится переключением WSL2 в
mirrored-режим. На Windows-хосте создай файл `%USERPROFILE%\.wslconfig`:

```ini
[wsl2]
networkingMode=mirrored
dnsTunneling=true
firewall=true
autoProxy=true

[experimental]
hostAddressLoopback=true
```

И перезапусти WSL:

```powershell
wsl --shutdown
```

В mirrored-режиме гость видит сетевые интерфейсы Windows напрямую (включая
VPN), и сеть начинает работать.

---

## 2. Системные зависимости в Ubuntu

```bash
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    ca-certificates curl git build-essential pkg-config \
    libgl1 libglu1-mesa libxrandr2 libxinerama1 libxcursor1 libxi6 \
    libxext6 libx11-6 libxkbcommon0 libegl1 locales

sudo locale-gen en_US.UTF-8
```

Библиотеки `libgl1`/`libxrandr2`/etc нужны Mujoco для GUI; рендер уходит на
WSLg (X-сервер встроен в современный WSL — отдельный VcXsrv не требуется).

---

## 3. Pixi

```bash
curl -fsSL https://pixi.sh/install.sh | bash
echo 'export PATH="$HOME/.pixi/bin:$PATH"' >> ~/.bashrc
exec bash
pixi --version
```

---

## 4. Клонирование проекта

ВАЖНО: клонируй проект в **нативную FS WSL** (`~/projects/...`), а не в
`/mnt/c/...`. Сборка `colcon build` через `/mnt/c/` работает в разы медленнее
из-за оверхеда 9p.

```bash
# rewrite ssh→https, чтобы submodules клонировались без ключей
git config --global url."https://github.com/".insteadOf "git@github.com:"

mkdir -p ~/projects/ogonek-spider
cd ~/projects/ogonek-spider
git clone --recurse-submodules https://github.com/ogonek-spider/ros2.git ros2
```

Если `git clone --recurse-submodules` зависает на одном из submodule (обычно
`mujoco_ros2_control`), убей процесс и подними нужные submodule поодиночке с
shallow clone:

```bash
cd ~/projects/ogonek-spider/ros2
git submodule update --init --depth 1 -- mujoco_ros2_control urdf_parser_py kdl_parser_py
```

---

## 5. twai_proto

Внешняя библиотека, нужна для C++ hardware interface. Кладётся **рядом** с
`ros2/`, в `../libs/lib/twai_proto`:

```bash
mkdir -p ~/projects/ogonek-spider/libs/lib
cd ~/projects/ogonek-spider/libs/lib
git clone https://github.com/ogonek-spider/twaiproto.git twai_proto
```

---

## 6. Pixi-окружение Kilted

Это самый долгий шаг — pixi скачает весь ROS2 Kilted desktop, Mujoco и
зависимости (~3-5 ГБ).

```bash
cd ~/projects/ogonek-spider/ros2/pixi-robostack
pixi install -e kilted
```

---

## 7. Сборка

```bash
cd ~/projects/ogonek-spider/ros2/pixi-robostack
pixi shell -e kilted
cd ..
colcon build --symlink-install
source install/local_setup.bash
```

---

## 8. Запуск

### Симуляция (Mujoco)

```bash
ros2 launch spider_ros_control spider-mujoco.launch.py
```

GUI Mujoco откроется как обычное Windows-окно (через WSLg).

### Реальное железо

Под Linux последовательный порт называется иначе, чем под macOS.
В `spider_ros_control/description/urdf/spider.urdf.xacro` поменяй:

```diff
- <param name="serial_port">/dev/tty.usbmodem101</param>
+ <param name="serial_port">/dev/ttyACM0</param>
```

(или `/dev/ttyUSB0` — зависит от драйвера)

#### Проброс USB в WSL

WSL2 по умолчанию USB не видит. Нужен `usbipd-win`:

1. На Windows-хосте (admin PowerShell):
   ```powershell
   winget install --interactive --exact dorssel.usbipd-win
   ```
2. После перезагрузки:
   ```powershell
   usbipd list                       # увидишь BUSID нужного устройства
   usbipd bind   --busid <BUSID>     # один раз, чтобы пометить
   usbipd attach --wsl --busid <BUSID>
   ```
3. Внутри Ubuntu:
   ```bash
   ls -l /dev/ttyACM*
   sudo usermod -aG dialout $USER     # один раз; затем перелогиниться
   ```

Запуск:

```bash
ros2 launch spider_ros_control spider.launch.py
```

---

## VS Code

Рекомендуется работать через расширение **Remote - WSL**: открой папку
`~/projects/ogonek-spider/ros2` из WSL, и весь IntelliSense/clangd/python будет
видеть pixi-окружение.
