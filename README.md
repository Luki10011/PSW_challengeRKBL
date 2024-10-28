# Pracownia Systemów Wizyjnych 2024 (zima)

Repozytorium zawiera kod do wykorzystania przez studentów w celu realizacji zajęć z Pracowni Systemów Wizyjnych w semestrze zimowym 2024/2025.

## Uruchomienie

### Wymagania wstępne

Repozytorium zostało przygotowane do wykorzystania wewnątrz kontenera [Docker](https://www.docker.com/) działającego pod systemem Ubuntu (preferowana wersja 22.04).

**Uwaga:** System operacyjnym Ubuntu jest wymagany ze względu na obsługę GUI (wymaga innego podejścia przy wykorzystaniu Windowsa).

W celu właściwego przygotowania dockera należy wykonać następujące kroki:

1. [Zainstaluj](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) Docker Engine.
2. [Skonfiguruj](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user) Dockera do wykorzystania jako "non-root user".
3. (Opcjonalnie, ale rekomendowane) [Zainstaluj](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) Nvidia Container Toolkit (bez "experimental packages" oraz bez "rootless mode").

### Budowanie kontenera

W celu zbudowania kontenera wystarczy wywołać skrypt [docker_build.sh](./docker_build.sh):

```
./docker_build.sh
```

Powyższy skrypt buduje obraz zawarty w pliku [Dockerfile](./.devcontainer/Dockerfile).
Wykorzystuje on dwa obrazy bazowe:

1. [Nvidia OpenGL Runtime](https://hub.docker.com/r/nvidia/opengl/tags)
2. [PX4 with ROS Noetic](https://hub.docker.com/r/px4io/px4-dev-ros-noetic)

Pierwszy z nich jest potrzebny do obsługi Gazebo GUI, natomiast drugi dostarcza wszystkich pakietów potrzebnych do uruchomienia PX4 (kontrola BSP), w tym pakietów ROS Noetic.
Na ich podstawie, dostarczony Dockerfile instaluje bibliotekę [Eigen](https://eigen.tuxfamily.org), buduje [PX4 stack](https://github.com/PX4/PX4-Autopilot) oraz instaluje pakiety pomagające w pracy z ROS i buduje przestrzeń roboczą catkin.

### Uruchomienie kontenera

W celu uruchomienia zbudowanego kontenera, wystarczy uruchomić skrypt [docker_run.sh](./docker_run.sh):

```
./docker_run.sh
```

Powyższy skrypt uruchamia kontener lub dołącza się do już uruchomionego, z kilkoma dodatkowymi argumentami (możesz je sprawdzić - większość z nich jest potrzebna do zapewnienia połączenia z GUI pomiędzy Dockerem i Ubuntu).

**Uwaga:** Skrypt montuje również lokalny folder jako folder `src` w przestrzeni roboczej catkin wewnątrz kontenera Docker.
Dzięki temu wszystkie zmiany zrobione lokalnie są widoczne wewnątrz kontenera (i odwrotnie), co powinno być użyteczne podczas rozwoju oprogramowania.

Po zakończeniu skryptu, terminal zostaje automatycznie dołączony do kontenera.
Przed uruchomieniem czegokolwiek, musisz przebudować przestrzeń roboczą catkin (żeby dołączyć nowy pakiet:)

```
cd ~/sim_ws
catkin build
```

Następnie musisz zaktualizować pewne ścieżki za pomocą skryptu [environment_vars.sh]:

```
. ~/sim_ws/src/psw_challenge/environment_vars.sh
```

**Uwaga:** Pamiętaj o kropce przed ścieżką.

## Challenge

Challenge może zostać uruchomiony za pomocą skryptu [challenge.launch](./launch/challenge.launch):

```
roslaunch psw_challenge challenge.launch
```

Skrypt uruchamia symulator Gazebo, dodaje model UAV Iris i uruchamia node (`iris_node`).
Możesz komunikować się z nim za pomocą dwóch topiców:

- `/iris_control/pose` ([std_msgs/Bool](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Bool.html)) - node publikuje obecne położenie drona (pozycja + orientacja),
- `/iris_control/cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)) - node subskrybuje, aby odczytywać prędkościowe komendy sterujące.

Oprócz tego, model Iris UAV zapewnia strumień wideo z kamery zamontowanej na dronie:

- `/iris/usb_cam/camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html)) - parametry kamery,
- `/iris/usb_cam/image_raw` ([sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)) - obraz RGB.


### Opis toru

Tor, przez który należy przelecieć, składa się z jednakowych bramek oznakowanych markerami ArUco.
Znaczniki zostały przygotowane z wykorzystaniem [generatora](https://chev.me/arucogen/) ze słownika 4x4(50, 100, 250, 1000).
Markery kodują kolejne liczby 1, 2, ..., 5, które oznaczają numery bramek (należy przez nie przelecieć w zadanej kolejności).