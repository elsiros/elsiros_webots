# Установка на ubuntu

## Installing dependencies

1. Install GameController - [https://github.com/elsiros/GameController](https://github.com/elsiros/GameController)
2. Install Webots R2021b - [https://cyberbotics.com/](https://cyberbotics.com/)

## Installing elsiros

### Clone code

```bash
git clone https://github.com/elsiros/elsiros_webots
```

### Install libs from requirements.txt

```bash
cd /path/to/elsiros_webots
pip install -r requirements.txt
```

### Install protobuf-compiler

```bash
sudo apt-get install libprotobuf-dev protobuf-compiler
```

### Build controller and protobuf messages

```bash
cd /path/to/elsiros_webots/controllers/player
make
```

### Test player controller

```bash
    cd /path/to/webots
    ./webots /path/to/elsiros_webots/worlds/elsiros_training.wbt
```

```bash
    cd /path/to/elsiros_webots/controllers/player
    python communication_manager_test.py
```
