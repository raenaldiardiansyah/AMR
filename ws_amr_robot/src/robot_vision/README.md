# robot_vision

Kamera Kinect v1 untuk AMR: deteksi objek (YOLO11n), depth, AprilTag checkpoint.
Rencana lengkap: `_dokumentasi_lokal/09_rencana_kamera_kinect.md` (lokal).

## Struktur

| Folder | Isi |
|---|---|
| `robot_vision/` | Logika yang sama untuk laptop dan Pi (deteksi, marker, depth) + node ROS 2 |
| `tools_laptop/` | Script uji di laptop Windows tanpa ROS (webcam / file rekaman) |
| `config/` | Parameter YAML (kamera, checkpoint AprilTag) |
| `launch/` | Launch file ROS 2 |
| `models/` | Bobot model (`.pt`, NCNN). **Tidak di-commit**; unduh/ekspor ulang |

Aturan: logika di `robot_vision/` tidak boleh bergantung pada ROS, supaya bisa diuji di laptop.
Node ROS 2 hanya membungkus logika itu.

## Env laptop (Windows)

Venv di luar repo: `D:\AMR\venv_vision` (Python 3.13).

```
py -3.13 -m venv D:\AMR\venv_vision
D:\AMR\venv_vision\Scripts\python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
D:\AMR\venv_vision\Scripts\python -m pip install -r tools_laptop\requirements-laptop.txt
D:\AMR\venv_vision\Scripts\python -m pip uninstall -y opencv-python
D:\AMR\venv_vision\Scripts\python -m pip install --force-reinstall --no-deps opencv-contrib-python
```

`opencv-python` (ditarik ultralytics) bentrok dengan `opencv-contrib-python`; keduanya memakai nama modul `cv2`,
jadi yang biasa dibuang dan versi contrib dipasang ulang.

## Env Pi 5 (Ubuntu 24.04 + Jazzy)

Belum disiapkan (tahap K1). Butuh `libfreenect-dev`, `ros-jazzy-vision-msgs`, `ros-jazzy-cv-bridge`, `ultralytics`.
