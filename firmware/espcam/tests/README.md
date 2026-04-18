# ESP-CAM Tests

## Red Ball Detection Test

Live visualization of red ball detection with centroid tracking, distance estimation, and UART packet monitoring.

### Quick Start

**1. Install dependencies:**
```bash
pip install pyserial websockets
```

**2. Flash firmware:**
```bash
cd /home/admin/Projects/Project_Talos/firmware/espcam
source ~/esp/esp-idf/export.sh
idf.py -p /dev/ttyACM0 flash
```

**3. Run the test bridge:**
```bash
python3 test_red_ball_serial.py --port /dev/ttyACM0 --baud 115200
```

The script will:
- Auto-open your browser to `http://localhost:9000/test_red_ball.html`
- Stream live camera feed with ball detection overlay
- Show centroid, blob size, distance estimates, and raw UART packets

**4. Point the camera at a red ball**

### What You'll See

- **Camera View (left)**: 320×240 feed with crosshair at center, red circle around detected ball
- **Detection Panel**: Whether ball detected, centroid coordinates, blob area, pixel radius
- **Position Panel**: Offset from center, bearing angle
- **Distance Panel**: Estimated distance (pinhole model + TOF sensor)
- **UART Packet**: Raw 11-byte packet sent to main robot controller

### Troubleshooting

**Port already in use:**
```bash
# Use different HTTP port
python3 test_red_ball_serial.py --port /dev/ttyACM0 --http-port 8080
# Then open: http://localhost:8080/test_red_ball.html
```

**Serial port not found:**
```bash
# Find correct port
ls /dev/tty*

# Use it
python3 test_red_ball_serial.py --port /dev/ttyUSB0
```

**No data appearing:**
- Check ESP-CAM is powered and connected
- Verify baud rate matches firmware (default 115200)
- Check that firmware has `TEST_SELECT TEST_RED_BALL` in main.c
