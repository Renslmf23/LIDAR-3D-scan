from pyrplidar import PyRPlidar

def disconnect():
    lidar = PyRPlidar()
    lidar.connect(port="/dev/ttyUSB0", baudrate=256000, timeout=3)
    # Linux   : "/dev/ttyUSB0"
    # MacOS   : "/dev/cu.SLAB_USBtoUART"
    # Windows : "COM5"


    lidar.stop()
    lidar.set_motor_pwm(0)

    lidar.disconnect()

disconnect()