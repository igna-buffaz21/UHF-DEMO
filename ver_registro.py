
import serial
import time

puerto = serial.Serial()
puerto.port = "COM3"
puerto.baudrate = 115200
puerto.timeout = 0.5
puerto.dtr = False
puerto.rts = False

try:
    puerto.open()
    time.sleep(3)

    puerto.reset_input_buffer()

    puerto.write(b"@DUMP@\n")
    puerto.flush()

    respuesta = bytearray()
    inicio = time.monotonic()

    while time.monotonic() - inicio < 4:
        datos = puerto.read(puerto.in_waiting or 1)
        respuesta.extend(datos)

        if b"=== FIN REGISTRO ===" in respuesta:
            break

    print(respuesta.decode("utf-8", errors="replace"))

finally:
    if puerto.is_open:
        puerto.close()
