
import serial
import time

PUERTO = "COM3"
BAUDRATE = 115200
DIRECCION = 0x01


def crear_comando(cmd, datos=b""):
    trama = bytes([
        0xA0,
        3 + len(datos),
        DIRECCION,
        cmd
    ]) + datos

    checksum = (-sum(trama)) & 0xFF
    return trama + bytes([checksum])


def recibir_tramas(lector, segundos=1.5):
    buffer = bytearray()
    tramas = []
    inicio = time.monotonic()

    while time.monotonic() - inicio < segundos:
        datos = lector.read(lector.in_waiting or 1)
        buffer.extend(datos)

        while buffer:
            if buffer[0] != 0xA0:
                del buffer[0]
                continue

            if len(buffer) < 2:
                break

            longitud = buffer[1] + 2

            if longitud < 5 or longitud > 257:
                del buffer[0]
                continue

            if len(buffer) < longitud:
                break

            trama = bytes(buffer[:longitud])
            del buffer[:longitud]

            if sum(trama) & 0xFF == 0:
                tramas.append(trama)
            else:
                print("Checksum invalido:",
                      trama.hex(" ").upper())

    return tramas


def enviar(lector, cmd, datos=b"", espera=1):
    comando = crear_comando(cmd, datos)

    lector.reset_input_buffer()

    print("TX:", comando.hex(" ").upper())

    lector.write(comando)
    lector.flush()

    tramas = recibir_tramas(lector, espera)

    for trama in tramas:
        print("RX:", trama.hex(" ").upper())

    if not tramas:
        print("Sin respuesta")

    return tramas


def seleccionar_antena(lector, numero):
    print(f"\nSeleccionando ANT{numero}")

    respuestas = enviar(
        lector,
        0x74,
        bytes([numero - 1])
    )

    for trama in respuestas:
        if trama[3] == 0x74 and len(trama) == 6:
            if trama[4] == 0x10:
                print("Antena seleccionada correctamente")
                return True

            print(f"Error del lector: 0x{trama[4]:02X}")

    return False


def leer_etiquetas(lector):
    print("\nBuscando etiquetas RFID...")

    # Inventario EPC de una ronda
    respuestas = enviar(
        lector,
        0x89,
        bytes([0x01]),
        espera=2
    )

    etiquetas = set()

    for trama in respuestas:
        if trama[3] != 0x89:
            continue

        # Respuesta de error
        if trama[1] == 0x04:
            print(f"Estado del lector: 0x{trama[4]:02X}")
            continue

        # Respuesta de fin de inventario
        if trama[1] == 0x0A:
            print("Inventario finalizado")
            continue

        # Posible respuesta con EPC
        if len(trama) < 10:
            continue

        pc = int.from_bytes(trama[5:7], "big")
        longitud_epc = ((pc >> 11) & 0x1F) * 2

        if not 0 < longitud_epc <= 62:
            continue

        fin_epc = 7 + longitud_epc

        if fin_epc != len(trama) - 2:
            continue

        epc = trama[7:fin_epc].hex().upper()
        etiquetas.add(epc)

    print("\n=== RESULTADO ===")

    if etiquetas:
        for epc in sorted(etiquetas):
            print("Etiqueta EPC:", epc)
    else:
        print("No se detectaron etiquetas EPC")

    print("Etiquetas unicas:", len(etiquetas))

    return etiquetas


def main():
    print("=== TEST RFID CM2000-4 ===")

    numero = int(input(
        "Puerto con antena conectada (1-4): "
    ))

    if numero not in (1, 2, 3, 4):
        print("Puerto invalido")
        return

    confirmar = input(
        f"Confirmas antena fisica en ANT{numero}? (s/n): "
    ).strip().lower()

    if confirmar != "s":
        print("Prueba cancelada")
        return

    lector = serial.Serial()
    lector.port = PUERTO
    lector.baudrate = BAUDRATE
    lector.timeout = 0.1
    lector.dtr = False
    lector.rts = False

    try:
        lector.open()
        time.sleep(3)

        if seleccionar_antena(lector, numero):
            leer_etiquetas(lector)
        else:
            print("No se pudo seleccionar la antena")

    finally:
        if lector.is_open:
            lector.close()


if __name__ == "__main__":
    try:
        main()
    except (ValueError, serial.SerialException) as e:
        print("ERROR:", e)
