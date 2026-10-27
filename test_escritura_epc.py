"""Prueba controlada de escritura EPC de 96 bits por protocolo A0.

Requiere Python 3 + pyserial, firmware ESP32 como puente USB <-> UART2,
antena adecuada y una SOLA etiqueta regrabable en el campo de lectura.

Protocolo A0 documentado para lectores UHF compatibles. La escritura
0x82 no esta aun validada en el firmware particular del CM2000-4.
"""
import time
import serial

PUERTO = "COM3"
BAUDRATE = 115200
DIRECCION = 0x01
EXITO = 0x10


def crear_comando(cmd, datos=b""):
    if 3 + len(datos) > 255:
        raise ValueError("Datos demasiado largos para el protocolo")
    trama = bytes([0xA0, 3 + len(datos), DIRECCION, cmd]) + datos
    return trama + bytes([(-sum(trama)) & 0xFF])


def recibir_tramas(lector, segundos=1.5):
    buffer = bytearray()
    tramas = []
    inicio = time.monotonic()
    while time.monotonic() - inicio < segundos:
        buffer.extend(lector.read(lector.in_waiting or 1))
        while buffer:
            if buffer[0] != 0xA0:
                del buffer[0]
                continue
            if len(buffer) < 2:
                break
            total = buffer[1] + 2
            if total < 5 or total > 257:
                del buffer[0]
                continue
            if len(buffer) < total:
                break
            trama = bytes(buffer[:total])
            if sum(trama) & 0xFF:
                print("  Trama descartada por checksum:", trama.hex(' ').upper())
                del buffer[0]
                continue
            del buffer[:total]
            if trama[2] == DIRECCION:
                tramas.append(trama)
    return tramas


def enviar(lector, cmd, datos=b"", espera=1.0):
    trama = crear_comando(cmd, datos)
    lector.reset_input_buffer()
    print("TX:", trama.hex(" ").upper())
    lector.write(trama)
    lector.flush()
    respuestas = recibir_tramas(lector, espera)
    if not respuestas:
        print("RX: sin respuesta")
    for r in respuestas:
        print("RX:", r.hex(" ").upper())
    return [r for r in respuestas if r[3] == cmd]


def confirmar_exito(respuestas, cmd):
    return any(r[1] == 0x04 and r[3] == cmd and r[4] == EXITO
               for r in respuestas)


def seleccionar_antena(lector, numero):
    if numero not in range(1, 5):
        raise ValueError("La antena debe estar entre 1 y 4")
    res = enviar(lector, 0x74, bytes([numero - 1]))
    if not confirmar_exito(res, 0x74):
        raise RuntimeError("El lector no confirmo seleccion de antena")


def quitar_filtro(lector):
    # Mode=1 -> desactivar; EpcLen=0; sin datos EPC
    res = enviar(lector, 0x85, b"\x01\x00")
    if not confirmar_exito(res, 0x85):
        raise RuntimeError("No se pudo limpiar el filtro EPC")


def consultar_filtro(lector):
    res = enviar(lector, 0x86)
    estados = [r for r in res if len(r) >= 6]
    if not estados:
        raise RuntimeError("No se obtuvo estado del filtro EPC")
    r = estados[0]
    estado = r[4]
    if estado == 0x01:
        return None
    if estado != 0x00 or len(r) < 7:
        raise RuntimeError(f"Estado del filtro inesperado: 0x{estado:02X}")
    n = r[5]
    if len(r) != n + 7:  # A0 LEN ADR CMD MODE EPC_LEN EPC CHECK
        raise RuntimeError("Respuesta de filtro con longitud inesperada")
    return r[6:-1]


def fijar_filtro(lector, epc):
    res = enviar(lector, 0x85, bytes([0x00, len(epc)]) + epc)
    if not confirmar_exito(res, 0x85):
        raise RuntimeError("No se pudo activar filtro por EPC")
    epc_leido = consultar_filtro(lector)
    if epc_leido != epc:
        raise RuntimeError("El filtro consultado no coincide con el EPC original")


def inventariar(lector, espera=2.0):
    respuestas = enviar(lector, 0x89, b"\x01", espera=espera)
    epcs = set()
    for r in respuestas:
        # El error tiene Len=4; resumen de inventario suele tener Len=8.
        if r[1] in (0x04, 0x08):
            continue
        if len(r) < 10:
            continue
        pc = int.from_bytes(r[5:7], 'big')
        epc_len = ((pc >> 11) & 0x1F) * 2
        final = 7 + epc_len
        if 0 < epc_len <= 62 and final == len(r) - 2:
            epcs.add(r[7:final].hex().upper())
    return epcs


def escribir_epc(lector, nuevo_epc, contrasena):
    # MemBank 0x01=EPC; WordAdd 0x02, WordCnt 0x06 para 96 bits.
    parametros = contrasena + bytes([0x01, 0x02, len(nuevo_epc) // 2]) + nuevo_epc
    return enviar(lector, 0x82, parametros, espera=2.5)


def describir_respuesta_escritura(respuestas):
    if not respuestas:
        print("No se recibio confirmacion de escritura; se verificara con inventario.")
        return
    for r in respuestas:
        if r[1] == 0x04:
            print(f"Respuesta corta de escritura, codigo 0x{r[4]:02X} (no se asume exito).")
        elif len(r) >= 11:
            n_datos = r[6]
            offset_error = 7 + n_datos
            if offset_error + 3 < len(r):
                cantidad = int.from_bytes(r[4:6], 'big')
                estado = r[offset_error]
                writes = r[offset_error + 2]
                print(f"Reporte escritura: tags={cantidad}, estado=0x{estado:02X}, escrituras={writes}")
                continue
            print("Respuesta 0x82 con campos no esperados; se verificara mediante lectura.")


def epc96(prompt):
    s = input(prompt).strip().replace(' ', '').upper()
    if len(s) != 24:
        raise ValueError("Para esta primera prueba los EPC deben tener 24 caracteres hex (96 bits)")
    try:
        return bytes.fromhex(s)
    except ValueError as e:
        raise ValueError("EPC debe tener solo caracteres hexadecimales") from e


def main():
    print("=== ESCRITURA CONTROLADA DE EPC / CM2000-4 ===")
    print("ATENCION: usar una sola etiqueta propia y regrabable, sin otras etiquetas cerca.")
    print("El modulo debe tener una antena UHF apropiada conectada.")
    ant = int(input("Antena conectada (1-4): ").strip())
    actual = epc96("EPC actual (24 caracteres): ")
    nuevo = epc96("Nuevo EPC  (24 caracteres): ")
    if actual == nuevo:
        raise ValueError("Los EPC son iguales; no hay nada para escribir")
    # Contraseña de acceso por defecto para etiquetas sin contraseña configurada.
    pwd_hex = input("Access password (8 hex, Enter para 00000000): ").strip() or '00000000'
    if len(pwd_hex) != 8:
        raise ValueError("Access password debe tener 8 caracteres hexadecimales")
    pwd = bytes.fromhex(pwd_hex)
    print(f"\nVas a reemplazar: {actual.hex().upper()} -> {nuevo.hex().upper()}")
    print("El cambio es persistente en la memoria de la etiqueta.")
    if input("Escribi ESCRIBIR para confirmar: ").strip() != 'ESCRIBIR':
        print("Cancelado sin modificar la etiqueta.")
        return

    puerto = serial.Serial()
    puerto.port = PUERTO
    puerto.baudrate = BAUDRATE
    puerto.timeout = 0.1
    puerto.dtr = False
    puerto.rts = False
    abierto = False
    intento_escritura = False
    limpieza_final_confirmada = False
    try:
        puerto.open()
        abierto = True
        time.sleep(3)
        seleccionar_antena(puerto, ant)
        # Evitar que un filtro de una prueba anterior limite el inventario.
        print("\nLimpiando filtros anteriores...")
        quitar_filtro(puerto)
        print("\nVerificando la unica etiqueta presente...")
        presentes = inventariar(puerto)
        print("EPC detectados:", presentes)
        if presentes != {actual.hex().upper()}:
            raise RuntimeError("No hay exactamente una etiqueta con el EPC esperado. NO se escribio.")
        print("\nActivando filtro por el EPC original...")
        fijar_filtro(puerto, actual)
        print("\nEnviando una unica escritura (sin reintentos automaticos)...")
        intento_escritura = True
        respuestas = escribir_epc(puerto, nuevo, pwd)
        describir_respuesta_escritura(respuestas)
    finally:
        if abierto:
            try:
                print("\nQuitando filtro EPC...")
                quitar_filtro(puerto)
                limpieza_final_confirmada = True
            except Exception as e:
                print("ADVERTENCIA: no se confirmo limpieza de filtro:", e)
            if intento_escritura and limpieza_final_confirmada:
                try:
                    print("\nVerificando EPC mediante nuevo inventario...")
                    encontrados = inventariar(puerto, espera=2.0)
                    print("EPC detectados:", encontrados)
                    if encontrados == {nuevo.hex().upper()}:
                        print("EXITO VERIFICADO: solo se detecta el EPC nuevo")
                    else:
                        print("NO CONFIRMADO: revisar EPC y memoria antes de intentar otra escritura")
                except Exception as e:
                    print("No se pudo completar la verificacion:", e)
            if intento_escritura and not limpieza_final_confirmada:
                print("NO CONFIRMADO: se requiere limpiar el filtro antes de comprobar el EPC")
            puerto.close()


if __name__ == '__main__':
    try:
        main()
    except (ValueError, RuntimeError, serial.SerialException) as err:
        print("ERROR:", err)
