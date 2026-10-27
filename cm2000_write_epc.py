"""Preparación offline de EPC según UHFAPI.dll y verificación de lectura A0.

No inicia una GUI. Reutiliza el constructor, decodificador e intérprete existentes.
La DLL usa A5 5A, incompatible con el encapsulado A0 del monitor. La escritura
permanece bloqueada. Ver CM2000_WRITE_EPC.md y UHFAPI_ANALISIS.md.
"""

from __future__ import annotations

import argparse
import re
import sys
import time

import serial

from cm2000_live import (
    BAUDRATE, DIRECCION, COMANDO_ANTENA, COMANDO_INVENTARIO,
    DecodificadorTramas, crear_comando, extraer_lectura,
)


class ErrorRFID(RuntimeError):
    pass


def validar_epc(texto: str) -> bytes:
    """Esta primera prueba acepta exclusivamente EPC de 96 bits."""
    if re.fullmatch(r"[0-9a-fA-F]{24}", texto) is None:
        raise ValueError("El EPC debe tener exactamente 24 caracteres hexadecimales (96 bits).")
    return bytes.fromhex(texto)


def validar_password(texto: str) -> bytes:
    if re.fullmatch(r"[0-9a-fA-F]{8}", texto) is None:
        raise ValueError("La contraseña de acceso debe tener 8 caracteres hexadecimales.")
    return bytes.fromhex(texto)


def construir_bloque_epc(epc: bytes) -> bytes:
    """Algoritmo descrito por el usuario para writeDataToEpc: PC + EPC."""
    if len(epc) != 12:
        raise ValueError("Solo se admite EPC de 12 bytes en esta prueba.")
    pc = (len(epc) // 2) << 11
    return pc.to_bytes(2, "big") + epc


def construir_trama_sdk(actual: bytes, nuevo: bytes, password: bytes) -> bytes:
    """Solo inspección: serialización A5 5A de la DLL analizada, nunca A0.

    El filtro comienza tras CRC y PC (4 bytes). No garantiza unicidad física:
    puede coincidir con etiquetas duplicadas o con EPC más largos de igual prefijo.
    """
    if len(actual) != 12 or len(password) != 4:
        raise ValueError("Se requiere EPC actual de 12 bytes y contraseña de 4 bytes.")
    bloque = construir_bloque_epc(nuevo)
    datos = (
        password + b"\x01"
        + (4 * 8).to_bytes(2, "big")
        + (len(actual) * 8).to_bytes(2, "big") + actual
        + b"\x01\x00\x01"
        + (len(bloque) // 2).to_bytes(2, "big") + bloque
    )
    cuerpo = (len(datos) + 8).to_bytes(2, "big") + b"\x86" + datos
    checksum = 0
    for valor in cuerpo:
        checksum ^= valor
    trama = b"\xA5\x5A" + cuerpo + bytes([checksum]) + b"\r\n"
    # El firmware actual demora cualquier '@' y consume la secuencia @DUMP@\n.
    # Rechazarlo evita además una pausa de 100 ms en mitad de una trama.
    if b"@" in trama:
        raise ValueError("La trama contiene 0x40 ('@'), interceptado por el puente ESP32 actual.")
    return trama


def interpretar_respuesta_sdk(trama: bytes):
    """Decodificación offline de un ACK completo A5 5A; no habilita escritura.

    RecvData espera comando solicitado + 1; UHFWriteData_RecvData acepta 01 00.
    Los significados de los demás estados no están documentados en esta DLL.
    """
    if (len(trama) != 10 or trama[:2] != b"\xA5\x5A"
            or int.from_bytes(trama[2:4], "big") != len(trama)
            or trama[-2:] != b"\r\n" or trama[4] != 0x87):
        raise ErrorRFID("ACK de escritura SDK inesperado: se requiere A5 5A, comando 87 y dos bytes de estado.")
    checksum = 0
    for valor in trama[2:-2]:
        checksum ^= valor
    if checksum:
        raise ErrorRFID("Checksum XOR incorrecto en ACK del SDK.")
    if trama[5:7] != b"\x01\x00":
        raise ErrorRFID(f"Escritura no confirmada por el SDK; estado bruto: {trama[5:7].hex(' ').upper()}.")
    # Este ACK solo confirma la respuesta. Aún haría falta verificar la memoria.


class ConexionLectura:
    """Solo permite comandos ya usados para seleccionar antena e inventariar."""

    def __init__(self, serie):
        self.serie = serie
        self.dec = DecodificadorTramas()

    def enviar(self, comando: int, datos: bytes):
        if comando not in (COMANDO_ANTENA, COMANDO_INVENTARIO):
            raise ErrorRFID("El envío de comandos de escritura no está habilitado.")
        trama = crear_comando(comando, datos)
        print("TX:", trama.hex(" ").upper(), flush=True)
        if self.serie.write(trama) != len(trama):
            raise ErrorRFID("Envío serial incompleto.")
        self.serie.flush()

    def recibir(self):
        datos = self.serie.read(self.serie.in_waiting or 1)
        if datos:
            print("RX bytes:", datos.hex(" ").upper(), flush=True)
        invalidas = self.dec.invalidas
        tramas = self.dec.alimentar(datos)
        if self.dec.invalidas != invalidas:
            raise ErrorRFID("Trama inválida/checksum incorrecto; verificación abortada.")
        for trama in tramas:
            print("RX trama:", trama.hex(" ").upper(), flush=True)
            if trama[2] != DIRECCION:
                raise ErrorRFID("Respuesta de una dirección inesperada.")
        return tramas

    def seleccionar_antena(self, antena: int):
        self.enviar(COMANDO_ANTENA, bytes([antena - 1]))
        limite = time.monotonic() + 2.5
        while time.monotonic() < limite:
            tramas = self.recibir()
            if not tramas:
                continue
            if len(tramas) != 1 or tramas[0] != crear_comando(COMANDO_ANTENA, b"\x10"):
                raise ErrorRFID("Selección de antena rechazada o respuesta inesperada.")
            if self.dec.buffer:
                raise ErrorRFID("Datos adicionales incompletos tras seleccionar antena.")
            return
        raise ErrorRFID("Timeout al seleccionar antena.")

    def inventariar(self, antena: int) -> set[str]:
        self.enviar(COMANDO_INVENTARIO, b"\x01")
        limite = time.monotonic() + 6.0
        etiquetas = set()
        while time.monotonic() < limite:
            terminada = False
            for trama in self.recibir():
                if terminada or trama[3] != COMANDO_INVENTARIO:
                    raise ErrorRFID("Respuesta fuera de la ronda solicitada.")
                if trama[1] == 0x04:
                    raise ErrorRFID(f"Estado/error de inventario: 0x{trama[4]:02X}.")
                if trama[1] in (0x08, 0x0A):
                    terminada = True
                    continue
                lectura = extraer_lectura(trama)
                if lectura is None or lectura.antena != antena:
                    raise ErrorRFID("Lectura no reconocida o de otra antena.")
                etiquetas.add(lectura.epc)
            if terminada:
                if self.dec.buffer:
                    raise ErrorRFID("Trama incompleta al finalizar el inventario.")
                return etiquetas
        raise ErrorRFID("Timeout: no llegó el fin de inventario; no se inicia otra ronda.")


def verificar_epc(puerto: str, antena: int, nuevo: bytes):
    """Comprueba presencia en tres rondas; no certifica una escritura ni identidad TID."""
    serie = serial.Serial()
    serie.port = puerto
    serie.baudrate = BAUDRATE
    serie.timeout = 0.12
    serie.write_timeout = 1.0
    serie.exclusive = True  # Linux: excluye otras aperturas que también usan flock.
    serie.dtr = False
    serie.rts = False
    try:
        serie.open()
        time.sleep(2.5)
        # No inventar un comando STOP. El monitor debe detenerse y cerrarse antes.
        # Esperar más que una ronda conocida y rechazar cualquier tráfico previo.
        limite = time.monotonic() + 6.5
        while time.monotonic() < limite:
            datos = serie.read(serie.in_waiting or 1)
            if datos:
                print("RX previo:", datos.hex(" ").upper(), flush=True)
                raise ErrorRFID("Hay tráfico previo. Detené y cerrá el monitor antes de continuar.")
        conexion = ConexionLectura(serie)
        conexion.seleccionar_antena(antena)
        for ronda in range(1, 4):
            etiquetas = conexion.inventariar(antena)
            print(f"Ronda {ronda}: {', '.join(sorted(etiquetas)) or 'sin etiquetas'}")
            if etiquetas != {nuevo.hex().upper()}:
                raise ErrorRFID("El inventario no contiene exclusivamente el EPC solicitado.")
        print("VERIFICACIÓN DE LECTURA: EPC solicitado observado en tres rondas completas.")
        print("Esto no demuestra unicidad física ni confirma una escritura realizada por este script.")
    finally:
        if serie.is_open:
            serie.close()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--epc-actual", required=True)
    parser.add_argument("--epc-nuevo", required=True)
    modos = parser.add_mutually_exclusive_group()
    modos.add_argument("--simular", action="store_true", help="Preparar sin abrir el puerto (predeterminado).")
    modos.add_argument("--verificar", action="store_true", help="Solo lectura del EPC nuevo.")
    modos.add_argument("--escribir", action="store_true", help="Bloqueado: DLL A5 5A y monitor A0.")
    parser.add_argument("--password", help="Contraseña real de acceso, 8 dígitos hex; sin valor supuesto.")
    parser.add_argument("--unidad-filtro", choices=("bits",), default="bits", help="Bits, confirmado en la DLL.")
    parser.add_argument("--puerto")
    parser.add_argument("--antena", type=int, choices=(1, 2, 3, 4))
    args = parser.parse_args(argv)
    try:
        actual = validar_epc(args.epc_actual)
        nuevo = validar_epc(args.epc_nuevo)
        if args.escribir:
            raise ErrorRFID(
                "ESCRITURA BLOQUEADA: UHFAPI.dll usa A5 5A con XOR, pero el monitor usa A0 "
                "con checksum aditivo. Falta el protocolo de escritura A0 o validar soporte "
                "A5 5A en este lector. No se abrió el puerto ni se envió 0x86."
            )
        if args.verificar:
            if not args.puerto or args.antena is None:
                raise ValueError("Verificar requiere --puerto y --antena.")
            verificar_epc(args.puerto, args.antena, nuevo)
        else:
            if actual == nuevo:
                raise ValueError("El EPC nuevo debe ser distinto del actual para esta prueba.")
            if args.password is None:
                raise ValueError("Simular requiere --password explícito.")
            trama = construir_trama_sdk(actual, nuevo, validar_password(args.password))
            print("SIMULACIÓN OFFLINE: no se abrió el puerto ni se enviaron comandos.")
            print("Protocolo de la DLL: A5 5A, XOR, terminación 0D 0A. No es A0.")
            print("Filtro EPC: dirección 32 bits; longitud 96 bits.")
            print("PC + EPC:", construir_bloque_epc(nuevo).hex(" ").upper())
            print(f"LEN: 0x{len(trama):04X}; escritura: banco 1, palabra 1, 7 palabras.")
            print("Trama SDK offline (incluye contraseña):", trama.hex(" ").upper())
            print("No transmitir: el soporte A5 5A en el lector conectado no fue verificado.")
        return 0
    except (ValueError, ErrorRFID, serial.SerialException, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    except (KeyboardInterrupt, EOFError):
        print("Operación interrumpida; no se confirmó ninguna escritura.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
