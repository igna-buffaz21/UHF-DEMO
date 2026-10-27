"""CM2000 Live — monitor EPC para CM2000-4 + ESP32 por puerto COM.

Requisitos: Python 3.9+, pyserial. GUI incluida en Python (tkinter).
Este programa usa únicamente el protocolo A0 que ya funcionó con el lector.
Permite consultar y configurar la potencia de ANT1-ANT4 desde un control explícito.
La escritura de potencia es persistente; no se realiza mientras se arrastra el control.
La escritura EPC de 96 bits modifica solo Words 2–7, uno por uno, con inventarios
de verificación y sin reintentos automáticos de escritura.
Consulta opcional de temperatura interna 0x7B entre rondas; su compatibilidad
con el CM2000-4 debe verificarse con respuestas del lector físico.
"""

from __future__ import annotations

import csv
import math
import queue
import re
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from tkinter import filedialog, messagebox
import tkinter as tk
from tkinter import ttk

import serial
from serial.tools import list_ports

BAUDRATE = 115200
DIRECCION = 0x01
COMANDO_ANTENA = 0x74
COMANDO_INVENTARIO = 0x89
COMANDO_LEER_POTENCIA = 0x77
COMANDO_FIJAR_POTENCIA = 0x76
COMANDO_FILTRO = 0x85
COMANDO_CONSULTAR_FILTRO = 0x86
COMANDO_ESCRIBIR = 0x82
COMANDO_TEMPERATURA = 0x7B
POTENCIA_MIN = 5
POTENCIA_MAX = 30  # Límite conservador de la interfaz; el protocolo puede soportar otros valores.

BG = "#0b1220"
PANEL = "#121d30"
PANEL_ALT = "#18263c"
BORDER = "#263951"
TEXT = "#eef4fc"
MUTED = "#91a4be"
ACCENT = "#35c7ac"
ACCENT_DARK = "#193e42"
ERROR = "#fa929b"


def crear_comando(comando: int, datos: bytes = b"") -> bytes:
    if 3 + len(datos) > 255:
        raise ValueError("Datos demasiado largos para el protocolo A0.")
    cuerpo = bytes([0xA0, 3 + len(datos), DIRECCION, comando]) + datos
    return cuerpo + bytes([(-sum(cuerpo)) & 0xFF])


def abrir_serie(puerto):
    serie = serial.Serial()
    serie.port = puerto
    serie.baudrate = BAUDRATE
    serie.timeout = 0.12
    serie.write_timeout = 1.0
    serie.dtr = False
    serie.rts = False
    serie.open()
    return serie


def validar_hex(valor, longitud, nombre):
    if not re.fullmatch(rf"[0-9a-fA-F]{{{longitud}}}", valor):
        raise ValueError(f"{nombre}: se requieren exactamente {longitud} caracteres hexadecimales.")
    return valor.upper()


def palabras_modificadas(actual, nuevo):
    actual = validar_hex(actual, 24, "EPC actual")
    nuevo = validar_hex(nuevo, 24, "Nuevo EPC")
    return [(2 + i, bytes.fromhex(nuevo[i*4:i*4+4]))
            for i in range(6) if actual[i*4:i*4+4] != nuevo[i*4:i*4+4]]


def estado_escritura(trama):
    """El estado largo sigue a DataLen bytes; el contador no determina éxito."""
    if trama is None or len(trama) < 6 or trama[3] != COMANDO_ESCRIBIR:
        return None
    if len(trama) == 6:
        return trama[4]
    if len(trama) >= 11:
        offset = 7 + trama[6]
        if offset + 4 == len(trama):
            return trama[offset]
    return None


def diagnostico_trama(trama, escritura_tx=False):
    # Un puente con eco también podría devolver el propio comando 0x82.
    if trama[3] == COMANDO_ESCRIBIR and (escritura_tx or len(trama) == 14):
        return (trama[:4].hex(" ").upper() + " ** ** ** ** "
                + trama[8:-1].hex(" ").upper() + " **")
    return trama.hex(" ").upper()


class DecodificadorTramas:
    """Recibe bloques UART arbitrarios y devuelve solo tramas A0 completas/válidas."""

    def __init__(self):
        self.buffer = bytearray()
        self.invalidas = 0

    def alimentar(self, datos: bytes) -> list[bytes]:
        self.buffer.extend(datos)
        resultado = []
        while self.buffer:
            pos = self.buffer.find(b"\xA0")
            if pos < 0:
                self.buffer.clear()
                break
            if pos:
                del self.buffer[:pos]
            if len(self.buffer) < 2:
                break
            longitud = self.buffer[1] + 2
            if longitud < 5 or longitud > 257:
                del self.buffer[0]
                self.invalidas += 1
                continue
            if len(self.buffer) < longitud:
                break
            trama = bytes(self.buffer[:longitud])
            if sum(trama) & 0xFF:
                # Ante checksum incorrecto, re-sincronizamos en vez de descartar
                # una trama entera que podría contener una cabecera válida.
                del self.buffer[0]
                self.invalidas += 1
                continue
            del self.buffer[:longitud]
            resultado.append(trama)
        return resultado



def interpretar_potencias(trama: bytes) -> tuple[int, int, int, int] | None:
    """0x77 devuelve una potencia común (1 byte) o las 4 (4 bytes)."""
    if len(trama) == 6 and trama[1] == 0x04 and trama[3] == COMANDO_LEER_POTENCIA:
        if trama[4] <= 33:
            return (trama[4],) * 4
    if len(trama) == 9 and trama[1] == 0x07 and trama[3] == COMANDO_LEER_POTENCIA:
        valores = tuple(trama[4:8])
        if len(valores) == 4 and all(v <= 33 for v in valores):
            return valores
    return None


class ErrorTemperatura(RuntimeError):
    def __init__(self, estado, mensaje):
        super().__init__(mensaje)
        self.estado = estado


def interpretar_temperatura(trama: bytes) -> int:
    """Formato solicitado: PlusMinus 0=positivo, 1=negativo; nunca inferir °C."""
    if (len(trama) < 5 or trama[0] != 0xA0 or len(trama) != trama[1] + 2
            or trama[2] != DIRECCION or trama[3] != COMANDO_TEMPERATURA
            or sum(trama) & 0xFF):
        raise ErrorTemperatura("Respuesta inválida", "Trama 0x7B inválida (cabecera, longitud, dirección, comando o checksum).")
    if trama[1] == 4:
        raise ErrorTemperatura("No compatible / consulta rechazada",
                               f"El lector devolvió error 0x{trama[4]:02X}; no es una temperatura. "
                               "Compatibilidad de 0x7B no confirmada.")
    if len(trama) != 7 or trama[1] != 5 or trama[4] not in (0, 1):
        raise ErrorTemperatura("Respuesta inválida", "Formato o signo de temperatura no reconocido.")
    return -trama[5] if trama[4] == 1 else trama[5]


class TrabajadorPotencia(threading.Thread):
    """Consulta o modifica la potencia con el puerto serial reservado en exclusiva.

    Al aplicar, lee primero las cuatro potencias actuales, cambia únicamente
    la seleccionada y luego consulta nuevamente para verificar el resultado.
    """

    def __init__(self, puerto: str, operacion: str, antena: int,
                 potencia: int, eventos: queue.Queue):
        super().__init__(daemon=True)
        self.puerto = puerto
        self.operacion = operacion
        self.antena = antena
        self.potencia = potencia
        self.eventos = eventos

    def _evento(self, tipo: str, dato):
        self.eventos.put((tipo, dato))

    def _enviar_y_esperar(self, serie, dec, comando, datos=b"", segundos=2.5):
        respuestas = OperacionesRFID(serie, self._evento).enviar(comando, datos, segundos)
        return respuestas[0] if respuestas else None

    def _consultar(self, serie, dec):
        respuesta = self._enviar_y_esperar(serie, dec, COMANDO_LEER_POTENCIA)
        if respuesta is None:
            raise RuntimeError("Sin respuesta del comando 0x77 (consultar potencia).")
        potencias = interpretar_potencias(respuesta)
        if potencias is None:
            raise RuntimeError(f"Respuesta de potencia inesperada: {respuesta.hex(' ').upper()}")
        self._evento("power_values", potencias)
        return potencias

    def run(self):
        try:
            serie = abrir_serie(self.puerto)
            try:
                # En algunas placas el ESP32 se reinicia al abrir el USB/COM.
                time.sleep(2.5)
                dec = DecodificadorTramas()
                actuales = self._consultar(serie, dec)
                if self.operacion == "consultar":
                    self._evento("info", "Potencias consultadas correctamente.")
                    return

                if self.operacion != "aplicar":
                    raise RuntimeError("Operación de potencia desconocida.")
                if not POTENCIA_MIN <= self.potencia <= POTENCIA_MAX:
                    raise RuntimeError("La potencia seleccionada está fuera del rango de la interfaz.")
                if self.antena not in (1, 2, 3, 4):
                    raise RuntimeError("Antena no válida.")

                nuevas = list(actuales)
                nuevas[self.antena - 1] = self.potencia
                if tuple(nuevas) == actuales:
                    self._evento("info", f"ANT{self.antena} ya está en {self.potencia} dBm. No se escribió memoria.")
                    self._evento("power_applied", (self.antena, self.potencia))
                    return

                # Se usa la forma de 4 parámetros (ANT1, ANT2, ANT3, ANT4)
                # que implementa el programa Arduino original del usuario.
                respuesta = self._enviar_y_esperar(
                    serie, dec, COMANDO_FIJAR_POTENCIA, bytes(nuevas), segundos=3.0
                )
                if respuesta is None:
                    raise RuntimeError("No hubo confirmación del comando 0x76. No asumir que se aplicó.")
                if len(respuesta) != 6 or respuesta[1] != 0x04 or respuesta[4] != 0x10:
                    raise RuntimeError(f"El lector rechazó la configuración: {respuesta.hex(' ').upper()}")

                time.sleep(0.2)
                verificado = self._consultar(serie, dec)
                if verificado != tuple(nuevas):
                    raise RuntimeError(
                        f"El lector confirmó 0x76 pero la lectura posterior no coincide: "
                        f"se esperaba {nuevas}, respondió {list(verificado)}."
                    )
                self._evento("power_applied", (self.antena, self.potencia))
            finally:
                if serie.is_open:
                    serie.close()
        except (serial.SerialException, OSError, ValueError, RuntimeError) as exc:
            self._evento("power_error", str(exc))
        finally:
            self._evento("power_done", None)


@dataclass(frozen=True)
class Lectura:
    epc: str
    antena: int
    rssi_bruto: int


def extraer_lectura(trama: bytes) -> Lectura | None:
    """0x89: A0 LEN ADDR 89 FreqAnt PC(2) EPC(n) RSSI CHECK."""
    if len(trama) < 11 or trama[3] != COMANDO_INVENTARIO:
        return None
    if trama[1] in (4, 8, 10):  # Estado/error o resumen de inventario
        return None
    pc = int.from_bytes(trama[5:7], "big")
    largo_epc = ((pc >> 11) & 0x1F) * 2
    if not 0 < largo_epc <= 62:
        return None
    fin = 7 + largo_epc
    if fin != len(trama) - 2:
        return None
    # FreqAnt utiliza sus 2 bits bajos para la antena (00=ANT1, 03=ANT4).
    return Lectura(
        epc=trama[7:fin].hex().upper(),
        antena=(trama[4] & 0x03) + 1,
        rssi_bruto=trama[-2],
    )


class TrabajadorRFID(threading.Thread):
    """El puerto COM solo se abre y usa desde este hilo."""

    def __init__(self, puerto: str, antena: int, pausa_ms: int,
                 eventos: queue.Queue, detener: threading.Event):
        super().__init__(daemon=True)
        self.puerto = puerto
        self.antena = antena
        self.pausa_ms = pausa_ms
        self.eventos = eventos
        self.detener = detener
        # Se entrega al escritor solo después de terminar la ronda y salir del hilo.
        self.transferir = False
        self.serie_transferida = None
        self.transferencia_segura = True
        self.pedir_temperatura = threading.Event()

    def _evento(self, tipo: str, dato):
        self.eventos.put((tipo, dato))

    def _esperar_respuesta_antena(self, serie, dec: DecodificadorTramas):
        limite = time.monotonic() + 2.5
        while time.monotonic() < limite and not self.detener.is_set():
            paquete = serie.read(serie.in_waiting or 1)
            for trama in dec.alimentar(paquete):
                if trama[2] != DIRECCION or trama[3] != COMANDO_ANTENA:
                    continue
                if len(trama) == 6 and trama[4] == 0x10:
                    return True
                self._evento("error", f"Selección de antena rechazada: {trama.hex(' ').upper()}")
                return False
        if not self.detener.is_set():
            self._evento("error", "El lector no confirmó la selección de antena (0x74).")
        return False

    def run(self):
        try:
            serie = abrir_serie(self.puerto)
            try:
                self._evento("info", f"COM abierto: {self.puerto} @ {BAUDRATE}")
                # Algunos ESP32 se reinician cuando se abre el puerto COM.
                time.sleep(2.5)
                if self.detener.is_set():
                    return
                serie.reset_input_buffer()
                dec = DecodificadorTramas()

                serie.write(crear_comando(COMANDO_ANTENA, bytes([self.antena - 1])))
                serie.flush()
                if not self._esperar_respuesta_antena(serie, dec):
                    return
                self._evento("info", f"ANT{self.antena} seleccionada. Inventario continuo iniciado.")
                self._evento("running", True)
                op = OperacionesRFID(serie, self._evento)
                op.dec = dec

                while not self.detener.is_set():
                    # Único propietario del UART: consultar antes de iniciar otra ronda.
                    if self.pedir_temperatura.is_set():
                        self.pedir_temperatura.clear()
                        try:
                            op.medir_temperatura(True, self.antena)
                        except (serial.SerialException, OSError) as exc:
                            self._evento("temperature_error", ("Sin respuesta", f"Error UART: {exc}"))
                            raise
                        if self.detener.is_set():
                            break
                    # Cada comando solicita UNA ronda. Solo iniciamos otra al
                    # recibir su resumen (0x0A); así evitamos solapar rondas.
                    serie.write(crear_comando(COMANDO_INVENTARIO, b"\x01"))
                    serie.flush()
                    fin = time.monotonic() + 6.0
                    terminada = False
                    # Terminar la ronda en curso antes de liberar/transferir el UART.
                    while time.monotonic() < fin:
                        recibidos = serie.read(serie.in_waiting or 1)
                        for trama in dec.alimentar(recibidos):
                            if trama[2] != DIRECCION or trama[3] != COMANDO_INVENTARIO:
                                continue
                            if trama[1] == 0x04:
                                self.transferencia_segura = False
                                self._evento("error", f"Error de inventario (0x89): 0x{trama[4]:02X}")
                                return
                            if trama[1] in (0x08, 0x0A):
                                terminada = True
                                self._evento("round", None)
                                continue
                            lectura = extraer_lectura(trama)
                            if lectura is not None:
                                self._evento("tag", lectura)
                            else:
                                self._evento("info", f"Respuesta 0x89 no reconocida: {trama.hex(' ').upper()}")
                        if terminada:
                            break
                    if not terminada:
                        self.transferencia_segura = False
                        self._evento("error", "No llegó el fin de ronda 0x89. Se detuvo para evitar comandos solapados.")
                        break
                    if self.detener.is_set():
                        break
                    if self.detener.wait(self.pausa_ms / 1000):
                        break
            finally:
                if self.transferir and self.transferencia_segura and serie.is_open:
                    self.serie_transferida = serie
                elif serie.is_open:
                    serie.close()
        except (serial.SerialException, OSError, ValueError) as exc:
            self.transferencia_segura = False
            if self.serie_transferida is not None:
                self.serie_transferida.close()
                self.serie_transferida = None
            self._evento("error", f"Error del puerto serie: {exc}")
        finally:
            if self.pedir_temperatura.is_set():
                self.pedir_temperatura.clear()
                self._evento("temperature_error", ("Sin respuesta", "Consulta cancelada al detener el inventario."))
            self._evento("stopped", None)


class OperacionesRFID:
    """Transacciones A0 secuenciales sobre una conexión de propiedad exclusiva."""

    def __init__(self, serie, evento):
        self.serie = serie
        self.evento = evento
        self.dec = DecodificadorTramas()

    def enviar(self, comando, datos=b"", segundos=2.5, diagnostico_bruto=False):
        if comando != COMANDO_TEMPERATURA:
            self.dec.buffer.clear()
            self.serie.reset_input_buffer()
        tx = crear_comando(comando, datos)
        # Ocultar también checksum: depende de los bytes de la contraseña.
        visible = diagnostico_trama(tx, escritura_tx=comando == COMANDO_ESCRIBIR)
        self.evento("info", "TX: " + visible)
        self.serie.write(tx)
        self.serie.flush()
        respuestas = []
        recibidos = bytearray()
        invalidas = self.dec.invalidas
        limite = time.monotonic() + segundos
        while time.monotonic() < limite:
            bloque = self.serie.read(self.serie.in_waiting or 1)
            if diagnostico_bruto:
                recibidos.extend(bloque)
            fin = False
            for trama in self.dec.alimentar(bloque):
                self.evento("info", "RX: " + diagnostico_trama(trama))
                if trama[2] != DIRECCION or trama[3] != comando:
                    if comando == COMANDO_TEMPERATURA and trama[2] == DIRECCION:
                        tag = extraer_lectura(trama)
                        if tag is not None:
                            self.evento("tag", tag)
                    continue
                respuestas.append(trama)
                if comando == COMANDO_INVENTARIO:
                    fin |= trama[1] in (4, 8, 10)
                else:
                    fin = True
            if fin:
                if diagnostico_bruto:
                    self.evento("info", "Temperatura RX bruto: " + recibidos.hex(" ").upper())
                if self.dec.invalidas != invalidas:
                    raise RuntimeError("Respuesta con checksum inválido; operación no confirmada.")
                return respuestas
        if diagnostico_bruto:
            self.evento("info", "Temperatura RX bruto: " + (recibidos.hex(" ").upper() or "sin bytes"))
            if recibidos:
                raise ErrorTemperatura("Respuesta inválida", "Respuesta 0x7B incompleta, corrupta o ajena al protocolo; ver RX bruto.")
        if comando == COMANDO_INVENTARIO:
            raise RuntimeError("Inventario incompleto: no llegó el fin de ronda 0x89.")
        return respuestas

    def medir_temperatura(self, lectura_activa, antena):
        try:
            respuestas = self.enviar(COMANDO_TEMPERATURA, diagnostico_bruto=True)
            if not respuestas:
                raise ErrorTemperatura("Sin respuesta", "Sin respuesta a 0x7B; compatibilidad no confirmada.")
            if len(respuestas) != 1:
                raise ErrorTemperatura("Respuesta inválida", "Múltiples respuestas 0x7B; medición descartada.")
            valor = interpretar_temperatura(respuestas[0])
            self.evento("temperature_value", (datetime.now(), valor, lectura_activa, antena))
        except (ErrorTemperatura, RuntimeError) as exc:
            self.evento("temperature_error", (getattr(exc, "estado", "Respuesta inválida"), str(exc)))
        finally:
            # Una longitud corrupta de 0x7B no debe retener las próximas tramas EPC.
            # Conservar cualquier fragmento identificado como inventario.
            if len(self.dec.buffer) >= 4 and self.dec.buffer[3] == COMANDO_TEMPERATURA:
                self.dec.buffer.clear()

    def confirmar(self, comando, datos):
        respuestas = self.enviar(comando, datos)
        if len(respuestas) != 1 or len(respuestas[0]) != 6 or respuestas[0][4] != 0x10:
            raise RuntimeError(f"No se confirmó el comando 0x{comando:02X}.")

    def consultar_filtro(self):
        respuestas = self.enviar(COMANDO_CONSULTAR_FILTRO)
        if len(respuestas) != 1:
            raise RuntimeError("No se pudo consultar el filtro EPC (0x86).")
        r = respuestas[0]
        if len(r) in (6, 7) and r[4] == 1 and (len(r) == 6 or r[5] == 0):
            return None
        if len(r) >= 7 and r[4] == 0 and len(r) == r[5] + 7:
            return r[6:-1].hex().upper()
        raise RuntimeError("Respuesta de filtro EPC inválida.")

    def quitar_filtro(self):
        self.confirmar(COMANDO_FILTRO, b"\x01\x00")
        if self.consultar_filtro() is not None:
            raise RuntimeError("El filtro EPC sigue activo.")

    def fijar_filtro(self, epc):
        self.confirmar(COMANDO_FILTRO, b"\x00\x0C" + bytes.fromhex(epc))
        if self.consultar_filtro() != epc:
            raise RuntimeError("El filtro consultado no coincide con el EPC esperado.")

    def inventariar(self):
        lecturas = {}
        for r in self.enviar(COMANDO_INVENTARIO, b"\x01", segundos=6.0):
            if r[1] == 4:
                raise RuntimeError(f"Error de inventario: 0x{r[4]:02X}.")
            if r[1] in (8, 10):
                continue
            tag = extraer_lectura(r)
            if tag is None:
                raise RuntimeError("Inventario con una respuesta no reconocida.")
            lecturas[tag.epc] = tag
        return lecturas

    def verificar_unica(self, esperado):
        lecturas = self.inventariar()
        if set(lecturas) != {esperado}:
            raise RuntimeError(f"Se esperaba únicamente {esperado}; detectados: "
                               + (", ".join(sorted(lecturas)) or "ninguno") + ".")
        return lecturas[esperado]


class TrabajadorTemperatura(threading.Thread):
    """Consulta aislada cuando ningún otro worker posee el puerto."""

    def __init__(self, puerto, antena, eventos):
        super().__init__(daemon=True)
        self.puerto, self.antena, self.eventos = puerto, antena, eventos

    def _evento(self, tipo, dato):
        self.eventos.put((tipo, dato))

    def run(self):
        try:
            serie = abrir_serie(self.puerto)
            try:
                time.sleep(2.5)
                OperacionesRFID(serie, self._evento).medir_temperatura(False, self.antena)
            finally:
                serie.close()
        except (serial.SerialException, OSError, ValueError) as exc:
            self._evento("temperature_error", ("Sin respuesta", f"Error de conexión: {exc}"))
        finally:
            self._evento("temperature_done", None)


class TrabajadorEscritura(threading.Thread):
    def __init__(self, puerto, antena, actual, nuevo, password, eventos, serie=None):
        super().__init__(daemon=True)
        self.puerto, self.antena = puerto, antena
        self.actual, self.nuevo, self.password = actual, nuevo, password
        self.eventos, self.serie = eventos, serie

    def _evento(self, tipo, dato):
        self.eventos.put((tipo, dato))

    def ejecutar(self, op):
        actual = validar_hex(self.actual, 24, "EPC actual")
        nuevo = validar_hex(self.nuevo, 24, "Nuevo EPC")
        password = bytes.fromhex(validar_hex(self.password, 8, "Access Password"))
        if self.antena not in (1, 2, 3, 4):
            raise ValueError("Antena inválida.")
        cambios = palabras_modificadas(actual, nuevo)
        ultimo = None
        intentos = 0
        palabra = None
        exito = False
        resultado = None
        try:
            op.confirmar(COMANDO_ANTENA, bytes([self.antena - 1]))
            op.quitar_filtro()
            op.verificar_unica(actual)
            ultimo = actual
            self._evento("write_progress", (0, len(cambios), "Etiqueta original verificada"))
            for numero, (palabra, word) in enumerate(cambios, 1):
                self._evento("write_progress", (numero - 1, len(cambios), f"Verificando Word {palabra}"))
                op.verificar_unica(actual)
                op.fijar_filtro(actual)
                pos = (palabra - 2) * 4
                esperado = actual[:pos] + word.hex().upper() + actual[pos+4:]
                intentos += 1
                # Nunca reintentar este comando, ni aun si write/flush/read fallan.
                error_tx = None
                try:
                    respuestas = op.enviar(COMANDO_ESCRIBIR,
                                           password + bytes([1, palabra, 1]) + word)
                    estados = [estado_escritura(r) for r in respuestas]
                except (serial.SerialException, OSError, RuntimeError) as exc:
                    error_tx = type(exc).__name__
                    estados = []
                errores = {0x33: "error de escritura", 0x40: "error de acceso/contraseña",
                           0x36: "etiqueta no encontrada", 0x37: "acceso fallido"}
                rechazados = [s for s in estados if s is not None and s != 0x10]
                rechazo = ""
                if rechazados:
                    s = rechazados[0]
                    rechazo = f"0x{s:02X}: {errores.get(s, 'estado desconocido')}."
                    self._evento("info", f"Word {palabra}: {rechazo} Se verificará el estado real.")
                # Verificar también cuando la respuesta es un error, ambigua o timeout.
                try:
                    op.quitar_filtro()
                    encontrados = op.inventariar()
                except (serial.SerialException, OSError, RuntimeError) as exc:
                    raise RuntimeError(f"{rechazo} Verificación posterior fallida: {exc}") from exc
                if len(encontrados) == 1:
                    ultimo = next(iter(encontrados))
                if rechazo:
                    raise RuntimeError(rechazo)
                if set(encontrados) != {esperado}:
                    raise RuntimeError("El inventario posterior no confirma el EPC intermedio "
                                       f"{esperado}; detectados: {', '.join(encontrados) or 'ninguno'}.")
                if estados != [0x10]:
                    self._evento("info", "Respuesta 0x82 ambigua/timeout"
                                 + (f" ({error_tx})" if error_tx else "")
                                 + ": modificación confirmada por inventario; sin reintentos.")
                actual = esperado
                self._evento("write_progress", (numero, len(cambios), f"Word {palabra} verificado: {actual}"))
            # Ronda completa adicional, sin filtro, aun después del último Word.
            resultado = op.verificar_unica(nuevo)
            ultimo = nuevo
            exito = True
        except (serial.SerialException, OSError, ValueError, RuntimeError) as exc:
            texto = (f"Word {palabra}: " if palabra is not None else "Preparación: ") + str(exc)
            texto += f" Último EPC verificado: {ultimo or 'ninguno'}."
            if intentos:
                texto += " La etiqueta podría haber quedado parcialmente modificada. No se reintentó la escritura."
            self._evento("write_error", texto)
        finally:
            try:
                op.quitar_filtro()
            except (serial.SerialException, OSError, RuntimeError) as exc:
                exito = False
                self._evento("write_error", "ADVERTENCIA: limpieza final del filtro no confirmada: "
                             + str(exc) + f" Último EPC verificado: {ultimo or 'ninguno'}.")
        if exito:
            self._evento("write_success" if cambios else "write_unchanged", (self.actual, resultado))

    def run(self):
        serie = self.serie
        try:
            if serie is None:
                serie = abrir_serie(self.puerto)
                time.sleep(2.5)
            self.ejecutar(OperacionesRFID(serie, self._evento))
        except (serial.SerialException, OSError, ValueError, RuntimeError) as exc:
            # No incluir parámetros de comando ni contraseñas en excepciones de transporte.
            self._evento("write_error", "No se pudo preparar la conexión de escritura: " + type(exc).__name__)
        finally:
            self.password = ""
            try:
                if serie is not None and serie.is_open:
                    serie.close()
            finally:
                self._evento("write_done", None)


@dataclass
class Registro:
    epc: str
    count: int
    antena: int
    rssi_raw: int
    primera: datetime
    ultima: datetime
    ultimo_monotonico: float


class Aplicacion(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("CM2000 Live | Monitor RFID")
        self.geometry(f"1120x{min(900, max(630, self.winfo_screenheight() - 80))}")
        self.minsize(920, 630)
        self.configure(bg=BG)

        self.eventos = queue.Queue()
        self.stop_event = None
        self.worker = None
        self.activo = False
        self.registros: dict[str, Registro] = {}
        self.actualizar_filas: set[str] = set()
        self.instantes = deque()
        self.total = 0
        self.rondas = 0
        self._last_stats = 0

        self.var_puerto = tk.StringVar(value="/dev/ttyUSB0")
        self.var_antena = tk.StringVar(value="1")
        self.var_intervalo = tk.StringVar(value="150")
        self.var_estado = tk.StringVar(value="Desconectado")
        self.var_unicos = tk.StringVar(value="0")
        self.var_total = tk.StringVar(value="0")
        self.var_velocidad = tk.StringVar(value="0,0 /s")
        self.var_rondas = tk.StringVar(value="0")
        self.var_potencia_actual = tk.StringVar(value="Sin consultar")
        self.var_potencias_detalle = tk.StringVar(value="ANT1: —    ANT2: —    ANT3: —    ANT4: —")
        self.var_potencia_objetivo = tk.IntVar(value=20)
        self.potencias_actuales = None
        self.trabajador_potencia = None
        self.operando_potencia = False
        self.escribiendo = False
        self.trabajador_escritura = None
        self.escritura_pendiente = None
        self.var_epc_actual = tk.StringVar()
        self.var_epc_nuevo = tk.StringVar()
        self.var_password = tk.StringVar(value="00000000")
        self.var_escritura = tk.StringVar(value="Seleccioná una etiqueta de la tabla. Solo EPC de 96 bits.")
        self.consultando_temperatura = False
        self.confirmando_operacion = False
        self.trabajador_temperatura = None
        self.temperatura_en_inventario = False
        self.historial_temperatura = []
        self.ultima_temperatura = None
        self.alerta_temperatura = False
        self.proxima_temperatura = 0.0
        self.var_temperatura = tk.StringVar(value="CM2000-4: — °C")
        self.var_sensor = tk.StringVar(value="Sin consultar · compatibilidad no confirmada")
        self.var_ultima_temperatura = tk.StringVar(value="Última lectura válida: —")
        self.var_auto_temperatura = tk.BooleanVar(value=False)
        self.var_intervalo_temperatura = tk.StringVar(value="5")
        self.var_umbral_temperatura = tk.StringVar(value="")
        self.var_alerta_temperatura = tk.StringVar(value="Umbral opcional y orientativo; no es un límite oficial de Chainway.")

        self._estilos()
        self._construir()
        self.refrescar_puertos()
        self.after(100, self._procesar_eventos)
        self.after(500, self._monitor_temperatura)
        self.protocol("WM_DELETE_WINDOW", self._cerrar)

    def _estilos(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("RFID.Treeview", background=PANEL, foreground=TEXT,
                        fieldbackground=PANEL, borderwidth=0, rowheight=35,
                        font=("Segoe UI", 10))
        style.map("RFID.Treeview", background=[("selected", "#20665f")],
                  foreground=[("selected", "#ffffff")])
        style.configure("RFID.Treeview.Heading", background=PANEL_ALT,
                        foreground=MUTED, borderwidth=0, relief="flat",
                        font=("Segoe UI", 10, "bold"), padding=(10, 10))
        style.map("RFID.Treeview.Heading", background=[("active", "#20344f")])
        style.configure("RFID.TCombobox", fieldbackground=PANEL_ALT,
                        background=PANEL_ALT, foreground=TEXT, arrowcolor=TEXT,
                        padding=7, borderwidth=0)
        style.map("RFID.TCombobox", fieldbackground=[("readonly", PANEL_ALT)],
                  foreground=[("readonly", TEXT)])

    def _etiqueta(self, padre, texto, size=11, weight="normal", fg=TEXT, **kw):
        return tk.Label(padre, text=texto, bg=padre.cget("bg"), fg=fg,
                        font=("Segoe UI", size, weight), **kw)

    def _boton(self, padre, texto, accion, bg=PANEL_ALT, fg=TEXT, ancho=None):
        return tk.Button(padre, text=texto, command=accion, bg=bg, fg=fg,
                         activebackground="#246d66" if bg == ACCENT else "#273e59",
                         activeforeground="white", font=("Segoe UI", 10, "bold"),
                         bd=0, cursor="hand2", padx=14, pady=10, width=ancho,
                         disabledforeground="#6f8197", relief="flat")

    def _construir(self):
        # La sección adicional debe seguir siendo accesible en pantallas de 768 px.
        canvas = tk.Canvas(self, bg=BG, highlightthickness=0)
        scroll = ttk.Scrollbar(self, orient="vertical", command=canvas.yview)
        scroll.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        canvas.configure(yscrollcommand=scroll.set)
        contenido = tk.Frame(canvas, bg=BG)
        ventana = canvas.create_window((0, 0), window=contenido, anchor="nw")

        def ajustar_contenido(_event=None):
            canvas.itemconfigure(ventana, width=canvas.winfo_width(),
                                 height=max(canvas.winfo_height(), contenido.winfo_reqheight()))
            canvas.configure(scrollregion=canvas.bbox("all"))

        canvas.bind("<Configure>", ajustar_contenido)
        contenido.bind("<Configure>", ajustar_contenido)

        top = tk.Frame(contenido, bg=BG)
        top.pack(fill="x", padx=24, pady=(22, 15))
        self._etiqueta(top, "CM2000  /  LIVE", 21, "bold").pack(side="left")
        self._etiqueta(top, "  Monitor de etiquetas UHF", 11, fg=MUTED).pack(side="left", pady=(8, 0))
        self.estado_label = self._etiqueta(top, "●  Desconectado", 11, "bold", fg=MUTED)
        self.estado_label.pack(side="right", pady=8)

        toolbar = tk.Frame(contenido, bg=PANEL, highlightbackground=BORDER,
                           highlightthickness=1)
        toolbar.pack(fill="x", padx=24, pady=(0, 15))
        row = tk.Frame(toolbar, bg=PANEL)
        row.pack(fill="x", padx=17, pady=15)
        self._etiqueta(row, "Puerto", 10, fg=MUTED).pack(side="left")
        self.combo_puerto = ttk.Combobox(row, textvariable=self.var_puerto,
                                         width=11, style="RFID.TCombobox")
        self.combo_puerto.pack(side="left", padx=(8, 6))
        self.btn_recargar = self._boton(row, "↻", self.refrescar_puertos)
        self.btn_recargar.pack(side="left", padx=(0, 22))

        self._etiqueta(row, "Antena", 10, fg=MUTED).pack(side="left")
        self.combo_antena = ttk.Combobox(row, textvariable=self.var_antena,
                                         values=["1", "2", "3", "4"], width=4,
                                         state="readonly", style="RFID.TCombobox")
        self.combo_antena.pack(side="left", padx=(8, 22))

        self._etiqueta(row, "Pausa entre rondas (ms)", 10, fg=MUTED).pack(side="left")
        self.combo_intervalo = ttk.Combobox(row, textvariable=self.var_intervalo,
                                            values=["0", "50", "150", "300", "500", "1000"],
                                            width=6, style="RFID.TCombobox")
        self.combo_intervalo.pack(side="left", padx=(8, 10))
        self.btn_iniciar = self._boton(row, "▶  Iniciar", self.iniciar, bg=ACCENT, fg=BG)
        self.btn_iniciar.pack(side="right", padx=(8, 0))
        self.btn_detener = self._boton(row, "■  Detener", self.detener)
        self.btn_detener.pack(side="right")
        self.btn_detener.configure(state="disabled")

        potencia_panel = tk.Frame(contenido, bg=PANEL, highlightbackground=BORDER,
                                 highlightthickness=1)
        potencia_panel.pack(fill="x", padx=24, pady=(0, 15))
        potencia_row = tk.Frame(potencia_panel, bg=PANEL)
        potencia_row.pack(fill="x", padx=17, pady=(11, 2))

        potencia_texto = tk.Frame(potencia_row, bg=PANEL)
        potencia_texto.pack(side="left", fill="x", expand=True)
        self._etiqueta(potencia_texto, "Potencia RF · antena seleccionada", 11,
                       "bold").pack(anchor="w")
        tk.Label(potencia_texto, textvariable=self.var_potencia_actual,
                 bg=PANEL, fg=ACCENT, font=("Segoe UI", 11, "bold")).pack(anchor="w", pady=(2, 0))
        tk.Label(potencia_texto, textvariable=self.var_potencias_detalle,
                 bg=PANEL, fg=MUTED, font=("Consolas", 9)).pack(anchor="w", pady=(1, 0))

        self.scale_potencia = tk.Scale(
            potencia_row, from_=POTENCIA_MIN, to=POTENCIA_MAX,
            orient="horizontal", variable=self.var_potencia_objetivo,
            resolution=1, length=205, label="Nuevo valor (dBm)",
            bg=PANEL, fg=TEXT, troughcolor=PANEL_ALT, activebackground=ACCENT,
            highlightthickness=0, bd=0, font=("Segoe UI", 9)
        )
        self.scale_potencia.pack(side="left", padx=(12, 12))
        self.btn_consultar_potencia = self._boton(
            potencia_row, "Consultar", self.consultar_potencia
        )
        self.btn_consultar_potencia.pack(side="left", padx=(0, 8))
        self.btn_aplicar_potencia = self._boton(
            potencia_row, "Aplicar potencia", self.aplicar_potencia, bg=ACCENT, fg=BG
        )
        self.btn_aplicar_potencia.pack(side="left")

        self._etiqueta(
            potencia_panel,
            "El cambio solo se envía al tocar Aplicar. El comando 0x76 puede guardarse en memoria del lector.",
            9, fg=MUTED
        ).pack(anchor="w", padx=17, pady=(0, 11))
        self.combo_antena.bind("<<ComboboxSelected>>", self._al_cambiar_antena)

        escritura = tk.Frame(contenido, bg=PANEL, highlightbackground=BORDER, highlightthickness=1)
        escritura.pack(fill="x", padx=24, pady=(0, 12))
        self._etiqueta(escritura, "Escritura EPC", 11, "bold").grid(
            row=0, column=0, sticky="w", padx=14, pady=(8, 3))
        self._etiqueta(escritura, "Antena: la seleccionada arriba · 96 bits · cambio persistente",
                       9, fg=MUTED).grid(row=0, column=1, columnspan=3, sticky="w")
        self.campos_escritura = []
        for col, (titulo, variable, opciones) in enumerate([
            ("EPC actual", self.var_epc_actual, {"state": "readonly", "width": 26}),
            ("Nuevo EPC", self.var_epc_nuevo, {"width": 26}),
            ("Access Password", self.var_password, {"show": "•", "width": 10}),
        ]):
            celda = tk.Frame(escritura, bg=PANEL)
            celda.grid(row=1, column=col, sticky="ew", padx=14)
            self._etiqueta(celda, titulo, 9, fg=MUTED).pack(anchor="w")
            campo = tk.Entry(celda, textvariable=variable, bg=PANEL_ALT, fg=TEXT,
                             readonlybackground=PANEL_ALT, insertbackground=TEXT,
                             font=("Consolas", 10), relief="flat", **opciones)
            campo.pack(fill="x", ipady=4)
            self.campos_escritura.append(campo)
        self.btn_escribir = self._boton(escritura, "Escribir EPC", self.escribir_epc, bg=ACCENT, fg=BG)
        self.btn_escribir.grid(row=1, column=3, padx=14, sticky="se")
        tk.Label(escritura, textvariable=self.var_escritura, bg=PANEL, fg=MUTED,
                 font=("Segoe UI", 9), wraplength=950, anchor="w", justify="left").grid(
                     row=2, column=0, columnspan=4, sticky="ew", padx=14, pady=(6, 2))
        self.progreso_escritura = ttk.Progressbar(escritura, maximum=6)
        self.progreso_escritura.grid(row=3, column=0, columnspan=4, sticky="ew", padx=14, pady=(0, 8))
        escritura.grid_columnconfigure(1, weight=1)

        temperatura_panel = tk.Frame(contenido, bg=PANEL, highlightbackground=BORDER, highlightthickness=1)
        temperatura_panel.pack(fill="x", padx=24, pady=(0, 12))
        self._etiqueta(temperatura_panel, "Temperatura del lector", 11, "bold").pack(anchor="w", padx=14, pady=(8, 0))
        fila_temp = tk.Frame(temperatura_panel, bg=PANEL)
        fila_temp.pack(fill="x", padx=14, pady=5)
        self.label_temperatura = tk.Label(fila_temp, textvariable=self.var_temperatura,
                                         bg=PANEL, fg=ACCENT, font=("Segoe UI", 18, "bold"))
        self.label_temperatura.pack(side="left")
        self.btn_temperatura = self._boton(fila_temp, "Consultar temperatura", self.consultar_temperatura)
        self.btn_temperatura.pack(side="right")
        opciones_temp = tk.Frame(temperatura_panel, bg=PANEL)
        opciones_temp.pack(fill="x", padx=14)
        tk.Checkbutton(opciones_temp, text="Monitoreo automático", variable=self.var_auto_temperatura,
                       command=self._configurar_auto_temperatura, bg=PANEL, fg=TEXT,
                       selectcolor=PANEL_ALT, activebackground=PANEL, activeforeground=TEXT).pack(side="left")
        self._etiqueta(opciones_temp, "Cada (s)", 9, fg=MUTED).pack(side="left", padx=(12, 4))
        self.entrada_intervalo_temperatura = tk.Entry(opciones_temp, textvariable=self.var_intervalo_temperatura,
                         width=5, bg=PANEL_ALT, fg=TEXT, insertbackground=TEXT)
        self.entrada_intervalo_temperatura.pack(side="left")
        self._etiqueta(opciones_temp, "Advertir por encima de (°C)", 9, fg=MUTED).pack(side="left", padx=(12, 4))
        self.entrada_umbral_temperatura = tk.Entry(opciones_temp, textvariable=self.var_umbral_temperatura,
                         width=6, bg=PANEL_ALT, fg=TEXT, insertbackground=TEXT)
        self.entrada_umbral_temperatura.pack(side="left")
        self.entrada_umbral_temperatura.bind("<FocusOut>", self._cambiar_umbral_temperatura)
        self.entrada_umbral_temperatura.bind("<Return>", self._cambiar_umbral_temperatura)
        self._boton(opciones_temp, "Exportar temperatura CSV", self.exportar_temperatura_csv).pack(side="right")
        for variable in (self.var_sensor, self.var_ultima_temperatura, self.var_alerta_temperatura):
            tk.Label(temperatura_panel, textvariable=variable, bg=PANEL, fg=MUTED,
                     font=("Segoe UI", 9), wraplength=950, anchor="w", justify="left").pack(fill="x", padx=14)
        self._etiqueta(temperatura_panel, "Temperatura interna reportada por el lector. No mide la superficie ni el ESP32.",
                       9, fg=MUTED).pack(anchor="w", padx=14, pady=(2, 8))

        tarjetas = tk.Frame(contenido, bg=BG)
        tarjetas.pack(fill="x", padx=24, pady=(0, 15))
        for i, (titulo, valor) in enumerate([
            ("EPC ÚNICOS", self.var_unicos),
            ("DETECCIONES", self.var_total),
            ("LECTURAS / SEGUNDO", self.var_velocidad),
            ("RONDAS", self.var_rondas),
        ]):
            card = tk.Frame(tarjetas, bg=PANEL, highlightthickness=1,
                            highlightbackground=BORDER)
            card.grid(row=0, column=i, sticky="nsew", padx=(0, 10) if i < 3 else 0)
            tarjetas.grid_columnconfigure(i, weight=1)
            self._etiqueta(card, titulo, 9, "bold", fg=MUTED).pack(anchor="w", padx=16, pady=(13, 3))
            tk.Label(card, textvariable=valor, bg=PANEL, fg=TEXT,
                     font=("Segoe UI", 23, "bold")).pack(anchor="w", padx=16, pady=(0, 13))

        tabla_container = tk.Frame(contenido, bg=PANEL, highlightbackground=BORDER,
                                   highlightthickness=1)
        tabla_container.pack(fill="both", expand=True, padx=24, pady=(0, 12))
        head = tk.Frame(tabla_container, bg=PANEL)
        head.pack(fill="x", padx=15, pady=(12, 10))
        self._etiqueta(head, "Etiquetas detectadas", 12, "bold").pack(side="left")
        self._etiqueta(head, "Count = reportes EPC recibidos (no unidades físicas)",
                       9, fg=MUTED).pack(side="right")

        cuerpo = tk.Frame(tabla_container, bg=PANEL)
        cuerpo.pack(fill="both", expand=True, padx=14, pady=(0, 14))
        cols = ("epc", "count", "antena", "rssi", "primera", "ultima", "hace")
        self.tabla = ttk.Treeview(cuerpo, columns=cols, show="headings",
                                   height=5, style="RFID.Treeview", selectmode="browse")
        definiciones = [
            ("epc", "EPC", 285, "w"),
            ("count", "Count", 75, "center"),
            ("antena", "Antena", 70, "center"),
            ("rssi", "RSSI bruto", 115, "center"),
            ("primera", "Primera vez", 105, "center"),
            ("ultima", "Última vez", 105, "center"),
            ("hace", "Hace", 90, "center"),
        ]
        for col, titulo, w, anchor in definiciones:
            self.tabla.heading(col, text=titulo)
            self.tabla.column(col, width=w, minwidth=60, stretch=col == "epc", anchor=anchor)
        sc = ttk.Scrollbar(cuerpo, orient="vertical", command=self.tabla.yview)
        self.tabla.configure(yscrollcommand=sc.set)
        sc.pack(side="right", fill="y")
        self.tabla.pack(side="left", fill="both", expand=True)
        self.tabla.bind("<<TreeviewSelect>>", self._seleccionar_epc)

        bottom = tk.Frame(contenido, bg=BG)
        bottom.pack(fill="x", padx=24, pady=(0, 10))
        self._boton(bottom, "Limpiar resultados", self.limpiar).pack(side="left")
        self._boton(bottom, "Exportar CSV", self.exportar_csv).pack(side="left", padx=9)
        self._etiqueta(bottom, "Una antena física conectada al puerto seleccionado; no cambiar en caliente.",
                       9, fg=MUTED).pack(side="right")

        logwrap = tk.Frame(contenido, bg=PANEL, highlightbackground=BORDER, highlightthickness=1)
        logwrap.pack(fill="x", padx=24, pady=(0, 22))
        self._etiqueta(logwrap, "Registro / diagnóstico", 10, "bold", fg=MUTED).pack(anchor="w", padx=12, pady=(8, 0))
        self.log = tk.Text(logwrap, height=3, state="disabled", wrap="word",
                           bg=PANEL, fg=MUTED, insertbackground=TEXT,
                           bd=0, highlightthickness=0,
                           font=("Consolas", 9))
        self.log.pack(fill="x", padx=12, pady=(4, 10))
        self._log("Listo. Cargá el puente UART en el ESP32 y elegí el puerto COM.")
        # Reservar espacio para el log y los controles inferiores ANTES de
        # expandir la tabla; de lo contrario se ocultan en pantallas de 768 px.
        tabla_container.pack_forget()
        bottom.pack_forget()
        logwrap.pack_forget()
        logwrap.pack(side="bottom", fill="x", padx=24, pady=(0, 12))
        bottom.pack(side="bottom", fill="x", padx=24, pady=(0, 8))
        tabla_container.pack(fill="both", expand=True, padx=24, pady=(0, 10))

    def _log(self, texto):
        self.log.configure(state="normal")
        self.log.insert("end", f"[{datetime.now():%H:%M:%S}] {texto}\n")
        lineas = int(self.log.index("end-1c").split(".")[0])
        if lineas > 110:
            self.log.delete("1.0", f"{lineas - 100}.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _estado(self, texto, fg=MUTED):
        self.var_estado.set(texto)
        self.estado_label.configure(text="●  " + texto, fg=fg)

    def refrescar_puertos(self):
        puertos = [p.device for p in list_ports.comports()]
        self.combo_puerto["values"] = puertos
        if puertos and self.var_puerto.get().strip() not in puertos:
            self.var_puerto.set(puertos[0])

    def _intervalo_temperatura(self):
        try:
            valor = float(self.var_intervalo_temperatura.get().replace(",", "."))
            if not math.isfinite(valor) or valor < 1:
                raise ValueError
            return valor
        except ValueError:
            raise ValueError("El intervalo debe ser un número de al menos 1 segundo.")

    def _confirmar_operacion(self, *args, **kwargs):
        # Los diálogos Tk ejecutan un bucle de eventos anidado: el temporizador
        # automático también corre mientras el usuario confirma una operación.
        self.confirmando_operacion = True
        try:
            return messagebox.askyesno(*args, **kwargs)
        finally:
            self.confirmando_operacion = False

    def _configurar_auto_temperatura(self):
        if self.var_auto_temperatura.get():
            try:
                self._intervalo_temperatura()
            except ValueError as exc:
                self.var_auto_temperatura.set(False)
                messagebox.showerror("Temperatura", str(exc))
                return
            self.proxima_temperatura = 0
        self._log("Monitoreo de temperatura " + ("activado." if self.var_auto_temperatura.get() else "desactivado."))

    def _monitor_temperatura(self):
        if self.var_auto_temperatura.get() and time.monotonic() >= self.proxima_temperatura:
            if (not self.consultando_temperatura and not self.escribiendo
                    and not self.operando_potencia and not self.confirmando_operacion):
                try:
                    self.proxima_temperatura = time.monotonic() + self._intervalo_temperatura()
                except ValueError as exc:
                    self.var_auto_temperatura.set(False)
                    self.var_sensor.set(str(exc) + " Monitoreo pausado.")
                else:
                    self.consultar_temperatura(automatico=True)
        self.after(500, self._monitor_temperatura)

    def consultar_temperatura(self, automatico=False):
        if self.consultando_temperatura or self.escribiendo or self.operando_potencia:
            return
        if self.worker and self.worker.is_alive():
            if not self.activo or self.stop_event.is_set():
                return
            self.consultando_temperatura = True
            self.temperatura_en_inventario = True
            self.worker.pedir_temperatura.set()
        else:
            # No abrir otro puerto mientras falta procesar el cierre del inventario.
            if self.activo:
                return
            puerto = self.var_puerto.get().strip()
            if not puerto:
                self.var_sensor.set("Seleccioná un puerto para consultar 0x7B.")
                if automatico:
                    self.var_auto_temperatura.set(False)
                return
            try:
                antena = int(self.var_antena.get())
                if antena not in (1, 2, 3, 4):
                    raise ValueError
            except ValueError:
                self.var_sensor.set("Seleccioná una antena válida para el registro histórico.")
                return
            self.consultando_temperatura = True
            self.temperatura_en_inventario = False
            self._bloquear_escritura(True)
            self.trabajador_temperatura = TrabajadorTemperatura(puerto, antena, self.eventos)
            self.trabajador_temperatura.start()
        self.var_sensor.set("Consultando 0x7B" + (" entre rondas de inventario..." if self.temperatura_en_inventario else "..."))
        self.btn_temperatura.configure(state="disabled")

    def _evaluar_umbral_temperatura(self):
        texto = self.var_umbral_temperatura.get().strip()
        if not texto:
            self.alerta_temperatura = False
            self.label_temperatura.configure(fg=ACCENT)
            self.var_alerta_temperatura.set("Umbral opcional y orientativo; no es un límite oficial de Chainway.")
            return
        try:
            umbral = float(texto.replace(",", "."))
            if not math.isfinite(umbral):
                raise ValueError
        except ValueError:
            self.var_alerta_temperatura.set("Umbral inválido: ingresá un número o dejalo vacío para desactivar la advertencia.")
            return
        supera = self.ultima_temperatura is not None and self.ultima_temperatura > umbral
        self.label_temperatura.configure(fg=ERROR if supera else ACCENT)
        self.var_alerta_temperatura.set(
            f"{'ADVERTENCIA: temperatura superior a' if supera else 'Umbral de advertencia:'} {umbral:g} °C. "
            "Valor orientativo configurado por el usuario; no es un límite oficial.")
        if supera and not self.alerta_temperatura:
            self._log(f"ADVERTENCIA de temperatura: {self.ultima_temperatura} °C supera el umbral orientativo de {umbral:g} °C.")
        self.alerta_temperatura = supera

    def _cambiar_umbral_temperatura(self, _event=None):
        self._evaluar_umbral_temperatura()

    def _evento_temperatura(self, tipo, dato):
        if tipo == "temperature_value":
            fecha, valor, activo, antena = dato
            potencia = self.potencias_actuales[antena - 1] if self.potencias_actuales is not None else None
            self.historial_temperatura.append((fecha, valor, activo, antena, potencia))
            self.ultima_temperatura = valor
            self.var_temperatura.set(f"CM2000-4: {valor} °C")
            self.var_ultima_temperatura.set(f"Última lectura válida: {fecha:%Y-%m-%d %H:%M:%S}")
            self.var_sensor.set("Disponible · temperatura interna reportada por 0x7B")
            self._evaluar_umbral_temperatura()
            self._log(f"Temperatura interna CM2000-4: {valor} °C · inventario {'activo' if activo else 'detenido'}.")
        elif tipo == "temperature_error":
            estado, mensaje = dato
            self.var_auto_temperatura.set(False)
            self.var_temperatura.set("CM2000-4: — °C")
            self.var_sensor.set(f"{estado}: {mensaje} Monitoreo automático pausado; ver diagnóstico TX/RX.")
            self._log("Temperatura: " + self.var_sensor.get())
            if self.temperatura_en_inventario:
                self._log("Se prioriza el inventario RFID. No se harán más consultas automáticas de temperatura hasta reactivarlas.")
        if tipo == "temperature_done" or self.temperatura_en_inventario:
            self.consultando_temperatura = False
            if tipo == "temperature_done":
                self._bloquear_escritura(False)
            self.temperatura_en_inventario = False
            self.btn_temperatura.configure(state="normal" if not self.escribiendo and not self.operando_potencia else "disabled")

    def exportar_temperatura_csv(self):
        if not self.historial_temperatura:
            messagebox.showinfo("Temperatura", "Todavía no hay mediciones válidas para exportar.")
            return
        ruta = filedialog.asksaveasfilename(
            title="Exportar temperatura interna", defaultextension=".csv",
            filetypes=[("Archivo CSV", "*.csv")], initialfile=f"temperatura_{datetime.now():%Y%m%d_%H%M%S}.csv")
        if not ruta:
            return
        try:
            with open(ruta, "w", newline="", encoding="utf-8-sig") as archivo:
                writer = csv.writer(archivo)
                writer.writerow(["Fecha_hora", "Temperatura_interna_C", "Inventario", "Antena", "Potencia_dBm_ultima_consultada"])
                for fecha, valor, activo, antena, potencia in self.historial_temperatura:
                    writer.writerow([fecha.isoformat(timespec="seconds"), valor, "activo" if activo else "detenido",
                                     antena, potencia if potencia is not None else ""])
            self._log(f"Historial de temperatura exportado: {ruta}")
        except OSError as exc:
            messagebox.showerror("Temperatura", str(exc))

    def _al_cambiar_antena(self, _event=None):
        self._mostrar_potencia_actual()

    def _mostrar_potencia_actual(self):
        try:
            antena = int(self.var_antena.get())
        except ValueError:
            return
        if self.potencias_actuales is None:
            self.var_potencia_actual.set(f"ANT{antena}: sin consultar")
        else:
            valor = self.potencias_actuales[antena - 1]
            self.var_potencia_actual.set(f"ANT{antena}: {valor} dBm (consultado)")
            if POTENCIA_MIN <= valor <= POTENCIA_MAX:
                self.var_potencia_objetivo.set(valor)

    def _controles_bloqueados(self, bloqueado: bool):
        self.btn_consultar_potencia.configure(state="disabled" if bloqueado else "normal")
        self.btn_aplicar_potencia.configure(state="disabled" if bloqueado else "normal")
        self.scale_potencia.configure(state="disabled" if bloqueado else "normal")
        self.btn_escribir.configure(state="disabled" if self.escribiendo or self.operando_potencia or self.consultando_temperatura else "normal")
        self.btn_temperatura.configure(state="disabled" if self.escribiendo or self.operando_potencia or self.consultando_temperatura else "normal")

    def _seleccionar_epc(self, _event=None):
        if not self.escribiendo:
            seleccion = self.tabla.selection()
            self.var_epc_actual.set(seleccion[0] if seleccion else "")

    def escribir_epc(self):
        if self.escribiendo or self.operando_potencia or self.consultando_temperatura:
            return
        try:
            seleccion = self.tabla.selection()
            if not seleccion or seleccion[0] != self.var_epc_actual.get():
                raise ValueError("Seleccioná la etiqueta que querés modificar en la tabla.")
            actual = validar_hex(self.var_epc_actual.get(), 24, "EPC actual")
            nuevo = validar_hex(self.var_epc_nuevo.get(), 24, "Nuevo EPC")
            password = validar_hex(self.var_password.get() or "00000000", 8, "Access Password")
            antena = int(self.var_antena.get())
            puerto = self.var_puerto.get().strip()
            if not puerto or antena not in (1, 2, 3, 4):
                raise ValueError("Seleccioná un puerto y una antena válida.")
            cambios = palabras_modificadas(actual, nuevo)
        except ValueError as exc:
            messagebox.showerror("Escritura EPC", str(exc))
            return
        if not cambios:
            self.var_escritura.set("Los EPC son iguales. No se realizó ninguna escritura.")
            self._log(self.var_escritura.get())
            return
        if not self._confirmar_operacion("Confirmar escritura persistente",
                                  f"EPC original: {actual}\nNuevo EPC: {nuevo}\nAntena: ANT{antena}\n\n"
                                  f"Se modificarán {len(cambios)} palabras, una por vez. "
                                  "El cambio es persistente. Dejá solo esta etiqueta en el campo RF.\n\n"
                                  "La lectura continua se detendrá. ¿Escribir EPC?"):
            return
        self.escribiendo = True
        self.escritura_pendiente = (puerto, antena, actual, nuevo, password)
        self.var_password.set("00000000")
        self._bloquear_escritura(True)
        self.progreso_escritura.configure(value=0, maximum=len(cambios))
        self.var_escritura.set("Esperando que termine el inventario y se libere la conexión...")
        if self.worker and self.worker.is_alive():
            self.worker.transferir = True
            self.stop_event.set()
        self._esperar_para_escribir()

    def _bloquear_escritura(self, bloqueado):
        self._controles_bloqueados(bloqueado)
        self.btn_iniciar.configure(state="disabled" if bloqueado else "normal")
        self.btn_detener.configure(state="disabled")
        for control in (self.combo_puerto, self.combo_intervalo, self.btn_recargar):
            control.configure(state="disabled" if bloqueado else "normal")
        self.combo_antena.configure(state="disabled" if bloqueado else "readonly")
        for control in self.campos_escritura[1:]:
            control.configure(state="disabled" if bloqueado else "normal")

    def _esperar_para_escribir(self):
        # El evento 'stopped' puede llegar antes de que Thread.run haya retornado.
        # Esperar is_alive() evita el acceso concurrente incluso en ese intervalo.
        if self.worker and self.worker.is_alive():
            self.after(50, self._esperar_para_escribir)
            return
        if not self.eventos.empty():
            self.after(100, self._esperar_para_escribir)
            return
        if self.worker and not self.worker.transferencia_segura:
            self.escritura_pendiente = None
            self.eventos.put(("write_error", "La última ronda de lectura no terminó correctamente. No se escribió."))
            self.eventos.put(("write_done", None))
            return
        serie = self.worker.serie_transferida if self.worker else None
        if self.worker:
            self.worker.serie_transferida = None
        self.trabajador_escritura = TrabajadorEscritura(*self.escritura_pendiente, self.eventos, serie=serie)
        self.escritura_pendiente = None
        self._estado("Escribiendo EPC...", "#eecb87")
        self.trabajador_escritura.start()

    def _operar_potencia(self, operacion: str):
        if self.consultando_temperatura or self.escribiendo or self.activo or self.operando_potencia or (self.worker and self.worker.is_alive()):
            messagebox.showinfo("Potencia", "Detené la lectura antes de consultar o cambiar la potencia.")
            return
        puerto = self.var_puerto.get().strip()
        try:
            antena = int(self.var_antena.get())
            potencia = int(self.var_potencia_objetivo.get())
        except ValueError:
            messagebox.showerror("Potencia", "Revisá la antena y el valor de potencia.")
            return
        if not puerto or antena not in (1, 2, 3, 4):
            messagebox.showerror("Potencia", "Elegí un puerto COM y una antena válida.")
            return
        if operacion == "aplicar":
            if not POTENCIA_MIN <= potencia <= POTENCIA_MAX:
                messagebox.showerror("Potencia", "Seleccioná entre 5 y 30 dBm.")
                return
            if not self._confirmar_operacion(
                "Confirmar potencia RF",
                f"Se configurará ANT{antena} a {potencia} dBm.\n"
                "Las otras antenas conservarán sus valores actuales.\n\n"
                "El comando puede guardar cambios en la memoria del lector. "
                "Usá una antena RF compatible y respetá los límites regionales.\n\n"
                "¿Continuar?"
            ):
                return
        self.operando_potencia = True
        self._controles_bloqueados(True)
        self.btn_iniciar.configure(state="disabled")
        self.combo_puerto.configure(state="disabled")
        self.combo_antena.configure(state="disabled")
        self.btn_recargar.configure(state="disabled")
        self._estado("Consultando potencia..." if operacion == "consultar" else "Aplicando potencia...", "#eecb87")
        self.trabajador_potencia = TrabajadorPotencia(
            puerto, operacion, antena, potencia, self.eventos
        )
        self.trabajador_potencia.start()

    def consultar_potencia(self):
        self._operar_potencia("consultar")

    def aplicar_potencia(self):
        self._operar_potencia("aplicar")

    def iniciar(self):
        if self.consultando_temperatura or self.escribiendo or self.operando_potencia or (self.worker is not None and self.worker.is_alive()):
            return
        puerto = self.var_puerto.get().strip()
        try:
            antena = int(self.var_antena.get())
            pausa = int(self.var_intervalo.get())
        except ValueError:
            messagebox.showerror("Configuración", "Antena y pausa deben ser números válidos.")
            return
        if not puerto or antena not in (1, 2, 3, 4) or not 0 <= pausa <= 5000:
            messagebox.showerror("Configuración", "Revisá el puerto, la antena y la pausa (0 a 5000 ms).")
            return
        if not self._confirmar_operacion(
                "Confirmar conexión RF",
                f"¿Hay una antena UHF de 50 Ω correctamente conectada a ANT{antena}?\n\n"
                "No inicies lecturas RF con el conector de antena vacío."):
            return
        self.stop_event = threading.Event()
        self.worker = TrabajadorRFID(puerto, antena, pausa, self.eventos, self.stop_event)
        self.activo = True
        self.btn_iniciar.configure(state="disabled")
        self.btn_detener.configure(state="normal")
        self.combo_puerto.configure(state="disabled")
        self.combo_antena.configure(state="disabled")
        self.combo_intervalo.configure(state="disabled")
        self.btn_recargar.configure(state="disabled")
        self._controles_bloqueados(True)
        self._estado("Conectando...", "#eecb87")
        self.worker.start()

    def detener(self):
        if self.stop_event:
            self.stop_event.set()
        self._estado("Deteniendo...", "#eecb87")
        self.btn_detener.configure(state="disabled")

    def limpiar(self):
        if self.escribiendo:
            return
        self.var_epc_actual.set("")
        self.registros.clear()
        self.actualizar_filas.clear()
        self.instantes.clear()
        self.total = 0
        self.rondas = 0
        for item in self.tabla.get_children():
            self.tabla.delete(item)
        self._actualizar_stats()
        self._log("Resultados limpiados. El inventario puede continuar activo.")

    def _registrar_tag(self, tag: Lectura):
        ahora = datetime.now()
        t = time.monotonic()
        registro = self.registros.get(tag.epc)
        if registro is None:
            registro = Registro(tag.epc, 0, tag.antena, tag.rssi_bruto, ahora, ahora, t)
            self.registros[tag.epc] = registro
        registro.count += 1
        registro.ultima = ahora
        registro.ultimo_monotonico = t
        registro.antena = tag.antena
        registro.rssi_raw = tag.rssi_bruto
        self.total += 1
        self.instantes.append(t)
        self.actualizar_filas.add(tag.epc)

    def _tiempo_relativo(self, t: float) -> str:
        seg = max(0, int(time.monotonic() - t))
        if seg < 60:
            return f"{seg} s"
        return f"{seg // 60} m {seg % 60} s"

    def _dibujar_tabla(self):
        # Actualizar solamente los EPC modificados; mantener el último visto arriba.
        pendientes = sorted(self.actualizar_filas,
                            key=lambda epc: self.registros[epc].ultimo_monotonico)
        self.actualizar_filas.clear()
        for epc in pendientes:
            registro = self.registros[epc]
            valores = (
                registro.epc, registro.count, f"ANT{registro.antena}",
                f"0x{registro.rssi_raw:02X} ({registro.rssi_raw})",
                registro.primera.strftime("%H:%M:%S"),
                registro.ultima.strftime("%H:%M:%S"),
                self._tiempo_relativo(registro.ultimo_monotonico),
            )
            if self.tabla.exists(epc):
                self.tabla.item(epc, values=valores)
            else:
                self.tabla.insert("", 0, iid=epc, values=valores)
            self.tabla.move(epc, "", 0)

        # Actualizar la antigüedad aunque la etiqueta no vuelva a aparecer.
        for iid in self.tabla.get_children():
            reg = self.registros.get(iid)
            if reg:
                self.tabla.set(iid, "hace", self._tiempo_relativo(reg.ultimo_monotonico))

    def _actualizar_stats(self):
        limite = time.monotonic() - 5
        while self.instantes and self.instantes[0] < limite:
            self.instantes.popleft()
        self.var_unicos.set(str(len(self.registros)))
        self.var_total.set(str(self.total))
        self.var_rondas.set(str(self.rondas))
        self.var_velocidad.set(f"{len(self.instantes) / 5:.1f} /s".replace(".", ","))

    def _procesar_eventos(self):
        procesados = 0
        while procesados < 3000:
            try:
                tipo, dato = self.eventos.get_nowait()
            except queue.Empty:
                break
            procesados += 1
            if tipo.startswith("temperature_"):
                self._evento_temperatura(tipo, dato)
            elif tipo == "tag":
                self._registrar_tag(dato)
            elif tipo == "round":
                self.rondas += 1
            elif tipo == "power_values":
                self.potencias_actuales = tuple(dato)
                self.var_potencias_detalle.set(
                    "    ".join(f"ANT{i}: {v} dBm" for i, v in enumerate(dato, 1))
                )
                self._mostrar_potencia_actual()
            elif tipo == "power_applied":
                antena, valor = dato
                self._log(f"Potencia verificada: ANT{antena} = {valor} dBm.")
                self._estado("Potencia verificada", ACCENT)
            elif tipo == "power_error":
                self._log("ERROR de potencia: " + dato)
                self._estado("Error de potencia", ERROR)
            elif tipo == "power_done":
                self.operando_potencia = False
                self._controles_bloqueados(False)
                self.btn_iniciar.configure(state="normal")
                self.combo_puerto.configure(state="normal")
                self.combo_antena.configure(state="readonly")
                self.btn_recargar.configure(state="normal")
                if self.var_estado.get() in ("Consultando potencia...", "Aplicando potencia..."):
                    self._estado("Potencia consultada", ACCENT)
                self._log("Consulta/configuración de potencia finalizada. Puerto cerrado.")
            elif tipo == "write_progress":
                cantidad, total, texto = dato
                self.progreso_escritura.configure(value=cantidad, maximum=max(1, total))
                self.var_escritura.set(f"{cantidad}/{total} · {texto}")
                self._log(self.var_escritura.get())
            elif tipo in ("write_success", "write_unchanged"):
                original, tag = dato
                if tipo == "write_success":
                    # Conservar el original como histórico; el EPC nuevo comienza
                    # una medición nueva, incluso si había una fila previa con ese EPC.
                    previo = self.registros.pop(tag.epc, None)
                    if previo:
                        self._log(f"Nueva medición de {tag.epc}; Count previo archivado en diagnóstico: {previo.count}.")
                    self._registrar_tag(tag)
                    self._dibujar_tabla()
                    self.tabla.selection_set(tag.epc)
                    self.tabla.see(tag.epc)
                    self.var_epc_actual.set(tag.epc)
                    texto = f"Escritura EPC exitosa: {tag.epc}. Única etiqueta confirmada por inventario."
                else:
                    texto = "Los EPC son iguales. Sin escritura; EPC confirmado por inventario."
                self.var_escritura.set(texto)
                self._log(texto)
                self._estado("Escritura EPC exitosa" if tipo == "write_success" else "Sin cambios", ACCENT)
            elif tipo == "write_error":
                previo = self.var_escritura.get()
                self.var_escritura.set((previo + "\n" if previo.startswith("ERROR:") else "ERROR: ") + dato)
                self._log("ERROR de escritura: " + dato)
                self._estado("Escritura EPC no confirmada", ERROR)
            elif tipo == "write_done":
                self.escribiendo = False
                self._bloquear_escritura(False)
                self._log("Operación EPC finalizada. Puerto cerrado; lectura detenida.")
            elif tipo == "info":
                self._log(dato)
            elif tipo == "running":
                self._estado("Leyendo EPC", ACCENT)
            elif tipo == "error":
                self._log("ERROR: " + dato)
                self._estado("Error de lectura", ERROR)
            elif tipo == "stopped":
                self.activo = False
                if self.escribiendo:
                    self._log("Inventario detenido. Esperando la salida del hilo para transferir la conexión.")
                    continue
                self.btn_iniciar.configure(state="normal")
                self.btn_detener.configure(state="disabled")
                self.combo_puerto.configure(state="normal")
                self.combo_antena.configure(state="readonly")
                self.combo_intervalo.configure(state="normal")
                self.btn_recargar.configure(state="normal")
                self._controles_bloqueados(False)
                if self.var_estado.get() != "Error de lectura":
                    self._estado("Detenido", MUTED)
                self._log("Puerto serie cerrado.")
        if self.actualizar_filas:
            self._dibujar_tabla()
        elif self.registros:
            # Para no actualizar cientos de filas cada 100 ms.
            if time.monotonic() - self._last_stats > 1:
                self._dibujar_tabla()
        if time.monotonic() - self._last_stats > 0.4:
            self._actualizar_stats()
            self._last_stats = time.monotonic()
        self.after(100, self._procesar_eventos)

    def exportar_csv(self):
        if not self.registros:
            messagebox.showinfo("Exportar", "Todavía no hay etiquetas para exportar.")
            return
        ruta = filedialog.asksaveasfilename(
            title="Guardar lecturas EPC", defaultextension=".csv",
            filetypes=[("Archivo CSV", "*.csv")],
            initialfile=f"rfid_{datetime.now():%Y%m%d_%H%M%S}.csv",
        )
        if not ruta:
            return
        try:
            with open(ruta, "w", newline="", encoding="utf-8-sig") as archivo:
                writer = csv.writer(archivo)
                writer.writerow(["EPC", "Count", "Antena", "RSSI_bruto", "RSSI_hex",
                                 "Primera_lectura", "Ultima_lectura"])
                for reg in sorted(self.registros.values(), key=lambda r: r.ultima, reverse=True):
                    writer.writerow([
                        reg.epc, reg.count, reg.antena, reg.rssi_raw,
                        f"0x{reg.rssi_raw:02X}",
                        reg.primera.isoformat(timespec="seconds"),
                        reg.ultima.isoformat(timespec="seconds"),
                    ])
            self._log(f"CSV exportado: {ruta}")
        except OSError as exc:
            messagebox.showerror("Exportar", str(exc))

    def _cerrar(self):
        if self.escribiendo:
            messagebox.showinfo("Escritura EPC", "Esperá a que termine la operación y la limpieza del filtro antes de cerrar.")
            return
        if self.stop_event:
            self.stop_event.set()
        self.destroy()


if __name__ == "__main__":
    Aplicacion().mainloop()
