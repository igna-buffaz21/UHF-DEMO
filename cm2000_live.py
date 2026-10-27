"""CM2000 Live — monitor EPC para CM2000-4 + ESP32 por puerto COM.

Requisitos: Python 3.9+, pyserial. GUI incluida en Python (tkinter).
Este programa usa únicamente el protocolo A0 que ya funcionó con el lector.
Permite consultar y configurar la potencia de ANT1-ANT4 desde un control explícito.
La escritura de potencia es persistente; no se realiza mientras se arrastra el control.
"""

from __future__ import annotations

import csv
import queue
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
    cuerpo = bytes([0xA0, 3 + len(datos), DIRECCION, comando]) + datos
    return cuerpo + bytes([(-sum(cuerpo)) & 0xFF])


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

    @staticmethod
    def _esperar_respuesta(serie, dec, comando: int, segundos: float = 2.5):
        limite = time.monotonic() + segundos
        while time.monotonic() < limite:
            datos = serie.read(serie.in_waiting or 1)
            for trama in dec.alimentar(datos):
                if trama[2] == DIRECCION and trama[3] == comando:
                    return trama
        return None

    def _enviar_y_esperar(self, serie, dec, comando, datos=b"", segundos=2.5):
        dec.buffer.clear()
        serie.reset_input_buffer()
        tx = crear_comando(comando, datos)
        self._evento("info", f"Potencia TX: {tx.hex(' ').upper()}")
        serie.write(tx)
        serie.flush()
        trama = self._esperar_respuesta(serie, dec, comando, segundos)
        if trama is not None:
            self._evento("info", f"Potencia RX: {trama.hex(' ').upper()}")
        return trama

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
            serie = serial.Serial()
            serie.port = self.puerto
            serie.baudrate = BAUDRATE
            serie.timeout = 0.12
            serie.write_timeout = 1.0
            serie.dtr = False
            serie.rts = False
            try:
                serie.open()
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
            serie = serial.Serial()
            serie.port = self.puerto
            serie.baudrate = BAUDRATE
            serie.timeout = 0.12
            serie.write_timeout = 1.0
            serie.dtr = False
            serie.rts = False
            try:
                serie.open()
                self._evento("info", f"COM abierto: {self.puerto} @ {BAUDRATE}")
                # Algunos ESP32 se reinician cuando se abre el puerto COM.
                if self.detener.wait(2.5):
                    return
                serie.reset_input_buffer()
                dec = DecodificadorTramas()

                serie.write(crear_comando(COMANDO_ANTENA, bytes([self.antena - 1])))
                serie.flush()
                if not self._esperar_respuesta_antena(serie, dec):
                    return
                self._evento("info", f"ANT{self.antena} seleccionada. Inventario continuo iniciado.")
                self._evento("running", True)

                while not self.detener.is_set():
                    # Cada comando solicita UNA ronda. Solo iniciamos otra al
                    # recibir su resumen (0x0A); así evitamos solapar rondas.
                    serie.write(crear_comando(COMANDO_INVENTARIO, b"\x01"))
                    serie.flush()
                    fin = time.monotonic() + 6.0
                    terminada = False
                    while time.monotonic() < fin and not self.detener.is_set():
                        recibidos = serie.read(serie.in_waiting or 1)
                        for trama in dec.alimentar(recibidos):
                            if trama[2] != DIRECCION or trama[3] != COMANDO_INVENTARIO:
                                continue
                            if trama[1] == 0x04:
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
                    if self.detener.is_set():
                        break
                    if not terminada:
                        self._evento("error", "No llegó el fin de ronda 0x89. Se detuvo para evitar comandos solapados.")
                        break
                    if self.detener.wait(self.pausa_ms / 1000):
                        break
            finally:
                if serie.is_open:
                    serie.close()
        except (serial.SerialException, OSError, ValueError) as exc:
            self._evento("error", f"Error del puerto serie: {exc}")
        finally:
            self._evento("stopped", None)


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
        self.geometry("1120x760")
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

        self._estilos()
        self._construir()
        self.refrescar_puertos()
        self.after(100, self._procesar_eventos)
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
        top = tk.Frame(self, bg=BG)
        top.pack(fill="x", padx=24, pady=(22, 15))
        self._etiqueta(top, "CM2000  /  LIVE", 21, "bold").pack(side="left")
        self._etiqueta(top, "  Monitor de etiquetas UHF", 11, fg=MUTED).pack(side="left", pady=(8, 0))
        self.estado_label = self._etiqueta(top, "●  Desconectado", 11, "bold", fg=MUTED)
        self.estado_label.pack(side="right", pady=8)

        toolbar = tk.Frame(self, bg=PANEL, highlightbackground=BORDER,
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

        potencia_panel = tk.Frame(self, bg=PANEL, highlightbackground=BORDER,
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

        tarjetas = tk.Frame(self, bg=BG)
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

        tabla_container = tk.Frame(self, bg=PANEL, highlightbackground=BORDER,
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

        bottom = tk.Frame(self, bg=BG)
        bottom.pack(fill="x", padx=24, pady=(0, 10))
        self._boton(bottom, "Limpiar resultados", self.limpiar).pack(side="left")
        self._boton(bottom, "Exportar CSV", self.exportar_csv).pack(side="left", padx=9)
        self._etiqueta(bottom, "Una antena física conectada al puerto seleccionado; no cambiar en caliente.",
                       9, fg=MUTED).pack(side="right")

        logwrap = tk.Frame(self, bg=PANEL, highlightbackground=BORDER, highlightthickness=1)
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

    def _operar_potencia(self, operacion: str):
        if self.activo or self.operando_potencia or (self.worker and self.worker.is_alive()):
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
            if not messagebox.askyesno(
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
        if self.operando_potencia or (self.worker is not None and self.worker.is_alive()):
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
        if not messagebox.askyesno(
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
            if tipo == "tag":
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
            elif tipo == "info":
                self._log(dato)
            elif tipo == "running":
                self._estado("Leyendo EPC", ACCENT)
            elif tipo == "error":
                self._log("ERROR: " + dato)
                self._estado("Error de lectura", ERROR)
            elif tipo == "stopped":
                self.activo = False
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
        if self.stop_event:
            self.stop_event.set()
        self.destroy()


if __name__ == "__main__":
    Aplicacion().mainloop()
