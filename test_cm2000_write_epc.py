"""Pruebas sin hardware: serialización, barreras de envío y lectura de verificación."""

import contextlib
import io
import unittest
from unittest.mock import patch

import cm2000_write_epc as epc


ACTUAL = "E2000017221101441890ABCD"
NUEVO = "112233445566778899AABBCC"


class SerieFalsa:
    def __init__(self, respuestas):
        self.respuestas = list(respuestas)
        self.enviadas = []

    def write(self, datos):
        self.enviadas.append(datos)
        return len(datos)

    def flush(self):
        pass

    @property
    def in_waiting(self):
        return len(self.respuestas[0]) if self.respuestas else 0

    def read(self, cantidad):
        if self.respuestas:
            return self.respuestas.pop(0)
        return b""


def lectura(epc_hex):
    return epc.crear_comando(0x89, b"\x00\x30\x00" + bytes.fromhex(epc_hex) + b"\x40")


class PruebasEPC(unittest.TestCase):
    def setUp(self):
        self.salida = contextlib.redirect_stdout(io.StringIO())
        self.salida.__enter__()
        self.addCleanup(self.salida.__exit__, None, None, None)

    def test_validacion_estricta(self):
        self.assertEqual(epc.validar_epc(NUEVO.lower()), bytes.fromhex(NUEVO))
        for valor in ("", NUEVO[:-1], NUEVO + "00", "0x" + NUEVO, " " + NUEVO, "G" * 24):
            with self.subTest(valor=valor), self.assertRaises(ValueError):
                epc.validar_epc(valor)
        for valor in ("", "0000000", "0000 0000", "GG000000"):
            with self.assertRaises(ValueError):
                epc.validar_password(valor)

    def test_vector_sdk_a55a_no_es_a0(self):
        trama = epc.construir_trama_sdk(bytes.fromhex(ACTUAL), bytes.fromhex(NUEVO), b"\0" * 4)
        # Vector derivado del desensamblado, no una captura del lector.
        self.assertEqual(trama, bytes.fromhex(
            "A5 5A 0030 86 00000000 01 0020 0060 " + ACTUAL
            + " 01 0001 0007 3000 " + NUEVO + " 61 0D0A"
        ))
        self.assertEqual(len(trama), 48)
        self.assertEqual(epc.DecodificadorTramas().alimentar(trama), [])

    def test_ack_sdk_y_errores(self):
        epc.interpretar_respuesta_sdk(bytes.fromhex("A5 5A 000A 87 0100 8C 0D0A"))
        for trama in (bytes.fromhex("A5 5A 000A 87 0001 8C 0D0A"),
                      bytes.fromhex("A5 5A 000A 87 0100 00 0D0A"),
                      bytes.fromhex("A5 5A 000A 86 0100 8D 0D0A"),
                      bytes.fromhex("A5 5A 000B 87 0100 8D 0D0A"),
                      epc.crear_comando(0x86, b"\x10")):
            with self.subTest(trama=trama), self.assertRaises(epc.ErrorRFID):
                epc.interpretar_respuesta_sdk(trama)

    def test_rechaza_intercepcion_esp32(self):
        for actual, nuevo, pwd in ((b"@" + b"\0" * 11, bytes.fromhex(NUEVO), b"\0" * 4),
                                   (bytes.fromhex(ACTUAL), b"@" + b"\0" * 11, b"\0" * 4),
                                   (bytes.fromhex(ACTUAL), bytes.fromhex(NUEVO), b"@\0\0\0")):
            with self.assertRaisesRegex(ValueError, "0x40"):
                epc.construir_trama_sdk(actual, nuevo, pwd)

    def test_bloqueo_escritura_antes_de_abrir_puerto(self):
        with patch.object(epc.serial, "Serial") as abrir, contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(epc.main(["--epc-actual", ACTUAL, "--epc-nuevo", NUEVO, "--escribir"]), 2)
            abrir.assert_not_called()
        with self.assertRaises(epc.ErrorRFID):
            epc.ConexionLectura(SerieFalsa([])).enviar(0x86, b"")

    def test_simulacion_no_abre_puerto(self):
        with patch.object(epc.serial, "Serial") as abrir:
            self.assertEqual(epc.main(["--epc-actual", ACTUAL, "--epc-nuevo", NUEVO,
                                      "--password", "00000000", "--unidad-filtro", "bits"]), 0)
            abrir.assert_not_called()

    def test_inventario_fragmentado_y_multiples_epc(self):
        respuesta = lectura(ACTUAL) + lectura(NUEVO) + epc.crear_comando(0x89, b"\0" * 7)
        serie = SerieFalsa([respuesta[:3], respuesta[3:17], respuesta[17:]])
        self.assertEqual(epc.ConexionLectura(serie).inventariar(1), {ACTUAL, NUEVO})
        self.assertEqual(serie.enviadas, [epc.crear_comando(0x89, b"\x01")])

    def test_errores_checksum_y_timeout(self):
        malo = bytearray(lectura(NUEVO))
        malo[-1] ^= 1
        for respuesta in (bytes(malo), epc.crear_comando(0x89, b"\x01")):
            with self.assertRaises(epc.ErrorRFID):
                epc.ConexionLectura(SerieFalsa([respuesta])).inventariar(1)
        serie = SerieFalsa([])
        with patch.object(epc.time, "monotonic", side_effect=[0, 7]), self.assertRaisesRegex(epc.ErrorRFID, "Timeout"):
            epc.ConexionLectura(serie).inventariar(1)
        self.assertEqual(len(serie.enviadas), 1)

    def test_verificacion_exige_tres_rondas_y_cierra_puerto(self):
        objetivo = bytes.fromhex(NUEVO)
        for rondas, falla in (([{NUEVO}] * 3, False),
                              ([{NUEVO}, {ACTUAL, NUEVO}], True),
                              ([set()], True), ([{ACTUAL}], True)):
            with self.subTest(rondas=rondas), \
                    patch.object(epc.serial, "Serial") as abrir, \
                    patch.object(epc, "ConexionLectura") as conexion, \
                    patch.object(epc.time, "sleep"), \
                    patch.object(epc.time, "monotonic", side_effect=[0, 7]):
                conexion.return_value.inventariar.side_effect = rondas
                if falla:
                    with self.assertRaises(epc.ErrorRFID):
                        epc.verificar_epc("/dev/falso", 1, objetivo)
                else:
                    epc.verificar_epc("/dev/falso", 1, objetivo)
                    self.assertEqual(conexion.return_value.inventariar.call_count, 3)
                abrir.return_value.close.assert_called_once()
                abrir.return_value.write.assert_not_called()


if __name__ == "__main__":
    unittest.main()
