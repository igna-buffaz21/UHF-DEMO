# Preparación de escritura EPC CM2000-4

Estado: escritura real bloqueada por incompatibilidad de protocolo detectada en
la DLL aportada. No se modificaron cm2000_live.py ni esp32/esp32.ino. No se ejecutó
la DLL ni se enviaron comandos al hardware. No hay Kill, Lock ni Permalock.

## Resultado del análisis

UHFAPI.dll construye la escritura 86 dentro de **A5 5A**, con longitud total de
2 bytes big-endian, sin ADDR, checksum XOR y terminación 0D 0A. El monitor usa
**A0**, longitud de 1 byte, dirección 01 y checksum aditivo. No se puede trasladar
el payload de la DLL a A0 sin confirmar el protocolo de escritura del lector.

La longitud del filtro está en bits: el código copia ceil(FILTER_LEN / 8) bytes.
La respuesta del SDK usa comando 87 y acepta payload 01 00. Esto no documenta una
respuesta equivalente en A0. Las direcciones, hash y evidencia reproducible del
desensamblado están en [UHFAPI_ANALISIS.md](UHFAPI_ANALISIS.md).

El simulador anterior combinaba A0 con el payload propuesto. Fue corregido:
ahora construye exclusivamente una trama SDK A5 5A para inspección offline.
La verificación por lectura continúa usando A0. No hay envío de escritura en
ningún modo y no existe una opción para omitir el bloqueo.

## Simulación desde Linux

Reutiliza pyserial y, por importar el monitor, tkinter. No abre una ventana.
Ambos EPC deben tener exactamente 24 dígitos hexadecimales, sin espacios ni 0x.
La contraseña de acceso no tiene valor por defecto.

```bash
.venv/bin/python cm2000_write_epc.py \
  --epc-actual E2000017221101441890ABCD \
  --epc-nuevo 112233445566778899AABBCC \
  --password 00000000 --simular
```

La contraseña cero es solo un ejemplo; usar la real. No se abre el puerto.
`--unidad-filtro bits` sigue aceptándose; bytes se rechaza tras analizar la DLL.
Sin modo explícito también se ejecuta solo la simulación.

Para estos datos, la salida reconstruida de la DLL es:

```text
A5 5A 00 30 86 00 00 00 00 01 00 20 00 60
E2 00 00 17 22 11 01 44 18 90 AB CD
01 00 01 00 07 30 00 11 22 33 44 55 66 77 88 99 AA BB CC 61 0D 0A
```

Tiene 48 bytes, longitud 0030 y XOR 61. No es una captura del lector ni debe
transmitirse manualmente. La salida incluye la contraseña completa.

El bloque PC + EPC sigue el algoritmo de writeDataToEpc descrito por el usuario:
PC 3000 y 12 bytes EPC, 7 palabras en banco 1 desde palabra 1. Ese algoritmo pone
a cero los demás bits PC; la DLL de transporte no confirma su compatibilidad con
la etiqueta utilizada. El filtro EPC usa dirección 32 y longitud 96, interpretadas
en bits; falta confirmar la semántica de dirección y el soporte A5 5A del lector.

El puente ESP32 intercepta 40 (`@`). El simulador rechaza cualquier trama que
contenga ese byte, incluida contraseña o checksum, para evitar consumo de
@DUMP@ seguido de salto de línea o demoras de 100 ms en medio de una trama.

## Verificación independiente por lectura

1. Detener el inventario y cerrar el monitor y otras herramientas seriales.
   No se manda un STOP desconocido en A0. El script espera más que una ronda
   conocida y rechaza cualquier tráfico previo.
2. Conectar una antena compatible y dejar físicamente una sola etiqueta en el
   campo, retirando también las etiquetas que tengan EPC duplicados.
3. Para verificar el EPC ya grabado mediante una herramienta del fabricante:

```bash
.venv/bin/python -m serial.tools.list_ports
.venv/bin/python cm2000_write_epc.py \
  --epc-actual E2000017221101441890ABCD \
  --epc-nuevo 112233445566778899AABBCC \
  --puerto /dev/ttyUSB0 --antena 1 --verificar
```

El EPC actual se valida pero no selecciona una etiqueta en este modo. Se usa
inventario sin filtro para detectar otros EPC visibles. Selecciona antena con 74
y ejecuta tres rondas 89 completas; cada una debe contener exclusivamente el EPC
nuevo. Se registran TX, RX bruto y tramas válidas en consola. Esto comprueba la
observación del EPC solicitado, no la identidad física ni una escritura del script.

Código 0: simulación completada o lectura coincidente, según el modo. Código 2:
parámetros inválidos, escritura bloqueada, puerto ocupado/inaccesible, tráfico
previo, timeout, checksum incorrecto, estado/error o EPC distinto. Código 130:
interrupción. El puerto se cierra ante errores y no hay reintentos automáticos.

## Lo necesario para completar escritura

- Manual/SDK de escritura A0 correspondiente al firmware real, o evidencia mediante
  consultas de lectura documentadas de que este lector admite también A5 5A.
- Confirmar la selección: un filtro por EPC coincide con duplicados y puede
  coincidir con un EPC más largo del mismo prefijo. El inventario deduplicado no
  cuenta etiquetas físicas; se requiere aislamiento para la prueba o selección
  inequívoca documentada, por ejemplo mediante identidad TID cuando sea aplicable.
- Probar el ACK y verificar memoria tras una única escritura con confirmación
  explícita. Un ACK favorable por sí solo no confirma el EPC. Ante timeout no se
  debe reintentar automáticamente: puede haber escritura parcial.

## Pruebas sin hardware

```bash
.venv/bin/python -m unittest -v test_cm2000_write_epc
```

Cubren el vector A5 5A reconstruido, ACK XOR, estados inesperados, bloqueo de
escritura antes de abrir el puerto, validación de EPC, intercepción del puente,
lectura fragmentada, timeouts y rechazo de verificación con EPC distintos.
