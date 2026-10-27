# Análisis estático de UHFAPI.dll

Archivo analizado: UHFAPI.dll, 519680 bytes, PE32 i386 nativo, sin ejecutar.
SHA-256: `e0024c9722a0a6788b53c263caef951f1d4c91870ee753826f71fd5d8968e015`.
Herramienta: GNU objdump, sintaxis Intel. Direcciones virtuales con ImageBase
`10000000`; restar esa base para obtener RVA. No son offsets del archivo.

## Hallazgo principal

La ruta exportada `UHFWriteData_SendData` **no usa A0**. Construye parámetros de
escritura 86 y llama a un constructor **A5 5A**. La ruta síncrona `UHFWriteData`
también utiliza A5 5A. No hay evidencia en estas rutas de que el mismo comando
86 ni sus parámetros puedan trasladarse al encapsulado A0 del monitor.

| Campo | Monitor existente | Ruta de escritura de esta DLL |
|---|---|---|
| Cabecera | A0 | A5 5A |
| Longitud | 1 byte, total menos 2 | 2 bytes BE, longitud total |
| Dirección | 01 | Sin campo ADDR en esta trama |
| Escritura | Sin documentar | 86 |
| Respuesta de escritura | Sin documentar | 87, payload 01 00 aceptado |
| Checksum | Complemento de suma, incluye cabecera | XOR desde longitud hasta fin de payload |
| Terminador | Sin terminador separado | 0D 0A |

## Evidencia de construcción

`UHFWriteData_SendData`: VA `10035560` (RVA `35560`).

- Copia 4 bytes de contraseña, banco de filtro y dirección/longitud BE.
- En `10035683–100356B9`, divide FILTER_LEN por 8 y copia un byte adicional
  cuando hay resto. Confirma que FILTER_LEN está en **bits**.
- FILTER_ADDR se copia BE sin conversión; la rutina por sí sola no demuestra su
  unidad. La dirección 32 bits para EPC corresponde al mapa de memoria descrito
  en el contexto de la prueba; falta confirmar su semántica en el lector real.
- Copia banco de destino y dirección BE. WRITE_LEN ocupa dos bytes, aunque
  esta función recibe un contador de un byte, y copia `2 * WRITE_LEN` bytes.
- En `100357BB`: `push 0x86`; en `100357C0`: `call 0x10015890`.

Constructor compartido: VA `10015890` (RVA `15890`).

```text
100158AE  mov BYTE PTR [eax],0xa5
100158C0  mov BYTE PTR [edx],0x5a
100158CF  add ecx,0x8       ; payload + 8 = longitud total
100158D2  shr ecx,0x8      ; longitud big-endian
10015946  mov [ebp-0x8],2  ; checksum comienza en longitud
1001596D  xor ecx,eax      ; XOR de longitud, comando y datos
1001598E  mov BYTE PTR [edx],0xd
100159A0  mov BYTE PTR [ecx],0xa
```

Formato observado:

```text
A5 5A | TOTAL_LEN_BE16 | 86 | PASSWORD(4) |
FILTER_BANK(1) | FILTER_ADDR_BE16 | FILTER_LEN_BITS_BE16 |
FILTER_DATA(ceil(bits/8)) | WRITE_BANK(1) | WRITE_ADDR_BE16 |
WRITE_LEN_BE16 | WRITE_DATA(2*words) | XOR | 0D 0A
```

`UHFWriteData` síncrona está en `100302A0`. En `1003050E–10030513` pasa 86 a
`10015770`, que llama a `10015340`. Esta última rutina también comienza
escribiendo A5 5A (`10015385`, `10015399`). No es una vía A0 alternativa.

## Evidencia de respuesta

`UHFWriteData_RecvData`: VA `100357F0` (RVA `357F0`). Pasa 86 al parser
`100159C0`. Este valida A5 5A, longitud, XOR y terminación; en `10015C67–10015C74`
exige comando recibido igual al solicitado más uno: **87**.

En `1003582F–10035846`, el wrapper exige los primeros dos bytes de payload
**01 00** para devolver cero. El parser offline implementado acepta exactamente
esos dos bytes; la DLL no establece aquí una longitud exacta de payload y no
ofrece una tabla de significados de otros estados. No se asignan significados
inventados a los demás valores ni se toma el ACK como verificación de memoria.

Ejemplo de ACK mínimo reconstruido, no capturado en hardware:

```text
A5 5A 00 0A 87 01 00 8C 0D 0A
```

El STOP del SDK también pertenece a esta familia: `UHFStopInventory_SendData`,
VA `100350E0`, pasa **8C** al constructor A5 5A. No autoriza enviar 8C dentro de A0.
Además, `UHFSetANT_SendData` en `100369C0` utiliza **28**, mientras el monitor
selecciona antena con **74**. Esto refuerza la diferencia entre ambas familias.

## Consecuencia para el proyecto

Se elimina la simulación híbrida A0 + payload 86 que solo seguía la propuesta
inicial. El simulador ahora reproduce el encapsulado de esta DLL sin transmitir.
La verificación de lectura sigue usando exclusivamente el protocolo A0 existente.

Para habilitar escritura se necesita documentación/SDK de escritura **A0**
correspondiente al firmware conectado, o confirmar primero por consultas de
lectura documentadas que el módulo admite también A5 5A. El binario no demuestra
ese soporte del hardware ni que el filtro por EPC identifique una única etiqueta.

## Cómo reproducir la inspección en Linux

```bash
sha256sum UHFAPI.dll
objdump -d -Mintel --start-address=0x10035560 --stop-address=0x10035869 UHFAPI.dll
objdump -d -Mintel --start-address=0x10015890 --stop-address=0x100159b8 UHFAPI.dll
objdump -d -Mintel --start-address=0x10015c5f --stop-address=0x10015ca2 UHFAPI.dll
```
