
#include <Arduino.h>
#include <Preferences.h>

HardwareSerial RFID(2);
Preferences prefs;

const size_t MAX_LOG = 1200;
uint8_t registro[MAX_LOG];
size_t usados = 0;

bool pendiente = false;
unsigned long ultimoDato = 0;

const uint8_t magic[] = {
  '@', 'D', 'U', 'M', 'P', '@', '\n'
};

size_t coincidencia = 0;
unsigned long ultimoMagic = 0;

void registrar(char origen, uint8_t dato) {
  if (usados + 2 > MAX_LOG) return;

  registro[usados++] = origen;
  registro[usados++] = dato;

  pendiente = true;
  ultimoDato = millis();
}

void guardar() {
  if (!pendiente) return;

  prefs.putBytes("log", registro, usados);
  pendiente = false;
}

void mostrarRegistro() {
  guardar();

  Serial.println("\n=== INICIO REGISTRO ===");

  char anterior = 0;

  for (size_t i = 0; i + 1 < usados; i += 2) {
    char origen = registro[i];
    uint8_t dato = registro[i + 1];

    if (origen != anterior) {
      if (anterior != 0) Serial.println();

      Serial.print(origen == 'T'
        ? "\nPC -> RFID: "
        : "\nRFID -> PC: ");

      anterior = origen;
    }

    if (dato < 16) Serial.print("0");
    Serial.print(dato, HEX);
    Serial.print(" ");
  }

  Serial.println("\n=== FIN REGISTRO ===");
}

void reenviar(uint8_t dato) {
  RFID.write(dato);
  registrar('T', dato);
}

void recibirPC(uint8_t dato) {
  if (coincidencia == 0) {
    if (dato == magic[0]) {
      coincidencia = 1;
      ultimoMagic = millis();
    } else {
      reenviar(dato);
    }
    return;
  }

  if (dato == magic[coincidencia]) {
    coincidencia++;
    ultimoMagic = millis();

    if (coincidencia == sizeof(magic)) {
      coincidencia = 0;
      mostrarRegistro();
    }
    return;
  }

  for (size_t i = 0; i < coincidencia; i++) {
    reenviar(magic[i]);
  }

  coincidencia = 0;

  if (dato == magic[0]) {
    coincidencia = 1;
    ultimoMagic = millis();
  } else {
    reenviar(dato);
  }
}

void setup() {
  Serial.begin(115200);
  RFID.begin(115200, SERIAL_8N1, 16, 17);

  prefs.begin("uhftrace", false);

  size_t n = prefs.getBytesLength("log");

  if (n > 0 && n <= MAX_LOG && n % 2 == 0) {
    usados = prefs.getBytes("log", registro, n);
  }
}

void loop() {
  while (Serial.available()) {
    recibirPC((uint8_t)Serial.read());
  }

  while (RFID.available()) {
    uint8_t dato = (uint8_t)RFID.read();

    Serial.write(dato);
    registrar('R', dato);
  }

  if (coincidencia > 0 &&
      millis() - ultimoMagic > 100) {
    for (size_t i = 0; i < coincidencia; i++) {
      reenviar(magic[i]);
    }
    coincidencia = 0;
  }

  if (pendiente &&
      millis() - ultimoDato > 500) {
    guardar();
  }
}
