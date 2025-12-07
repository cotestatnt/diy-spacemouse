#line 1 "D:\\diy-spacemouse\\useless\\test_vl53l0x\\src\\ServoConsole.h"
#ifndef SERVO_CONSOLE_H
#define SERVO_CONSOLE_H

#include <Arduino.h>

// Definizioni necessarie
typedef struct {
  int     servoIndex;
  uint8_t servoPin;
} ISR_servo_t;

// Classe per il controllo dei servo via seriale
class ServoConsole {
private:
  const ISR_servo_t* servos;
  int numServos;

public:
  ServoConsole(const ISR_servo_t* _servos, int _numServos) 
    : servos(_servos), numServos(_numServos) {}

  void begin() {
    Serial.println("\n=== Servo Console Ready ===");
    Serial.println("Comandi disponibili:");
    Serial.println("  s <servo> <posizione>  - Imposta posizione servo (0-180)");
    Serial.println("  l                       - Lista servo");
    Serial.println("  h                       - Aiuto");
    Serial.println("==============================\n");
  }

  void handleSerialInput() {
    if (!Serial.available()) return;

    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() == 0) return;

    char command = input[0];

    switch (command) {
      case 's':
      case 'S':
        handleSetServo(input);
        break;
      case 'l':
      case 'L':
        listServos();
        break;
      case 'h':
      case 'H':
        showHelp();
        break;
      default:
        Serial.println("Comando non riconosciuto. Digita 'h' per aiuto.");
    }
  }

private:
  void handleSetServo(const String& input) {
    int firstSpace = input.indexOf(' ');
    if (firstSpace == -1) {
      Serial.println("Errore: comando incompleto. Uso: s <servo> <posizione>");
      return;
    }

    int secondSpace = input.indexOf(' ', firstSpace + 1);
    if (secondSpace == -1) {
      Serial.println("Errore: comando incompleto. Uso: s <servo> <posizione>");
      return;
    }

    String servoStr = input.substring(firstSpace + 1, secondSpace);
    String posStr = input.substring(secondSpace + 1);

    int servoNum = servoStr.toInt();
    int position = posStr.toInt();

    if (servoNum < 0 || servoNum >= numServos) {
      Serial.print("Errore: servo ");
      Serial.print(servoNum);
      Serial.print(" non valido. Range: 0-");
      Serial.println(numServos - 1);
      return;
    }

    if (position < 0 || position > 180) {
      Serial.println("Errore: posizione deve essere tra 0 e 180 gradi.");
      return;
    }

    int servoIndex = servos[servoNum].servoIndex;
    if (servoIndex < 0) {
      Serial.print("Errore: servo ");
      Serial.print(servoNum);
      Serial.println(" non inizializzato.");
      return;
    }

    NRF52_ISR_Servos.setPosition(servoIndex, position);

    Serial.print("Servo ");
    Serial.print(servoNum);
    Serial.print(" impostato a ");
    Serial.print(position);
    Serial.println(" gradi");
  }

  void listServos() {
    Serial.println("\n=== Lista Servo ===");
    for (int i = 0; i < numServos; i++) {
      Serial.print("Servo ");
      Serial.print(i);
      Serial.print(": Pin ");
      Serial.print(servos[i].servoPin);
      Serial.print(" (Indice: ");
      Serial.print(servos[i].servoIndex);
      Serial.println(")");
    }
    Serial.println("===================\n");
  }

  void showHelp() {
    Serial.println("\n=== Aiuto Servo Console ===");
    Serial.println("Comandi disponibili:");
    Serial.println("  s <servo> <posizione>");
    Serial.println("    Imposta la posizione di un servo");
    Serial.println("    <servo>: numero del servo (0-" + String(numServos - 1) + ")");
    Serial.println("    <posizione>: gradi (0-180)");
    Serial.println("    Esempio: s 0 90");
    Serial.println("");
    Serial.println("  l");
    Serial.println("    Elenca tutti i servo disponibili");
    Serial.println("");
    Serial.println("  h");
    Serial.println("    Mostra questo aiuto");
    Serial.println("============================\n");
  }
};

#endif
