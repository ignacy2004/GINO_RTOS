# Podsumowanie Przetestowanych Funkcjonalności (GINO_RTOS)

Krótki opis aktualnie działających i przetestowanych na robocie elementów oprogramowania wbudowanego **GINO_RTOS** (stan przed dzisiejszymi zmianami).

---

## 1. Architektura Systemu (FreeRTOS)
* **Mikrokontroler:** STM32F401CCU6 (84 MHz) z systemem **FreeRTOS** (CMSIS-RTOS v2).
* **Podział na zadania (Tasks):**
  * **`MotorTask` (`High`)** – obsługa maszyny stanów 3 silników krokowych (`J1`, `J2`, `Z`), generacja impulsów `STEP/DIR/EN` oraz kontrola krańcówek i limitów maksymalnych (`MAX_STEPS`).
  * **`DispatcherTask` (`AboveNormal`)** – inicjalizacja USB CDC, monitorowanie przycisku bezpieczeństwa `E-STOP` (`PB14`) oraz odbieranie i wykonywanie rozkazów z kolejki `CommandQueue`.
  * **`KinematicsTask` (`Normal`)** – moduł pod obliczenia kinematyki prostej i odwrotnej SCARA.
  * **`ToolTask` (`Normal`)** oraz **`CommTask` (`BelowNormal`)** – obsługa narzędzia i sygnalizacji LED (`PB8`).
* **Cyfrowy Bliźniak (`system_state`):** Współdzielona struktura `RobotState_t` chroniona mutexem FreeRTOS, przechowująca bieżącą pozycję w krokach, współrzędne kartezjańskie, stan bazowania (`isHomed`), stan pompy (`pumpState`) oraz flagę awaryjną (`emergencyStop`).

---

## 2. Komunikacja USB CDC i Protokół Binarny
* Komunikacja z aplikacją PC (**GINO_HMI**) odbywa się przez natywny port **USB CDC (Virtual COM Port)**.
* **Parser ramek binarnych (`serial_interface`):**
  * Odbiór bajt po bajcie w przerwaniu USB z synchronizacją na nagłówku `0xAA 0x55` oraz weryfikacją sumy kontrolnej **XOR (`CRC`)**.
  * Poprawne ramki trafiają bezblokująco do kolejki FreeRTOS `CommandQueue` i są przekazywane do modułu `dispatcher`.

---

## 3. Sterowanie Napędami (`tmc2209_stepper`) i Ruch Ręczny (Joint Move)
* **Sterowanie w przestrzeni przegubów (`CMD_JOINT_MOVE = 0x01`):**
  * Robot odbiera zadane kąty $J_1, J_2$ [°] oraz pozycję osi $Z$ [mm] z suwaków zakładki **Manual Control** w aplikacji HMI.
  * `Dispatcher` przelicza wartości na kroki według przełożeń i mikrokroków zdefiniowanych w `config.h` (`STEPS_PER_DEG_J1`, `STEPS_PER_DEG_J2`, `STEPS_PER_MM_Z`) i płynnie przemieszcza wszystkie 3 osie do pozycji docelowej (`STATE_MOVING`).
* **Limity programowe (Soft Limits):**
  * Zabezpieczenie przed wyjechaniem poza maksymalny zakres roboczy poszczególnych osi (`J1_MAX_STEPS`, `J2_MAX_STEPS`, `Z_MAX_STEPS`).

---

## 4. Obsługa Krańcówek i Bazowanie (`CMD_HOME = 0x03`)
* **Ochrona krańcówkowa podczas jazdy (`STATE_MOVING`):**
  * Najechanie na krańcówkę podczas ruchu w jej kierunku natychmiast zatrzymuje oś i zeruje jej pozycję (`0`), jednocześnie pozwalając na swobodny odjazd w kierunku przeciwnym.
* **Procedura bazowania (`CMD_HOME = 0x03` / `STATE_HOMING`):**
  * Po kliknięciu przycisku **HOME** w aplikacji HMI wszystkie osie jadą w kierunku swoich wyłączników krańcowych.
  * Po wyzwoleniu krańcówki każda oś zatrzymuje się, przypisuje pozycję zerową (`currentPosition = 0`, `targetPosition = 0`, `stepsToMove = 0`) i ustawia flagę zbazowania (`isHomed = true`).

---

## 5. Efektor Końcowy i Bezpieczeństwo (`E-STOP`)
* **Sterowanie pompą próżniową (`tool_controller`, `CMD_EFFECTOR = 0x04 / 0x05`):**
  * Załączanie i wyłączanie wyjścia MOSFET pompy (`PB13`) z poziomu rozkazów HMI.
* **Zatrzymanie awaryjne (`CMD_ESTOP = 0x10 / 0xFF` oraz fizyczny przycisk `PB14`):**
  * Natychmiastowe zatrzymanie wszystkich silników krokowych, wyłączenie pompy próżniowej i zablokowanie wykonywania kolejnych ruchów do czasu ponownego zbazowania (`CMD_HOME`).
