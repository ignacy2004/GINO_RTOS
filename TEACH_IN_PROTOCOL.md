# Specyfikacja Protokołu Binarnego 16B: Teach-In oraz Wykonywanie Programu SCADA (Handshake ACK)

Ten dokument opisuje zaktualizowany protokół komunikacji binarnej **16-bajtowej (z 3 parametrami `float`)**, obsługę trybu **Teach-In** oraz mechanizm wykonywania programu krok po kroku ze zwrotkami potwierdzenia (**Handshake ACK**) w firmware **STM32 (`GINO_RTOS`)**. Możesz przekazać ten plik bezpośrednio agentowi od kodu **Python HMI (`GINO_HMI`)**.

---

## 1. Struktura Ramki Binarnej (16 Bajtów – 3 Parametry)

Każda ramka binarna (zarówno wysyłana z **PC $\rightarrow$ STM32**, jak i odbierana ze **STM32 $\rightarrow$ PC**) ma dokładnie **16 bajtów** i zawiera **3 parametry typu `float` (`param1`, `param2`, `param3`)**.

### Struktura `CommandFrame_t` w C (`serial_interface.h`):
```c
#pragma pack(push, 1)
typedef struct {
    uint8_t sync1;   // 0xAA
    uint8_t sync2;   // 0x55
    uint8_t cmd;     // ID komendy
    float   param1;  // Parametr 1 (4B float, Little-Endian)
    float   param2;  // Parametr 2 (4B float, Little-Endian)
    float   param3;  // Parametr 3 (4B float, Little-Endian)
    uint8_t crc;     // XOR pierwszych 15 bajtów (0..14)
} CommandFrame_t;    // Rozmiar całkowity: 16 bajtów
#pragma pack(pop)
```

### Mapa Bajtów Ramki (16 B) i Format `struct` w Pythonie:
* Format pakowania w Pythonie (bez CRC): **`<BBBfff`** (15 bajtów)
* Format całej ramki z CRC: **`<BBBfffB`** (16 bajtów)

| Offset | Pole | Typ (C / Python) | Rozmiar | Opis |
| :---: | :--- | :---: | :---: | :--- |
| `0` | `sync1` | `uint8_t` / `B` | 1 B | Stały bajt synchronizacji 1: **`0xAA`** (170) |
| `1` | `sync2` | `uint8_t` / `B` | 1 B | Stały bajt synchronizacji 2: **`0x55`** (85) |
| `2` | `cmd` | `uint8_t` / `B` | 1 B | Kod rozkazu / odpowiedzi |
| `3..6` | `param1` | `float` / `f` | 4 B | Parametr 1 (IEEE 754 Little-Endian) |
| `7..10` | `param2` | `float` / `f` | 4 B | Parametr 2 (IEEE 754 Little-Endian) |
| `11..14` | `param3` | `float` / `f` | 4 B | Parametr 3 (IEEE 754 Little-Endian) |
| `15` | `crc` | `uint8_t` / `B` | 1 B | Suma kontrolna **XOR** z bajtów `0..14` (15 bajtów) |

---

## 2. Tabela Wszystkich Komend Binarnych

```c
#define CMD_JOINT_MOVE      0x01  // PC -> STM32: Ruch przegubowy (J1 [deg], J2 [deg], Z [mm])
#define CMD_LINEAR_MOVE     0x02  // PC -> STM32: Ruch kartezjański (X [mm], Y [mm], Z [mm])
#define CMD_HOME            0x03  // PC -> STM32: Bazowanie wszystkich osi (0.0, 0.0, 0.0)
#define CMD_EFFECTOR_HMI    0x04  // PC -> STM32: Sterowanie narzędziem/pompą (tool_id, state 0/1, duration_ms)
#define CMD_EFFECTOR_ACTION 0x05  // PC -> STM32: Alias dla akcji efektora
#define CMD_TEACH_MODE      0x06  // PC -> STM32: Zwalnianie / blokowanie napędów w trybie Teach-In
#define CMD_TEACH_READ      0x07  // PC -> STM32: Bazowanie pomiarowe Teach-In (zliczanie kroków)
#define CMD_TEACH_RESPONSE  0x08  // STM32 -> PC: Ramka zwrotna z nauczoną pozycją (X, Y, Z) w mm
#define CMD_DELAY           0x09  // PC -> STM32: Odczekaj zadany czas w ms (param1 = delay_ms)
#define CMD_ACK             0x0A  // STM32 -> PC: Zwrotka potwierdzająca zakończenie wykonania komendy
#define CMD_ESTOP           0x10  // PC -> STM32: Zatrzymanie awaryjne
#define CMD_ESTOP_LEGACY    0xFF  // PC -> STM32: Zatrzymanie awaryjne (alias HMI)
```

---

## 3. Wykonywanie Programu z Bazy SCADA (Stop-and-Wait / Handshake `CMD_ACK = 0x0A`)

Wykonywanie wczytanego programu z zakładki **SCADA** działa w trybie **krok po kroku ze zwrotką (`CMD_ACK`)**:
1. **HMI wysyła 1 ramkę** odpowiadającą aktualnemu krokowi z tabeli programu (`row = 0`).
2. **STM32 wykonuje polecenie** (jedzie silnikami do celu, bazuje się, włącza/wyłącza pompę lub odlicza czas `DELAY`).
3. **Dopiero po fizycznym zakończeniu polecenia STM32 odsyła ramkę `CMD_ACK (0x0A)`**.
4. **HMI po odebraniu `CMD_ACK (0x0A)`** ze statusem `OK (1.0)` przechodzi do kolejnego wiersza tabeli (`row + 1`) i wysyła następną ramkę aż do końca programu!

### 3.1. Format Ramki Zwrotnej `CMD_ACK (0x0A)` (STM32 $\rightarrow$ PC):
* **`cmd`:** `0x0A` (`CMD_ACK`)
* **`param1`:** Kod zakończonej komendy jako `float` (np. `2.0` dla `CMD_LINEAR_MOVE`, `4.0` dla `CMD_EFFECTOR_HMI`, `9.0` dla `CMD_DELAY`)
* **`param2`:** Status wykonania:
  * **`1.0` (`ACK_OK`)** – Polecenie wykonane pomyślnie, robot gotowy na kolejny krok programu.
  * **`0.0` (`ACK_ERROR`)** – Błąd wykonania (np. punkt $(X, Y, Z)$ w `CMD_LINEAR_MOVE` poza zasięgiem kinematyki odwrotnej). W takim przypadku HMI powinno zatrzymać wykonywanie programu i wyświetlić komunikat błędu.
* **`param3`:** `0.0`

### 3.2. Mapowanie Kroków Programu SCADA na Ramki Binarne (PC $\rightarrow$ STM32):

W tabeli kroków SCADA (`tab_scada.py` / baza SQLite `STEPS`) występują następujące typy akcji (`action_type`), które HMI powinno wysyłać jako poniższe ramki:

| `action_type` w bazie SQLite | Komenda binarna (`cmd`) | `param1` | `param2` | `param3` | Kiedy STM32 odsyła `CMD_ACK (0x0A)`? |
| :--- | :--- | :---: | :---: | :---: | :--- |
| **`MOVE`** | `CMD_LINEAR_MOVE` (`0x02`) | `x` [mm] | `y` [mm] | `z` [mm] | Gdy wszystkie 3 silniki dojadą do punktu i przejdą w `STATE_IDLE` |
| **`PUMP_ON`** / **`GRIPPER_CLOSE`** | `CMD_EFFECTOR_HMI` (`0x04`) | `1.0` (ID) | `1.0` (ON) | `0.0` | Natychmiast po załączeniu pinu pompy (`PB13 = HIGH`) |
| **`PUMP_OFF`** / **`GRIPPER_OPEN`** | `CMD_EFFECTOR_HMI` (`0x04`) | `1.0` (ID) | `0.0` (OFF) | `0.0` | Natychmiast po wyłączeniu pinu pompy (`PB13 = LOW`) |
| **`DELAY`** | `CMD_DELAY` (`0x09`) | `delay_ms` | `0.0` | `0.0` | Po odmierzeniu dokładnie `delay_ms` milisekund w wątku `ToolTask` na STM32 |
| *(Bazowanie przed startem)* | `CMD_HOME` (`0x03`) | `0.0` | `0.0` | `0.0` | Gdy wszystkie 3 silniki najadą na krańcówki i wyzerują pozycję |

---

## 4. Obsługa Trybu Teach-In (`0x06`, `0x07`, `0x08`)

### 4.1. `CMD_TEACH_MODE (0x06)` – Zwalnianie / blokowanie silników (PC $\rightarrow$ STM32)
Wysyłane po kliknięciu przycisków `Release J1`, `Release J2`, `Release J3 (Z)` lub `Release All Drives`:
* **`param1` (ID Osi):** `0.0` = `ALL`, `1.0` = `J1`, `2.0` = `J2`, `3.0` = `J3 (Z)`
* **`param2` (Stan):** `1.0` = zwolnij napęd (`EN = HIGH`), `0.0` = zablokuj napęd (`EN = LOW`)
* **`param3`:** `0.0`
* *(STM32 odsyła po przełączeniu również `CMD_ACK (0x0A)` z `param1 = 6.0, param2 = 1.0`).*

### 4.2. `CMD_TEACH_READ (0x07)` – Odczyt nauczonej pozycji przez powrót do bazy (PC $\rightarrow$ STM32)
Wysyłane po kliknięciu przycisku **`2. Read Position & Add to List`**:
* **`param1`:** `0.0`, **`param2`:** `0.0`, **`param3`:** `0.0`
* Wszystkie 3 silniki wracają do krańcówek w stanie `STATE_TEACH_IN_RETURN`, zliczając kroki `teachInStepsCount`.
* Po dotarciu wszystkich 3 osi do krańcówek `MotorTask` przekazuje kroki do `SystemState` i zeruje liczniki, a `KinematicsTask` wylicza pozycję $(X, Y, Z)$ i odsyła ramkę **`CMD_TEACH_RESPONSE (0x08)`**.

### 4.3. `CMD_TEACH_RESPONSE (0x08)` – Ramka z wyliczoną pozycją $X, Y, Z$ (STM32 $\rightarrow$ PC)
* **`cmd`:** `0x08` (`CMD_TEACH_RESPONSE`)
* **`param1`:** Współrzędna **$X$ [mm]** (`float`)
* **`param2`:** Współrzędna **$Y$ [mm]** (`float`)
* **`param3`:** Współrzędna **$Z$ [mm]** (`float`)

---

## 5. Wytyczne Implementacyjne dla Agenta Python HMI (`GINO_HMI`)

### 1. `src/core/serial_interface.py`:
* Zaktualizuj `Command(IntEnum)`:
  ```python
  class Command(IntEnum):
      MOVE_JOINT     = 0x01
      MOVE_LINEAR    = 0x02
      HOME           = 0x03
      EFFECTOR       = 0x04
      TEACH_MODE     = 0x06
      TEACH_READ     = 0x07
      TEACH_RESPONSE = 0x08
      DELAY          = 0x09
      ACK            = 0x0A
      SET_SPEED      = 0x10
      ESTOP          = 0xFF
  ```
* Dodaj dwa sygnały w `SerialInterface`:
  ```python
  teach_position_received = pyqtSignal(float, float, float)  # x, y, z (z CMD_TEACH_RESPONSE 0x08)
  command_ack_received = pyqtSignal(int, bool)               # (ack_cmd_id, is_success) (z CMD_ACK 0x0A)
  ```
* Zmodyfikuj `send_frame`, aby pakowało **16 bajtów** (`<BBBfff` + `<B` CRC):
  ```python
  def send_frame(self, cmd: Command, param1: float = 0.0, param2: float = 0.0, param3: float = 0.0):
      if not (self.serial_connection and self.serial_connection.is_open):
          print(f"[SERIAL TX] Port closed. Dropping frame: {cmd.name}")
          return
      try:
          payload = struct.pack('<BBBfff', self.SYNC_1, self.SYNC_2, cmd.value, param1, param2, param3)
          crc = self.calculate_crc8(payload)
          full_frame = payload + struct.pack('<B', crc)
          self.serial_connection.write(full_frame)
          print(f"[SERIAL TX] Sent {cmd.name}: P1={param1:.2f}, P2={param2:.2f}, P3={param3:.2f} | CRC: {crc}")
      except Exception as e:
          self.error_occurred.emit(f"Frame send error: {str(e)}")
  ```
* W wątku `run()` w `SerialInterface` odbieraj strumień bajtów i szukaj 16-bajtowych ramek binarnych zaczynających się od `0xAA 0x55`:
  ```python
  # Po odebraniu 16 bajtów ramki (frame) i sprawdzeniu calculate_crc8(frame[:15]) == frame[15]:
  _, _, cmd_id, p1, p2, p3, crc = struct.unpack('<BBBfffB', frame)
  if cmd_id == Command.TEACH_RESPONSE:
      self.teach_position_received.emit(p1, p2, p3)
  elif cmd_id == Command.ACK:
      acked_cmd = int(round(p1))
      is_ok = (p2 > 0.5)
      self.command_ack_received.emit(acked_cmd, is_ok)
  ```

### 2. `src/core/scada_controller.py` i `src/gui/tabs/tab_scada.py`:
* Dodaj w `TabScada` przycisk **`Run Selected Program`** (oraz opcjonalnie `Stop Program`).
* W `ScadaController` zaimplementuj maszynę stanów wykonującą kroki programu po kolei:
  1. Po kliknięciu **`Run Selected Program`** pobierz listę kroków `self.current_steps = self.db.get_program_steps(program_name)`, ustaw `self.current_step_idx = 0`, `self.is_program_running = True` i wywołaj `self._send_next_step()`.
  2. W `_send_next_step()` pobierz słownik `step = self.current_steps[self.current_step_idx]`, podświetl aktualnie wykonywany wiersz w tabeli `self.tab.table.selectRow(self.current_step_idx)` i wyślij odpowiednią ramkę:
     * `"MOVE"` $\rightarrow$ `self.robot.send_frame(Command.MOVE_LINEAR, step["x"], step["y"], step["z"])`
     * `"PUMP_ON"` lub `"GRIPPER_CLOSE"` $\rightarrow$ `self.robot.send_frame(Command.EFFECTOR, 1.0, 1.0, 0.0)`
     * `"PUMP_OFF"` lub `"GRIPPER_OPEN"` $\rightarrow$ `self.robot.send_frame(Command.EFFECTOR, 1.0, 0.0, 0.0)`
     * `"DELAY"` $\rightarrow$ `self.robot.send_frame(Command.DELAY, float(step["time"] or 0), 0.0, 0.0)`
  3. Podepnij sygnał `self.robot.command_ack_received.connect(self.on_command_ack)`:
     * Gdy przyjdzie `on_command_ack(acked_cmd, is_ok)` podczas gdy `self.is_program_running == True`:
       * Jeśli `not is_ok`: zatrzymaj wykonywanie programu (`self.is_program_running = False`) i pokaż ostrzeżenie (np. punkt poza zasięgiem).
       * Jeśli `is_ok`: zwiększ `self.current_step_idx += 1`. Jeśli `self.current_step_idx < len(self.current_steps)`, wywołaj `self._send_next_step()`. W przeciwnym razie zakończ program (`self.is_program_running = False`) i wyświetl komunikat o ukończeniu programu.
