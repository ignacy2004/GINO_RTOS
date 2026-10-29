# GINO RTOS – Firmware Manipulatora SCARA

Oprogramowanie wbudowane czasu rzeczywistego (RTOS) dla 3-osiowego robota przemysłowego typu **SCARA** ("GINO"). Firmware został zbudowany na bazie mikrokontrolera **STM32F401CCU6** (płytka WeAct Blackpill) zamontowanego na dedykowanej płycie bazowej (Carrier Board), z wykorzystaniem systemu operacyjnego **FreeRTOS** (CMSIS-RTOS v2) oraz środowiska **CMake / GCC ARM**.

---

## Spis Treści

1. [Główne Cechy Systemu](#1-główne-cechy-systemu)
2. [Architektura Oprogramowania](#2-architektura-oprogramowania)
   - [Zadania FreeRTOS (Tasks)](#21-zadania-freertos-tasks)
   - [Mechanizmy IPC (Kolejki i Mutexy)](#22-mechanizmy-ipc-kolejki-i-mutexy)
3. [Moduły Aplikacyjne](#3-moduły-aplikacyjne)
4. [Autorski Algorytm Teach-In](#4-autorski-algorytm-teach-in-inżynieria-odwrotna-pozycji)
5. [Protokół Komunikacji (PC HMI <-> STM32)](#5-protokół-komunikacji-pc-hmi---stm32)
   - [Struktura Ramki Binarnej (20 Bajtów)](#51-struktura-ramki-binarnej-20-bajtów)
   - [Tabela Komend](#52-tabela-komend)
   - [Komendy Tekstowe (G-Code / M-Code)](#53-komendy-tekstowe-g-code--m-code)
6. [Przypisanie Pinów (Hardware Pinout)](#6-przypisanie-pinów-hardware-pinout)
7. [Struktura Projektu](#7-struktura-projektu)
8. [Kompilacja i Wgrywanie](#8-kompilacja-i-wgrywanie)
   - [Wymagania Narzędziowe](#81-wymagania-narzędziowe)
   - [Budowanie Projektu (CMake Presets)](#82-budowanie-projektu-cmake-presets)
   - [Wgrywanie Firmware (Flash)](#83-wgrywanie-firmware-flash)

---

## 1. Główne Cechy Systemu

- **Mikrokontroler:** STM32F401CCU6 (ARM Cortex-M4 z FPU, 84 MHz, 256 kB Flash, 64 kB SRAM).
- **System Czasu Rzeczywistego:** FreeRTOS v10 z warstwą CMSIS-RTOS v2 i optymalnym podziałem priorytetów.
- **Sterowanie Napędami:** Bezpośrednie sterowanie 3 osiami za pośrednictwem cichych i precyzyjnych sterowników **TMC2209** w trybie STEP/DIR/EN.
- **Cyfrowy Bliźniak (Digital Twin):** Bezpieczna, chroniona mutexem struktura stanu robota (`RobotState_t`), agregująca pozycje kątowe/krokowe, kartezjańskie, stan chwytaka i flagi bezpieczeństwa.
- **Natywna Komunikacja USB CDC:** Wirtualny port szeregowy USB Full-Speed (12 Mbps) eliminujący potrzebę zewnętrznych konwerterów UART-USB.
- **Dwustopniowe Bezpieczeństwo E-STOP:** Sprzętowe odcięcie zasilania/sygnału ENABLE oraz obsługa programowego przerwania bezpieczeństwa w czasie rzeczywistym.
- **Obsługa Narzędzia Końcowego:** Zintegrowany kontroler chwytaka podciśnieniowego (pompa próżniowa sterowana tranzystorem MOSFET / przekaźnikiem).

---

## 2. Architektura Oprogramowania

Oprogramowanie zostało zaprojektowane według wzorca **Głównego Dyspozytora (Dispatcher Pattern)** z separacją zadań krytycznych czasowo od obsługi protokołów i telemetrii.

```mermaid
flowchart TD
    USB_IN["Przerwanie USB CDC\n(ISR_USB_CDC)"] -->|20-bajtowa ramka\nxQueueSendFromISR| CQ["CommandQueue\n(10 ramek x 20 B)"]
    
    subgraph RTOS_CORE ["Warstwa Zadań FreeRTOS"]
        DISP["DispatcherTask\n(AboveNormal)\nDyspozytor komend"]
        MOTOR["MotorTask\n(High)\nGeneracja kroków i FSM TMC2209"]
        KIN["KinematicsTask\n(Normal)\nKinematyka SCARA"]
        TOOL["ToolTask\n(Normal)\nSterowanie pompą"]
        COMM["CommTask\n(BelowNormal)\nTelemetria i LED statusu"]
        
        CQ -->|xQueueReceive| DISP
        DISP -->|Zlecenie ruchu| MOTOR
        DISP -->|Przeliczenie trajektorii| KIN
        DISP -->|Załączenie próżni| TOOL
    end
    
    subgraph STATE ["Cyfrowy Bliźniak (Digital Twin)"]
        MUTEX[("StateMutex")]
        ROBOT_STATE["RobotState_t\n- Cartesian (X, Y, Z)\n- Steps (J1, J2, J3)\n- Homing / E-STOP\n- Pump State"]
        MUTEX --- ROBOT_STATE
    end
    
    MOTOR <-->|Aktualizacja kroków| STATE
    KIN <-->|Odczyt/Zapis (X, Y, Z)| STATE
    TOOL <-->|Stan pompy| STATE
    COMM -->|Bezpieczny odczyt stanu| STATE
    
    COMM -->|Odpowiedź / Telemetria| USB_OUT["USB CDC Transmit\n(Wysyłka do PC HMI)"]
    
    HW_ESTOP["Fizyczny Przycisk E-STOP (PB14)"] -->|Natychmiastowe zatrzymanie| DISP
```

### 2.1. Zadania FreeRTOS (Tasks)

| Nazwa Zadania | Funkcja Wejściowa | Priorytet CMSIS | Rozmiar Stosu | Rola i Odpowiedzialność |
| :--- | :--- | :--- | :---: | :--- |
| **`MotorTask`** | `StartMotorTask` | `High` | 2048 B (512 słów) | Najwyższy priorytet aplikacyjny. Odpowiada za taktowanie silników krokowych (STEP/DIR), obsługę maszyny stanów TMC2209 oraz zatrzymanie na krańcówkach. |
| **`DispatcherTask`** | `StartDefaultTask` | `AboveNormal` | 2048 B (512 słów) | Główny dyspozytor. Odbiera ramki z kolejki `CommandQueue`, weryfikuje poprawność, parsuje ID rozkazu i przekazuje zlecenia do podsystemów. Inicjalizuje stos USB. |
| **`KinematicsTask`** | `StartKinematicsTask` | `Normal` | 2048 B (512 słów) | Obliczenia numeryczne kinematyki prostej (FK) i odwrotnej (IK) manipulatora SCARA (przeliczanie współrzędnych $X, Y, Z \leftrightarrow J_1, J_2, J_3$). |
| **`ToolTask`** | `StartToolTask` | `Normal` | 1024 B (256 słów) | Sterowanie osprzętem efektora końcowego (załączanie pompy próżniowej, czasy podtrzymania ssania). |
| **`CommTask`** | `StartCommTask` | `BelowNormal` | 2048 B (512 słów) | Formowanie pakietów telemetrycznych, cykliczne raportowanie stanu robota do HMI oraz sterowanie diodą sygnalizacyjną transmisji. |

### 2.2. Mechanizmy IPC (Kolejki i Mutexy)

- **`CommandQueue`:**
  - Typ: Kolejka FreeRTOS (`osMessageQueueId_t`).
  - Pojemność: 10 elementów o rozmiarze 20 bajtów każdy.
  - Zastosowanie: Odbiór danych bezpośrednio z przerwania USB CDC za pomocą `xQueueSendFromISR` i asynchroniczne przekazywanie do `DispatcherTask`.
- **`stateMutex`:**
  - Typ: Mutex FreeRTOS (`SemaphoreHandle_t`).
  - Lokalizacja: Moduł [`Modules/system_state`](Modules/system_state/).
  - Zastosowanie: Zapewnienie atomowości operacji na globalnej strukturze `RobotState_t` i ochrona przed wyścigami danych (*Race Conditions*) pomiędzy wieloma wątkami.

---

## 3. Moduły Aplikacyjne

Firmware podzielony jest na wyspecjalizowane biblioteki w katalogu `Modules/`:

```
Modules/
├── system_state/          # Cyfrowy Bliźniak (Digital Twin) i ochrona stanu
│   ├── system_state.h
│   └── system_state.c
├── tmc2209_stepper/       # Sterownik silników krokowych i maszyna stanów
│   ├── tmc2209_stepper.h
│   └── tmc2209_stepper.c
└── tool_controller/       # Zarządzanie osprzętem chwytaka (pompa próżniowa)
    ├── tool_controller.h
    └── tool_controller.c
```

### Moduł `system_state`
Zarządza współdzielonym stanem systemu:
```c
typedef struct {
    Point3D_t    currentCartesian; // Pozycja X, Y, Z w mm
    JointSteps_t currentSteps;     // Pozycje w krokach dla J1, J2, J3
    bool         isHomed;          // Czy manipulator został zbazowany
    bool         pumpState;        // Status załączenia pompy podciśnienia
    bool         emergencyStop;    // Flaga aktywnego E-STOP
} RobotState_t;
```

### Moduł `tmc2209_stepper`
Implementuje skończoną maszynę stanów (FSM) sterownika krokowego:
- `STATE_IDLE` – Silnik w spoczynku, prąd trzymania aktywny.
- `STATE_MOVING` – Płynna generacja impulsów STEP z zadanym kierunkiem.
- `STATE_HOMING` – Poszukiwanie punktu zerowego (ruch do momentu wyzwolenia krańcówki).
- `STATE_TEACH_IN` – Zwolnienie sygnału ENABLE (silnik swobodny do ręcznego pozycjonowania).
- `STATE_TEACH_IN_RETURN` – Powrót do punktu zerowego ze zliczaniem przebytych kroków.

### Moduł `tool_controller`
Zapewnia bezpieczne załączanie i wyłączanie podciśnieniowego chwytaka robota z natychmiastowym odzwierciedleniem stanu w strukturze `system_state`.

---

## 4. Autorski Algorytm Teach-In (Inżynieria Odwrotna Pozycji)

Z uwagi na zastosowanie silników krokowych bez drogich enkoderów absolutnych, manipulator realizuje autorski proces uczenia pozycji zademonstrowanej:

1. **Zwolnienie napędów (`M18` / `STATE_TEACH_IN`):** Firmware wystawia stan wysoki na pinach ENABLE sterowników TMC2209, odłączając prąd cewek.
2. **Ręczne pozycjonowanie:** Operator manualnie przemieszcza ramię robota i chwytak do żądanej pozycji docelowej.
3. **Żądanie zapisu pozycji (`STATE_TEACH_IN_RETURN`):** Operator w HMI klika zapis/bazowanie.
4. **Zliczanie kroków powrotnych:** Silniki wykonują ruch w kierunku wyłączników krańcowych (HOME). Każdy krok inkrementuje rejestr `teachInStepsCount`.
5. **Kalkulacja kątów i współrzędnych:** Po napotkaniu krańcówki znana jest dokładna liczba kroków oddalenia od zera. Kinematyka wylicza pozycję startową ($J_1, J_2, J_3$) oraz punkt w przestrzeni ($X, Y, Z$), który trafia do bazy receptur SCADA.

---

## 5. Protokół Komunikacji (PC HMI <-> STM32)

Wymiana danych pomiędzy komputerem nadrzędnym (aplikacja Python HMI) a kontrolerem STM32 realizowana jest przez wirtualny port szeregowy USB CDC przy użyciu ramek binarnych o stałej długości.

### 5.1. Struktura Ramki Binarnej (20 Bajtów)

Stały rozmiar 20 bajtów gwarantuje optymalne wyrównanie pamięci (32-bit alignment) i bezproblemową obsługę sprzętowego transferu. Kodowanie liczb zmiennoprzecinkowych: **IEEE 754 Little-Endian** (format `<BBffffB` w Pythonie).

| Bajt | Pole | Typ | Wartość domyślna / Format | Opis |
| :---: | :--- | :--- | :--- | :--- |
| `0` | **SYNC1** | `uint8_t` | `0xAA` (170) | Pierwszy bajt synchronizacji |
| `1` | **SYNC2** | `uint8_t` | `0x55` (85) | Drugi bajt synchronizacji |
| `2` | **CMD** | `uint8_t` | Identyfikator komendy | Kod rozkazu sterującego |
| `3..6` | **PARAM_1** | `float` | 4 bajty Little-Endian | Pierwszy parametr rozkazu |
| `7..10` | **PARAM_2** | `float` | 4 bajty Little-Endian | Drugi parametr rozkazu |
| `11..14` | **PARAM_3** | `float` | 4 bajty Little-Endian | Trzeci parametr rozkazu |
| `15..18` | **PARAM_4** | `float` | 4 bajty Little-Endian | Czwarty parametr rozkazu (np. Feedrate) |
| `19` | **CRC / XOR**| `uint8_t` | Suma kontrolna | Suma kontrolna nagłówka i danych |

### 5.2. Tabela Komend

| CMD | Nazwa Rozkazu | PARAM_1 | PARAM_2 | PARAM_3 | PARAM_4 |
| :---: | :--- | :--- | :--- | :--- | :--- |
| `0x01` | **Ruch Przegubowy (Joint)** | Kąt $J_1$ [°] | Kąt $J_2$ [°] | Pozycja $J_3$ [mm] | Prędkość / Feedrate |
| `0x02` | **Ruch Kartezjański (Linear)** | Pozycja $X$ [mm] | Pozycja $Y$ [mm] | Pozycja $Z$ [mm] | Prędkość liniowa |
| `0x05` | **Akcja Efektora (Narzędzie)** | ID Narzędzia (np. `1.0` - pompa) | Stan (`1.0` = ON, `0.0` = OFF) | Czas trwania [ms] | `0.0` |
| `0x10` | **Zatrzymanie Awaryjne (E-STOP)** | `0.0` | `0.0` | `0.0` | `0.0` |

### 5.3. Komendy Tekstowe (G-Code / M-Code)

Do celów diagnostycznych, kalibracji i procedury Teach-In obsługiwane są również polecenia tekstowe:
- `M17` – Załączenie zasilania silników krokowych (ENABLE LOW).
- `M18` – Zwolnienie zasilania silników (ENABLE HIGH – tryb Teach-In).
- `M114` – Żądanie natychmiastowego odesłania bieżących współrzędnych robota.

---

## 6. Przypisanie Pinów (Hardware Pinout)

Poniższe mapowanie odpowiada fizycznym połączeniom pomiędzy mikrokontrolerem STM32F401CCU6 a płytą bazową robota GINO:

| Sekcja | Sygnał | Etykieta w Kodzie | Pin STM32 | Typ Pinu | Tryb Pracy | Uwagi |
| :--- | :--- | :--- | :---: | :---: | :---: | :--- |
| **Oś J1 (Ramię)** | STEP | `J1_STEP_Pin` | **PA0** | GPIO Output | Push-Pull | Krok silnika osi J1 |
| | DIR | `J1_DIR_Pin` | **PA1** | GPIO Output | Push-Pull | Kierunek obrotu J1 |
| | ENABLE | `J1_ENABLE_Pin` | **PA2** | GPIO Output | Push-Pull | Niski (0) = Aktywny |
| | ENDSTOP | `J1_LIMIT_Pin` | **PA3** | GPIO Input | Pull-Up | Krańcówka bazująca J1 |
| **Oś J2 (Przedramię)**| STEP | `J2_STEP_Pin` | **PA4** | GPIO Output | Push-Pull | Krok silnika osi J2 |
| | DIR | `J2_DIR_Pin` | **PA5** | GPIO Output | Push-Pull | Kierunek obrotu J2 |
| | ENABLE | `J2_ENABLE_Pin` | **PA6** | GPIO Output | Push-Pull | Niski (0) = Aktywny |
| | ENDSTOP | `J2_LIMIT_Pin` | **PA7** | GPIO Input | Pull-Up | Krańcówka bazująca J2 |
| **Oś J3 (Oś Z)** | STEP | `Z_STEP_Pin` | **PB0** | GPIO Output | Push-Pull | Krok napędu śrubowego Z |
| | DIR | `Z_DIR_Pin` | **PB1** | GPIO Output | Push-Pull | Kierunek ruchu osi Z |
| | ENABLE | `Z_ENABLE_Pin` | **PB10** | GPIO Output | Push-Pull | Niski (0) = Aktywny |
| | ENDSTOP | `Z_LIMIT_Pin` | **PB12** | GPIO Input | Pull-Up | Krańcówka bazująca Z |
| **Efektor Końcowy** | POMPA | `PUMP_PIN_Pin` | **PB13** | GPIO Output | Push-Pull | Sterowanie MOSFET-em pompy |
| **Bezpieczeństwo** | E-STOP SENSE | `ESTOP_PIN_Pin` | **PB14** | GPIO Input | Pull-Up | Detekcja wciśnięcia grzyba E-STOP |
| **Interfejs USB** | USB D- | `OTG_FS_DM` | **PA11** | Alternatywna | USB_OTG_FS | Transmisja USB CDC |
| | USB D+ | `OTG_FS_DP` | **PA12** | Alternatywna | USB_OTG_FS | Transmisja USB CDC |
| **Programator SWD** | SWDIO | `SYS_JTMS-SWDIO` | **PA13** | Alternatywna | Debug | Interfejs ST-Link |
| | SWCLK | `SYS_JTCK-SWCLK` | **PA14** | Alternatywna | Debug | Interfejs ST-Link |
| **Oscylator HSE** | OSC_IN | `RCC_OSC_IN` | **PH0** | Wejście | HSE (25 MHz)| Zewnętrzny rezonator kwarcowy |
| | OSC_OUT | `RCC_OSC_OUT` | **PH1** | Wyjście | HSE (25 MHz)| Zewnętrzny rezonator kwarcowy |

---

## 7. Struktura Projektu

```
GINO_RTOS/
├── CMakeLists.txt              # Główny plik konfiguracyjny CMake
├── CMakePresets.json           # Presety kompilacji (Debug / Release)
├── GINO_RTOS.ioc               # Projekt konfiguracyjny STM32CubeMX
├── STM32F401XX_FLASH.ld        # Skrypt konsolidatora (Linker Script)
├── startup_stm32f401xc.s       # Plik startowy asemblera ARM
│
├── Core/
│   ├── Inc/                    # Nagłówki HAL, FreeRTOSConfig.h, main.h
│   └── Src/
│       ├── main.c              # Punkt wejścia systemu i zegary
│       ├── freertos.c          # Definicje zadań, kolejek i pętli FreeRTOS
│       ├── gpio.c              # Konfiguracja wejść/wyjść mikrokontrolera
│       └── stm32f4xx_it.c      # Obsługa wektorów przerwań
│
├── Modules/                    # Dedykowane moduły aplikacyjne robota
│   ├── system_state/           # Zarządzanie stanem i synchronizacja (Digital Twin)
│   ├── tmc2209_stepper/        # Obsługa sterowników silników krokowych TMC2209
│   └── tool_controller/        # Kontrola efektora końcowego (pompy)
│
├── USB_DEVICE/                 # Stos USB CDC (Virtual COM Port)
│   ├── App/                    # Deskryptory i interfejs usbd_cdc_if
│   └── Target/                 # Niskopoziomowa konfiguracja sprzętowa USB
│
├── Middlewares/
│   ├── Third_Party/FreeRTOS/   # Źródła jądra FreeRTOS
│   └── ST/STM32_USB_Device_Library/ # Biblioteka ST USB Device
│
└── Drivers/
    ├── CMSIS/                  # Nagłówki ARM Cortex-M4 CMSIS
    └── STM32F4xx_HAL_Driver/   # Sterowniki STM32 HAL
```

---

## 8. Kompilacja i Wgrywanie

### 8.1. Wymagania Narzędziowe

Upewnij się, że w zmiennej środowiskowej `PATH` znajdują się następujące narzędzia:
- **Kompilator:** `arm-none-eabi-gcc` (wersja $\ge 10.3$)
- **System budowania:** `CMake` (wersja $\ge 3.22$) oraz `Ninja`
- **Programator:** `STM32CubeProgrammer`, `OpenOCD` lub `st-flash`

### 8.2. Budowanie Projektu (CMake Presets)

Projekt posiada predefiniowane presety w pliku `CMakePresets.json`.

#### Wersja Debug:
```bash
# Konfiguracja
cmake --preset Debug

# Kompilacja
cmake --build --preset Debug
```

#### Wersja Release:
```bash
# Konfiguracja
cmake --preset Release

# Kompilacja
cmake --build --preset Release
```

Plik binarny `GINO_RTOS.elf` oraz pliki pomocnicze zostaną wygenerowane w katalogu `build/Debug/` lub `build/Release/`.

### 8.3. Wgrywanie Firmware (Flash)

#### Za pomocą STM32CubeProgrammer (CLI):
```bash
STM32_Programmer_CLI -c port=SWD -w build/Debug/GINO_RTOS.elf -v -rst
```

#### Za pomocą OpenOCD:
```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/Debug/GINO_RTOS.elf verify reset exit"
```

Po zaprogramowaniu mikrokontroler zresetuje się, uruchomi jądro FreeRTOS oraz zgłosi się w systemie operacyjnym jako wirtualny port COM (USB CDC) gotowy do komunikacji z aplikacją **GINO_HMI**.

