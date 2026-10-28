# SCARA Robot – Architektura Firmware'u i Przypisanie Pinów

Niniejszy dokument stanowi pełną specyfikację architektury oprogramowania (opartej na systemie czasu rzeczywistego FreeRTOS) oraz przypisania pinów dla mikrokontrolera STM32 Blackpill (F401/F411), uwzględniając autorską płytę bazową (Carrier Board).

## 1. Architektura FreeRTOS (Wątki, Kolejki, Mutexy)

System został zaprojektowany z myślą o determinizmie i bezpieczeństwie danych. Wykorzystuje wzorzec **Głównego Dyspozytora (Dispatcher)**, aby uniknąć konfliktów w sterowaniu urządzeniami.

### 1.1. Zasoby Systemowe (IPC - Komunikacja Międzyprocesowa)

* **`CommandQueue` (Kolejka RTOS):**
  * **Pojemność:** np. 10 elementów.
  * **Typ danych:** Struktura `CommandFrame_t` (zdekodowane 20 bajtów).
  * **Rola:** Główny rurociąg komunikacyjny. Gwarantuje, że komendy z HMI wykonają się w dokładnie takiej kolejności, w jakiej zostały wysłane.

* **`StateMutex` (Mutex RTOS):**
  * **Lokalizacja:** Moduł `system_state.c`.
  * **Rola:** Chroni globalną strukturę `RobotState_t` (Cyfrowego Bliźniaka). Zapewnia, że żaden Task nie zmodyfikuje współrzędnych robota w momencie, gdy inny Task próbuje je odczytać (zapobiega zjawisku *Race Condition*).

### 1.2. Przerwania i Wątki (Tasks)

1. **`ISR_USB_CDC` (Przerwanie Sprzętowe USB - Poza FreeRTOS):**
   * *Wyzwalacz:* Nadejście danych z portu USB-C (od Pythona).
   * *Działanie:* Dekoduje 20-bajtową ramkę, sprawdza sumę kontrolną (CRC/XOR). Jeśli ramka jest poprawna, rzutuje ją na strukturę `CommandFrame_t` i używa funkcji `xQueueSendFromISR`, aby natychmiast wrzucić ją do kolejki `CommandQueue`, po czym wznawia normalną pracę procesora.

2. **`Task_Main` (Główny Dyspozytor / Motion Task):**
   * *Priorytet:* High (Wysoki).
   * *Działanie:* Działa w nieskończonej pętli. Blokuje się i zasypia na funkcji `xQueueReceive`, czekając na nową komendę w `CommandQueue`.
   * Gdy komenda nadejdzie, wybudza się, czyta bajt `CMD` i rozdziela pracę (Ruch, Pompa, E-STOP).

3. **`Task_Telemetry` (Opcjonalny wątek raportujący):**
   * *Priorytet:* Normal (Normalny).
   * *Działanie:* Pobiera bezpiecznie aktualną pozycję robota (używając `SystemState_Get`, co angażuje `StateMutex`) i wysyła ją z powrotem do HMI po USB. Odpowiada również za miganie dedykowaną, zieloną diodą komunikacji na płycie PCB.

## 2. Logika Inżynierii Odwrotnej Pozycji (Teach-In)

Z uwagi na brak enkoderów absolutnych, system wykorzystuje autorski algorytm inżynierii odwrotnej podczas procesu uczenia ramienia (zdefiniowany w maszynie stanów jako `STATE_TEACH_IN_RETURN`):

1. HMI wysyła komendę zwolnienia napędów (np. `M18`).
2. Operator ręcznie przestawia ramię do pożądanej pozycji docelowej.
3. HMI wysyła komendę bazowania (szukania punktu zerowego).
4. Ramię powraca do punktu zerowego (HOME). Podczas tego ruchu, za każdym fizycznym krokiem silnika inkrementowana jest zmienna `teachInStepsCount`, aż do uderzenia w krańcówkę (Endstop).
5. Posiadając precyzyjną liczbę kroków od "nieznanego punktu" do punktu zerowego, moduł `scara_kinematics` wylicza kąt startowy.

## 3. Przypisanie Pinów (Hardware Pinout - Płyta PCB)

Poniższa tabela przedstawia mapowanie pinów zintegrowane z projektem płyty głównej (Carrier Board) z KiCada.

### 3.1. Moduł Napędów (TMC2209)

*Uwaga projektowa: Sygnał ENABLE jest dodatkowo ryglowany sprzętowo przez przycisk E-STOP (NC).*

| Oś | Sygnał | Sugerowany Pin | Typ Pinu w CubeMX | Uwagi | 
| ----- | ----- | ----- | ----- | ----- | 
| **J1 (Ramię)** | STEP | `PA0` | GPIO_Output |  | 
| **J1 (Ramię)** | DIR | `PA1` | GPIO_Output |  | 
| **J1 (Ramię)** | EN | `PA2` | GPIO_Output | Stan niski (0) włącza silnik | 
| **J1 (Ramię)** | ENDSTOP | `PA3` | GPIO_Input | Pull-Up (odczyt ze złącza JST-XH) | 
| **J2 (Przedr.)** | STEP | `PA4` | GPIO_Output |  | 
| **J2 (Przedr.)** | DIR | `PA5` | GPIO_Output |  | 
| **J2 (Przedr.)** | EN | `PA6` | GPIO_Output | Stan niski (0) włącza silnik | 
| **J2 (Przedr.)** | ENDSTOP | `PA7` | GPIO_Input | Pull-Up (odczyt ze złącza JST-XH) | 
| **J3 (Oś Z)** | STEP | `PB0` | GPIO_Output |  | 
| **J3 (Oś Z)** | DIR | `PB1` | GPIO_Output |  | 
| **J3 (Oś Z)** | EN | `PB2` | GPIO_Output | Stan niski (0) włącza silnik | 
| **J3 (Oś Z)** | ENDSTOP | `PB10` | GPIO_Input | Pull-Up (odczyt ze złącza JST-XH) | 

### 3.2. Sekcja Efektora Końcowego (Pompa)

Złącze sterujące przekaźnikiem lub tranzystorem MOSFET dla pompy próżniowej. Z uwagi na uproszczenie projektu zrezygnowano z dodatkowych narzędzi i pętli sprzętowej detekcji.

| Komponent | Sygnał | Sugerowany Pin | Typ Pinu w CubeMX | Uwagi | 
| ----- | ----- | ----- | ----- | ----- | 
| **Pompa ssąca** | PUMP_CTRL | `PB12` | GPIO_Output | Sterowanie przekaźnikiem / MOSFET | 

### 3.3. Interfejs Użytkownika, Bezpieczeństwo i Komunikacja

Moduły dodatkowe znajdujące się na zaprojektowanej płycie głównej.

| Komponent | Sygnał | Sugerowany Pin | Typ Pinu w CubeMX | Uwagi | 
| ----- | ----- | ----- | ----- | ----- | 
| **E-STOP Sense** | ESTOP_IN | `PB14` | GPIO_Input | Pull-Up; informuje firmware o wbitym grzybie | 
| **Dioda Komunikacji** | COMMS_LED | `PB5` | GPIO_Output | Zielona LED, miga podczas transmisji z HMI | 
| **Rozszerzenie I2C** | I2C_SCL | `PB6` | I2C1_SCL | Magistrala pod wyświetlacz OLED | 
| **Rozszerzenie I2C** | I2C_SDA | `PB7` | I2C1_SDA | Magistrala pod wyświetlacz OLED | 

### 3.4. Zastrzeżone Piny Systemowe Blackpilla

| Pin | Funkcja | Powód rezerwacji | 
| ----- | ----- | ----- | 
| `PA11`, `PA12` | USB_DM, USB_DP | Niezbędne do Wirtualnego Portu COM (USB CDC) | 
| `PA13`, `PA14` | SWDIO, SWCLK | Niezbędne do flashowania i debugowania (ST-Link) | 
| `PC13` | User LED | Wbudowana niebieska dioda na płytce STM32 | 