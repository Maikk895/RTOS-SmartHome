# RTOS-SmartHome
RTOS-SmartHome
Projekt demonstruje wykorzystanie systemu operacyjnego czasu rzeczywistego FreeRTOS do zarządzania wieloma zadaniami w systemie automatyki domowej. Głównym celem jest pokazanie mechanizmów wielozadaniowości, synchronizacji za pomocą semaforów oraz ochrony zasobów wspólnych przy użyciu muteksów.

Architektura Systemu (FreeRTOS)
Sercem projektu jest podział logiki na niezależne zadania (Tasks), które są zarządzane przez harmonogram FreeRTOS. Wykorzystano następujące mechanizmy:

W systemie zdefiniowano 5 priorytetowych zadań:

-RFID (Priorytet 40): Obsługa czytnika kart dostępu (najwyższy priorytet).
-OLED_Disp (Priorytet 32): Zarządzanie wyświetlaczem i interfejsem użytkownika.
-Heating (Priorytet 28): Kontrola systemu ogrzewania.
-Encoder (Priorytet 26): Obsługa wejść sterujących (enkodera).
-Temp_Read (Priorytet 24): Odczyt danych z czujników temperatury.

Mechanizmy Synchronizacji

Dla zapewnienia stabilności i uniknięcia konfliktów zastosowano:

Muteks (I2C_Mutex): Chroni dostęp do magistrali I2C, z której korzysta zarówno wyświetlacz OLED, jak i czujnik temperatury.

Semafory Binarne: Wykorzystywane do powiadamiania zadań o zdarzeniach (np. Temp_Sem, OLED_Sem, Rfid01).

Specyfikacja Sprzętowa
Mikrokontroler: STM32L432KCUx (Cortex-M4, 32MHz).

Płytka rozwojowa: NUCLEO-L432KC.

Interfejsy: I2C1 (Fast Mode), USART2 (komunikacja VCP), GPIO.

Przypisanie Pinów (Wybrane)

Funkcja	Pin	Etykieta
I2C1 SDA/SCL	PA10 / PA9	
Obsługa czujników/ekranu

Wyjście Grzania	PB4	
Heating_on

Dioda Statusu	PB3	
LD3 [Green]

Przyciski Temp	PB0 / PB7	
Temp_down / Temp_up