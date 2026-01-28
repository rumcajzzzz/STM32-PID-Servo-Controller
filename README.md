# Regulator PID Serwomechanizmu - STM32F746

Projekt realizuje cyfrowy sterownik pozycji serwa z wykorzystaniem mikrokontrolera STM32. System odczytuje nastawy PID oraz pozycję z potencjometrów przez ADC (z filtracją EMA), przelicza sygnał sterujący w pętli zamkniętej i generuje sygnał PWM.

## Instrukcja uruchomienia (Krok po kroku)

Jeśli pierwszy raz korzystasz ze środowiska STM32CubeIDE, wykonaj poniższe kroki w podanej kolejności:

### 1. Rozwiązywanie problemów z wersją (Manual Version Fix)
Jeśli Twoja wersja STM32CubeIDE różni się od tej użytej w projekcie, plik `.ioc` może się nie otworzyć. Należy to poprawić **wewnątrz archiwum** przed importem:
1. Otwórz pobrane archiwum `.zip` (bez rozpakowywania całego projektu).
2. Znajdź w nim plik `PID-REGULATOR.ioc` i otwórz go w dowolnym edytorze tekstu (np. Notatnik).
3. Znajdź linie:
   * `MxCube.Version=X.X.X`
4. Zmień numery wersji na takie, które masz zainstalowane (np. `6.12.1`).
5. Zapisz zmiany bezpośrednio w archiwum.

### 2. Import projektu (Archive Mode)
1. Otwórz **STM32CubeIDE**.
2. Wybierz `File` -> `Import...` -> `General` -> `Existing Projects into Workspace`.
3. Zaznacz opcję **Select archive file** i wskaż pobrany plik `.zip`.
4. Kliknij `Finish`. IDE automatycznie wypakuje i zaimportuje projekt do Twojego workspace.

### 3. Generowanie kodu i kompilacja (Kluczowy krok!)
**Uwaga:** Musisz wygenerować pliki konfiguracyjne przed budowaniem projektu, aby dopasować biblioteki HAL do swojej wersji IDE.

1. **Generowanie kodu z `.ioc`**: 
   * Otwórz plik `PID-REGULATOR.ioc` wewnątrz CubeIDE.
   * Kliknij ikonę **żółtego koła zębatego** (Device Configuration Tool Code Generation) na górnym pasku lub naciśnij `Ctrl + S`.
   * Potwierdź chęć generowania kodu – to zaktualizuje pliki `main.c` oraz sterowniki HAL.
2. **Budowanie i Wgrywanie**:
   * Kliknij ikonę **Młotka** (Build) i upewnij się, że nie ma błędów.
   * Podłącz płytkę **Nucleo-F746ZG** i kliknij ikonę **Zielonej strzałki** (Run).

## Specyfikacja techniczna

* **Algorytm sterowania:** Implementacja pełnego regulatora **PID** (Proporcjonalno-Całkująco-Różniczkujący) działającego w czasie rzeczywistym.
    * **Anti-Windup:** Logika ograniczająca nasycenie członu całkującego, zapobiegająca akumulacji błędu i dużym przeregulowaniom.
    * **Deadband:** Programowa strefa nieczułości eliminująca niepożądane drgania serwa przy minimalnych wahaniach błędu pozycji.
* **Filtracja sygnałów:** Cyfrowy filtr dolnoprzepustowy **EMA** (Exponential Moving Average) ze współczynnikiem $\alpha=0.2$.
    * Efektywna eliminacja szumów kwantyzacji ADC oraz zakłóceń elektromagnetycznych indukowanych na ścieżkach potencjometrów.
* **System zabezpieczeń:**
    * **Stop Awaryjny (Emergency Stop):** Natychmiastowe zatrzymanie algorytmu i odcięcie sygnału sterującego PWM po wykryciu przerwania zewnętrznego (przycisk na pinie PA5).
    * **Saturacja wyjścia:** Sztywne ograniczenie sygnału sterującego do zakresu bezpiecznego dla serwomechanizmu (standard czasowy 500-2500 μs).
* **Interfejs użytkownika i diagnostyka:**
    * **LCD 2x16 (I2C):** Dynamiczne wyświetlanie w czasie rzeczywistym aktualnej pozycji, wartości zadanej (setpoint) oraz bieżących nastaw parametrów $K_p, K_i, K_d$.
    * **Telemetria UART:** Transmisja danych diagnostycznych (115200 bps) umożliwiająca wizualizację odpowiedzi układu na skok jednostkowy w narzędziach typu Serial Plotter.
* **Konfiguracja sprzętowa (Hardware Abstraction):**
    * Precyzyjne generowanie sygnału PWM (50 Hz) z wykorzystaniem 16-bitowych timerów sprzętowych (**TIM**).
    * Wielokanałowe przetwarzanie analogowo-cyfrowe (**ADC**) z wykorzystaniem trybu skanowania sekwencyjnego dla 4 niezależnych potencjometrów.

## Podłączenie sprzętowe
| Komponent | Pin STM32 | Funkcja |
| :--- | :--- | :--- |
| **Serwo (PWM)** | PA15 (TIM2_CH1) | Sterowanie kątem |
| **LCD (SDA/SCL)** | PB9 / PB8 | Interfejs I2C |
| **Zadajnik Pozycji** | PC0 | ADC3_IN10 |
| **Nastawa Kp** | PC2 | ADC3_IN12 |
| **Nastawa Ki** | PC3 | ADC3_IN13 |
| **Nastawa Kd** | PF3 | ADC3_IN9 |
| **Stop Awaryjny** | PA5 | Przycisk zewnętrzny |

---
*Projekt wykonany w ramach przedmiotu Systemy Mikroprocesorowe.*