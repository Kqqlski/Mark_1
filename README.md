#include <U8g2lib.h>
#include <DHT.h>
#include <EEPROM.h>
#include <Wire.h>

U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


// Adresy w EEPROM dla parametrów (progów)
const int EEPROM_ADDR_TEMP = 0;           // Adres dla progu temperatury
const int EEPROM_ADDR_SOIL = 4;           // Adres dla progu wilgotności gleby
const int EEPROM_ADDR_HUMIDITY = 8;       // Adres dla progu wilgotności powietrza
const int EEPROM_ADDR_AIR_QUALITY = 12;   // Adres dla progu jakości powietrza

// Adresy w EEPROM dla histerezy
const int EEPROM_ADDR_TEMP_HYSTERESIS = 16;           // Adres dla histerezy temperatury
const int EEPROM_ADDR_SOIL_HYSTERESIS = 20;           // Adres dla histerezy wilgotności gleby
const int EEPROM_ADDR_HUMIDITY_HYSTERESIS = 24;       // Adres dla histerezy wilgotności powietrza
const int EEPROM_ADDR_AIR_QUALITY_HYSTERESIS = 28;    // Adres dla histerezy jakości powietrza

int lastJoyX = 512;  // Inicjalizacja na środek zakresu (np. 512 dla joysticka o zakresie 0-1023)
int lastJoyY = 512;  // Inicjalizacja na środek zakresu
const int tolerance = 50;  // Tolerancja wokół środka
const int centerValue = 512;  // Środkowa wartość dla joysticka o zakresie 0-1023


const unsigned long debounceDelay = 200;  // 200 ms na debounce
unsigned long lastButtonPress = 0;


// Progi automatyki
int thresholdTemperature = 25;   // Próg temperatury (°C)
int thresholdSoilMoisture = 30;  // Próg wilgotności gleby (%)
int thresholdHumidity = 60;      // Próg wilgotności powietrza (%)
int thresholdAirQuality = 150;   // Próg jakości powietrza (ppm)


// Definicje dla czujników i komponentów
#define DHTPIN 2                // Pin czujnika DHT22
#define DHTTYPE DHT22           // Typ DHT
#define AIR_QUALITY_PIN 0       // Pin czujnika jakości powietrza
#define SOIL_MOISTURE_PIN 1     // Pin czujnika wilgotności gleby
#define JOYSTICK_BUTTON_PIN 3   // Pin przycisku joysticka
#define JOYSTICK_Y_PIN A2       // Pin przycisku joysticka ośY
#define JOYSTICK_X_PIN A3       // Pin przycisku joysticka ośX
#define UPPER_FLOAT_PIN 9       // Górny czujnik pływakowy
#define LOWER_FLOAT_PIN 10      // Dolny czujnik pływakowy


// Definicje dla przekaźników
#define RELAY_MAIN_FAN 5      // Wentylator Główny
#define RELAY_STEAM_GEN 6     // Generator Pary
#define RELAY_IRRIGATION 7    // Pompa Nawadniania
#define RELAY_TANK_PUMP 8     // Pompa Zbiornika Generatora Pary

DHT dht(DHTPIN, DHTTYPE);  // Tworzenie obiektu czujnika DHT

void displayTankStatus();
void loadFromEEPROM();
void startAnimation();
void handleMenu();
void readJoystick();          // Deklaracja funkcji odczytu joysticka
void resetActivityTimer();    // Deklaracja funkcji resetującej timer aktywności
void displaySensorReadings();
void Level2();
void Level3();
void handleRelayAutomation();

// Debouncing
unsigned long lastDebounceTime = 0;
unsigned long lastActivityTime = 0;      // Czas ostatniej aktywności 
unsigned long inactivityTimeout = 5000;  // Czas bezczynności przed powrotem do menu 

// Wartości joysticka
int joyX, joyY, joyBtn, lastJoyBtn;
int currentMenuLevel = 1;                // Poziom menu (1 = główne menu)
int selectedParam = 0;                   // Numer wybranego parametru
int menuStartIndex = 0;                  // Zmienna przechowująca indeks pierwszej widocznej pozycji w menu

// Zmienna do przechowywania ostatnich progów  
float maxAirQualityThreshold = 100.0;
float AirQualityHysteresis = 5.0;
float maxTemperatureThreshold = 100.0;
float TemperatureHysteresis = 2.0;
float maxSoilMoistureThreshold = 100.0;
float SoilMoistureHysteresis = 5.0;
float maxHumidityThreshold = 100.0;
float HumidityHysteresis = 5.0;

unsigned long lastDisplayUpdate = 0;
const unsigned long displayUpdateInterval = 500; // Odświeżanie co 500 ms


void resetActivityTimer() {
  lastActivityTime = millis();
}


// Funkcja do sprawdzania ruchu w poziomie (X)
bool joystickMovedX() {
  // Sprawdzamy, czy zmiana przekroczyła tolerancję oraz czy nie jesteśmy w okolicach środka
  if (abs(joyX - lastJoyX) > 100 && abs(joyX - centerValue) > tolerance && millis() - lastDebounceTime > debounceDelay) {
    lastDebounceTime = millis();  // Zresetowanie czasu debouncingu

    if (joyX > centerValue + tolerance) {  // Ruch w prawo
      lastJoyX = joyX;
      return true;  // Zwracamy true dla ruchu w prawo
    }
    else if (joyX < centerValue - tolerance) {  // Ruch w lewo
      lastJoyX = joyX;
      return true;  // Zwracamy true dla ruchu w lewo
    }
  }
  return false;
}

// Funkcja do sprawdzania ruchu w pionie (Y)
bool joystickMovedY() {
  // Sprawdzamy, czy zmiana przekroczyła tolerancję oraz czy nie jesteśmy w okolicach środka
  if (abs(joyY - lastJoyY) > 100 && abs(joyY - centerValue) > tolerance && millis() - lastDebounceTime > debounceDelay) {
    lastDebounceTime = millis();  // Zresetowanie czasu debouncingu

    if (joyY > centerValue + tolerance) {  // Ruch w dół
      lastJoyY = joyY;
      return true;  // Zwracamy true dla ruchu w dół
    }
    else if (joyY < centerValue - tolerance) {  // Ruch w górę
      lastJoyY = joyY;
      return true;  // Zwracamy true dla ruchu w górę
    }
  }
  return false;
}


// Funkcja do detekcji wciśnięcia przycisku joysticka
bool joystickButtonPressed() {
  int currentButtonState = digitalRead(JOYSTICK_BUTTON_PIN);  // Odczyt stanu przycisku
  if (currentButtonState == LOW && lastJoyBtn == HIGH) {  // Przycisk wciśnięty (zakładając, że stan LOW oznacza wciśnięcie)
    lastJoyBtn = LOW;  // Zapisz stan przycisku
    return true;  // Przyciski zostały wciśnięte
  }
  else if (currentButtonState == HIGH && lastJoyBtn == LOW) {  // Jeśli przycisk został zwolniony
    lastJoyBtn = HIGH;  // Zaktualizuj stan
  }
  return false;  // Jeśli przycisk nie został wciśnięty
}


// Funkcja wyświetlająca dane z czujników (Poziom 1)
void displaySensorReadings() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  int airQualityRaw = analogRead(A0); // Surowa wartość czujnika jakości powietrza
  int airQuality = map(airQualityRaw, 0, 1023, 0, 100); // Mapowanie do %
  int soilMoisture = analogRead(A1); // Odczyt wilgotności gleby
  soilMoisture = map(soilMoisture, 0, 1023, 0, 100); // Mapowanie do %

  u8g2.clearBuffer();  
  u8g2.setFont(u8g2_font_5x8_tr); 
  u8g2.setCursor(0, 10);
  u8g2.print("Aktualne Odczyty: ");

  u8g2.setCursor(5, 20);
  u8g2.print("Temp. Pow.: ");
  u8g2.print(temperature, 1);
  u8g2.print(" C");

  u8g2.setCursor(5, 30);
  u8g2.print("Wilg. Pow.: ");
  u8g2.print(humidity, 1);
  u8g2.print(" %");

  u8g2.setCursor(5, 40);
  u8g2.print("Wilg. Gleby: ");
  u8g2.print(soilMoisture, 1);
  u8g2.print(" %");

  u8g2.setCursor(5, 50);
  u8g2.print("Zan. Pow.: ");
  u8g2.print(airQuality, 1);
  u8g2.print(" %");

  u8g2.setCursor(5, 60);
  u8g2.print("Zbiornik: ");
  if (digitalRead(UPPER_FLOAT_PIN) == LOW) {
    u8g2.print("G 2 G :)");
  } else if (digitalRead(LOWER_FLOAT_PIN) == HIGH) {
    u8g2.print("LoL nope :D");
  } else {
    u8g2.print("ok");
  }

  u8g2.sendBuffer();
}

// Funkcja do przewijania w dół
void scrollDownMenu() {
  if (menuStartIndex < 4) { // 4 to liczba pozycji widocznych na ekranie
    menuStartIndex++;
  }
}

// Funkcja do przewijania w górę
void scrollUpMenu() {
  if (menuStartIndex > 0) {
    menuStartIndex--;
  }
}

// Funkcja do wyświetlania menu edycji parametrów (Poziom 2)
void Level2() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tr);

  u8g2.setCursor(0, 10);
  u8g2.print("Parametry do edycji:");

  int yPos = 30;  // Pozycja Y początkowa
  for (int i = menuStartIndex; i < menuStartIndex + 4 && i <= 7; i++) {  // Pętla wyświetlająca 4 opcje
    u8g2.setCursor(0, yPos);
    if (selectedParam == i) {
      u8g2.print("> ");
    } else {
      u8g2.print("  ");
    }

    // Wyświetlanie nazw parametrów i wartości threshold + hysteresisa obok siebie
    switch (i) {
      case 0:
        u8g2.print("Temp. Pow.: ");
        u8g2.print(thresholdTemperature, 1);
        u8g2.print(" Hyst. Temp.: ");
        u8g2.print(TemperatureHysteresis, 1);
        break;
      case 1:
        u8g2.print("Wilg. Pow.: ");
        u8g2.print(thresholdHumidity);
        u8g2.print("% Hyst. Wilg.: ");
        u8g2.print(HumidityHysteresis);
        break;
      case 2:
        u8g2.print("Wilg. Gleby: ");
        u8g2.print(thresholdSoilMoisture);
        u8g2.print("% Hyst. Gleby: ");
        u8g2.print(SoilMoistureHysteresis);
        break;
      case 3:
        u8g2.print("Zan. Pow.: ");
        u8g2.print(thresholdAirQuality);
        u8g2.print("% Hyst. Zan.: ");
        u8g2.print(AirQualityHysteresis);
        break;
      
    }

    yPos += 10; // Przesunięcie o 10 pikseli w dół na każdą kolejną linię
  }

  u8g2.sendBuffer();
}

// Funkcja edycji parametrów (Poziom 3)
void Level3() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tr);

  u8g2.setCursor(5, 10);
  switch (selectedParam) {
    case 0: u8g2.print("Edycja: Temp. Pow."); break;
    case 1: u8g2.print("Edycja: Wilg. Pow."); break;
    case 2: u8g2.print("Edycja: Wilg. Gleby"); break;
    case 3: u8g2.print("Edycja: Zan. Pow."); break;
    case 4: u8g2.print("Edycja: Hyst. Temp."); break;
    case 5: u8g2.print("Edycja: Hyst. Wilg."); break;
    case 6: u8g2.print("Edycja: Hyst. Gleby"); break;
    case 7: u8g2.print("Edycja: Hyst. Zan."); break;
  }

  u8g2.setCursor(0, 30);
  u8g2.print("Wartosc: ");
  switch (selectedParam) {
    case 0: u8g2.print(thresholdTemperature, 1); break;
    case 1: u8g2.print(thresholdHumidity); break;
    case 2: u8g2.print(thresholdSoilMoisture); break;
    case 3: u8g2.print(thresholdAirQuality); break;
    case 4: u8g2.print(TemperatureHysteresis, 1); break;
    case 5: u8g2.print(HumidityHysteresis); break;
    case 6: u8g2.print(SoilMoistureHysteresis); break;
    case 7: u8g2.print(AirQualityHysteresis); break;
  }

  u8g2.setCursor(5, 50);
  u8g2.print("Joystick: +/- OK");

  u8g2.sendBuffer();
}

// Funkcja do zmiany wybranego parametru w menu
void navigateMenu() {
  // Sprawdzamy ruch w górę
  if (joystickMovedY() && joyY < centerValue - tolerance) {
    if (selectedParam > 0) selectedParam--;  // Przemieszczamy się do góry w menu
  }
  
  // Sprawdzamy ruch w dół
  if (joystickMovedY() && joyY > centerValue + tolerance) {
    if (selectedParam < 7) selectedParam++;  // Przemieszczamy się w dół w menu
  }
}

// Funkcja do zmiany parametrów (przykład, można dopasować do własnych potrzeb)
void editParam() {
  // Sprawdzamy ruch w prawo
  if (joystickMovedX() && joyX > centerValue + tolerance) {
    switch (selectedParam) {
      case 0: thresholdTemperature++; break;
      case 1: thresholdHumidity++; break;
      case 2: thresholdSoilMoisture++; break;
      case 3: thresholdAirQuality++; break;
      case 4: TemperatureHysteresis++; break;
      case 5: HumidityHysteresis++; break;
      case 6: SoilMoistureHysteresis++; break;
      case 7: AirQualityHysteresis++; break;
    }
  }
  
  // Sprawdzamy ruch w lewo
  if (joystickMovedX() && joyX < centerValue - tolerance) {
    switch (selectedParam) {
      case 0: thresholdTemperature--; break;
      case 1: thresholdHumidity--; break;
      case 2: thresholdSoilMoisture--; break;
      case 3: thresholdAirQuality--; break;
      case 4: TemperatureHysteresis--; break;
      case 5: HumidityHysteresis--; break;
      case 6: SoilMoistureHysteresis--; break;
      case 7: AirQualityHysteresis--; break;
    }
  }
}


// Adresy EEPROM dla parametrów
const int EEPROM_ADDR_AUTOMATION = 4;  // Adres dla flagi automatyki

// Parametry progowe
bool automationEnabled = true;  // Flaga włączająca/wyłączająca automatykę

// Funkcja zapisu do EEPROM (dla progów i histerez)
void saveToEEPROM() {
    EEPROM.put(EEPROM_ADDR_TEMP, thresholdTemperature);
    EEPROM.put(EEPROM_ADDR_SOIL, thresholdSoilMoisture);
    EEPROM.put(EEPROM_ADDR_HUMIDITY, thresholdHumidity);
    EEPROM.put(EEPROM_ADDR_AIR_QUALITY, thresholdAirQuality);

    EEPROM.put(EEPROM_ADDR_TEMP + 4, TemperatureHysteresis);  // Histereza temperatury
    EEPROM.put(EEPROM_ADDR_SOIL + 4, SoilMoistureHysteresis); // Histereza wilgotności gleby
    EEPROM.put(EEPROM_ADDR_HUMIDITY + 4, HumidityHysteresis);  // Histereza wilgotności powietrza
    EEPROM.put(EEPROM_ADDR_AIR_QUALITY + 4, AirQualityHysteresis); // Histereza jakości powietrza
}

// Funkcja odczytu z EEPROM (dla progów i histerez)
void loadFromEEPROM() {
    EEPROM.get(EEPROM_ADDR_TEMP, thresholdTemperature);
    EEPROM.get(EEPROM_ADDR_SOIL, thresholdSoilMoisture);
    EEPROM.get(EEPROM_ADDR_HUMIDITY, thresholdHumidity);
    EEPROM.get(EEPROM_ADDR_AIR_QUALITY, thresholdAirQuality);

    EEPROM.get(EEPROM_ADDR_TEMP + 4, TemperatureHysteresis);  // Histereza temperatury
    EEPROM.get(EEPROM_ADDR_SOIL + 4, SoilMoistureHysteresis); // Histereza wilgotności gleby
    EEPROM.get(EEPROM_ADDR_HUMIDITY + 4, HumidityHysteresis);  // Histereza wilgotności powietrza
    EEPROM.get(EEPROM_ADDR_AIR_QUALITY + 4, AirQualityHysteresis); // Histereza jakości powietrza
}


// Funkcja animacji startowej
void startAnimation() {
  u8g2.clearBuffer();  // Clear the screen before starting animation
  delay(2000);  // Pause for 2 seconds before starting the animation

  u8g2.setFont(u8g2_font_5x8_tr);

  // Dots animation
  for (int i = 1; i <= 3; i++) {
    u8g2.clearBuffer();
    for (int j = 1; j <= i; j++) {
      u8g2.drawStr(10 * (j - 1) + 40, 32, ".");
    }
    u8g2.sendBuffer();
    delay(500);
  }

  // Text animation
  const char* text1 = "Follow the white";
  const char* text2 = "Rabbit";

  // Animacja pierwszego tekstu
  for (int i = 1; i <= strlen(text1); i++) {
    char tempText1[21];
    strncpy(tempText1, text1, i);
    tempText1[i] = '\0';
    u8g2.clearBuffer();
    u8g2.drawStr(10, 28, tempText1);
    u8g2.sendBuffer();
    delay(100);
  }

  // Animacja drugiego tekstu
  for (int i = 1; i <= strlen(text2); i++) {
    char tempText2[21];
    strncpy(tempText2, text2, i);
    tempText2[i] = '\0';
    u8g2.clearBuffer();
    u8g2.drawStr(10, 28, text1);  // Zachowanie pierwszego tekstu
    u8g2.drawStr(40, 44, tempText2);
    u8g2.sendBuffer();
    delay(100);
  }

  delay(2000);  // Wait for 2 seconds after animation
  u8g2.clearBuffer();  // Clear the screen at the end of animation
}


void handleMenu() {
  if (millis() - lastActivityTime > inactivityTimeout && currentMenuLevel != 1) {
    currentMenuLevel = 1;
  }


  switch (currentMenuLevel) {
    case 1:
      displaySensorReadings(); // Wyświetlanie odczytów
      break;

    case 2:
      if (joyY < 500) selectedParam = (selectedParam + 1) % 4; // Nawigacja w dół
      if (joyY > 600) selectedParam = (selectedParam - 1 + 4) % 4; // Nawigacja w górę
      Level2();
      break;

    case 3:
      switch (selectedParam) {
        case 0: // Temperatura
          if (joyX < 500) thresholdTemperature = max(0.0, thresholdTemperature - 0.1);
          if (joyX > 600) thresholdTemperature = min(maxTemperatureThreshold, thresholdTemperature + 0.1);
          break;
        case 1: // Wilgotność powietrza
          if (joyX < 500) thresholdHumidity = max(0.0, thresholdHumidity - 1);
          if (joyX > 600) thresholdHumidity = min(maxHumidityThreshold, thresholdHumidity + 1);
          break;
        case 2: // Wilgotność gleby
          if (joyX < 500) thresholdSoilMoisture = max(0.0, thresholdSoilMoisture - 1);
          if (joyX > 600) thresholdSoilMoisture = min(maxSoilMoistureThreshold, thresholdSoilMoisture + 1);
          break;
        case 3: // Jakość powietrza
          if (joyX < 500) thresholdAirQuality = max(0.0, thresholdAirQuality - 1);
          if (joyX > 600) thresholdAirQuality = min(maxAirQualityThreshold, thresholdAirQuality + 1);
          break;
        case 4: // Hyst. Temperatura
          if (joyX < 500) TemperatureHysteresis = max(0.0, TemperatureHysteresis - 0.1);
          if (joyX > 600) TemperatureHysteresis = min(10.0, TemperatureHysteresis + 0.1);
          break;
        case 5: // Hyst. Wilgotność
          if (joyX < 500) HumidityHysteresis = max(0.0, HumidityHysteresis - 1);
          if (joyX > 600) HumidityHysteresis = min(10.0, HumidityHysteresis + 1);
          break;
        case 6: // Hyst. Wilgotność gleby
          if (joyX < 500) SoilMoistureHysteresis = max(0.0, SoilMoistureHysteresis - 1);
          if (joyX > 600) SoilMoistureHysteresis = min(10.0, SoilMoistureHysteresis + 1);
          break;
        case 7: // Hyst. Jakość powietrza
          if (joyX < 500) AirQualityHysteresis = max(0.0, AirQualityHysteresis - 1);
          if (joyX > 600) AirQualityHysteresis = min(10.0, AirQualityHysteresis + 1);
          break;

      }

      Level3();
      resetActivityTimer();

      // Zapis do EEPROM po kliknięciu joysticka
      if (joystickButtonPressed()) {
        switch (selectedParam) {
          case 0:
            EEPROM.put(EEPROM_ADDR_TEMP, thresholdTemperature);
            break;
          case 1:
            EEPROM.put(EEPROM_ADDR_HUMIDITY, thresholdHumidity);
            break;
          case 2:
            EEPROM.put(EEPROM_ADDR_SOIL, thresholdSoilMoisture);
            break;
          case 3:
            EEPROM.put(EEPROM_ADDR_AIR_QUALITY, thresholdAirQuality);
            break;
          case 4:
            EEPROM.put(EEPROM_ADDR_TEMP + 4, TemperatureHysteresis);
            break;
          case 5:
            EEPROM.put(EEPROM_ADDR_HUMIDITY + 4, HumidityHysteresis);
            break;
          case 6:
            EEPROM.put(EEPROM_ADDR_SOIL + 4, SoilMoistureHysteresis);
            break;
          case 7:
            EEPROM.put(EEPROM_ADDR_AIR_QUALITY + 4, AirQualityHysteresis);
            break;
        }

        // Wyświetl komunikat o zapisie
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_5x8_tr);
        u8g2.drawStr(10, 30, "Zapisano!");
        u8g2.sendBuffer();
        delay(1000); // Krótka pauza na wyświetlenie komunikatu
      }
      break;
  }
}


void loop() {
    handleMenu();
    handleRelayAutomation();
}


void setup() {
  u8g2.begin();
  Wire.begin();
  Wire.setClock(100000); // Obniżenie prędkości do 100 kHz
  startAnimation();  // Uruchom animację startową
  loadFromEEPROM();
  dht.begin();
  resetActivityTimer();

  // Konfiguracja pinów
  pinMode(JOYSTICK_BUTTON_PIN, INPUT_PULLUP);
  pinMode(JOYSTICK_Y_PIN, INPUT_PULLUP);
  pinMode(JOYSTICK_X_PIN, INPUT_PULLUP);
  pinMode(UPPER_FLOAT_PIN, INPUT_PULLUP);
  pinMode(LOWER_FLOAT_PIN, INPUT_PULLUP);
  pinMode(RELAY_MAIN_FAN, OUTPUT);
  pinMode(RELAY_STEAM_GEN, OUTPUT);
  pinMode(RELAY_IRRIGATION, OUTPUT);
  pinMode(RELAY_TANK_PUMP, OUTPUT);
  
  // Wyłączenie przekaźników (odwrócona logika)
  digitalWrite(RELAY_MAIN_FAN, HIGH);
  digitalWrite(RELAY_STEAM_GEN, HIGH);
  digitalWrite(RELAY_IRRIGATION, HIGH);
  digitalWrite(RELAY_TANK_PUMP, HIGH);

  Serial.begin(9600);
  loadFromEEPROM();  // Załaduj wartości progów z EEPROM
}


// Obsługa automatyczna przekaźników
void handleRelayAutomation() {
  bool upperFloatActive = digitalRead(UPPER_FLOAT_PIN) == LOW; // Górny czujnik aktywny (LOW)
  bool lowerFloatActive = digitalRead(LOWER_FLOAT_PIN) == LOW; // Dolny czujnik aktywny (LOW)
  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();
  int airQuality = map(analogRead(AIR_QUALITY_PIN), 0, 1023, 0, 100);
  int soilMoisture = map(analogRead(SOIL_MOISTURE_PIN), 1023, 0, 0, 100);

  // Automatyka dla pompy zbiornika generatora pary
  if (!lowerFloatActive) {
    digitalWrite(RELAY_TANK_PUMP, LOW); // Włącz pompę (logika odwrócona: LOW = ON)
  } else if (upperFloatActive) {
    digitalWrite(RELAY_TANK_PUMP, HIGH); // Wyłącz pompę (logika odwrócona: HIGH = OFF)
  }

  // Automatyka dla temperatury
  if (temp > thresholdTemperature) {
    digitalWrite(RELAY_MAIN_FAN, LOW); // Włącz wentylator
  } else {
    digitalWrite(RELAY_MAIN_FAN, HIGH); // Wyłącz wentylator
  }

  // Automatyka dla wilgotności gleby
  if (soilMoisture < thresholdSoilMoisture) {
    digitalWrite(RELAY_IRRIGATION, LOW); // Włącz pompę nawadniania
  } else {
    digitalWrite(RELAY_IRRIGATION, HIGH); // Wyłącz pompę nawadniania
  }

  // Automatyka dla wilgotności powietrza
  if (humidity < thresholdHumidity) {
    digitalWrite(RELAY_STEAM_GEN, LOW); // Włącz generator pary
  } else {
    digitalWrite(RELAY_STEAM_GEN, HIGH); // Wyłącz generator pary
  }

  // Automatyka dla jakości powietrza
  if (airQuality < thresholdAirQuality) {
    digitalWrite(RELAY_MAIN_FAN, LOW); // Włącz wentylator
  } else {
    digitalWrite(RELAY_MAIN_FAN, HIGH); // Wyłącz wentylator
  }
}
