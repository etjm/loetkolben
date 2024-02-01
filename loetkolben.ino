#include <Adafruit_GFX.h>
#include <Adafruit_GrayOLED.h>
#include <Adafruit_SPITFT_Macros.h>
#include <Adafruit_SPITFT.h>
#include <gfxfont.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include "PID.hpp"

#define TFT_CS 10
#define TFT_RST 8 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC 9
#define HEATING_PIN 5
#define inp_roatenc1 2
#define inp_roatenc2 3
#define inp_button 4
#define TEMP_SENSE A0
#define MAX_TEMP 450
#define HistSize 101

// uint16_t ?  = 0bRRRRRGGGGGGBBBBB;
uint16_t COLOR_GRAY = 0b0100001000001000;

unsigned long encTime1 = 0;
unsigned long encTime2 = 0;

unsigned long buttonTime = 0;
bool buttonLastState = false;
bool buttonState = false; // always true, why ?
bool buttonState1 = false;
bool buttonState2 = true;
bool buttonDetected = false;

int selectedField = 100;
int selectedFieldPrev = 0;
bool fieldActive = false;
int numberOfFields = 1;

char window = 1;
char window_prev = 0;

int temperature = 100;
float Set = 0.0;
float Mes = 100.0;
int PWM = 0;
float prevSet = 0.0;
float prevMes = 0.0;
int prevPWM = 1;
// temperature chart
char history[HistSize] = {0};
char history2[HistSize] = {0};
int currentHistPos = 0;
int test = 50;

int inWindow = 0;
int buttonClicked = 0;
int tempMemory = 0;
bool tempMemoryButtonState = false;

// to detect rotation of the button
bool enc1 = false;
bool enc2 = false;
// counts the rotations of the button
int counter = 0;
// same as counter
int counter2 = 0;
// Counter for field selection
int counter3 = 0;
// Counter for SelectSettings
int counterSettings = 0;
int counterSettings2 = 0;
bool selectSettingsAktiv = false;
bool buttonhit = false;
bool buttonhit2 = false;
// stops the display from blinking
bool initializeMainWindow = true;
bool stopInitializingMainWindow = false;
bool selectAktiv = true;
bool oneTimeOption = true;
bool oneTimeOption2 = true;
bool oneTimeOption3 = true;
int buttonCounter = 2;
bool test2 = false;
bool oneTimeOption4 = false;
bool buttonMemory = false;
int counter4 = 0;

// For 1.44" and 1.8" TFT with ST7735 use:
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

PID PIDHeater1(0.0, 0.0, 0.0);

void setup()
{
  // put your setup code here, to run once:
  // Use this initializer if using a 1.8" TFT screen:
  Serial.begin(115200);

  tft.initR(INITR_BLACKTAB); // Init ST7735S chip, black tab
  tft.setRotation(3);
  tft.setTextWrap(true);

  tft.fillScreen(ST77XX_BLACK);

  tft.drawPixel(160, 150, ST77XX_BLUE);
  tft.setCursor(0, 140);
  tft.setTextColor(ST7735_CYAN);
  tft.print("Test1");

  tft.setTextColor(ST77XX_YELLOW);

  pinMode(HEATING_PIN, OUTPUT);
  analogWrite(HEATING_PIN, 0);

  pinMode(inp_button, INPUT_PULLUP);
  pinMode(inp_roatenc1, INPUT_PULLUP);
  pinMode(inp_roatenc2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(inp_roatenc1), ISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(inp_roatenc2), ISR2, CHANGE);
  tft.setTextSize(2);

  PIDHeater1.start();
}

void deletePlot(int x, int y, char *data, int size, int lastIndex, uint16_t Color = ST77XX_BLACK)
{
  for (int i = 1; i < size; i++) // 1. Wert wird Ignoriert
  {
    tft.drawPixel(x + i - 1, y + 50 - ((data[(lastIndex + i) % size]) / 2), Color);
  }
}

void printPlot(int x, int y, char *data, int size, int lastIndex, uint16_t Color = ST77XX_RED)
{
  // add OFFSET?
  for (int i = 1; i < size; i++)
  {
    tft.drawPixel(x + i - 1, y + 50 - ((data[(lastIndex + i + 1) % size]) / 2), Color);
  }
  /*
  for (int i = 0; i < HistSize; i++)
    tft.drawPixel(x+i,y+40-(history[(currentHistPos + i)%HistSize])/2,ST77XX_BLACK);
  for (int i = 0; i < HistSize; i++)
    tft.drawPixel(x+i,y+28-(history[(currentHistPos + 1 + i)%HistSize])/2,ST77XX_RED);
  */
}

float getSet()
{
  return 110.0;
}

void drawDegCel(int xpos, int ypos)
{
  tft.drawCircle(xpos + 2, ypos + 2, 2, ST77XX_YELLOW);
  tft.setCursor(xpos + 6, ypos);
  tft.println('C');
}

void drawScreen1Dyn()
{
  // Print Input fields

  // Print set temperature if didn't change
  if (Set != prevSet or selectedField != selectedFieldPrev)
  {
    tft.fillRect(60, 0, 12 * 5, 14, ST77XX_BLACK);
    tft.setCursor(60, 0);
    if (selectedField == 1)
    {
      tft.setTextColor(COLOR_GRAY);
      if (fieldActive)
      {
        tft.setTextColor(ST77XX_ORANGE);
      }
      tft.print(Set, 1);
      tft.setTextColor(ST77XX_YELLOW);
    }
    else
      tft.print(Set, 1);
    prevSet = Set;
  }

  if (selectedField != selectedFieldPrev)
  {
    if (selectedField == 2)
    {
      tft.setTextColor(COLOR_GRAY);
      if (fieldActive)
      {
        tft.setTextColor(ST77XX_ORANGE);
      }
    }
    tft.setTextSize(1);
    tft.setCursor(10, 50);
    tft.print("Settings");
    tft.setTextColor(ST77XX_YELLOW);
    tft.setTextSize(2);
  }
  selectedFieldPrev = selectedField;

  // Numbers
  if (Mes != prevMes)
  {
    tft.fillRect(60, 17, 12 * 5, 14, ST77XX_BLACK);
    tft.setCursor(60, 17);
    if (Mes < MAX_TEMP)
      tft.print(Mes, 1);
    else
      tft.print("NC  -");
    prevMes = Mes;
  }

  if (PWM != prevPWM)
  {
    tft.setCursor(45, 32);
    tft.fillRect(45, 33, 12 * 3, 14, ST77XX_BLACK);
    tft.setTextSize(1);
    tft.print(PWM, 1);
    tft.setTextSize(2);
    prevPWM = PWM;
  }
}
// 160 * 128 pixel

// funktioniert
// counter zurücksetzten noch optimieren
void openSettingsWindow()
{
  // counter2 speichern counter und selectAktiv stopt die SelectFunktion
  //  if buttons is clicked
  if (oneTimeOption3)
  {
    if (buttonState && buttonClicked == 0)
    {
      buttonClicked++;
      tempMemory = counter2;
      selectAktiv = false;
      // selectSettingsAktiv = true;
      counterSettings = 0;
      counterSettings2 = 0;
      tempMemoryButtonState = buttonState;
      buttonhit2 = true;
    }
  }
  if (counterSettings != counterSettings2)
  {
    if ((counterSettings > counterSettings2 || counterSettings < counterSettings2) && buttonhit2) //&& buttonhit == false
    {
      // button einfügen für die zweite ebene
      //button hit setzt selectSettingsAktiv nicht auf false;
      if (selectSettingsAktiv)
      {
        // aktivierung dieser Funktion passt nicht
        selectSettings();
      }
      counterSettings2 = counterSettings;
    }
  }

  // button handling anpassen, wenn selectSettingsFunktion betreten
  // button in dieser funktion pausieren und eine ebene tiefer weiterführen

  // erster klick, öffnet fenster
  if ((buttonClicked % 2) == true && oneTimeOption2)
  {
    // nur bei jedem ersten öffnen neu anzeigen
    tft.fillScreen(ST7735_BLACK);

    tft.setTextSize(1);

    tft.setCursor(10, 15);
    tft.print("Smart Standby:");
    tft.setCursor(125, 15);
    tft.print("On");

    tft.setCursor(10, 35);
    tft.print("Time till Standby:");
    tft.setCursor(125, 35);
    tft.print("30s");

    tft.setCursor(10, 55);
    tft.print("Number of Ch:");
    tft.setCursor(125, 55);
    tft.print("2");

    tft.setCursor(10, 85);
    tft.print("Set");

    tft.setCursor(50, 85);
    tft.print("Cancel");

    tft.setCursor(100, 85);
    tft.print("Restore");
    tempMemoryButtonState = buttonState;
    stopInitializingMainWindow = true;
    oneTimeOption2 = false;
    oneTimeOption3 = false;
    selectSettingsAktiv = true;
  }
  // zweiter klick schließt fenster wieder und hebt beschränkungen wieder auf
  if (oneTimeOption3)
  {
    if (!buttonState && stopInitializingMainWindow)
    {
      // if (test2)
      //{
      initializeMainWindow = true;
      stopInitializingMainWindow = false;
      buttonClicked = 0;
      oneTimeOption2 = true; // sorgt dafür das das Fenster nur bei jedem ersten Aufruf initialisiert wird(sonst flackert display)
      selectAktiv = true;    // aktiviert SelectFunktion wieder
      counter2 = tempMemory; // setzt Counter zurück
      selectSettingsAktiv = false;
      buttonhit2 = false;
      //}
    }
  }
}

void openPIDWindow()
{
  // counter2 speichern counter und selectAktiv stopt die SelectFunktion
  //  if buttons is clicked
  if (buttonState && buttonClicked == 0)
  {
    buttonClicked++;
    tempMemory = counter2;
    selectAktiv = false;
  }
  // erster klick, öffnet fenster
  if ((buttonClicked % 2) == true && oneTimeOption)
  {
    tft.fillScreen(ST7735_BLACK);

    tft.setTextSize(2); // größe noch
    tft.setCursor(15, 20);
    tft.print("P: 10");
    tft.setCursor(15, 42);
    tft.print("I: 1");
    tft.setCursor(15, 64);
    tft.print("D: 1");

    tft.setTextSize(1);
    tft.setCursor(100, 6);
    tft.print("Default");
    tft.setCursor(100, 20);
    tft.print("10");
    tft.setCursor(100, 42);
    tft.print("1");
    tft.setCursor(100, 64);
    tft.print("1.0");

    // tft.setTextSize(1);
    tft.setCursor(17, 100);
    tft.print("Set");
    tft.setCursor(50, 100);
    tft.print("Cancel");
    tft.setCursor(100, 100);
    tft.print("Restore");
    stopInitializingMainWindow = true;
    oneTimeOption = false;
  }
  // zweiter klick schließt fenster wieder und hebt beschränkungen wieder auf
  if (!buttonState && stopInitializingMainWindow)
  {
    initializeMainWindow = true;
    stopInitializingMainWindow = false;
    buttonClicked = 0;
    oneTimeOption = true;  // sorgt dafür das das Fenster nur bei jedem ersten Aufruf initialisiert wird(sonst flackert display)
    selectAktiv = true;    // aktiviert SelectFunktion wieder
    counter2 = tempMemory; // setzt Counter zurück
  }
}

void openMainWindow()
{
  tft.setTextSize(2);
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(0, 0);
  tft.print("Soll:");
  tft.setCursor(75, 0);
  tft.print(temperature);
  drawDegCel(120, 0);

  tft.setCursor(0, 17);
  tft.print("Ist:");
  tft.setCursor(75, 0);
  tft.print("100");
  drawDegCel(120, 18);

  tft.setTextSize(1);
  tft.setCursor(10, 33);
  tft.print("PWM:");

  tft.setCursor(10, 50);
  tft.print("Settings");
  tft.setTextSize(2);

  tft.setTextColor(COLOR_GRAY);
  tft.drawLine(19, 68, 19, 121, COLOR_GRAY);
  tft.drawPixel(18, 69, COLOR_GRAY);
  tft.drawPixel(20, 69, COLOR_GRAY);

  tft.drawLine(19, 121, 121, 121, COLOR_GRAY);
  tft.drawPixel(120, 120, COLOR_GRAY);
  tft.drawPixel(120, 122, COLOR_GRAY);

  tft.setTextSize(1);
  tft.setCursor(0, 70);
  tft.print(500);
  tft.setCursor(0, 115);
  tft.print(100);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  initializeMainWindow = false;
}

void drawThickLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
  int thickness = abs(y0 - y1);
  int starty = 0;
  if (y0 >= y1)
  {
    starty = y1;
  }
  else
  {
    starty = y0;
  }
  for (int i = 0; i < thickness; i++)
  {
    tft.drawLine(x0, starty + i, x1, starty + i, color);
  }
}

// reset red rectangle for selectfunktion
void resetMain()
{
  tft.fillRect(0, 0, 55, 15, ST7735_BLACK);
  tft.setCursor(0, 0);
  tft.setTextSize(2);
  tft.print("Soll:");

  tft.setCursor(10, 33);
  tft.fillRect(8, 33, 25, 10, ST7735_BLACK);
  tft.setTextSize(1);
  tft.print("PWM:");

  tft.setCursor(10, 50);
  tft.fillRect(9, 50, 49, 10, ST7735_BLACK);
  tft.setTextSize(1);
  tft.print("Settings");
  tft.setTextSize(2);
}

void selectMain()
{
  switch (counter2 % 3)
  {
  case 2:
    resetMain();
    // drawThickLine(0,0,55,15,ST7735_RED);
    tft.fillRect(0, 0, 55, 15, ST7735_RED);
    // tft.drawRoundRect(0,0,55,15,10,ST7735_RED);
    tft.setCursor(0, 0);
    tft.print("Soll:");
    break;
  case 1:
    resetMain();
    tft.setCursor(10, 33);
    tft.fillRect(8, 33, 25, 10, ST7735_RED);
    tft.setTextSize(1);
    tft.print("PWM:");
    break;
  case 0:
    resetMain();
    tft.setCursor(10, 50);
    tft.fillRect(9, 50, 49, 10, ST7735_RED);
    tft.setTextSize(1);
    tft.print("Settings");
    break;
  }
}

void resetSettings()
{
  tft.setCursor(10, 15);
  tft.fillRect(9, 14, 110, 9, ST7735_BLACK);
  tft.setTextSize(1);
  tft.print("Smart Standby:");

  tft.setCursor(10, 35);
  tft.fillRect(9, 34, 110, 9, ST7735_BLACK);
  tft.setTextSize(1);
  tft.print("Time till Standby:");

  tft.setCursor(10, 55);
  tft.fillRect(9, 54, 110, 9, ST7735_BLACK);
  tft.setTextSize(1);
  tft.print("Number of Ch:");

  tft.setCursor(10, 85);
  tft.fillRect(9, 84, 25, 9, ST7735_BLACK);
  tft.setTextSize(1);
  tft.print("Set");

  tft.setCursor(50, 85);
  tft.fillRect(49, 84, 45, 9, ST7735_BLACK);
  tft.setTextSize(1);
  tft.print("Cancel");

  tft.setCursor(100, 85);
  tft.fillRect(99, 84, 45, 9, ST7735_BLACK);
  tft.setTextSize(1);
  tft.print("Restore");
}

void selectSettings()
{
  // beim betreten der openSettingsFunktion, wird diese funktion aktiviert
  //  wenn sich button status ändert, dann funktion ausführen

  switch (counterSettings % 6)
  {
  case 5:
    resetSettings();
    tft.setCursor(10, 15);
    tft.fillRect(9, 14, 83, 9, ST7735_RED);
    tft.setTextSize(1);
    tft.print("Smart Standby:");
    break;
  case 4:
    resetSettings();
    tft.setCursor(10, 35);
    tft.fillRect(9, 34, 107, 9, ST7735_RED);
    tft.setTextSize(1);
    tft.print("Time till Standby:");
    break;
  case 3:
    resetSettings();
    tft.setCursor(10, 55);
    tft.fillRect(9, 54, 77, 9, ST7735_RED);
    tft.setTextSize(1);
    tft.print("Number of Ch:");
    break;
  case 2:
    resetSettings();
    tft.setCursor(10, 85);
    tft.fillRect(9, 84, 19, 9, ST7735_RED);
    tft.setTextSize(1);
    tft.print("Set");
    break;
  case 1:
    resetSettings();
    tft.setCursor(50, 85);
    tft.fillRect(49, 84, 36, 9, ST7735_RED);
    tft.setTextSize(1);
    tft.print("Cancel");
    break;
  case 0:
    resetSettings();
    tft.setCursor(100, 85);
    tft.fillRect(99, 84, 43, 9, ST7735_RED);
    tft.setTextSize(1);
    tft.print("Restore");
    break;
  }

  // beim ersten entry buttonMemory = button state setzen 
  if(oneTimeOption4){
    buttonMemory = buttonState;
    oneTimeOption4 = false;
  }
  // wenn buttonhit detected funktion aus switch auslösen und selectSettings deaktivieren
  if (buttonState != buttonMemory)
  {
    counter4++;
    //später true setzen
    test2 = true;
    buttonMemory = buttonState;
  }

  if (test2)
  {
    switch (counterSettings % 6)
    {
    case 5:
      // Smart Standby
      // beim ersten klick true und beim zweiten wieder false setzen
      buttonhit2 = false;
      break;
    case 4:
      // Time till Standby
      break;
    case 3:
      // Number of Ch
      break;
    case 2:
      // Set
      break;
    case 1:
      // Cancel
      // buttonState zurücksetzen
      // andere sachen zurücksetzen
      // zurücksetzung erfolgt nicht zu 100
      buttonCounter = 2;
      oneTimeOption3 = true;
      counterSettings = 0;
      //nicht richtig zurück gesetzt
      buttonhit2 = false;

      initializeMainWindow = true;
      stopInitializingMainWindow = false;
      buttonClicked = 0;
      counterSettings2 = 0;
      oneTimeOption2 = true; // sorgt dafür das das Fenster nur bei jedem ersten Aufruf initialisiert wird(sonst flackert display)
      selectAktiv = true;    // aktiviert SelectFunktion wieder
      counter2 = tempMemory; // setzt Counter zurück
      selectSettingsAktiv = false;
      buttonState = false;
      test2 = false;
      oneTimeOption4 = true;



      /*
      // alle zurücksetzen
      counter2 = 0;
      counter3 = 0;
      buttonhit = false;
      oneTimeOption3 = true;
      buttonState = false; // button counter = 2 setzen;
      buttonClicked = 0;
      tempMemory = 0;
      selectAktiv = true;
      counterSettings = 0;
      counterSettings2 = 0;
      buttonhit2 = false;
      oneTimeOption2 = true;
      tempMemoryButtonState = false;
      stopInitializingMainWindow = false;
      selectSettingsAktiv = false;
      initializeMainWindow = true;


      */
      break;
    case 0:
      // Restore
      break;
    default:
      break;
    }
  }
  
}

void selectPWM()
{
}

void drawGraph()
{
  test += ((random() % 5) - 2);
  while (test < 0)
    test = test + 100;
  test = test % 100;

  history[currentHistPos] = test;
  history2[currentHistPos] = (((int)Set) - 100) / 4;

  deletePlot(20, 70, history, HistSize, currentHistPos);
  deletePlot(20, 70, history2, HistSize, currentHistPos);
  printPlot(20, 70, history, HistSize, currentHistPos, ST77XX_BLUE);
  printPlot(20, 70, history2, HistSize, currentHistPos, ST77XX_RED);

  currentHistPos = (currentHistPos + 1) % HistSize;
}

// gibt für jeden 2. Button hit true zurück
// funktioniert Delay teils im weg
bool smoothButton()
{
  buttonState1 = digitalRead(inp_button);
  if (buttonState1 == false)
  {
  }
  if (buttonDetected == false and buttonState1 == false)
  {
    buttonDetected = true;
    buttonTime = millis();
  }
  if (buttonDetected == true and buttonState1 == true and buttonTime + 50 < millis())
  {
    buttonDetected = false;
  }
  /*if (buttonState1 == false && buttonLastState == true)
  {
    Serial.println("hit");
    buttonLastState = buttonState1;
  }*/
  if (buttonState1 != buttonLastState)
  {
    //Serial.println("hit");
    buttonCounter++;
    buttonLastState = buttonState1;
  }

  if ((buttonCounter % 4) == 0)
  {
    buttonhit = true;
  }
  else if (buttonCounter % 2 == 0)
  {
    buttonhit = false;
  }

  return buttonhit;
}

void changeSollTemp()
{

  if (buttonState && buttonClicked == 0)
  {
    buttonClicked++;
    tempMemory = counter2;
    selectAktiv = false;
    initializeMainWindow = false;
    counter = 0;
    stopInitializingMainWindow = true;
  }
  if ((buttonClicked % 2) == true)
  {
    //Serial.println("selectedField");
    //Serial.println(temperature);
    // hier temperatur ändern
  }
  if (!buttonState && stopInitializingMainWindow)
  {
    initializeMainWindow = true;
    stopInitializingMainWindow = false;
    buttonClicked = 0;
    selectAktiv = true;    // aktiviert SelectFunktion wieder
    counter2 = tempMemory; // setzt Counter zurück
  }
}

void loop()
{
  buttonState = smoothButton();
  delay(100);
  Serial.print(selectSettingsAktiv);

  /*
  PIDHeater1.setSetpoint(Set);
  PIDHeater1.setParameters(5.0, 0.0, 0.0);
  PIDHeater1.calc(getSet);
  float heaterPid_Out = PIDHeater1.getControlValue();
  tft.setTextSize(1);
  tft.setCursor(80, 50);
  tft.fillRect(80, 50, 140, 65, ST77XX_BLACK);
  tft.print(heaterPid_Out);
  tft.setTextSize(2);
  */


  if (initializeMainWindow)
  {
    openMainWindow();
    // drawScreen1Dyn();
  }
  // drawGraph();

  if (counter2 != counter3)
  {
    if ((counter2 > counter3 || counter2 < counter3) && buttonhit == false)
    {
      if (selectAktiv)
      { // selectAktiv wenn im window select nicht möglich und counter zurück setzten wo war zum zeit des betreten des fensters
        selectMain();
      }
      counter3 = counter2;
    }
  }

  switch (counter3 % 3)
  {
  case 0:
    openSettingsWindow();
    break;
  case 1:
    // openPIDWindow();
    break;
  case 2:
    changeSollTemp();
    break;

  default:
    break;
  }

  // test

  // showPWMState(heaterPid_Out);

  // if (Mes > MAX_TEMP){PWM=0;}
  analogWrite(HEATING_PIN, PWM);
}

void showPWMState(float heaterPidOut)
{
  // Show PWM State
  if (PWM > 0)
    tft.fillRect(150, 10, 10, 10, ST77XX_YELLOW);
  else
    tft.fillRect(150, 10, 10, 10, ST77XX_BLACK);

  if (PWM < 0)
    PWM = 0;
  if (PWM > 255)
    PWM = 255;

  if (heaterPidOut <= 0.0)
  {
    PWM = 0;
  }
  else if (heaterPidOut <= 255.0)
  {
    PWM = (int)heaterPidOut;
  }
  else
  {
    PWM = 255;
  }

  Mes = getTemp();
  if (!buttonState)
  {
    PWM = round(Set) % 255;
  }
  else
  {
    PWM = 0;
  }
}

float getTemp()
{
  int adc = analogRead(TEMP_SENSE);
  return (adc * 0.574503 + 43.5);
}

void ISR1()
{
  bool state = digitalRead(inp_roatenc1);
  unsigned long intTime = micros();
  if (enc1 != state)
  {                                 // For Some reason the interrupt is called multiple times for the same State
    if (intTime > encTime1 + 20000) // Software Debounce
    {
      if (state == false) // Start of an rotation
      {
        if (enc2 == true) // CCW Rot
        {
          // counter--;
          // counter3--;
        }
      }
      encTime1 = intTime;
    }
    enc1 = state;
  }
}

void ISR2()
{
  bool state = digitalRead(inp_roatenc2);
  unsigned long intTime = micros();
  if (enc2 != state)
  {
    if (intTime > encTime2 + 4000)
    {
      if (state == false) // Start of an rotation
      {
        if (enc1 == true) // CW Rot
        {
          counter++;
          counter2++;
          counterSettings++;
        }
        else
        {
          counter--;
          counter2--;
          counterSettings--;
        }
      }
      encTime2 = intTime;
    }
    enc2 = state;
  }
}
