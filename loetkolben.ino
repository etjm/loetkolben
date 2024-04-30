#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_GrayOLED.h>
#include <Adafruit_SPITFT_Macros.h>
#include <Adafruit_SPITFT.h>
#include <gfxfont.h>
#include "PID.hpp"

#define TFT_CS        10
#define TFT_RST       8 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC        6 // labelled A0
//define TFT_SDA       12 //<- Pin numbers of the hardware SPI bus
//define TFT_SCK       13
#define HEATING_PIN   9
#define inp_roatenc1  2
#define inp_roatenc2  3
#define inp_button    12
#define TEMP_SENSE    A0
#define MAX_TEMP      450
#define HistSize      101

// uint16_t ?  = 0bRRRRRGGGGGGBBBBB;
uint16_t COLOR_GRAY = 0b0100001000001000;

enum button
{
  noClick,
  click,
  release
};

enum selectedFieldSettings
{
  NONE_SETTINGS,
  SMART_STANDBY,
  STANDBY_TIMER,
  NUMBER_OF_CH,
  CANCEL_SETTINGS,
  SET_SETTINGS,
  RESTORE_SETTINGS
};

enum selectedFieldPwm
{
  NONE_PWM,
  P,
  I,
  D,
  CANCEL_PWM,
  SET_PWM,
  RESTORE_PWM
};

enum selectedField1
{
  NONE,
  SOLL,
  PWM,
  SETTINGS,
  NUM300,
  NUM350,
  CANCEL,
  SET,
  RESTORE
};

selectedFieldSettings fieldStateSettings = selectedFieldSettings::NONE_SETTINGS;
selectedFieldPwm fieldStatePwm = selectedFieldPwm::NONE_PWM;
selectedField1 fieldStateMain = selectedField1::NONE;
button state = button::noClick;

unsigned long encTime1 = 0;
unsigned long encTime2 = 0;

unsigned long buttonTime = 0;
bool buttonLastState = false;
bool buttonState = false;  // always true, why ?
bool buttonState1 = false; // nicht mehr gebraucht
bool buttonDetected = false;

int selectedField = 100;
int selectedFieldPrev = 0;
bool fieldActive = false;
int numberOfFields = 1;

int temperature = 100;
float set = 0.0;
float mes = 100.0;
int heaterPWM = 0;
float prevSet = 0.0;
float prevMes = 0.0;
int prevPWM = 1;
// temperature chart
char history[HistSize] = {0};
char history2[HistSize] = {0};
int currentHistPos = 0;
int test = 50;

unsigned long lastMessureTime = 0;
unsigned long messureDelta = 50; // ms

float p = 10.0; // eine nachkomma stelle und 0.1 verstellbar
float i = 1.00; // zwei nachkomma stellen und 0.01 verstellbar
float d = 1.0;  // eine nachkomma stelle und 0.1 verstellbar
// int set = 100;  // sollwert
// int mes = 0;    // istWert
int power = 0;  // wert für standby (wie viel Leistung gezogen wird)

String smartStandbyOn = "On";
int timeTillStandby = 30;
int numberOfCh = 2;
int preset1 = 300;
int preset2 = 350;

// to detect rotation of the button
bool enc1 = false;
bool enc2 = false;
// counts the rotations of the button
int counter = 0;
// Counter for SelectSettings
int counterSettings = 0;
bool selectSettingsAktiv = false;
bool buttonhit = false; // wird nicht mehr gebraucht

int counterLastState = 0;
int counterLastState2 = 0;
int counterLastState3 = 0;
bool openSettingsWindowBool = false;
bool openPIDWindowBool = false;
bool changeSollTempBool = false;
bool setSoll300Bool = false;
bool setSoll350Bool = false;
bool openSmartStandby = false;
bool openStandbyTimer = false;
bool openNumberOfCh = false;
bool openCancel = false;
bool openSet = false;
bool openRestore = false;
bool openP = false;
bool openI = false;
bool openD = false;
int tester = 0;
bool switchAktive = true;
int lastCounterStatePwm = 0;
selectedFieldPwm lastFieldStatePwm = NONE_PWM;
bool setFieldStateToDefault = true;
bool standbyModeAktive = false;
int counterLastState4 = 0;

// stops the display from blinking, so display will be only refreshed when needed
bool initializeMainWindow = true;
bool stopInitializingMainWindow = false;
bool selectAktiv = true;

bool oneTimeOption3 = false;
int buttonCounter = 2; // wird nicht mehr benötigt

// For 1.44" and 1.8" TFT with ST7735 use:
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

PID PIDHeater1 = PID(&mes,&heaterPWM,&set,p,i,d);

void setup()
{
  // Use this initializer if using a 1.8" TFT screen:
  Serial.begin(115200);

  tft.initR(INITR_BLACKTAB); // Init ST7735S chip, black tab
  tft.setRotation(3);
  tft.setTextWrap(true);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);

  pinMode(HEATING_PIN, OUTPUT);
  analogWrite(HEATING_PIN, 0);

  pinMode(inp_button, INPUT_PULLUP);
  pinMode(inp_roatenc1, INPUT_PULLUP);
  pinMode(inp_roatenc2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(inp_roatenc1), ISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(inp_roatenc2), ISR2, CHANGE);

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
  if (set != prevSet or selectedField != selectedFieldPrev)
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
      tft.print(set, 1);
      tft.setTextColor(ST77XX_YELLOW);
    }
    else
      tft.print(set, 1);
    prevSet = set;
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
  if (mes != prevMes)
  {
    tft.fillRect(60, 17, 12 * 5, 14, ST77XX_BLACK);
    tft.setCursor(60, 17);
    if (mes < MAX_TEMP)
      tft.print(mes, 0);
    else
      tft.print("NC  -");
    prevMes = mes;
  }

  if (heaterPWM != prevPWM)
  {
    tft.setCursor(45, 32);
    tft.fillRect(45, 33, 12 * 3, 14, ST77XX_BLACK);
    tft.setTextSize(1);
    tft.print(heaterPWM, 1);
    tft.setTextSize(2);
    prevPWM = heaterPWM;
  }
}
// 160 * 128 pixel

void openSettingsWindow()
{
  if (counter != counterLastState2)
  {
    if (selectSettingsAktiv)
    {
      selectSettings();
    }
    counterLastState2 = counter;
  }

  // erster klick, öffnet fenster
  if (state == click)
  {
    // nur bei jedem ersten öffnen neu anzeigen
    tft.fillScreen(ST7735_BLACK);

    tft.setTextSize(1);

    tft.setCursor(10, 15);
    tft.print("Smart Standby:");
    tft.setCursor(125, 15);
    tft.print(smartStandbyOn);

    tft.setCursor(10, 35);
    tft.print("Time till Standby:");
    printTimeTillStandby();

    tft.setCursor(10, 55);
    tft.print("Number of Ch:");
    tft.setCursor(125, 55);
    tft.print(numberOfCh);
    /*
    tft.setCursor(10, 85);
    tft.print("Set");
    */
    tft.setCursor(50, 85);
    tft.print("Cancel");

    tft.setCursor(100, 85);
    tft.print("Restore");
    stopInitializingMainWindow = true;
    selectSettingsAktiv = true;

    selectAktiv = false;
  }
  // prevents from instandly going in to a setting after entering the menu
  if (setFieldStateToDefault)
  {
    fieldStateSettings = NONE_SETTINGS;
    setFieldStateToDefault = false;
  }

  if (switchAktive)
  {
    if (state == click)
    {
      switch (fieldStateSettings)
      {
      case SMART_STANDBY:
        openSmartStandby = true;
        break;
      case STANDBY_TIMER:
        openStandbyTimer = true;
        break;
      case NUMBER_OF_CH:
        openNumberOfCh = true;
        break;
      case CANCEL_SETTINGS:
        openCancel = true;
        break;
      case SET_SETTINGS:
        openSet = true;
        break;
      case RESTORE_SETTINGS:
        openRestore = true;
        break;
      default:
        break;
      }
    }
  }
  if (openSmartStandby)
  {
    changeSmartStandby();
  }
  if (openStandbyTimer)
  {
    changeTimeTillStandby();
  }
  if (openNumberOfCh)
  {
    changeNumberOfCh();
  }
  if (openCancel)
  {
    openSmartStandby = false;
    openStandbyTimer = false;
    openNumberOfCh = false;
    openSet = false;
    openCancel = false;
    openRestore = false;
    openSettingsWindowBool = false;
    initializeMainWindow = true;
    selectAktiv = true;
    setFieldStateToDefault = true;
  }
  if (openSet)
  {
  }
  if (openRestore)
  {
    restoreSettings();
  }
}

void openPIDWindow()
{

  if (counter != counterLastState2)
  {
    if (selectSettingsAktiv)
    {
      selectPWM();
    }
    counterLastState2 = counter;
  }

  // erster klick, öffnet fenster
  if (state == click)
  {
    tft.fillScreen(ST7735_BLACK);

    tft.setTextSize(2); // größe noch
    tft.setCursor(15, 20);
    tft.print("P:");
    tft.setCursor(45, 20);
    tft.print(p, 1);
    tft.setCursor(15, 42);
    tft.print("I:");
    tft.setCursor(45, 42);
    tft.print(i, 2);
    tft.setCursor(15, 64);
    tft.print("D:");
    tft.setCursor(45, 64);
    tft.print(d, 1);

    tft.setTextSize(1);
    tft.setCursor(100, 6);
    tft.print("Default");
    tft.setCursor(100, 20);
    tft.print("10");
    tft.setCursor(100, 42);
    tft.print("1.00");
    tft.setCursor(100, 64);
    tft.print("1.0");

    // tft.setTextSize(1);
    /*
    tft.setCursor(17, 100);
    tft.print("Set");
    */
    tft.setCursor(50, 100);
    tft.print("Cancel");
    tft.setCursor(100, 100);
    tft.print("Restore");
    stopInitializingMainWindow = true;
    selectSettingsAktiv = true;
    selectAktiv = false;
  }

  // prevents from instandly going in to a setting after entering the menu
  if (setFieldStateToDefault)
  {
    fieldStatePwm = NONE_PWM;
    setFieldStateToDefault = false;
  }

  if (switchAktive)
  {
    if (state == click)
    {
      switch (fieldStatePwm)
      {
      case P:
        openP = true;
        break;
      case I:
        openI = true;
        break;
      case D:
        openD = true;
        break;
      case CANCEL_PWM:
        openCancel = true;
        break;
      case SET_PWM:
        openSet = true;
        break;
      case RESTORE_PWM:
        openRestore = true;
        break;
      default:
        break;
      }
    }
  }
  if (openP)
  {
    changeP();
  }
  if (openI)
  {
    changeI();
  }
  if (openD)
  {
    changeD();
  }
  if (openCancel)
  {
    openSmartStandby = false;
    openStandbyTimer = false;
    openNumberOfCh = false;
    openSet = false;
    openCancel = false;
    openRestore = false;
    openPIDWindowBool = false;
    initializeMainWindow = true;
    selectAktiv = true;
    setFieldStateToDefault = true;
  }
  if (openSet)
  {
    setPWM();
  }
  if (openRestore)
  {
    restorePWM();
  }
}

void openMainWindow()
{
  tft.setTextSize(2);
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(0, 0);
  tft.print("Soll:");
  tft.setCursor(75, 0);
  tft.print(set,0);
  drawDegCel(120, 0);

  tft.setCursor(0, 17);
  tft.print("Ist:");
  tft.setCursor(75, 17);
  tft.print(mes,0);
  drawDegCel(120, 18);

  tft.setTextSize(1);
  tft.setCursor(10, 33);
  tft.print("PWM");

  tft.setCursor(10, 50);
  tft.print("Settings");

  tft.setCursor(75, 50);
  tft.print(preset1);
  drawDegCel(94, 50);

  tft.setCursor(120, 50);
  tft.print(preset2);
  drawDegCel(139, 50);
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

// todo
void setPWM()
{
}

void restoreSettings()
{
  if (state == click)
  {
    smartStandbyOn = "On";
    timeTillStandby = 30;
    numberOfCh = 2;
    tft.setTextSize(1);
    tft.setCursor(125, 15);
    tft.fillRect(125, 15, 20, 7, ST7735_BLACK);
    tft.print(smartStandbyOn);
    tft.fillRect(125, 35, 20, 7, ST7735_BLACK);
    printTimeTillStandby();
    tft.setCursor(125, 55);
    tft.fillRect(125, 55, 20, 7, ST7735_BLACK);
    tft.print(numberOfCh);
    openRestore = false;
  }
}

void restorePWM()
{
  if (state == click)
  {
    p = 10;
    i = 1;
    d = 1;
    tft.setTextSize(2);
    tft.fillRect(45, 20, 50, 15, ST7735_BLACK);
    tft.setCursor(45, 20);
    tft.print(p, 1);
    tft.fillRect(45, 42, 50, 15, ST7735_BLACK);
    tft.setCursor(45, 42);
    tft.print(i, 2);
    tft.fillRect(45, 64, 50, 15, ST7735_BLACK);
    tft.setCursor(45, 64);
    tft.print(d, 1);
    openRestore = false;
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
  tft.fillRect(7, 32, 25, 10, ST7735_BLACK);
  tft.setTextSize(1);
  tft.print("PWM");

  tft.setCursor(10, 50);
  tft.fillRect(9, 49, 49, 10, ST7735_BLACK);
  tft.setTextSize(1);
  tft.print("Settings");

  tft.fillRect(74, 49, 40, 10, ST7735_BLACK);
  tft.setCursor(75, 50);
  tft.setTextSize(1);
  tft.print(preset1);
  drawDegCel(94, 50);

  tft.fillRect(119, 49, 40, 10, ST7735_BLACK);
  tft.setCursor(120, 50);
  tft.print(preset2);
  tft.setTextSize(1);
  drawDegCel(139, 50);
  tft.setTextSize(2);
}

void selectMain()
{
  switch (counter % 5)
  {
  case 0:
    resetMain();
    // drawThickLine(0,0,55,15,ST7735_RED);
    tft.fillRect(0, 0, 55, 15, ST7735_RED);
    // tft.drawRoundRect(0,0,55,15,10,ST7735_RED);
    tft.setCursor(0, 0);
    tft.print("Soll:");
    fieldStateMain = SOLL;
    break;
  case 1:
    resetMain();
    tft.setCursor(10, 33);
    tft.fillRect(7, 32, 23, 10, ST7735_RED);
    tft.setTextSize(1);
    tft.print("PWM");
    fieldStateMain = PWM;
    break;
  case 2:
    resetMain();
    tft.setCursor(10, 50);
    tft.fillRect(9, 49, 49, 10, ST7735_RED);
    tft.setTextSize(1);
    tft.print("Settings");
    fieldStateMain = SETTINGS;
    break;
  case 3:
    resetMain();
    tft.fillRect(74, 49, 32, 10, ST7735_RED);
    tft.setCursor(75, 50);
    tft.setTextSize(1);
    tft.print(preset1);
    drawDegCel(94, 50);
    fieldStateMain = NUM300;
    break;
  case 4:
    resetMain();
    tft.fillRect(119, 49, 32, 10, ST7735_RED);
    tft.setCursor(120, 50);
    tft.setTextSize(1);
    tft.print(preset2);
    drawDegCel(139, 50);
    fieldStateMain = NUM350;
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
  /*
  tft.setCursor(10, 85);
  tft.fillRect(9, 84, 25, 9, ST7735_BLACK);
  tft.setTextSize(1);
  tft.print("Set");
  */
  tft.setCursor(50, 85);
  tft.fillRect(49, 84, 45, 9, ST7735_BLACK);
  tft.setTextSize(1);
  tft.print("Cancel");

  tft.setCursor(100, 85);
  tft.fillRect(99, 84, 45, 9, ST7735_BLACK);
  tft.setTextSize(1);
  tft.print("Restore");
}

void resetPwm()
{
  tft.setCursor(15, 20);
  tft.fillRect(13, 18, 14, 18, ST7735_BLACK);
  tft.setTextSize(2);
  tft.print("P:");
  tft.setCursor(45, 20);
  tft.print(p, 1);

  tft.setCursor(15, 42);
  tft.fillRect(13, 40, 14, 18, ST7735_BLACK);
  tft.setTextSize(2);
  tft.print("I:");
  tft.setCursor(45, 42);
  tft.print(i, 2);

  tft.setCursor(15, 64);
  tft.fillRect(13, 62, 14, 18, ST7735_BLACK);
  tft.setTextSize(2);
  tft.print("D:");
  tft.setCursor(45, 64);
  tft.print(d, 1);
  /*
  tft.setCursor(17, 100);
  tft.fillRect(16, 99, 19, 9, ST7735_BLACK);
  tft.setTextSize(1);
  tft.print("Set");
  */
  tft.setCursor(50, 100);
  tft.fillRect(49, 99, 36, 9, ST7735_BLACK);
  tft.setTextSize(1);
  tft.print("Cancel");

  tft.setCursor(100, 100);
  tft.fillRect(99, 99, 43, 9, ST7735_BLACK);
  tft.setTextSize(1);
  tft.print("Restore");
}

void selectSettings()
{
  // beim betreten der openSettingsFunktion, wird diese funktion aktiviert
  //  wenn sich button status ändert, dann funktion ausführen

  switch (counterSettings % 5)
  {
  case 0:
    resetSettings();
    tft.setCursor(10, 15);
    tft.fillRect(9, 14, 83, 9, ST7735_RED);
    tft.setTextSize(1);
    tft.print("Smart Standby:");
    fieldStateSettings = SMART_STANDBY;
    break;
  case 1:
    resetSettings();
    tft.setCursor(10, 35);
    tft.fillRect(9, 34, 107, 9, ST7735_RED);
    tft.setTextSize(1);
    tft.print("Time till Standby:");
    fieldStateSettings = STANDBY_TIMER;
    break;
  case 2:
    resetSettings();
    tft.setCursor(10, 55);
    tft.fillRect(9, 54, 77, 9, ST7735_RED);
    tft.setTextSize(1);
    tft.print("Number of Ch:");
    fieldStateSettings = NUMBER_OF_CH;
    break;
    /*
  case 3:
    resetSettings();
    tft.setCursor(10, 85);
    tft.fillRect(9, 84, 19, 9, ST7735_RED);
    tft.setTextSize(1);
    tft.print("Set");
    fieldStateSettings = selectedFieldSettings::SET_SETTINGS;
    break;
    */
  case 3:
    resetSettings();
    tft.setCursor(50, 85);
    tft.fillRect(49, 84, 36, 9, ST7735_RED);
    tft.setTextSize(1);
    tft.print("Cancel");
    fieldStateSettings = selectedFieldSettings::CANCEL_SETTINGS;
    break;
  case 4:
    resetSettings();
    tft.setCursor(100, 85);
    tft.fillRect(99, 84, 43, 9, ST7735_RED);
    tft.setTextSize(1);
    tft.print("Restore");
    fieldStateSettings = selectedFieldSettings::RESTORE_SETTINGS;
    break;
  }
}

void selectPWM()
{

  switch (counterSettings % 5)
  {
  case 0:
    resetPwm();
    tft.setCursor(15, 20);
    tft.fillRect(13, 18, 14, 18, ST7735_RED);
    tft.setTextSize(2);
    tft.print("P:");
    tft.setCursor(45, 20);
    tft.print(p, 1);
    fieldStatePwm = P;
    break;
  case 1:
    resetPwm();
    tft.setCursor(15, 42);
    tft.fillRect(13, 40, 14, 18, ST7735_RED);
    tft.setTextSize(2);
    tft.print("I:");
    tft.setCursor(45, 42);
    tft.print(i, 2);
    fieldStatePwm = I;
    break;
  case 2:
    resetPwm();
    tft.setCursor(15, 64);
    tft.fillRect(13, 62, 14, 18, ST7735_RED);
    tft.setTextSize(2);
    tft.print("D:");
    tft.setCursor(45, 64);
    tft.print(d, 1);
    fieldStatePwm = D;
    break;
    /*
  case 3:
    resetPwm();
    tft.setCursor(17, 100);
    tft.fillRect(16, 99, 19, 9, ST7735_RED);
    tft.setTextSize(1);
    tft.print("Set");
    fieldStatePwm = selectedFieldPwm::SET_PWM;
    break;
    */
  case 3:
    resetPwm();
    tft.setCursor(50, 100);
    tft.fillRect(49, 99, 36, 9, ST7735_RED);
    tft.setTextSize(1);
    tft.print("Cancel");
    fieldStatePwm = selectedFieldPwm::CANCEL_PWM;
    break;
  case 4:
    resetPwm();
    tft.setCursor(100, 100);
    tft.fillRect(99, 99, 43, 9, ST7735_RED);
    tft.setTextSize(1);
    tft.print("Restore");
    fieldStatePwm = selectedFieldPwm::RESTORE_PWM;
    break;
  }
}

void drawGraph()
{
  // test += ((random() % 5) - 2);
  // while (test < 0)
  //   test = test + 100;
  // test = test % 100;

  
  char point1 = (mes-100)/4;
  char point2 = (((int)set)-100)/4;

  // cap values
  point1 = point1 < 0 ? 0 : point1;
  point1 = point1 > 100 ? 100 : point1;
  point2 = point2 < 0 ? 0 : point2;
  point2 = point2 > 100 ? 100 : point2 ;

  history[currentHistPos] = point1;
  history2[currentHistPos] = point2;

  deletePlot(20, 70, history, HistSize, currentHistPos);
  deletePlot(20, 70, history2, HistSize, currentHistPos);
  printPlot(20, 70, history, HistSize, currentHistPos, ST77XX_BLUE);
  printPlot(20, 70, history2, HistSize, currentHistPos, ST77XX_RED);

  currentHistPos = (currentHistPos + 1) % HistSize;
}

void changeP()
{
  if (tester == 0)
  {
    // eintritts position speicher mit status
    counterLastState3 = counter;
    lastCounterStatePwm = counterSettings;
    lastFieldStatePwm = fieldStatePwm;
  }
  if (state == click)
  {
    selectSettingsAktiv = false;
    tester++;
    switchAktive = false;
  }

  if(counter != 0)
    tft.fillRect(45, 20, 50, 15, ST7735_BLACK);
  p = p + 0.1*counter;
  counter = 0;
  tft.setTextSize(2);
  tft.setCursor(45, 20);
  tft.setTextColor(ST7735_ORANGE);
  tft.print(p, 1);
  tft.setTextColor(ST7735_YELLOW);
  counterLastState3 = counter;
  if (tester == 2)
  {
    tft.setCursor(45, 20);
    tft.print(p, 1);
    openP = false;
    selectSettingsAktiv = true;
    switchAktive = true;
    tester = 0;
    // auf selected position vom eintritt zurück setzen
    counterSettings = lastCounterStatePwm;
    fieldStatePwm = lastFieldStatePwm;
  }
}

void changeI()
{
  if (tester == 0)
  {
    counterLastState3 = counter;
    // eintritts position speicher mit status
    lastCounterStatePwm = counterSettings;
    lastFieldStatePwm = fieldStatePwm;
  }
  if (state == click)
  {
    selectSettingsAktiv = false;
    tester++;
    switchAktive = false;
  }
  if(counter != 0)
    tft.fillRect(45, 42, 50, 15, ST7735_BLACK);
  i = i + 0.01*counter;
  counter = 0;

  tft.setTextSize(2);
  tft.setCursor(45, 42);
  tft.setTextColor(ST7735_ORANGE);
  tft.print(i, 2);
  tft.setTextColor(ST7735_YELLOW);
  counterLastState3 = counter;
  if (tester == 2)
  {
    tft.setCursor(45, 42);
    tft.print(i, 2);
    openI = false;
    selectSettingsAktiv = true;
    switchAktive = true;
    tester = 0;
    // auf selected position vom eintritt zurück setzen
    counterSettings = lastCounterStatePwm;
    fieldStatePwm = lastFieldStatePwm;
  }
}

void changeD()
{
  if (tester == 0)
  {
    // eintritts position speicher mit status
    counterLastState3 = counter;
    lastCounterStatePwm = counterSettings;
    lastFieldStatePwm = fieldStatePwm;
  }
  if (state == click)
  {
    selectSettingsAktiv = false;
    tester++;
    switchAktive = false;
  }
  if(counter != 0)
    tft.fillRect(45, 64, 50, 15, ST7735_BLACK);
  d = d + 0.01*counter;
  counter = 0;
  tft.setTextSize(2);
  tft.setCursor(45, 64);
  tft.setTextColor(ST7735_ORANGE);
  tft.print(d, 1);
  tft.setTextColor(ST7735_YELLOW);
  counterLastState3 = counter;
  if (tester == 2)
  {
    tft.setCursor(45, 64);
    tft.print(d, 1);
    openD = false;
    selectSettingsAktiv = true;
    switchAktive = true;
    tester = 0;
    // auf selected position vom eintritt zurück setzen
    counterSettings = lastCounterStatePwm;
    fieldStatePwm = lastFieldStatePwm;
  }
}

void changeSmartStandby()
{
  if (state == click)
  {
    tft.fillRect(125, 15, 20, 7, ST7735_BLACK);
    if (smartStandbyOn == "On")
    {
      smartStandbyOn = "Off";
    }
    else
    {
      smartStandbyOn = "On";
    }
    tft.setCursor(125, 15);
    tft.print(smartStandbyOn);
  }
  openSmartStandby = false;
}

void changeTimeTillStandby()
{
  if (tester == 0)
  {
    // eintritts position speicher mit status
    counterLastState3 = counter;
    lastCounterStatePwm = counterSettings;
    lastFieldStatePwm = fieldStatePwm;
  }

  if (state == click)
  {
    selectSettingsAktiv = false;
    tester++;
    switchAktive = false;
  }

  tft.setTextSize(1);
  tft.fillRect(125, 35, 20, 7, ST7735_BLACK);
  if (counter > counterLastState3)
  {
    // erhöhe timeTillStandby
    timeTillStandby = timeTillStandby += 1;
  }
  else if (counter < counterLastState3)
  {
    // verringer timeTillStandby
    timeTillStandby = timeTillStandby -= 1;
    if (timeTillStandby < 10)
    {
      timeTillStandby = 10;
    }
  }
  tft.setTextColor(ST7735_ORANGE);
  printTimeTillStandby();
  tft.setTextColor(ST7735_YELLOW);
  counterLastState3 = counter;
  if (tester == 2)
  {
    printTimeTillStandby();
    openStandbyTimer = false;
    selectSettingsAktiv = true;
    switchAktive = true;
    tester = 0;
    // auf selected position vom eintritt zurück setzen
    counterSettings = lastCounterStatePwm;
    fieldStatePwm = lastFieldStatePwm;
  }
}

void printTimeTillStandby()
{
  tft.setCursor(125, 35);
  tft.print(timeTillStandby);
  if (timeTillStandby > 99)
  {
    tft.setCursor(143, 35);
    tft.print("s");
  }
  else
  {
    tft.setCursor(137, 35);
    tft.print("s");
  }
}

void changeNumberOfCh()
{
  if (tester == 0)
  {
    // eintritts position speicher mit status
    counterLastState3 = counter;
    lastCounterStatePwm = counterSettings;
    lastFieldStatePwm = fieldStatePwm;
  }

  if (state == click)
  {
    selectSettingsAktiv = false;
    tester++;
    switchAktive = false;
  }

  tft.setTextSize(1);
  tft.fillRect(125, 55, 20, 7, ST7735_BLACK);
  if (counter > counterLastState3)
  {
    // erhöhe timeTillStandby
    numberOfCh = numberOfCh += 1;
  }
  else if (counter < counterLastState3)
  {
    // verringer timeTillStandby
    numberOfCh = numberOfCh -= 1;
    if (numberOfCh < 0)
    {
      numberOfCh = 0;
    }
  }
  tft.setTextColor(ST7735_ORANGE);
  tft.setCursor(125, 55);
  tft.print(numberOfCh);
  tft.setTextColor(ST7735_YELLOW);
  counterLastState3 = counter;
  if (tester == 2)
  {
    tft.setCursor(125, 55);
    tft.print(numberOfCh);
    openNumberOfCh = false;
    selectSettingsAktiv = true;
    switchAktive = true;
    tester = 0;
    // auf selected position vom eintritt zurück setzen
    counterSettings = lastCounterStatePwm;
    fieldStatePwm = lastFieldStatePwm;
  }
}

void changeSollTemp()
{
  if (tester == 0)
  {
    counterLastState2 = counter;
  }
  if (state == click)
  {
    selectAktiv = false;
    initializeMainWindow = false;
    stopInitializingMainWindow = true;
  }

  if (state == click)
  {
    if (oneTimeOption3)
    {
      initializeMainWindow = true;
      stopInitializingMainWindow = false;
      selectAktiv = true; // aktiviert SelectFunktion wieder
      changeSollTempBool = false;
      oneTimeOption3 = false;
    }
  }

  if (state == click)
  {
    tester++;
    oneTimeOption3 = true;
  }

  if (tester == 2)
  {
    oneTimeOption3 = false;
    tester = 0;
  }

  tft.setTextSize(2);
  tft.setCursor(75, 0);
  tft.fillRect(65, 0, 45, 15, ST7735_BLACK);
  if (counter > counterLastState2)
  {
    // erhöhe soll temperatur
    set = set += 5;
  }
  else if (counter < counterLastState2)
  {
    // verringer soll temperatur
    set = set -= 5;
  }
  tft.setTextColor(ST7735_ORANGE);
  tft.print(set,0);
  tft.setTextColor(ST7735_YELLOW);
  counterLastState2 = counter;
}

void setSollTo300()
{
  if (state == click)
  {
    set = preset1;
    tft.setTextSize(2);
    tft.setCursor(75, 0);
    tft.fillRect(65, 0, 45, 15, ST7735_BLACK);
    tft.print(set,0);
    setSoll300Bool = false;
  }
}

void setSollTo350()
{
  if (state == click)
  {
    set = preset2;
    tft.setTextSize(2);
    tft.setCursor(75, 0);
    tft.fillRect(65, 0, 45, 15, ST7735_BLACK);
    tft.print(set,0);
    setSoll350Bool = false;
  }
}

void standbyMode()
{
  if(standbyModeAktive){
    tft.fillScreen(ST7735_BLACK);
    tft.setTextSize(2);
    tft.setCursor(60,50);
    tft.print("Standby");
  }
  if(state == click || counter != counterLastState4){
    //standby beenden
  }
  counterLastState4 = counter;
  // todo
}

void loop()
{
  readButton();
  // buttonState = smoothButton();
  // delay(100);

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
  counterSettings = counter;

  // deaktivieren damit display nicht flackert
  if (initializeMainWindow)
  {
    openMainWindow();
    // drawScreen1Dyn();
  }

  // schauen das display nicht flackert
  if (counter != counterLastState)
  {
    if (selectAktiv)
    { // selectAktiv wenn im window select nicht möglich und counter zurück setzten wo war zum zeit des betreten des fensters
      selectMain();
    }
    counterLastState = counter;
  }

  if (state == click)
  {
    switch (fieldStateMain)
    {
    case SETTINGS:
      openSettingsWindowBool = true;
      break;
    case PWM:
      openPIDWindowBool = true;
      break;
    case SOLL:
      changeSollTempBool = true;
      break;
    case NUM300:
      setSoll300Bool = true;
      break;
    case NUM350:
      setSoll350Bool = true;
      break;
    default:
      break;
    }
  }

  if (openSettingsWindowBool)
  {
    openSettingsWindow();
  }
  else if (openPIDWindowBool)
  {
    openPIDWindow();
  }
  else  if (changeSollTempBool)
  {
    changeSollTemp();
  }
  else  if (setSoll300Bool)
  {
    setSollTo300();
  }
  else  if (setSoll350Bool)
  {
    setSollTo350();
  }
  else {
    // draw dynamic stuff
    drawGraph();
    // tft.print()
    mes = getTemp();
    tft.setCursor(75, 17);
    tft.setTextSize(2);
    tft.fillRect(75, 17, 42, 16, ST77XX_BLACK);
    tft.print(mes,0);
  }

  // showPWMState(heaterPid_Out);
  unsigned long now = millis();
  unsigned long MessureTime = (now - lastMessureTime);
  if (MessureTime > messureDelta){
    analogWrite(HEATING_PIN,0);
    delay(5); // time to let the voltage settle
    mes = getTemp();
    lastMessureTime = now;
  }

  PIDHeater1.calc(); // Calc only if new measurement?

  if (mes > 390.0) // Protect tip from damages
    heaterPWM = 0;

  // if (mes > MAX_TEMP){PWM=0;}
  analogWrite(HEATING_PIN, heaterPWM);
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
        }
        else
        {
          counter--;
        }
      }
      encTime2 = intTime;
    }
    enc2 = state;
  }
}

void readButton()
{
  // Read button
  buttonState = digitalRead(inp_button);
  // todo Inverse

  // smooth Button
  if (buttonDetected == true and buttonState == true and buttonTime + 50 < millis())
  {
    buttonDetected = false;
  }
  if (buttonDetected == false and buttonState == false)
  {
    buttonDetected = true;
    buttonTime = millis();
  }

  // Edge detection
  state = button::noClick;
  if (buttonState != buttonLastState)
  {
    if (buttonState == true && buttonLastState == false)
      state = button::release;
    else if (buttonState == false && buttonLastState == true)
      state = button::click;
    buttonLastState = buttonState;
  }
}
