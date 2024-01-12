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
bool buttonState = false;
bool buttonDetected = false;

int selectedField = 0;
int selectedFieldPrev = 0;
bool fieldActive = false;
int numberOfFields = 1;

char window = 1;
char window_prev = 0;

float Set = 0.0;
float Mes = 100.0;
int PWM = 0;
float prevSet = 0.0;
float prevMes = 0.0;
int prevPWM = 1;

char history[HistSize] = {0};
char history2[HistSize] = {0};
int currentHistPos = 0;
int test = 50;

bool enc1 = false;
bool enc2 = false;
int counter = 0;
int counter2 = 0;

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

void drawStaticScreen1()
{
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
//Diferrent Windows
void openStandbywindow()
{
  tft.fillScreen(ST7735_BLACK);

  tft.setCursor(10, 15);
  tft.print("Smart Standby:");
  tft.setCursor(110, 15);
  tft.print("On");

  tft.setCursor(10, 35);
  tft.print("Time till Standby");
  tft.setCursor(110, 35);
  tft.print("30s");

  tft.setCursor(10, 55);
  tft.print("Number of Ch:");
  tft.setCursor(110, 55);
  tft.print("2");

  tft.setCursor(10, 85);
  tft.print("Set");

  tft.setCursor(60, 85);
  tft.print("Cancel");

  tft.setCursor(110, 85);
  tft.print("Restore");
}

void openPIDWindow()
{
  tft.fillScreen(ST7735_BLACK);

  tft.setCursor(15, 25);
  tft.print("P: 10");
  tft.setCursor(15, 35);
  tft.print("I: 1");
  tft.setCursor(15, 45);
  tft.print("D: 1");

  tft.setCursor(80, 15);
  tft.print("Default");
  tft.setCursor(80, 25);
  tft.print("10");
  tft.setCursor(80, 35);
  tft.print("1");
  tft.setCursor(80, 45);
  tft.print("1.0");

  tft.setCursor(15, 90);
  tft.print("Set");
  tft.setCursor(60, 90);
  tft.print("Cancel");
  tft.setCursor(110, 90);
  tft.print("Restore");
}

void openMainWindow()
{
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(0, 0);
  tft.print("Soll:");
  drawDegCel(120, 0);

  tft.setCursor(0, 17);
  tft.print("Ist:");
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

void resetMain(){
  tft.fillRect(0, 0, 55, 15, ST7735_BLACK);
  tft.setCursor(0, 0);
  //tft.setTextSize(2);
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

void selectSettings(){
  switch(counter2 % 3){
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
  //counter = 0;
}

void drawGraph(){
  test += ((random() % 5) - 2);
  while (test < 0)
    test = test + 100;
  test = test % 100;

  history[currentHistPos] = test;
  history2[currentHistPos] = (((int)Set) - 100) / 4;

  deletePlot(20,70,history,HistSize,currentHistPos);
  deletePlot(20,70,history2,HistSize,currentHistPos);
  printPlot(20,70,history,HistSize,currentHistPos,ST77XX_BLUE);
  printPlot(20,70,history2,HistSize,currentHistPos,ST77XX_RED);
  
  currentHistPos = (currentHistPos + 1) % HistSize;
}

void button(){
  buttonState = digitalRead(inp_button);

  // smooth Button
  if (buttonDetected == true and buttonState == true and buttonTime + 50 < millis())
  {
    buttonDetected = false;
  }
  if (buttonDetected == false and buttonState == false)
  {
    selectedField = (selectedField + 1) % numberOfFields;
    buttonDetected = true;
    buttonTime = millis();
  }

  if (buttonState)
    tft.fillRect(150, 0, 10, 10, ST77XX_RED);
  else
    tft.fillRect(150, 0, 10, 10, ST77XX_BLACK);

  if (buttonState != buttonLastState)
  {
    buttonLastState = buttonState;
    fieldActive = !fieldActive;
  }

  //! Abfrage der Eingabe

  if (!fieldActive)
  {
    selectedField += counter;
    selectedField %= numberOfFields;
  }
  else if (selectedField == 1)
  {
    Set += (counter * 2);
    if (Set > MAX_TEMP)
      Set = MAX_TEMP;
    if (Set < 100)
      Set = 100;
  }
  counter = 0;

}

void loop()
{
  // put your main code here, to run repeatedly:

  // tft.fillRect(0,20,10,14,ST77XX_BLACK);
  // tft.fillRect(0+12,20,10,14,ST77XX_BLACK);
  delay(100);
  // tft.print(99);
  PIDHeater1.setSetpoint(Set);
  PIDHeater1.setParameters(5.0, 0.0, 0.0);
  PIDHeater1.calc(getSet);
  float heaterPidOut = PIDHeater1.getControlValue();
  tft.setTextSize(1);
  tft.setCursor(80, 50);
  tft.fillRect(80, 50, 140, 65, ST77XX_BLACK);
  tft.print(heaterPidOut);
  tft.setTextSize(2);

  // Max Refresh rate?
  switch (window)
  {
  case 1:
    if (window != window_prev)
    {
      // tft.reset?
      openMainWindow();

      window_prev = window;
      numberOfFields = 2;
    }
    drawScreen1Dyn();
    // openStandbywindow();
    break;
  case 2:
    break;
  case 3:
    break;
  default:
    // Window vom Standby modus
    break;
  }

  drawGraph();
  int counter3 = 0;
  if(counter2 > counter3 || counter2 < counter3){
    selectSettings();
    counter3 = counter2;
  }
  button();
  

  
  // test

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

  // if (Mes > MAX_TEMP){PWM=0;}
  analogWrite(HEATING_PIN, PWM);
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
          //counter3--;
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
        }
        else
        {
          counter--;
          counter2--;
        }
      }
      encTime2 = intTime;
    }
    enc2 = state;
  }
}
