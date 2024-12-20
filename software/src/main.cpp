#include <Arduino.h>
#include <arduinoFFT.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <ESP8266WiFi.h>

const int samples = 1024; // This value MUST ALWAYS be a power of 2
const double samplingFrequency = 20000;

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

/*
Rectangle sizes:
  16 x 64 pixels
  for 8 rectangles
*/
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];
int currentSample;
volatile bool isReadyToRead;
bool isReadyToCompute;
double abscissa;

unsigned long lastTimeExecutedFFT;
unsigned long lastTimeExecutedOLED;

/* Create FFT object */
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency, true);

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType);
void IRAM_ATTR ISR(void);
void drawOLEDImageFromSamples(int frequencies);

void setup()
{
  Serial.begin(115200);

  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);
  Serial.print("CPU Frequency set to: ");
  Serial.println(ESP.getCpuFreqMHz());
  delay(2000);

  pinMode(A0, INPUT);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    while (1)
      ;
  }
  Serial.println("OLED Display initialized!");

  currentSample = 0;
  lastTimeExecutedFFT = millis();
  lastTimeExecutedOLED = millis();
  isReadyToRead = false;
  isReadyToCompute = false;

  noInterrupts();
  timer1_isr_init();
  Serial.println("ISR initialized");
  timer1_attachInterrupt(ISR);
  Serial.println("Interrupt function attached");
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
  Serial.println("Timer enabled");
  timer1_write(250); // 500 ticks for a sampling frequency of 10kHz and a pre-scaler of 16; Timer has a frequency of 80MHz
  interrupts();      // --> 1/(80MHz/16) = ticks/second --> 1/10kHz = samples/second --> (ticks/second) / (samples/second) = ticks/sample

  Serial.println("Finished setup");
}

void loop()
{
  if (isReadyToRead && !isReadyToCompute)
  {
    vReal[currentSample] = (double)analogRead(A0);
    currentSample++;
    isReadyToRead = false;
    if (currentSample >= samples)
    {
      currentSample = 0;
      isReadyToCompute = true;
    }
  }

  if (isReadyToCompute && lastTimeExecutedFFT + 50 < millis())
  {
    for (int i = 0; i < samples; i++)
    {
      vReal[i] -= 512; // remove dc value
      vImag[i] = 0.0;  // fill imaginary values with zero
    }
    // Serial.println("Data:");
    // PrintVector(vReal, samples, SCL_TIME);
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); /* Weigh data */
    // Serial.println("Weighed data:");
    // PrintVector(vReal, samples, SCL_TIME);
    FFT.compute(FFTDirection::Forward); /* Compute FFT */
    // Serial.println("Computed Real values:");
    // PrintVector(vReal, samples, SCL_INDEX);
    // Serial.println("Computed Imaginary values:");
    // PrintVector(vImag, samples, SCL_INDEX);
    FFT.complexToMagnitude(); /* Compute magnitudes */
    // Serial.println("Computed magnitudes:");
    // PrintVector(vReal, samples, SCL_FREQUENCY);

    double x; // Print peak value
    double v;
    Serial.print("Peak: ");
    FFT.majorPeak(&x, &v);
    Serial.print(x, 2);
    Serial.print(", ");
    Serial.println(v, 2);
    Serial.println();

    drawOLEDImageFromSamples(5);

    lastTimeExecutedFFT = millis();
    isReadyToCompute = false;
  }
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
    case SCL_INDEX:
      abscissa = (i * 1.0);
      break;
    case SCL_TIME:
      abscissa = ((i * 1.0) / samplingFrequency);
      break;
    case SCL_FREQUENCY:
      abscissa = ((i * 1.0 * samplingFrequency) / samples);
      break;
    }
    Serial.print(i);
    Serial.print(" ");
    Serial.print(abscissa, 6);
    if (scaleType == SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}

void IRAM_ATTR ISR(void)
{
  isReadyToRead = true;
}

void drawOLEDImageFromSamples(int frequencies)
{
  display.clearDisplay();

  int currentFrequency = 8192;
  // int currentFrequency = samplingFrequency / 2;  // max frequency you could show
  int sampleCount = samples / 4;
  int currentSample = samples /2;

  for (int i = 1; i <= frequencies; i++)
  {

    // Calculate average magnitude
    int avgMagnitude = 0;
    if (i == 1)
    {
      // for the max frequency you can't use the samples above it
      for (int i = 0; i < sampleCount / 2; i++)
      {
        avgMagnitude += vReal[currentSample - i];
      }
      avgMagnitude = avgMagnitude / sampleCount;
    }
    else
    {
      sampleCount /= 2;
      currentFrequency /= 2;
      currentSample /= 2;
      for (int i = 0; i < sampleCount / 2; i++)
      {
        avgMagnitude += vReal[currentSample + i];
        avgMagnitude += vReal[currentSample - (i + 1)];
      }
      avgMagnitude = avgMagnitude / sampleCount;
    }

    int rectHeight = map(avgMagnitude, 0, 2500, 0, 50);
    if (rectHeight > 50)
    {
      rectHeight = 50; // make sure that rectHeight isn't higher than the limit
    }
    else if (rectHeight <= 0)
    {
      rectHeight = 1;
    }

    int rectWidth = SCREEN_WIDTH / frequencies;
    int rectPosX = 128 - i * rectWidth;
    int rectPosY = 64 - rectHeight;

    display.fillRect(rectPosX, rectPosY, rectWidth, rectHeight, WHITE);


    Serial.print("Average magnitude at sample: ");
    Serial.print(currentSample);
    Serial.print(" with ");
    Serial.print(sampleCount);
    Serial.println(" samples used");

    Serial.print(currentFrequency);
    Serial.print("Hz, ");
    Serial.print(avgMagnitude);
    Serial.print(", ");
    Serial.println(rectHeight);
  }

  display.display();
}