#include <Arduino.h>
#include <arduinoFFT.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

const int samples = 1024;                // This value MUST ALWAYS be a power of 2
const double samplingFrequency = 10000; // 25us interval

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

/* Create FFT object */
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency, true);

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType);
void IRAM_ATTR ISR(void);
void drawOLEDImageFromSamples(int frequencies);

void setup()
{
  Serial.begin(115200);
  pinMode(A0, INPUT);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.drawRect(10, 10, 50, 30, WHITE);
  display.display();
  Serial.println("OLED Display initialized!");

  currentSample = 0;
  lastTimeExecutedFFT = millis();
  isReadyToRead = false;
  isReadyToCompute = false;

  noInterrupts();
  timer1_isr_init();
  Serial.println("ISR initialized");
  timer1_attachInterrupt(ISR);
  Serial.println("Interrupt function attached");
  timer1_enable(TIM_DIV1, TIM_EDGE, TIM_SINGLE);
  Serial.println("Timer enabled");
  timer1_write(8000); // 8000 ticks for a sampling frequency of 10kHz; Timer has a frequency of 80MHz
  interrupts();       // --> 1/80MHz = ticks/second --> 1/10kHz = samples/second --> (ticks/second) / (samples/second) = ticks/sample

  Serial.println("Finished setup");
}

void loop()
{
  if (isReadyToRead && !isReadyToCompute)
  {
    vReal[currentSample] = 1024.0 - (double)analogRead(A0);
    vImag[currentSample] = 0.0;
    // Serial.print(currentSample);
    // Serial.print(" ");
    // Serial.println(vReal[currentSample]);
    currentSample++;
    isReadyToRead = false;
    if (currentSample > samples)
    {
      currentSample = 0;
      isReadyToCompute = true;
    }
  }

  if (isReadyToCompute && lastTimeExecutedFFT + 500 < millis())
  {
    FFT.dcRemoval();
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

    
    double x;             // Print peak value
    double v;
    Serial.print("Peak: ");
    FFT.majorPeakParabola(&x, &v);
    Serial.print(x, 6);
    Serial.print(", ");
    Serial.println(v, 6);
    Serial.println();
    
    // drawOLEDImageFromSamples(8);

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
  timer1_write(8000);
  isReadyToRead = true;
}

void drawOLEDImageFromSamples(int frequencies)
{
  display.clearDisplay();

  int currentFrequency = samplingFrequency;
  int sampleCount = samples / 4;

  for (int i = 0; i < frequencies; i++)
  {
    currentFrequency = currentFrequency / 2;
    sampleCount = sampleCount / 2;
    int currentSample = samples * currentFrequency / samplingFrequency;

    // Calculate average magnitude
    int avgMagnitude = 0;
    if (i != 0)
    {
      for (int i = 0; i < sampleCount; i++)
      {
        avgMagnitude += vReal[currentSample + i];
        avgMagnitude += vReal[currentSample - (i + 1)];
      }
      avgMagnitude = avgMagnitude / (sampleCount*2);
    }
    else
    {
      // for the max frequency you can't use the samples above it
      for (int i = 0; i < sampleCount; i++)
      {
        avgMagnitude += vReal[currentSample - i];
      }
      avgMagnitude = avgMagnitude / sampleCount;
    }

    Serial.print("Average magnitude at sample: ");
    Serial.println(currentSample);

    Serial.print(currentFrequency);
    Serial.print("Hz, ");
    Serial.print(avgMagnitude);
    Serial.print(", ");

    int rectHeight = map(avgMagnitude, 0, 256, 0, 64);
    int rectWidth = SCREEN_WIDTH / frequencies;
    int rectPosX = i * SCREEN_WIDTH / frequencies;
    int rectPosY = 64;

    Serial.println(rectHeight);

    if (rectHeight != 0)
    {
      // display.fillRect(rectPosX, rectPosY, rectWidth, rectHeight, WHITE);
    }
  }

  display.display();
}