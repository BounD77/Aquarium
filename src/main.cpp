
using namespace std;

/*
 An example analogue clock using a TFT LCD screen to show the time
 use of some of the drawing commands with the modified Adafruit_TFT_AS library.
 For a more accurate clock, it would be better to use the RTClib library.
 But this is just a demo. 
 
 This sketch uses font 4 only.
 Make sure all the required fonts are loaded by editting the
 User_Setup.h file in the TFT_ILI9341 library folder.
 If using an UNO or Mega (ATmega328 or ATmega2560 processor) then for best
 performance use the F_AS_T option found in the User_Setup.h file in the
 TFT_ILI9341 library folder.
 The library uses the hardware SPI pins only:
   For UNO, Nano, Micro Pro ATmega328 based processors
      MOSI = pin 11, SCK = pin 13
   For Mega:
      MOSI = pin 51, SCK = pin 52
 The pins used for the TFT chip select (CS) and Data/command (DC) and Reset (RST)
 signal lines to the TFT must also be defined in the library User_Setup.h file.
 Sugested TFT connections for UNO and Atmega328 based boards
   sclk 13  // Don't change, this is the hardware SPI SCLK line
   mosi 11  // Don't change, this is the hardware SPI MOSI line
   cs   10  // Chip select for TFT display
   dc   9   // Data/command line
   rst  7   // Reset, you could connect this to the Arduino reset pin
 Suggested TFT connections for the MEGA and ATmega2560 based boards
   sclk 52  // Don't change, this is the hardware SPI SCLK line
   mosi 51  // Don't change, this is the hardware SPI MOSI line
   cs   47  // TFT chip select line
   dc   48  // TFT data/command line
   rst  44  // you could alternatively connect this to the Arduino reset
  #########################################################################
  ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
  ######       TO SELECT THE FONTS AND PINS YOU USE, SEE ABOVE       ######
  #########################################################################
 
 Based on a sketch by Gilchrist 6/2/2014 1.0
 */
// Видеообзоры и уроки работы с ARDUINO на YouTube-канале IOMOIO: https://www.youtube.com/channel/UCmNXABaTjX_iKH28TTJpiqA

#include <Arduino.h>
#include <Wire.h>        // I2C
#include <TFT_ILI9341.h> // Hardware-specific library
#include <SPI.h>
#include <URTouch.h> // Библиотека для работы с сенсорным экраном

#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
//#include <PZEM004T.h>

// Для управления очисткой экрана с помощью кнопки RESET на Arduino подключить вывод дисплея RESET через резистор к пину RESET на плате Arduino
// Для Mega 2560 вывод дисплея RESET, если не подключен в пин RESET на Arduino, подключить в 3.3V (без резистора), либо в 5V (с резистором)

#define TFT_GREY 0x5AEB
TFT_ILI9341 tft = TFT_ILI9341(); // Invoke custom library
#define t_SCK 3                  // Пин подключения вывода дисплея T_CLK
#define t_CS 4                   // Пин подключения вывода дисплея T_CS
#define t_MOSI 5                 // Пин подключения вывода дисплея T_DIN
#define t_MISO 6                 // Пин подключения вывода дисплея T_DOUT
#define t_IRQ 7                  // Пин подключения вывода дисплея T_IRQ

URTouch ts(t_SCK, t_CS, t_MOSI, t_MISO, t_IRQ); // Создаем объект сенсорного модуля и сообщаем библиотеке распиновку для работы с ним
// подключение термодатчика
#define PIN_DHT 2 // пин термодатчика

// подключение датчика освещенности  GY-2561  I2C

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   
   Connections
   ===========
   Connect SCL to I2C SCL Clock
   Connect SDA to I2C SDA Data
   Connect VDD to 3.3V or 5V (whatever your logic level is)
   Connect GROUND to common ground

   I2C Address
   ===========
   The address will be different depending on whether you leave
   the ADDR pin floating (addr 0x39), or tie it to ground or vcc. 
   The default addess is 0x39, which assumes the ADDR pin is floating
   (not connected to anything).  If you set the ADDR pin high
   or low, use TSL2561_ADDR_HIGH (0x49) or TSL2561_ADDR_LOW
   (0x29) respectively.
    
   History
   =======
   2013/JAN/31  - First version (KTOWN)
*/
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

// подключение pH-метра
#define SensorPHPin A0      // pH meter Analog output to Arduino Analog Input 0
#define Offset -1.81        // Компенсация смещения
//#define LED 13              // Номер вывода светодиода, который является индикатором нормальной работы скетча
#define samplingInterval 1000 // Интервал в мс между измерениями
#define ArrayLenth 40

// подключение измерителя мощности PZEM
//PZEM004T *pzem; // (RX,TX) connect to TX,RX of PZEM
//IPAddress ip(192, 168, 2, 1);

#define PIN_RESET_PZEM 31 // Пин сброса счетчика PZEM
#define PIN_FLOOD 32 // Пин датчика протечки
#define PIN_PRESSURE 39 // давление CO2

// выходные пины
#define PIN_HEATER 33 // нагреватель
#define PIN_LIGHT1 34 // свет 1
#define PIN_LIGHT2 35 // свет 2
#define PIN_LIGHT3 36 // свет 3
#define PIN_O2     37 // воздушный компрессор
#define PIN_CO2    38 // реле  CO2
#define PIN_FILTER 40 // фильтр
#define PIN_UF 41 // ультрафиолетовый светильник




void setup()
{
  // put your setup code here, to run once:
  tft.init();
  tft.setRotation(2);

  //tft.fillScreen(TFT_BLACK);
  //tft.fillScreen(TFT_RED);
  //tft.fillScreen(TFT_GREEN);
  //tft.fillScreen(TFT_BLUE);
  //tft.fillScreen(TFT_BLACK);
  tft.fillScreen(TFT_GREY);
  ts.InitTouch();                // Инициализируем сенсорный модуль дисплея
  ts.setPrecision(PREC_EXTREME); // Определяем необходимую точность обработки нажатий: PREC_LOW - низкая, PREC_MEDIUM - средняя, PREC_HI - высокая, PREC_EXTREME - максимальная
Serial.begin(115200);
  // Initialise the sensor 2561
  //use tsl.begin() to default to Wire,
  //tsl.begin(&Wire2) directs api to use Wire2, etc.
   if (!tsl.begin())   {
    // There was a problem detecting the TSL2561 ... check your connections 
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while (1) {}
      
  }
configureSensor();
tsl.enableAutoRange(true); // Auto-gain ... switches automatically between 1x and 16x 
   tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS); // fast but low resolution 
   

  // Display some basic information on this sensor 
//  displaySensorDetails();

  
  pinMode(PIN_DHT, INPUT_PULLUP);
  // Setup the sensor gain and integration time 
  
  // while (!Serial1)  {   }
  //pzem = new PZEM004T(&Serial1);
  //pzem->setAddress(ip); 
  pinMode(PIN_RESET_PZEM, INPUT_PULLUP);
  pinMode(PIN_FLOOD, INPUT_PULLUP);
 
  // выходные пины
  pinMode(PIN_HEATER, OUTPUT);
  pinMode(PIN_LIGHT1, OUTPUT);
  pinMode(PIN_LIGHT2, OUTPUT);
  pinMode(PIN_LIGHT3, OUTPUT);
  pinMode(PIN_O2, OUTPUT);
  pinMode(PIN_CO2, OUTPUT);
  pinMode(PIN_FILTER, OUTPUT);
  pinMode(PIN_UF, OUTPUT);

  
//test_work(); // проверка работоспособности нагрузок путем поочередного включения и измерения силы тока
}

void loop() {
  // Get a new sensor 2561 event 
  sensors_event_t event;
  tsl.getEvent(&event);
//displaySensorDetails();
  // Display the results (light is measured in lux) 
  if (event.light)   {
    Serial.print(event.light);
    Serial.println(" lux");
  }   else   {
    // If event.light = 0 lux the sensor is probably saturated
    //   and no reliable data could be generated! 
    Serial.println("Sensor overload");
  } 
  
  // put your main code here, to run repeatedly:
}

String utf8rus(String source) {
  int i, k;
  String target;
  unsigned char n;
  char m[2] = {'0', '\0'};

  k = source.length();
  i = 0;

  while (i < k)   {
    n = source[i];
    i++;

    if (n >= 0xC0)     {
      switch (n)       {
      case 0xD0:       {
        n = source[i];
        i++;
        if (n == 0x81) {
          n = 0xA8;
          break;
        }
        if (n >= 0x90 && n <= 0xBF)
          n = n + 0x30;
        break;
      }
      case 0xD1:       {
        n = source[i];
        i++;
        if (n == 0x91)         {
          n = 0xB8;
          break;
        }
        if (n >= 0x80 && n <= 0x8F)
          n = n + 0x70;
        break;
      }
      }
    }
    m[0] = n;
    target = target + String(m);
  }
  return target;
}

// функция возврата значения PH                                                                                              ////

//
//  Функция определения среднего значения напряжения                                          // Эта функция возвращает среднее арифметическое значение данных массива arr без учёта одного максимального и одного минимального значения массива.
double averagearray(uint16_t *arr, uint8_t number)
{            //
  uint8_t i; // Объявляем переменные для цикла и экстремумов
  uint16_t max, min;
  double avg;      // Объявляем переменную для вывода среднего значения
  long amount = 0; // Определяем переменную для подсчёта среднего значения
  if (number <= 0)
  {
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  } // В массиве arr не может быть 0 и менее элементов
  if (number < 5)
  {
    for (i = 0; i < number; i++)
    {
      amount += arr[i];
    }
    avg = amount / number;
    return avg; // Если в массиве arr менее 5 элементов, то среднее значение является средним арифметическим значением
  }
  else
  { // Если в массиве arr более 5 элементов, то среднее значение считаем иначе ...
    if (arr[0] < arr[1])    {
      min = arr[0];
      max = arr[1];
    } // Определяем минимальное и максимальное число из первых двух элементов массива
    else  {
      min = arr[1];
      max = arr[0];
    } // Определяем минимальное и максимальное число из первых двух элементов массива
    for (i = 2; i < number; i++)     { // Проходим по остальным элементам массива
      if (arr[i] < min)       {
        amount += min;
        min = arr[i];
      } // Если значение очередного элемента меньше минимального,  то добавляем к значению amount предыдущее минимальное значение  и обновляем значение min
      else if (arr[i] > max)       {
        amount += max;
        max = arr[i];
      } // Если значение очередного элемента больше максимального, то добавляем к значению amount предыдущее максимальное значение и обновляем значение max
      else       {
        amount += arr[i];
      }                                  // Если значение очередного элемента находится в пределах между min и max, то добавляем значение этого элемента к amount
    }                                    //
    avg = (double)amount / (number - 2); // Получаем среднее арифметическое значение (без учета значений первых двух элементов массива arr, т.к. они не добавлялись к amount)
  }                                      //
  return avg;                            // Возвращаем полученное среднее значение
} //

float dataPHMeter(void) { //
  static float pHValue, voltage;
  static uint16_t pHArray[ArrayLenth]; // Массив для определения среднего показания напряжения считанного с датчика
  static uint16_t pHArrayIndex = 0;
  // Объявляем переменные для хранения значений напряжения и pH
  //  Проводим измерения:                                                                       //
  pHArray[pHArrayIndex++] = analogRead(SensorPHPin); // Читаем данные в очередной элемент массива pHArray
  if (pHArrayIndex == ArrayLenth)
    pHArrayIndex = 0;                                       // Если достигли последнего элемента массива pHArray, то сбрасываем номер текущего элемента этого массива в 0
  voltage = averagearray(pHArray, ArrayLenth) * 5.0 / 1023; // Получаем среднее напряжение в мВ из массива напряжений pHArray
  return pHValue = 3.5 * voltage + Offset;                  // Преобразуем мВ в pH
} //



void configureSensor(void)
{
  // You can also manually set the gain or enable auto-gain support 
   //tsl.setGain(TSL2561_GAIN_1X);      // No gain ... use in bright light to avoid sensor saturation 
   //tsl.setGain(TSL2561_GAIN_16X);     // 16x gain ... use in low light to boost sensitivity 
   tsl.enableAutoRange(true); // Auto-gain ... switches automatically between 1x and 16x 

  // Changing the integration time gives you better sensor resolution (402ms = 16-bit data) 
   tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS); // fast but low resolution 
   //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  // medium resolution and speed   
   //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  // 16-bit data but slowest conversions 

  // Update these values depending on what you've set above! 
  Serial.println("------------------------------------");
  Serial.print("Gain:         ");
  Serial.println("Auto");
  Serial.print("Timing:       ");
  Serial.println("13 ms");
  Serial.println("------------------------------------");
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" lux");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" lux");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" lux");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}



void test_work(void)
{

}