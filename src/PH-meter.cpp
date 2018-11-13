#include <Arduino.h>
#define SensorPHPin        A0                                                                   // pH meter Analog output to Arduino Analog Input 0
#define Offset           -1.81                                                                // Компенсация смещения
#define LED              13                                                                   // Номер вывода светодиода, который является индикатором нормальной работы скетча
#define samplingInterval 20                                                                   // Интервал в мс между измерениями
#define ArrayLenth       40                                                                   // Количество выборок
int pHArray[ArrayLenth];                                                                      // Массив для определения среднего показания напряжения считанного с датчика
int pHArrayIndex=0;                                                                           // Индекс элемента массива pHArray значения которого требуется изменить
// функция возврата значения PH                                                                                              ////
float dataPHMeter (void){                                                                              //
      static float pHValue, voltage;                                                            // Объявляем переменные для хранения значений напряжения и pH
//  Проводим измерения:                                                                       //
    pHArray[pHArrayIndex++] = analogRead(SensorPHPin);                                      // Читаем данные в очередной элемент массива pHArray
        if(pHArrayIndex==ArrayLenth) pHArrayIndex=0;                                          // Если достигли последнего элемента массива pHArray, то сбрасываем номер текущего элемента этого массива в 0
        voltage = averagearray(pHArray, ArrayLenth) * 5.0 / 1023;                             // Получаем среднее напряжение в мВ из массива напряжений pHArray
        pHValue = 3.5 * voltage + Offset;                                                     // Преобразуем мВ в pH
}                                                                                              //
                                                                                              //
//  Функция определения среднего значения напряжения                                          // Эта функция возвращает среднее арифметическое значение данных массива arr без учёта одного максимального и одного минимального значения массива.
double averagearray(int* arr, int number){                                                    //
    int i,max,min;                                                                            // Объявляем переменные для цикла и экстремумов
    double avg;                                                                               // Объявляем переменную для вывода среднего значения
    long amount=0;                                                                            // Определяем переменную для подсчёта среднего значения
    if(number<=0){ Serial.println("Error number for the array to avraging!/n");  return 0;}   // В массиве arr не может быть 0 и менее элементов
    if(number< 5){ for(i=0; i<number; i++){amount+=arr[i];} avg = amount/number; return avg;  // Если в массиве arr менее 5 элементов, то среднее значение является средним арифметическим значением
    }else{                                                                                    // Если в массиве arr более 5 элементов, то среднее значение считаем иначе ...
        if(arr[0]<arr[1]){ min = arr[0]; max=arr[1];}                                         // Определяем минимальное и максимальное число из первых двух элементов массива
        else             { min = arr[1]; max=arr[0];}                                         // Определяем минимальное и максимальное число из первых двух элементов массива
        for(i=2; i<number; i++){                                                              // Проходим по остальным элементам массива
                 if(arr[i]<min){ amount+=min; min=arr[i]; }                                   // Если значение очередного элемента меньше минимального,  то добавляем к значению amount предыдущее минимальное значение  и обновляем значение min
            else if(arr[i]>max){ amount+=max; max=arr[i]; }                                   // Если значение очередного элемента больше максимального, то добавляем к значению amount предыдущее максимальное значение и обновляем значение max
            else               { amount+=arr[i];          }                                   // Если значение очередного элемента находится в пределах между min и max, то добавляем значение этого элемента к amount
        }                                                                                     //
        avg = (double) amount/(number-2);                                                     // Получаем среднее арифметическое значение (без учета значений первых двух элементов массива arr, т.к. они не добавлялись к amount)
    }                                                                                         //
    return avg;                                                                               // Возвращаем полученное среднее значение
}                                                                                             //