// --- DS1302 en Arduino Mega 2560 ---
// VCC→5V, GND→GND, CLK→D6, DTA(DAT/IO)→D7, RST(CE)→D8

#include <virtuabotixRTC.h>
#include <string.h>
#include <stdlib.h>

virtuabotixRTC myRtc(6, 7, 8);  // <-- NO usar el nombre 'rtc'

int monthFromDateMacro(const char* d){
  if(!strncmp(d,"Jan",3)) return 1;  if(!strncmp(d,"Feb",3)) return 2;
  if(!strncmp(d,"Mar",3)) return 3;  if(!strncmp(d,"Apr",3)) return 4;
  if(!strncmp(d,"May",3)) return 5;  if(!strncmp(d,"Jun",3)) return 6;
  if(!strncmp(d,"Jul",3)) return 7;  if(!strncmp(d,"Aug",3)) return 8;
  if(!strncmp(d,"Sep",3)) return 9;  if(!strncmp(d,"Oct",3)) return 10;
  if(!strncmp(d,"Nov",3)) return 11; return 12; // Dec
}

// 1=Dom,...,7=Sab (formato de la librería)
int dayOfWeekLib(int y,int m,int d){
  static int t[] = {0,3,2,5,0,3,5,1,4,6,2,4};
  if(m<3) y -= 1;
  int w = (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7; // 0..6 (0=Dom)
  return (w==0) ? 1 : (w+1); // 1..7
}

void setup(){
  const char* DATE = __DATE__;        // "Mmm dd yyyy"
  const char* TIME = __TIME__;        // "hh:mm:ss"

  int month = monthFromDateMacro(DATE);
  int day   = atoi(&DATE[4]);   // maneja día con espacio
  int year  = atoi(&DATE[7]);

  int hour   = (TIME[0]-'0')*10 + (TIME[1]-'0');
  int minute = (TIME[3]-'0')*10 + (TIME[4]-'0');
  int second = (TIME[6]-'0')*10 + (TIME[7]-'0');

  int dow = dayOfWeekLib(year, month, day);
  myRtc.setDS1302Time(second, minute, hour, dow, day, month, year);
}

void loop(){}
