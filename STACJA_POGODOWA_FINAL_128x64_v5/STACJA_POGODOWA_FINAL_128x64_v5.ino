#include <EEPROM.h>       // 0: LB, 1: LG, 2: RB, 3: RR, 4: step/s
#include <Wire.h>
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; // I2C

#include "U8glib.h"
#include <virtuabotixRTC.h>  
#include <dht.h>
dht DHT;
#define DHT11_PIN A3
#define lG 3
#define lB 5
#define rR 6
#define rB 9
#define BUZZ A0
///ENCODER
#define eSW 4
#define eL 7 /// PD7 DDRD / PIND // PORTD
#define eR 2 /// PD2

#define fH 8
#define nPI 3.141592653589

#define grafW 128
#define grafH 16
//EKRAN (SPI) + TIME DS1302 
//OBIEKTY
//U8GLIB_ST7920_128X64 u8g (13,11,10,U8G_PIN_NONE);
U8GLIB_ST7920_128X64_1X u8g (10, U8G_PIN_NONE);
virtuabotixRTC myRTC(A2, A1, 8);

void chkEprom();
//------------------------------------------------
String temp;
String hum;
String pres;

float tFloat;
int tMed, tMin, tMax;
int tSpectr;
unsigned int tStep = 3, tStepCnt=0; // (step w sekundach)

char tData[20];
unsigned long timers [3]={0,0,0};           // dht refresh    encoder deb   fx ref
uint16_t del [2] = {2000, 50};            //    dht           debounce        ref
byte rnd_seed =0;
byte lFX = 12;

volatile byte enState = 0;                // 0-nic   1-right   2-left
volatile byte enPind = 0x80;
const char *DOW [7] ={"PN","WT","SR","CZ","PT","SB","NE"};
unsigned long bTimer = 0;

int8_t menuItem = 0;
#define mIt 8
const char *menuIt [mIt] ={"< L BLU","< L GRN","R BLU >","R RED >","S/STEP","DI/AN","F-FX","Opc3"};

const char mAfix[mIt] = {'%','%','%','%','s','*','*','z'};
const byte mInc[mIt] = {5,5,5,5,1,1,1,1};
byte ledArr[4]={255,128,32,64};                             // 0    1    2    3    4    5    6    7 ...
byte ledArrF[mIt]={60,10,20,60,5};                          // B    G    B    R  step  op1  op2  op3
const byte itemMax[mIt] = {100,100,100,100,240,1,1,30};     // 100 100  100  100 240    1    x    x
byte ledArrPrev[mIt]={}; // B G B R
byte edit = 0;

//chwilowe
byte graf[grafW]={};

unsigned int enData = 0;
byte sSec = 0;

enum mStates {main=0, opcje=1, count=2, ring=3};
mStates cState = main;
enum mStyles {digital=0, analog=1};
mStyles mStyle = digital;
enum fxStates {fOff=0, fOn=1};
fxStates fxState = fOn;

void setup(void) {
  Serial.begin(9600);

   //u8g.setRot180();
   if (!bmp.begin()) {
  }
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  chkEprom();
  
  // assign default color value
  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
    u8g.setColorIndex(255);     // white
  }
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
    u8g.setColorIndex(3);         // max intensity
  }
  else if ( u8g.getMode() == U8G_MODE_BW ) {
    u8g.setColorIndex(1);         // pixel on
  }
  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255,255,255);
  }
  pinMode(lG,OUTPUT);
  pinMode(lB,OUTPUT);
  pinMode(rR,OUTPUT);
  pinMode(rB,OUTPUT);
  pinMode(BUZZ,OUTPUT);
  //ENCODER
  pinMode(eSW,INPUT);
  pinMode(eL,INPUT);
  pinMode(eR,INPUT);
  attachInterrupt(digitalPinToInterrupt(eR), encoder, LOW);

  tFloat = bmp.readTemperature()*100.f;
  tMed = static_cast<float>(tFloat);
  tMin = tMed-25;
  tMax = tMed+25;

}

void encoder()
{
  // PIND 10000100
  //PIND &= 10000100 ? enState=1 : enState=2;
  //if (PIND & 0x84 == 0x84) {enState=1;}
  //else {enState=2;}
  //if (timers[1]+del[1]<millis())
  cli();
  enPind ^= PIND;
  enPind = enPind >> 7;
  (enPind == 0x01) ? enState=2 : enState=1;

  //if (digitalRead(eL)==HIGH) enState=2;
  //else enState=1;
  
}

void chkEprom()
{
  for (byte i=0; i<mIt; i++)
  {
    ledArrF[i]=EEPROM.read(i); // wczytaj z EEPROM
    ledArrPrev[i]=ledArrF[i];  // wyrownaj
  }
  
}

void drawHands(float angle, byte leng)
{
      float aSecRad = ((angle-90)*nPI)/180.f;
      float aSecSin = sin(aSecRad);
      float aSecCos = cos(aSecRad);
      float aSecXF = round(aSecCos*leng+64);
      float aSecYF = round(aSecSin*leng+32);
      byte aSecX = static_cast<float>(aSecXF);
      byte aSecY = static_cast<float>(aSecYF);
      u8g.drawLine(64,32,aSecX,aSecY);
}
void drawIndi(byte r, byte ll, byte sl)
{

 u8g.drawLine(64,32-r,64,32-r+ll); //h
 u8g.drawLine(64,32+r,64,32+r-ll); //h
  
 u8g.drawLine(64-r,32,64-r+ll,32);
 u8g.drawLine(64+r,32,64+r-ll,32);  
 for (byte i=0; i<12; i++)
 {
  float aRad = (i*30*nPI)/180.f; // 12 indykatorow
  
  byte aCosX = static_cast<float>(round(cos(aRad)*r+64));
  byte aSinY = static_cast<float>(round(sin(aRad)*r+32));
    
  byte aCosXL = static_cast<float>(round(cos(aRad)*(r-sl)+64));
  byte aSinYL = static_cast<float>(round(sin(aRad)*(r-sl)+32));
  
  u8g.drawLine(aCosX,aSinY,aCosXL,aSinYL);
 }
}

void loop(void) {
  void draw();
/*
Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print("Approx altitude = ");
    Serial.print(bmp.readAltitude(1013.25));
    Serial.println(" m");
*/

/// AKTUALIZUJ LEDY w GLOWNEJ PETLI++++++++++++++++++
  for (byte i=0; i<4; i++)
  {
    ledArr[i]=static_cast<byte>(ledArrF[i]*2.55);
  }
  analogWrite(lB,ledArr[0]);
  analogWrite(lG,ledArr[1]);
  analogWrite(rB,ledArr[2]);
  analogWrite(rR,ledArr[3]);

  if (ledArrF[5]==0) mStyle=digital;
  if (ledArrF[5]==1) mStyle=analog;

  if (ledArrF[6]==0) fxState=fOff;
  if (ledArrF[6]==1) fxState=fOn;
/// AKTUALIZUJ LEDY w GLOWNEJ PETLI++++++++++++++++++

  myRTC.updateTime();

  ///////////////////////////DISPLAY MANAGMENT
    u8g.firstPage();  
  do {
    draw();
  } while( u8g.nextPage() );

}


void draw(void) {
void encoder();

/// MAIN /// ---------------------------------------- CZUJNIKI 1/1000ms (1s)
  if (timers[0]+del[0]<millis()){
int chk = DHT.read11(DHT11_PIN);
temp="T: "+String(bmp.readTemperature(), 2)+"*C";
hum="H: "+String(DHT.humidity, 0)+"%";
pres="P: "+String(bmp.readPressure()/100.f,4)+" hPa";


// WYPELNIAJ GRAF //
  tStepCnt++;
  if (tStepCnt>=ledArrF[4])
   {
   tFloat = bmp.readTemperature()*100.f;
   tMed = static_cast<float>(tFloat);
   if (tMed<tMin) tMin=tMed;
   if (tMed>tMax) tMax=tMed;
   tSpectr = tMax-tMin;
  
   for (byte i=1; i<grafW; i++)
   {
     graf[i-1]=graf[i];
   }
    graf[grafW-1]=map(tMed,tMin,tMax,0,grafH);
    tStepCnt=0;
   }
  timers[0]=millis();
  }
/// MAIN /// -------------------------------------------------- CZUJNIKI END

  if (cState == main){
  /// ENCODER /// ---------------------------------------- MAIN
  if (timers[1]+del[1]<millis() && enState > 0)
  {
    if (mStyle==digital)
    {
    if (enState==1) enData+=5 ;
    else{ 
      if (enData<5) enData=0;
      else enData-=5;
      }
    }
        if (mStyle==analog)
    {
    if (enState==1) lFX+=2 ;
    else{ 
      if (lFX<2) lFX=0;
      else lFX-=2;
      }
    }
    timers[1]=millis();
    enState=0;
    sei();
  }}
  
  if (cState == opcje && edit==0){
  /// ENCODER /// ---------------------------------------- OPCJE
  if ((timers[1]+del[1]<millis()) && enState > 0)
  {
    if (enState == 1 && menuItem<(mIt-1)) menuItem+=1;
    else if (menuItem>0) menuItem-=1;

    timers[1]=millis();
    enState=0;
    sei();
  }}

    if (cState == opcje && edit==1){
  /// ENCODER /// ---------------------------------------- OPCJE / EDYCJA
  if ((timers[1]+del[1]<millis()) && enState > 0)
  {
    if (enState == 1 && ledArrF[menuItem]<itemMax[menuItem]) ledArrF[menuItem]+=mInc[menuItem];
    else if (ledArrF[menuItem]>0) ledArrF[menuItem]-=mInc[menuItem];

    timers[1]=millis();
    enState=0;
    sei();
  }}

if (cState != opcje){ 
  /// BUTTON CHECK ///
      // count
  while (digitalRead(eSW)==LOW)
  {
    bTimer++;
    delay(10);
    if (bTimer<100) {analogWrite(lB,255); analogWrite(rR,0);}
    else {analogWrite(lB,0); analogWrite(rR,255);}
  }
      // check
          // -- start ring
  if (bTimer>15 && bTimer<100 && enData>0)
  {
    cState = count;
  }
          // -- opcje
  else if (bTimer>=100 && bTimer<200)
  {
    cState = opcje;
  }
  
  bTimer = 0;

  /// MAIN MENU
  /// RINGER ///
 
  if (cState == count)
  {
    if (sSec != myRTC.seconds)
       {
        sSec = myRTC.seconds;
        enData--;
       }
    if (enData <= 0) {cState = ring;}
  }
  /// START RING ///
  if (cState == ring)
  {
        tone(BUZZ,262,200);
    delay(200);
        tone(BUZZ,294,400);
    delay(400);
        tone(BUZZ,330,400);
    delay(400);
        tone(BUZZ,262,200);
    delay(400);
        tone(BUZZ,262,200);
    delay(200);
        tone(BUZZ,349,200);
    delay(400);
        tone(BUZZ,349,400);
    delay(400);
        tone(BUZZ,330,400);
    delay(500);
    
    cState = main;
  }
  if (mStyle==digital){
  /// /// /// /// /// /// /// /// 
  u8g.setFont(u8g_font_ncenB08);
  u8g.setPrintPos(96,60);
  u8g.print("("+String(enData)+")s");

  //''''''''''''''''''''''''''''' CZUJNIKI
  u8g.setPrintPos(4,48);
   u8g.print(temp);
  u8g.setPrintPos(70,48);
   u8g.print(hum);
  u8g.setPrintPos(4,60);
   u8g.print(pres);
  //''''''''''''''''''''''''''''' DATA  
  u8g.setPrintPos(98,22);
  u8g.print(DOW[myRTC.dayofweek-1]);
  u8g.setPrintPos(94,10);
  u8g.print(String(myRTC.dayofmonth)+"/"+String(myRTC.month));
    //''''''''''''''''''''''''''''' ADD-ONS
      u8g.setPrintPos(24,63);

  //''''''''''''''''''''''''''''' CZAS    
  u8g.setFont(u8g_font_freedoomr25n);
  u8g.setPrintPos(2,27);
  u8g.print(String(myRTC.hours, DEC)+":"+String(myRTC.minutes, DEC));
  //''''''''''''''''''''''''''''' SEKUNDY
  u8g.drawFrame(4,29,120,5);
  u8g.drawBox(4,30,(myRTC.seconds+1)*2,3);

  //u8g.setPrintPos(6,53);
  //u8g.print(String(myRTC.seconds));
  /*SYMBOLE
  if (enData>0){
  u8g.drawStr( 98, 30, "<A>");
  }*/

   
   // u8g.drawBitmap( 0, 0, 128, 240, screen);
  }
  //''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''' ANALOG
    if (mStyle==analog){
      const byte sHand=29, mHand=24, hHand=15;
      
      u8g.drawCircle(64,32,30);

      float aSecAng = (myRTC.seconds)*6;
      drawHands(aSecAng, sHand);
            aSecAng = (myRTC.minutes)*6;
      drawHands(aSecAng, mHand);
             aSecAng = (myRTC.hours)*30;
      drawHands(aSecAng, hHand);

      drawIndi(30, 6, 3);

  u8g.setFont(u8g_font_ncenB08);
  //''''''''''''''''''''''''''''' CZUJNIKI A
  static int Tint = round(bmp.readTemperature());
  static int Pint = round(bmp.readPressure()/100);
  static int Hint;
    if (timers[0]+del[0]<millis()){
      int chk = DHT.read11(DHT11_PIN);
      Hint = round(DHT.humidity);
      timers[0]=millis();
    }
  u8g.setPrintPos(3,8);
   u8g.print(String(Tint)+"*C");
  u8g.setPrintPos(3,64);
   u8g.print(String(Pint)+" hPa");
  u8g.setPrintPos(100,64);
   u8g.print(String(Hint)+"%");
  //''''''''''''''''''''''''''''' DATA A  
  u8g.setPrintPos(100,8);
  u8g.print(String(myRTC.dayofmonth)+"/"+String(myRTC.month));

    //''''''''''''''''''''''''''''' FX ANALOG
    if (fxState==fOn){
    int8_t rnd1 [7];
    int8_t rnd2 [7];
    rnd1[0]=0;
    rnd1[6]=0;
    rnd2[0]=0;
    rnd2[6]=0;

    if ((timers[2]+rnd_seed)<millis())
     {
      for (byte i=1; i<6; i++)
     {
       rnd1[i]=(random(lFX*2))-lFX;
       rnd2[i]=(random(lFX*2))-lFX;
     }
     timers[2]=millis();
     rnd_seed = random(1,15);
      }
    for (byte i=0; i<6; i++)
    {
      u8g.drawLine(2+(i*5),32+rnd1[i],2+((i+1)*5),32+rnd1[i+1]);
    }

    for (byte i=0; i<6; i++)
    {
      u8g.drawLine(94+(i*5),32+rnd2[i],94+((i+1)*5),32+rnd2[i+1]);
    }
   } // fx state end 
    } // KONIEC ANALOG
}
  /// OPCJE -------------------------- ///
if (cState == opcje){
  /// RYSUJ DLA OPCJI
  // ------------------------------RYSUJ AKTUALNY FRAME
  if (edit==0) u8g.drawFrame(2,fH*2+2,52,fH+2);
  else  u8g.drawFrame(52,fH*2+2,32,fH+2);
  /*
  byte pos = (fH)*menuItem;
  if (edit==0) u8g.drawFrame(0,pos+menuItem,54,fH+2);
  else  u8g.drawFrame(54,pos+menuItem,32,fH+2);
  */
  u8g.setFont(u8g_font_ncenB08);
  /*
  // ------------------------------RYSUJ BARY
    for (byte i=0; i<mIt; i++)
    {
      u8g.drawFrame(88,(fH*i)+i,40,fH+2);
      u8g.drawBox(88,(fH*i)+i,map(ledArrF[i],0,100,0,40),fH+2);
    }
  */
  //------------------------------RYSUJ ITEMY
  byte mOfset = 5;
  if (menuItem-1>=0)
  {
  u8g.setPrintPos(mOfset,(fH+1)*1+9);
  u8g.print(menuIt[menuItem-1]);
  u8g.setPrintPos(56,(fH+1)*1+9);
  u8g.print(String(ledArrF[menuItem-1])+mAfix[menuItem-1]); 

        if (menuItem-2>=0)
         {
           u8g.setPrintPos(mOfset,9);
           u8g.print(menuIt[menuItem-2]);
           u8g.setPrintPos(56,9);
           u8g.print(String(ledArrF[menuItem-2])+mAfix[menuItem-2]); 
         }
  }
  
  u8g.setPrintPos(mOfset,(fH+1)*2+9);
  u8g.print(menuIt[menuItem]);
  u8g.setPrintPos(56,(fH+1)*2+9);
  u8g.print(String(ledArrF[menuItem])+mAfix[menuItem]);

    if (menuItem+1<=mIt-1)
  {
  u8g.setPrintPos(mOfset,(fH+1)*3+9);
  u8g.print(menuIt[menuItem+1]);
  u8g.setPrintPos(56,(fH+1)*3+9);
  u8g.print(String(ledArrF[menuItem+1])+mAfix[menuItem+1]); 

      if (menuItem+2<=mIt-1)
       {
         u8g.setPrintPos(mOfset,(fH+1)*4+9);
         u8g.print(menuIt[menuItem+2]);
         u8g.setPrintPos(56,(fH+1)*4+9);
         u8g.print(String(ledArrF[menuItem+2])+mAfix[menuItem+2]); 
       }
  }
  /*
  for (byte i=0; i<5; i++)
  {
    u8g.setPrintPos(3,(fH+1)*i+9);
      u8g.print(menuIt[i]);
    u8g.setPrintPos(56,(fH+1)*i+9);
    u8g.print(String(ledArrF[i])+mAfix[i]);
  }
  */
    // ------------------------------RYSUJ INDYKATOR
  u8g.drawLine(2,0,2,64-grafH);
    byte ind = static_cast<byte>(((64-grafH)/mIt)*menuItem);
  u8g.drawBox(0,ind,2,6);
    
  // ------------------------------RYSUJ GRAF
  u8g.drawFrame(0,64-grafH,128,grafH);
  /* LINIE GRAFU
  for (byte l=4; l<128; l+=4)
  {
    u8g.drawLine(l,64-grafH,l,64);
  }
  */
  for (byte g=0; g<grafW; g++)
  {
    //if (graf[g]>0){u8g.drawPixel(g,64-graf[g]);}
    if (graf[g]>0)
    {
      u8g.drawLine(g,64-graf[g],g,64);
    }
  }   

  
  // ------------------------------PROCEDURA WYJSCIA
  while (digitalRead(eSW)==LOW)
  {
    bTimer++;
    delay(10);
    if (bTimer<100) analogWrite(lB,255);
    else analogWrite (lG,128);
    //Serial.println("bTimer: "+String(bTimer));
  }
    if (bTimer>10 && bTimer<100)
  {
    if (edit==0) edit=1;
    else edit=0;
    bTimer=0;
  }
  else if (bTimer>=100 && bTimer<200)
  {
    cState = main;
    bTimer=0;
    /// aktualizuj EEPROM jesli potrzeba
    for (byte i=0; i<mIt; i++)
    {
      if (ledArrF[i] != ledArrPrev[i]) 
      {
        EEPROM.write(i,ledArrF[i]);
        ledArrPrev[i]=ledArrF[i];
      }
    }
  }
  else bTimer=0;
 }  
}

/*  
void cDOWa()
{
  strcpy(cDOW, DOW[myRTC.dayofweek-1]);

  if (myRTC.dayofweek == 1) {strcpy(cDOW, "PN");}
  else if (myRTC.dayofweek == 2) {strcpy(cDOW, "WT");}
  else if (myRTC.dayofweek == 3) {strcpy(cDOW, "SR");}
  else if (myRTC.dayofweek == 4) {strcpy(cDOW, "CZ");}
  else if (myRTC.dayofweek == 5) {strcpy(cDOW, "PT");}
  else if (myRTC.dayofweek == 6) {strcpy(cDOW, "SB");}
  else if (myRTC.dayofweek == 7) {strcpy(cDOW, "NE");}
  
}*/
