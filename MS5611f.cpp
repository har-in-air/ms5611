#include "Arduino.h"
#include "spi.h"
#include "MS5611f.h"



MS5611::MS5611() {
	paSample_ = 0.0f;
	zCmSample_ = 0.0f;
	celsiusSample_ = 0;
	zCmAvg_ = 0.0f;
	zGood = 0;
	}

void MS5611::Configure(void) {
	CSB(1);

	ReadCoefficients();
    tref_ = S64(cal_[4])<<8;
    offT1_ = S64(cal_[1])<<16;
    sensT1_ = S64(cal_[0])<<15;
	}

void MS5611::InitializeSampleStateMachine(void) {
   TriggerTemperatureSample();
   sensorState_ = MS5611_READ_TEMPERATURE;
   zGood = 0;
   }


void MS5611::SampleStateMachine(void) {
   if (sensorState_ == MS5611_READ_TEMPERATURE) {
      D2_ = ReadTemperatureSample();
      TriggerPressureSample();
      //DBG(1); // turn on the debug pulse for timing the critical computation
      CalculateTemperatureCx10();
      celsiusSample_ = (tempCx100_ >= 0? (tempCx100_+50)/100 : (tempCx100_-50)/100);
      paSample_ = CalculatePressurePa();
	  zCmSample_ = Pa2Cm(paSample_);
	  zGood = 1;
      //DBG(0);
	  sensorState_ = MS5611_READ_PRESSURE;
      }
	else
	if (sensorState_ == MS5611_READ_PRESSURE) {
		D1_ = ReadPressureSample();   
		TriggerTemperatureSample();
		sensorState_ = MS5611_READ_TEMPERATURE;
	   }
   }


#ifdef MS5611_TEST

#define MAX_TEST_SAMPLES    100
extern char gszBuf[];
static float pa[MAX_TEST_SAMPLES];
static float z[MAX_TEST_SAMPLES];

void MS5611::Test(int nSamples) {
	S32 n;
    float paMean, zMean, zVariance, paVariance;
    paMean = 0.0f;
    zMean = 0.0f;
    paVariance = 0.0f;
    zVariance = 0.0f;
    for (n = 0; n < nSamples; n++) {
	    TriggerTemperatureSample();
	    delay(MS5611_SAMPLE_PERIOD_MS);
	    D2_ = ReadTemperatureSample();
	    CalculateTemperatureCx10();
		TriggerPressureSample();
		delay(MS5611_SAMPLE_PERIOD_MS);
		D1_ = ReadPressureSample();
		pa[n] = CalculatePressurePa();
        z[n] =  Pa2Cm(pa[n]);
        paMean += pa[n];
        zMean += z[n];
        }
    paMean /= nSamples;
    zMean /= nSamples;
    for (n = 0; n < nSamples; n++) {
        paVariance += (pa[n]-paMean)*(pa[n]-paMean);
        zVariance += (z[n]-zMean)*(z[n]-zMean);
        sprintf(gszBuf,"%f %f\r\n",pa[n],z[n]);
        Serial.print(gszBuf);
       }
    paVariance /= (nSamples-1);
    zVariance /= (nSamples-1);
    sprintf(gszBuf,"\r\npaVariance %f  zVariance %f\r\n",paVariance,zVariance);
    Serial.print(gszBuf);
    
	}
#endif

void MS5611::AveragedSample(int nSamples) {
	S32 tc,tAccum,n;
    float pa,pAccum;
	pAccum = 0.0f;
    tAccum = 0;
	n = nSamples;
    while (n--) {
		TriggerTemperatureSample();
		delay(MS5611_SAMPLE_PERIOD_MS);
		D2_ = ReadTemperatureSample();
		CalculateTemperatureCx10();
		TriggerPressureSample();
		delay(MS5611_SAMPLE_PERIOD_MS);
		D1_ = ReadPressureSample();
		pa = CalculatePressurePa();
		pAccum += pa;
		tAccum += tempCx100_;
		}
	tc = tAccum/nSamples;
	celsiusSample_ = (tc >= 0 ?  (tc+50)/100 : (tc-50)/100);
	paSample_ = (pAccum+nSamples/2)/nSamples;
	zCmAvg_ = zCmSample_ = Pa2Cm(paSample_);
	}
	
	

/// Fast Lookup+Interpolation method for converting pressure readings to altitude readings.
#include "pztbl.txt"

float MS5611::Pa2Cm(float paf)  {
   	S32 pa,inx,pa1,z1,z2;
    float zf;
    pa = S32(paf);

   	if (pa > PA_INIT) {
      	zf = float(gPZTbl[0]);
      	}
   	else {
      	inx = (PA_INIT - pa)>>10;
      	if (inx >= PZLUT_ENTRIES-1) {
         	zf = float(gPZTbl[PZLUT_ENTRIES-1]);
         	}
      	else {
         	pa1 = PA_INIT - (inx<<10);
         	z1 = gPZTbl[inx];
         	z2 = gPZTbl[inx+1];
         	zf = float(z1) + ((float(pa1)-paf)*float(z2-z1))/1024.0f;
         	}
      	}
   	return zf;
   	}


void MS5611::ReadCoefficients(void) {
    int cnt;
    for (cnt = 0; cnt < 6; cnt++) {
        SCK(0);
    	CSB(0);
		Write8(0xA2 + cnt*2);// skip the factory data in addr A0, and the checksum at last addr
		cal_[cnt] = Read16();
		CSB(1);
		}
	}


/// Trigger a pressure sample with max oversampling rate
void MS5611::TriggerPressureSample(void) {
    SCK(0);
    CSB(0);
	Write8(0x48);
	CSB(1);
   }

/// Trigger a temperature sample with max oversampling rate
void MS5611::TriggerTemperatureSample(void) {
    SCK(0);
    CSB(0);
    Write8(0x58);
    CSB(1);
   }

U32 MS5611::ReadTemperatureSample(void)	{
   U32 w;
   SCK(0);
   CSB(0);
   Write8(0x00);
   w = Read24();
   CSB(1);
   return w;
   }


U32 MS5611::ReadPressureSample(void)	{
   U32 w;
   SCK(0);
   CSB(0);
   Write8(0x00);
   w = Read24();
   CSB(1);
   return w;
   }

void MS5611::CalculateTemperatureCx10(void) {
	dT_ = (S64)D2_ - tref_;
	tempCx100_ = 2000 + ((dT_*((S32)cal_[5]))>>23);
	}


float MS5611::CalculatePressurePa(void) {
	float pa;
    S64 offset, sens,offset2,sens2,t2;
	offset = offT1_ + ((((S64)cal_[3])*(S64)dT_)>>7);
	sens = sensT1_ + ((((S64)cal_[2])*(S64)dT_)>>8);
    if (tempCx100_ < 2000) {
        t2 = ((dT_*dT_)>>31); 
        offset2 = (5*(tempCx100_-2000)*(tempCx100_-2000))/2;
        sens2 = offset2/2;
        } 
    else {
        t2 = 0;
        sens2 = 0;
        offset2 = 0;
        }
    tempCx100_ -= t2;
    offset -= offset2;
    sens -= sens2;
	pa = (float((S64)D1_ * sens)/2097152.0f - float(offset)) / 32768.0f;
	return pa;
	}


void MS5611::Write8(U08 out) {
	int i;
	for (i = 0; i < 8; i++)	{
		MOSI(out & 0x80 ? 1 : 0);
		SCK_DELAY();
		SCK(1);
		SCK_DELAY();
		out = (out << 1);
		SCK(0);
	    }
    }


U16 MS5611::Read16(void){
	int i,temp;
	U16 in;
	in = 0;
	for (i = 0; i < 16; i++)	{
		in = (in << 1);
		temp = MISO();
		SCK(1);
		SCK_DELAY();
	    if (temp == 1)	in = in | 0x0001;
		SCK(0);
		SCK_DELAY();
	    }
	return in;
    }

U32 MS5611::Read24(void){
	int i,temp;
	U32 in;
	in = 0;
	for (i = 0; i < 24; i++)	{
		in = (in << 1);
		temp = MISO();
		SCK(1);
		SCK_DELAY();
	    if (temp == 1)	in = in | 0x00000001;
		SCK(0);
		SCK_DELAY();
	    }
	return in;
    }





