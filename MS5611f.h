#ifndef MS5611F_H_
#define MS5611F_H_

#include "Arduino.h"

#define MS5611_TEST

#define MS5611_SAMPLE_PERIOD_MS         12

#define MS5611_READ_TEMPERATURE 		11
#define MS5611_READ_PRESSURE			22


class MS5611 {

	public : 

	MS5611();
	void TriggerPressureSample(void);
	void TriggerTemperatureSample(void);
	U32  ReadPressureSample(void);
	void AveragedSample(int nSamples);	
	U32  ReadTemperatureSample(void);
	void  CalculateTemperatureCx10(void);
	float  CalculatePressurePa(void);
	void CalculateSensorNoisePa(void);
	void Configure(void);
	void SampleStateMachine(void);
	void InitializeSampleStateMachine(void);
	void ReadCoefficients(void);
	float  Pa2Cm(float pa);
    void Test(int nSamples);	

	float zCmAvg_;
	float zCmSample_;
	float paSample_;
	S32 celsiusSample_;
	int zGood;

	private :


	U16 cal_[6];
	int sensorState_;
    S64 tref_;
    S64 offT1_;
    S64 sensT1_;	
    S32 tempCx100_;
    U32 D1_;
	U32 D2_;
	S64 dT_;	

	void Write8(U08 out);
	U16 Read16(void);
	U32 Read24(void);
	
	};

#endif // MS5611F_H_
