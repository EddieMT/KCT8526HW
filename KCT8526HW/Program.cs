//===================================================================================
//
// Program: KCT8526HW (v1.0)
// Date:    07/09/2018
//
// v1.0     Initial test program release in C# for KCT8526HW device on APS100 system
//
// Copyright (c) 2018 - Merlin Test Technologies, Inc.
//
//===================================================================================

using System;
using System.IO;
using System.Linq;
using System.Threading;
using System.Diagnostics;
using System.Collections.Generic;

using MT.APS100.Model;
using MT.TesterDriver;
using MT.TesterDriver.VNA;
using MT.Tools.HwResourceManager.Manager;
using MT.TesterDriver.Enums;
using NationalInstruments;
using MerlinTest.Tools.Diagnostic.Logging;

namespace KCT8526HW
{
    public enum MIPI { RX_BYPASS, RX_TX, TX_RX, TX_OFF, reset }

    public enum GPIO { WLAN_GAIN_TX, WLAN_GAIN_RX, WLAN_GAIN_RX_BYPASS, Power_off };
    public enum SETUP { DC_IDLE, DC_IDLE_OFF, DC_SETUP, DC_SETUP_OFF, DC_MEAS, DC_MEAS_OFF, HARM_2ND, HARM_3RD, HARM_2ND_3RD, HARM_OFF }
    public enum SWITCH_MODE { RX_BYPASS, RX_TX, TX_RX, TX_OFF }


    public class KCT8526HW : ApplicationBaseMrk2
    {
        public KCT8526HW()
        {
            // Initialize the network trace object.
            this.trace = new NetworkTraceSource("Application Code", "KCT8526HW");
            // Define the level of logging to display
            SourceSwitch myTraceSourceSwitch = new SourceSwitch("Data Logging Prototype");
            myTraceSourceSwitch.Level = SourceLevels.Information;
            trace.Switch = myTraceSourceSwitch;
        }

        private double[] RX_5150 = new double[2];
        private double[] RX_5500 = new double[2];
        private double[] RX_5500_Pout = new double[2];
        private double[] RX_5850 = new double[2];
        private double[] RX_5925 = new double[2];
        private double[] RX_Flatness = new double[2];
        private double[] RX_2410 = new double[2];

        private double[] RX_BYPASS_5150 = new double[2];
        private double[] RX_BYPASS_5500 = new double[2];
        private double[] RX_BYPASS_5500_Pout = new double[2];
        private double[] RX_BYPASS_5850 = new double[2];
        private double[] RX_BYPASS_5925 = new double[2];

        private double[] TX_5150 = new double[2];
        private double[] TX_5500 = new double[2];
        private double[] TX_5500_Pout = new double[2];
        private double[] TX_5850 = new double[2];
        private double[] TX_5925 = new double[2];
        private double[] TX_3800 = new double[2];
        private double[] TX_2600 = new double[2];
        private double[] TX_Flatness = new double[2];

        private double[] VDET_NORF = new double[2];
        private double[] VDET_10 = new double[2];
        private double[] VDET_18 = new double[2];
        private double[] VDET_22 = new double[2];
        private double[] VDET_25 = new double[2];
        private double[] VDET_SLOPE = new double[2];
        //private double[] VDET_INTERCEPT = new double[2];


        private double[] VDET_IDEALPOUT_5 = new double[2];
        private double[] VDET_IDEALPOUT_7 = new double[2];

        private double[] MEASURE_IDEAL_POUT_5 = new double[2];
        private double[] MEASURE_IDEAL_POUT_7 = new double[2];





        private float[] iData = { 0 };
        private float[] qData = { 0 };
        private float[] iData_OFF = { 0 };
        private float[] qData_OFF = { 0 };




        private Dictionary<int, List<double>> evm_cached = new Dictionary<int, List<double>>();
        private Dictionary<int, List<double>> evmfloor_cached = new Dictionary<int, List<double>>();

        //============================================ Utilities =============================================

        private void ProgramExit()
        {
            Console.Write("\nPress any key to quit. ");

            try
            {
                Console.ReadKey();
            }
            catch
            {
                throw new Exception("ApplicationLoad failed!");
            }

            Environment.Exit(0);
        }

        public int LoSharing(bool share)
        {
            if (!isOSPass) return 1;

            if (share == true)
            {

                SigGen1Hal.LOOutEnabled = true;
                SigGen1Hal.ExternalLO = false;



                DigitizerHal.LOOutEnabled = false;
                DigitizerHal.ExternalLO = true;

            }
            else
            {

                SigGen1Hal.LOOutEnabled = false;
                SigGen1Hal.ExternalLO = false;


                DigitizerHal.LOOutEnabled = false;
                DigitizerHal.ExternalLO = false;
            }

            return 1;
        }

        public int DeviceInit(GPIO mode)
        {
            if (!isOSPass) return 1;

            if (mode == GPIO.WLAN_GAIN_TX)
            {
                //@1

                DC.setVoltage(1, 3.3);//PA_EN
                Digital.cpu_df(1, 2, 1, 1);//C1
                Digital.cpu_df(1, 3, 1, 0);//C0
                //@2
                DC.setVoltage(2, 3.3);//PA_EN
                Digital.cpu_df(1, 7, 1, 1);//C1
                Digital.cpu_df(1, 8, 1, 0);//C0
            }
            else if (mode == GPIO.WLAN_GAIN_RX)
            {
                //@1
                DC.setVoltage(1, 0.0);//PA_EN
                Digital.cpu_df(1, 2, 1, 0);//C1
                Digital.cpu_df(1, 3, 1, 1);//C0
                //@2
                DC.setVoltage(2, 0.0);//PA_EN
                Digital.cpu_df(1, 7, 1, 0);//C1
                Digital.cpu_df(1, 8, 1, 1);//C0
            }
            else if (mode == GPIO.WLAN_GAIN_RX_BYPASS)
            {
                //@1
                DC.setVoltage(1, 0.0);//PA_EN
                Digital.cpu_df(1, 2, 1, 1);//C1
                Digital.cpu_df(1, 3, 1, 0);//C0
                //@2
                DC.setVoltage(2, 0.0);//PA_EN
                Digital.cpu_df(1, 7, 1, 1);//C1
                Digital.cpu_df(1, 8, 1, 0);//C0
            }
            else if (mode == GPIO.Power_off)
            {
                //@1
                DC.setVoltage(1, 0.0);//PA_EN
                Digital.cpu_df(1, 2, 1, 0);//C1
                Digital.cpu_df(1, 3, 1, 0);//C0
                //@2
                DC.setVoltage(2, 0.0);//PA_EN
                Digital.cpu_df(1, 7, 1, 0);//C1
                Digital.cpu_df(1, 8, 1, 0);//C0
            }
            return 1;
        }

        public int VsgVsaSetup(double sampleRate, double captureTime, double measLength, double measStepLength, double measOffset, TriggerTypeEnum triggerType, VsgMode arbMode, string modWaveform)
        {
            if (!isOSPass) return 1;

            uint numOfSamples = (uint)(sampleRate * captureTime);
            uint preTrigSamples = (uint)(sampleRate * measOffset);

            currentWaveform = Path.GetFileName(modWaveform);
            switch (arbMode)
            {
                case VsgMode.ARB:
                    SigGen1Hal.Waveform = modWaveform;
                    SigGen1Hal.Mode = arbMode;
                    SigGen1Hal.StopWaveform();
                    SigGen1Hal.PlayWaveform();
                    break;
                case VsgMode.CW:
                    SigGen1Hal.StopWaveform();
                    SigGen1Hal.Mode = VsgMode.CW;
                    break;
            }

            DigitizerHal.SamplingFreq = sampleRate;
            DigitizerHal.DwellTime = captureTime;
            DigitizerHal.NumberOfPowerMeasurements = 1;
            DigitizerHal.StepLength = measStepLength;
            DigitizerHal.MeasurementOffset = measOffset;
            DigitizerHal.MeasurementLength = captureTime;
            DigitizerHal.TriggerType = triggerType;




            return 1;
        }

        public int VsgVsaSetup_play(double sampleRate, double captureTime, double measLength, double measStepLength, double measOffset, TriggerTypeEnum triggerType, VsgMode arbMode, string modWaveform)
        {

            uint numOfSamples = (uint)(sampleRate * captureTime);
            uint preTrigSamples = (uint)(sampleRate * measOffset);

            currentWaveform = Path.GetFileName(modWaveform);
            switch (arbMode)
            {
                case VsgMode.ARB:
                    SigGen1Hal.Waveform = modWaveform;
                    SigGen1Hal.Mode = arbMode;
                    SigGen1Hal.StopWaveform();
                    SigGen1Hal.PlayWaveform();
                    break;
                case VsgMode.CW:
                    SigGen1Hal.StopWaveform();
                    SigGen1Hal.Mode = VsgMode.CW;
                    break;
            }


            return 1;
        }


        private int MeasureIdle(int testnumber, int siteNum)
        {
            double[] i_result;

            i_result = DC.HPMeas.getCurrentAverage();
            VDD_ICC[0] = i_result[0];  //VDD@1
            VCC2_ICC[0] = i_result[1];  //VCC2@1
            //VDD_ICC[1] = i_result[2];  //VDD@2
            //VCC1_ICC[1] = i_result[3];  //VCC2@2


            i_result = DC.LPMeas.getCurrentAverage();
            VCC1_ICC[0] = i_result[2];  //VCC1@1
            //VCC2_ICC[1] = i_result[3];  //VCC1@2


            Total_ICC[0] = VCC1_ICC[0] + VCC2_ICC[0] + VDD_ICC[0];//Total_ICC@1
            Total_ICC[1] = VCC1_ICC[1] + VCC2_ICC[1] + VDD_ICC[1];//Total_ICC@2


            //------------------Measure VDET------------------------------------

            Digital.con_pmu(1, 0, 0);

            if (siteNum == 0 && ActiveSites[siteNum])
            {
                Digital.cpu_df(1, MVPs["VDET"][siteNum].digPin, 0, 0);
                Digital.con_pmu(1, MVPs["VDET"][siteNum].digPin, 1);
                Digital.pmufi(1, 1, 0, 1.0, 0.0);//board number, chip number, double ri, cvh,cvl ;   chip represents pmu                    
                Util.WaitTime(2.0e-3);
                VDET_NORF[0] = Digital.vmeas(1, MVPs["VDET"][siteNum].digPin);
                //Util.WaitTime(2e-3);
                Digital.con_pmu(1, MVPs["VDET"][siteNum].digPin, 0);

            }

            if (siteNum == 1 && ActiveSites[siteNum])
            {
                Digital.cpu_df(1, MVPs["VDET"][siteNum].digPin, 0, 0);
                Digital.con_pmu(1, MVPs["VDET"][siteNum].digPin, 1);
                Digital.pmufi(1, 1, 0, 1.0, 0.0);//board number, chip number, double ri, cvh,cvl ;   chip represents pmu                    
                Util.WaitTime(2.0e-3);
                VDET_NORF[0] = Digital.vmeas(1, MVPs["VDET"][siteNum].digPin);
                //Util.WaitTime(2e-3);
                Digital.con_pmu(1, MVPs["VDET"][siteNum].digPin, 0);
            }
            return 1;
        }

        private int DcSetup(int testnumber)
        {

            return 1;
        }

        private int MeasureDC(int testnumber, int siteNum)
        {
            double[] i_result;

            i_result = DC.Meas.getCurrentAverage();
            VCC1_ICC[0] = i_result[2];  //VCC1@1
            VDD_ICC[0] = i_result[4];  //VDD@1
            VCC2_ICC[0] = i_result[5];  //VCC2@1
            //VCC2_ICC[1] = i_result[3];  //VCC2@2
            //VDD_ICC[1] = i_result[6];  //VDD@2
            //VCC1_ICC[1] = i_result[7];  //VCC1@2
            //i_result = DC.LPMeas.getCurrentAverage();



            Total_ICC[0] = VCC1_ICC[0] + VCC2_ICC[0] + VDD_ICC[0];//Total_ICC@1
            Total_ICC[1] = VCC1_ICC[1] + VCC2_ICC[1] + VDD_ICC[1];//Total_ICC@2
            return 1;

        }

        private int StoreResult(int testNumber, int siteNum)
        {

            return 1;
        }

        private int Servo(SERVO servoType, double startLevel, double poutTarget, double targetTolerance, double expectedGain, double limitP1dB, double limitP3dB,
                          double settlingTime, int maxLoopIterations, double centerFreq, ref double Pin, ref double Pout, ref double Pout1dB, ref double Pout3dB, ref double gainDelta)
        {
            double[] measPower = new double[0];
            double error = 100;
            double newPin = startLevel;
            double gain = 0;
            double gainInit = 0;
            double gainTargetP1dB = 100;
            double gainTargetP3dB = 100;
            //double gainDelta = 100;

            int loopTries = 0;
            bool loopDone = false;
            bool wasSuccessful = false;

            Pin = startLevel;

            SigGen1Hal.Level = (startLevel - srcCalFactor);
            Util.WaitTime(settlingTime);
            measPower = DigitizerHal.CapturePower();
            Pout = measPower[0] - measCalFactor;

            switch (servoType)
            {
                case SERVO.SERVO_P1DB:
                case SERVO.SERVO_P1DB_P3DB:
                    poutTarget = limitP1dB;
                    DigitizerHal.InputLevel = measPower[0] + 3;
                    Util.WaitTime(settlingTime);
                    measPower = DigitizerHal.CapturePower();
                    Pout = measPower[0] - measCalFactor;
                    DigitizerHal.InputLevel = limitP1dB + measCalFactor + 6;
                    Util.WaitTime(settlingTime);
                    break;
                case SERVO.SERVO_P3DB:
                    poutTarget = limitP3dB;
                    poutTarget = limitP1dB;
                    DigitizerHal.InputLevel = measPower[0] + 3;
                    Util.WaitTime(settlingTime);
                    measPower = DigitizerHal.CapturePower();
                    Pout = measPower[0] - measCalFactor;
                    DigitizerHal.InputLevel = limitP1dB + measCalFactor + 6;
                    Util.WaitTime(settlingTime);
                    break;
            }

            gainInit = Pout - startLevel;
            gainTargetP1dB = gainInit - 1;
            gainTargetP3dB = gainInit - 3;
            gain = gainInit;

            if (servoType == SERVO.SERVO_NONE)
            {
                loopTries = 1;
                loopDone = true;
                wasSuccessful = true;
            }
            else
            {
                do
                {
                    error = poutTarget - Pout;
                    if (Math.Abs(error) <= targetTolerance)
                    {
                        loopDone = true;
                        wasSuccessful = true;
                    }
                    else
                    {
                        if (gain <= (expectedGain - 15))
                        {
                            loopDone = true;
                            wasSuccessful = false;
                        }
                        else
                        {
                            newPin += error;

                            if (newPin > 8)
                            {
                                loopDone = true;
                                wasSuccessful = false;
                                Console.WriteLine("WARNING: Pin {0:F2} dBm exceeds maximum safe level of 8 dBm on servo loop iteration {1}.", newPin - srcCalFactor, loopTries + 1);
                            }
                        }
                    }
                    if (loopDone == false)
                    {
                        SigGen1Hal.Level = (newPin - srcCalFactor);
                        Util.WaitTime(settlingTime);

                        Pin = newPin;
                        measPower = DigitizerHal.CapturePower();
                        Pout = measPower[0] - measCalFactor;
                        gain = Pout - Pin;
                    }

                    loopTries++;

                } while (loopDone == false && loopTries < maxLoopIterations);
            }

            if (servoType == SERVO.SERVO_P1DB || servoType == SERVO.SERVO_P1DB_P3DB)
            {
                trace.TraceInformation("Doing a P1DB loop");
                loopTries = 0;
                loopDone = false;
                wasSuccessful = false;
                do
                {
                    gainDelta = gainInit - gain;
                    error = 1 - gainDelta;

                    trace.TraceInformation("Gain Init = {0}\tGain = {1} Gain Delta = {2} Loop tries {3}", gainInit, gain, gainDelta, loopTries);

                    if (Math.Abs(error) <= targetTolerance)
                    {
                        loopDone = true;
                        wasSuccessful = true;
                        Pout1dB = Pout;
                    }
                    else
                    {
                        newPin += error;

                        if (newPin > 8)
                        {
                            loopDone = true;
                            wasSuccessful = false;
                            Console.WriteLine("WARNING: Pin {0:F2} dBm exceeds maximum safe level of 8 dBm on servo loop iteration {1}.", newPin - srcCalFactor, loopTries + 1);
                        }
                    }
                    if (loopDone == false)
                    {
                        SigGen1Hal.Level = (newPin - srcCalFactor);
                        Util.WaitTime(settlingTime);

                        Pin = newPin;
                        measPower = DigitizerHal.CapturePower();
                        Pout = measPower[0] - measCalFactor;
                        gain = Pout - Pin;

                    }

                    loopTries++;

                } while (loopDone == false && loopTries < maxLoopIterations);
            }

            if (servoType == SERVO.SERVO_P3DB || servoType == SERVO.SERVO_P1DB_P3DB)
            {
                loopTries = 0;
                loopDone = false;
                wasSuccessful = false;
                do
                {
                    gainDelta = gainInit - gain;
                    error = 3 - gainDelta;
                    if (Math.Abs(error) <= targetTolerance)
                    {
                        loopDone = true;
                        wasSuccessful = true;
                        Pout3dB = Pout;
                    }
                    else
                    {
                        newPin += error;

                        if (newPin > 8)
                        {
                            loopDone = true;
                            wasSuccessful = false;
                            Console.WriteLine("WARNING: Pin {0:F2} dBm exceeds maximum safe level of 8 dBm on servo loop iteration {1}.", newPin - srcCalFactor, loopTries + 1);
                        }
                    }
                    if (loopDone == false)
                    {
                        SigGen1Hal.Level = (newPin - srcCalFactor);
                        Util.WaitTime(settlingTime);

                        Pin = newPin;
                        measPower = DigitizerHal.CapturePower();
                        Pout = measPower[0] - measCalFactor;
                        gain = Pout - Pin;
                    }

                    loopTries++;

                } while (loopDone == false && loopTries < maxLoopIterations);
            }

            if (wasSuccessful) return loopTries;
            else return -1;
        }

        private int ServoVDET(SERVO servoType, double startLevel, double poutTarget, double targetTolerance, double expectedGain, double limitP1dB, double limitP3dB,
                        double settlingTime, int maxLoopIterations, double centerFreq, ref double Pin, ref double Pout, ref double Pout1dB, ref double Pout3dB, double VDET_TARGET, ref double VDET, double VDET_SLOPE)
        {
            double[] measPower = new double[0];
            double error1 = 100;
            double error2 = 100;
            double newPin = startLevel;
            double gain = 0;
            double gainInit = 0;
            double gainTargetP1dB = 100;
            double gainTargetP3dB = 100;
            double gainDelta = 100;

            int loopTries = 0;
            bool loopDone = false;
            bool wasSuccessful = false;

            Pin = startLevel;

            SigGen1Hal.Level = (startLevel - srcCalFactor);
            Util.WaitTime(settlingTime);
            measPower = DigitizerHal.CapturePower();
            Pout = measPower[0] - measCalFactor;

            //---------------------------Measure VDET---------------------------------------


            Digital.cpu_df(1, 1, 0, 0);
            Digital.pmufi(1, 1, 0, 1.5, 0.0);//board number, chip number, double ri, cvh,cvl ;
            Digital.con_pmu(1, 1, 1);
            Util.WaitTime(2.0e-3);
            VDET = Digital.vmeas(1, 1);
            //Util.WaitTime(2e-3);
            Digital.con_pmu(1, 1, 0);

            //Digital.cpu_df(1, 6, 0, 0);
            //Digital.pmufi(1, 2, 0, 1.0, 0.0);//board number, chip number, double ri, cvh,cvl ;
            //Digital.con_pmu(1, 6, 1);
            //Util.WaitTime(2e-3);
            //VDET_2 = Digital.vmeas(1, 6);
            //// Util.WaitTime(2e-3);
            //Digital.con_pmu(1, 6, 0);


            switch (servoType)
            {
                case SERVO.SERVO_P1DB:
                case SERVO.SERVO_P1DB_P3DB:
                    poutTarget = limitP1dB;
                    DigitizerHal.InputLevel = measPower[0] + 3;
                    Util.WaitTime(settlingTime);
                    measPower = DigitizerHal.CapturePower();
                    Pout = measPower[0] - measCalFactor;
                    DigitizerHal.InputLevel = limitP1dB + measCalFactor + 6;
                    Util.WaitTime(settlingTime);
                    break;
                case SERVO.SERVO_P3DB:
                    poutTarget = limitP3dB;
                    poutTarget = limitP1dB;
                    DigitizerHal.InputLevel = measPower[0] + 3;
                    Util.WaitTime(settlingTime);
                    measPower = DigitizerHal.CapturePower();
                    Pout = measPower[0] - measCalFactor;
                    DigitizerHal.InputLevel = limitP1dB + measCalFactor + 6;
                    Util.WaitTime(settlingTime);
                    break;
            }

            gainInit = Pout - startLevel;
            gainTargetP1dB = gainInit - 1;
            gainTargetP3dB = gainInit - 3;
            gain = gainInit;

            if (servoType == SERVO.SERVO_NONE)
            {
                loopTries = 1;
                loopDone = true;
                wasSuccessful = true;
            }
            else
            {
                do
                {
                    error1 = VDET_TARGET - VDET;
                    error2 = poutTarget - Pout;
                    if (Math.Abs(error1) <= targetTolerance)
                    {
                        loopDone = true;
                        wasSuccessful = true;
                    }
                    else
                    {
                        if (gain <= (expectedGain - 15))
                        {
                            loopDone = true;
                            wasSuccessful = false;
                        }
                        else
                        {
                            newPin += error1 / VDET_SLOPE;

                            if (newPin > 8)
                            {
                                loopDone = true;
                                wasSuccessful = false;
                                Console.WriteLine("WARNING: Pin {0:F2} dBm exceeds maximum safe level of 8 dBm on servo loop iteration {1}.", newPin - srcCalFactor, loopTries + 1);
                            }
                        }
                    }
                    if (loopDone == false)
                    {
                        SigGen1Hal.Level = (newPin - srcCalFactor);
                        Util.WaitTime(settlingTime);

                        Pin = newPin;
                        measPower = DigitizerHal.CapturePower();
                        Pout = measPower[0] - measCalFactor;
                        gain = Pout - Pin;

                        //--------------------MEASURE VDET FOR POUT-----------------------------------

                        Digital.cpu_df(1, 1, 0, 0);
                        Digital.pmufi(1, 1, 0, 1.0, 0.0);//board number, chip number, double ri, cvh,cvl ;
                        Digital.con_pmu(1, 1, 1);
                        Util.WaitTime(2.0e-3);
                        VDET = Digital.vmeas(1, 1);
                        //Util.WaitTime(2e-3);
                        Digital.con_pmu(1, 1, 0);

                        //Digital.cpu_df(1, 6, 0, 0);
                        //Digital.pmufi(1, 2, 0, 1.0, 0.0);//board number, chip number, double ri, cvh,cvl ;
                        //Digital.con_pmu(1, 6, 1);
                        //Util.WaitTime(2e-3);
                        //VDET_2 = Digital.vmeas(1, 6);
                        //// Util.WaitTime(2e-3);
                        //Digital.con_pmu(1, 6, 0);



                    }

                    loopTries++;

                } while (loopDone == false && loopTries < maxLoopIterations);
            }

            if (servoType == SERVO.SERVO_P1DB || servoType == SERVO.SERVO_P1DB_P3DB)
            {
                trace.TraceInformation("Doing a P1DB loop");
                loopTries = 0;
                loopDone = false;
                wasSuccessful = false;
                do
                {
                    gainDelta = gainInit - gain;
                    error2 = 1 - gainDelta;

                    trace.TraceInformation("Gain Init = {0}\tGain = {1} Gain Delta = {2} Loop tries {3}", gainInit, gain, gainDelta, loopTries);

                    if (Math.Abs(error2) <= targetTolerance)
                    {
                        loopDone = true;
                        wasSuccessful = true;
                        Pout1dB = Pout;
                    }
                    else
                    {
                        newPin += error2;

                        if (newPin > 8)
                        {
                            loopDone = true;
                            wasSuccessful = false;
                            Console.WriteLine("WARNING: Pin {0:F2} dBm exceeds maximum safe level of 8 dBm on servo loop iteration {1}.", newPin - srcCalFactor, loopTries + 1);
                        }
                    }
                    if (loopDone == false)
                    {
                        SigGen1Hal.Level = (newPin - srcCalFactor);
                        Util.WaitTime(settlingTime);

                        Pin = newPin;
                        measPower = DigitizerHal.CapturePower();
                        Pout = measPower[0] - measCalFactor;
                        gain = Pout - Pin;
                    }

                    loopTries++;

                } while (loopDone == false && loopTries < maxLoopIterations);
            }

            if (servoType == SERVO.SERVO_P3DB || servoType == SERVO.SERVO_P1DB_P3DB)
            {
                loopTries = 0;
                loopDone = false;
                wasSuccessful = false;
                do
                {
                    gainDelta = gainInit - gain;
                    error2 = 3 - gainDelta;
                    if (Math.Abs(error2) <= targetTolerance)
                    {
                        loopDone = true;
                        wasSuccessful = true;
                        Pout3dB = Pout;
                    }
                    else
                    {
                        newPin += error2;

                        if (newPin > 8)
                        {
                            loopDone = true;
                            wasSuccessful = false;
                            Console.WriteLine("WARNING: Pin {0:F2} dBm exceeds maximum safe level of 8 dBm on servo loop iteration {1}.", newPin - srcCalFactor, loopTries + 1);
                        }
                    }
                    if (loopDone == false)
                    {
                        SigGen1Hal.Level = (newPin - srcCalFactor);
                        Util.WaitTime(settlingTime);

                        Pin = newPin;
                        measPower = DigitizerHal.CapturePower();
                        Pout = measPower[0] - measCalFactor;
                        gain = Pout - Pin;
                    }

                    loopTries++;

                } while (loopDone == false && loopTries < maxLoopIterations);
            }

            if (wasSuccessful) return loopTries;
            else return -1;
        }


        private void SetupVramp(double waveformMarkerDelay, double onTime, double offTime, double onRiseTime, double onFallTime, double onVoltsStart, double onVoltsStop, double offVolts, double delayTime, int Ch)
        {
            DC.LPSource.waveformMarkerDelay = waveformMarkerDelay;
            DC.LPSource.onTime = onTime;    //bust on time
            DC.LPSource.offTime = offTime;   // off time, remember one play per trigger so off time just has to be long enough to include burst, rise time, fall time and any delay

            DC.LPSource.onRiseTime = onRiseTime;
            DC.LPSource.onFallTime = onFallTime;
            DC.LPSource.onVoltsStart = onVoltsStart;  //if you want to ramp up make this lower than stop
            DC.LPSource.onVoltsStop = onVoltsStop;   //if you want to ramp down make this lower than start
            DC.LPSource.offVolts = offVolts;
            DC.LPSource.delayTime = delayTime;     //can add delay
            DC.LPSource.aoTrig = "PXI_Trig1";  //trigger on second waveform marker
            DC.LPSource.setVramp(Ch);  //Setup the ramp
        }
        //================================== Multithread Analysis Functions ==================================
        private List<NoiseData> noiseDatas = new List<NoiseData>();

        private void ProcessTestPointWlan(object WlanObj)
        {
            WlanRfmxAnalysis WlanThreadObj = WlanObj as WlanRfmxAnalysis;

            lock (WlanThreadObj)
            {
                try
                {
                    WlanThreadObj.AnalyzeIQ();
                }

                catch (Exception ex)
                {
                    throw new Exception("WLAN exception: " + ex.Message);
                }

                WlanThreadObj.ThreadComplete = true;
            }
        }

        public int Measure_OS(int testNumber,bool retest)
        {
            if (!isOSPass) return 0;

            int testNumberStart = testNumber;

            //double[] OS_DET_NEG = new double[ActiveSites.Length];//Digital pin1(VDET)
            //double[] OS_DET_POS = new double[ActiveSites.Length];
            double[] OS_C0 = new double[ActiveSites.Length];//Digital pin3(C0)
            double[] OS_C1 = new double[ActiveSites.Length];//Digital pin2(C1)

            if(retest)
            { 
                // VCC2@1
                PS.SetVoltage(2, 0.0);
                PS.SetCurrent(2, 1.0);
                DC.Meas.setHPRange(2, HRange.R_1A);

                // VCC1@1
                DC.setVoltage(3, 0.0);
                DC.setLPRange(3, LRange.R_200mA);
                DC.Meas.setLPRange(3, LRange.R_200mA);

                // VDD@1
                PS.SetVoltage(1, 0.0);
                PS.SetCurrent(1, 1.0);
                DC.Meas.setHPRange(1, HRange.R_1A);

                // PAEN@1
                DC.setVoltage(1, 0.0);
                DC.setLPRange(1, LRange.R_200mA);
            }

            Digital.cpu_df(1, 1, 1, 0);//DET@1
            Digital.cpu_df(1, 2, 1, 0);//LNAEN@1
            Digital.cpu_df(1, 3, 1, 0);//RXEN@1

            Digital.cpu_df(1, 6, 1, 0);//DET@2
            Digital.cpu_df(1, 7, 1, 0);//LNAEN@2
            Digital.cpu_df(1, 8, 1, 0);//RXEN@2

            Digital.con_pmu(1, 0, 0);

            //------------------Measure OS VDET@1----------------------------------------
            //Digital.cpu_df(1, 1, 0, 0);//LNAEN
            //Digital.pmufi(1, 1, 0, 2, -1);
            //Digital.con_pmu(1, 5, 1);//connect CH8 to PMU to avoid spur          
            //Util.WaitTime(2.0e-3);
            //Digital.con_pmu(1, 1, 1);//LNAEN
            //Digital.con_pmu(1, 5, 0);
            //Util.WaitTime(1e-3);
            //Digital.pmufi(1, 1, -0.5, 2, -1);
            //Util.WaitTime(5e-3);
            //OS_DET_NEG[0] = Digital.vmeas(1, 1);//VDET_NEG
            //Digital.pmufi(1, 1, 0.5, 2, -1);
            //Util.WaitTime(5e-3);
            //OS_DET_POS[0] = Digital.vmeas(1, 1);//VDET_NEG
            //Digital.con_pmu(1, 1, 0);


            //------------------Measure OS C1@2----------------------------------------
            Digital.cpu_df(1, 2, 0, 0);//LNAEN
            Digital.pmufi(1, 1, 0, 2, -1);
            Digital.con_pmu(1, 5, 1);//connect CH8 to PMU to avoid spur          
            Util.WaitTime(1.0e-3);
            Digital.con_pmu(1, 2, 1);//LNAEN
            Digital.con_pmu(1, 5, 0);
            Util.WaitTime(1e-3);
            Digital.pmufi(1, 1, 0.02, 2, -1);
            Util.WaitTime(1e-3);
            OS_C1[0] = Digital.vmeas(1, 2);//C1

            Digital.con_pmu(1, 2, 0);

            //------------------Measure OS C0@3----------------------------------------
            Digital.cpu_df(1, 3, 0, 0);//DET
            Digital.pmufi(1, 1, 0, 2, -1);
            Digital.con_pmu(1, 5, 1);//connect CH8 to PMU to avoid spur          
            Util.WaitTime(1.0e-3);
            Digital.con_pmu(1, 3, 1);//DET
            Digital.con_pmu(1, 5, 0);
            Util.WaitTime(1e-3);
            Digital.pmufi(1, 1, 0.02, 2, -1);
            Util.WaitTime(1e-3);
            OS_C0[0] = Digital.vmeas(1, 3);//C0

            //todo: retrive from limit file
            //var ttt = RetriveLimit(testNumber);
            if (OS_C0[0] > 0.6 || OS_C0[0] < 0.1)
            {
                isOSPass = false;
            }
            else
            {
                isOSPass = true;
            }

            Digital.con_pmu(1, 3, 0);


            //------------------Measure OS VDET@6----------------------------------------
            //Digital.cpu_df(1, 6, 0, 0);//LNAEN
            //Digital.pmufi(1, 1, 0, 2, -1);
            //Digital.con_pmu(1, 5, 1);//connect CH8 to PMU to avoid spur          
            //Util.WaitTime(2.0e-3);
            //Digital.con_pmu(1, 6, 1);//LNAEN
            //Digital.con_pmu(1, 5, 0);
            //Util.WaitTime(1e-3);
            //Digital.pmufi(1, 1, -0.5, 2, -1);
            //Util.WaitTime(5e-3);
            //OS_DET_NEG[0] = Digital.vmeas(1, 6);//VDET_NEG
            //Digital.pmufi(1, 1, 0.5, 2, -1);
            //Util.WaitTime(5e-3);
            //OS_DET_POS[0] = Digital.vmeas(1, 6);//VDET_NEG

            //Digital.con_pmu(1, 1, 0);

            //------------------Measure OS LNAEN@2----------------------------------------
            //Digital.cpu_df(1, 7, 0, 0);//LNAEN
            //Digital.pmufi(1, 1, 0, 2, -1);
            //Digital.con_pmu(1, 5, 1);//connect CH8 to PMU to avoid spur          
            //Util.WaitTime(2.0e-3);
            //Digital.con_pmu(1, 7, 1);//LNAEN
            //Digital.con_pmu(1, 5, 0);
            //Util.WaitTime(1e-3);
            //Digital.pmufi(1, 1, 0.02, 2, -1);
            //Util.WaitTime(5e-3);
            //OS_LNAEN[0] = Digital.vmeas(1, 7);

            //Digital.con_pmu(1, 7, 0);

            //------------------Measure OS RXEN@2----------------------------------------
            //Digital.cpu_df(1, 8, 0, 0);//DET
            //Digital.pmufi(1, 1, 0, 2, -1);
            //Digital.con_pmu(1, 5, 1);//connect CH8 to PMU to avoid spur          
            //Util.WaitTime(2.0e-3);
            //Digital.con_pmu(1, 8, 1);//DET
            //Digital.con_pmu(1, 5, 0);
            //Util.WaitTime(1e-3);
            //Digital.pmufi(1, 1, 0.02, 2, -1);
            //Util.WaitTime(5e-3);
            //OS_RXEN[0] = Digital.vmeas(1, 8);

            //Digital.con_pmu(1, 8, 0);


            Digital.cpu_df(1, 1, 1, 0);//DET@1
            Digital.cpu_df(1, 2, 1, 0);//LNAEN@1
            Digital.cpu_df(1, 3, 1, 0);//RXEN@1

            Digital.cpu_df(1, 6, 1, 0);//DET@2
            Digital.cpu_df(1, 7, 1, 0);//LNAEN@2
            Digital.cpu_df(1, 8, 1, 0);//RXEN@2

            if (writeToDatalog)
            {
                testNumber = testNumberStart;
                LogTestResult(testNumber++, OS_C0);
                LogTestResult(testNumber++, OS_C1);
                //if(retest == false)
                //{ 
                    //LogTestResult(testNumber++, OS_DET_NEG);
                    //LogTestResult(testNumber++, OS_DET_POS);
                //}
            }
            return 0;
        }

        public int Measure_End_OS(int testNumber)

        {
            int testNumberStart = testNumber;

            double[] OS_DET_NEG = new double[ActiveSites.Length];//Digital pin1(VDET)
            double[] OS_DET_POS = new double[ActiveSites.Length];
            double[] OS_C0 = new double[ActiveSites.Length];//Digital pin3(C0)
            double[] OS_C1 = new double[ActiveSites.Length];//Digital pin2(C1)

            Digital.cpu_df(1, 1, 1, 0);//DET@1
            Digital.cpu_df(1, 2, 1, 0);//LNAEN@1
            Digital.cpu_df(1, 3, 1, 0);//RXEN@1

            Digital.cpu_df(1, 6, 1, 0);//DET@2
            Digital.cpu_df(1, 7, 1, 0);//LNAEN@2
            Digital.cpu_df(1, 8, 1, 0);//RXEN@2

            Digital.con_pmu(1, 0, 0);

            //------------------Measure OS VDET@1----------------------------------------
            Digital.cpu_df(1, 1, 0, 0);//LNAEN
            Digital.pmufi(1, 1, 0, 2, -1);
            Digital.con_pmu(1, 5, 1);//connect CH8 to PMU to avoid spur          
            Util.WaitTime(2.0e-3);
            Digital.con_pmu(1, 1, 1);//LNAEN
            Digital.con_pmu(1, 5, 0);
            Util.WaitTime(1e-3);
            Digital.pmufi(1, 1, -0.5, 2, -1);
            Util.WaitTime(5e-3);
            OS_DET_NEG[0] = Digital.vmeas(1, 1);//VDET_NEG
            Digital.pmufi(1, 1, 0.5, 2, -1);
            Util.WaitTime(5e-3);
            OS_DET_POS[0] = Digital.vmeas(1, 1);//VDET_NEG
            Digital.con_pmu(1, 1, 0);


            //------------------Measure OS C1@2----------------------------------------
            Digital.cpu_df(1, 2, 0, 0);//LNAEN
            Digital.pmufi(1, 1, 0, 2, -1);
            Digital.con_pmu(1, 5, 1);//connect CH8 to PMU to avoid spur          
            Util.WaitTime(1.0e-3);
            Digital.con_pmu(1, 2, 1);//LNAEN
            Digital.con_pmu(1, 5, 0);
            Util.WaitTime(1e-3);
            Digital.pmufi(1, 1, 0.02, 2, -1);
            Util.WaitTime(1e-3);
            OS_C1[0] = Digital.vmeas(1, 2);//C1

            Digital.con_pmu(1, 2, 0);

            //------------------Measure OS C0@3----------------------------------------
            Digital.cpu_df(1, 3, 0, 0);//DET
            Digital.pmufi(1, 1, 0, 2, -1);
            Digital.con_pmu(1, 5, 1);//connect CH8 to PMU to avoid spur          
            Util.WaitTime(1.0e-3);
            Digital.con_pmu(1, 3, 1);//DET
            Digital.con_pmu(1, 5, 0);
            Util.WaitTime(1e-3);
            Digital.pmufi(1, 1, 0.02, 2, -1);
            Util.WaitTime(1e-3);
            OS_C0[0] = Digital.vmeas(1, 3);//C0

            Digital.con_pmu(1, 3, 0);


            //------------------Measure OS VDET@6----------------------------------------
            //Digital.cpu_df(1, 6, 0, 0);//LNAEN
            //Digital.pmufi(1, 1, 0, 2, -1);
            //Digital.con_pmu(1, 5, 1);//connect CH8 to PMU to avoid spur          
            //Util.WaitTime(2.0e-3);
            //Digital.con_pmu(1, 6, 1);//LNAEN
            //Digital.con_pmu(1, 5, 0);
            //Util.WaitTime(1e-3);
            //Digital.pmufi(1, 1, -0.5, 2, -1);
            //Util.WaitTime(5e-3);
            //OS_DET_NEG[0] = Digital.vmeas(1, 6);//VDET_NEG
            //Digital.pmufi(1, 1, 0.5, 2, -1);
            //Util.WaitTime(5e-3);
            //OS_DET_POS[0] = Digital.vmeas(1, 6);//VDET_NEG

            //Digital.con_pmu(1, 1, 0);

            //------------------Measure OS LNAEN@2----------------------------------------
            //Digital.cpu_df(1, 7, 0, 0);//LNAEN
            //Digital.pmufi(1, 1, 0, 2, -1);
            //Digital.con_pmu(1, 5, 1);//connect CH8 to PMU to avoid spur          
            //Util.WaitTime(2.0e-3);
            //Digital.con_pmu(1, 7, 1);//LNAEN
            //Digital.con_pmu(1, 5, 0);
            //Util.WaitTime(1e-3);
            //Digital.pmufi(1, 1, 0.02, 2, -1);
            //Util.WaitTime(5e-3);
            //OS_LNAEN[0] = Digital.vmeas(1, 7);

            //Digital.con_pmu(1, 7, 0);

            //------------------Measure OS RXEN@2----------------------------------------
            //Digital.cpu_df(1, 8, 0, 0);//DET
            //Digital.pmufi(1, 1, 0, 2, -1);
            //Digital.con_pmu(1, 5, 1);//connect CH8 to PMU to avoid spur          
            //Util.WaitTime(2.0e-3);
            //Digital.con_pmu(1, 8, 1);//DET
            //Digital.con_pmu(1, 5, 0);
            //Util.WaitTime(1e-3);
            //Digital.pmufi(1, 1, 0.02, 2, -1);
            //Util.WaitTime(5e-3);
            //OS_RXEN[0] = Digital.vmeas(1, 8);

            //Digital.con_pmu(1, 8, 0);


            Digital.cpu_df(1, 1, 1, 0);//DET@1
            Digital.cpu_df(1, 2, 1, 0);//LNAEN@1
            Digital.cpu_df(1, 3, 1, 0);//RXEN@1

            Digital.cpu_df(1, 6, 1, 0);//DET@2
            Digital.cpu_df(1, 7, 1, 0);//LNAEN@2
            Digital.cpu_df(1, 8, 1, 0);//RXEN@2

            if (writeToDatalog)
            {
                testNumber = testNumberStart;
                LogTestResult(testNumber++, OS_C0);
                LogTestResult(testNumber++, OS_C1);
                //LogTestResult(testNumber++, OS_DET_NEG);
                //LogTestResult(testNumber++, OS_DET_POS);

            }
            return 0;
        }

        public void Measure_Idle_Current(int testNumber, bool retest)
        {
            try
            {
                if (!isOSPass) return;

                trace.TraceInformation(testNumber + "  Measure_Idle_Current");
                int testNumberStart = testNumber;

                double[] Idle_IIH_PA_EN = new double[ActiveSites.Length];
                double[] Idle_IIL_PA_EN = new double[ActiveSites.Length];
                double[] Idle_IIH_C0 = new double[ActiveSites.Length];
                double[] Idle_IIL_C0 = new double[ActiveSites.Length];
                double[] Idle_IIH_C1 = new double[ActiveSites.Length];
                double[] Idle_IIL_C1 = new double[ActiveSites.Length];

                double[] Icq_VCC1 = new double[ActiveSites.Length];
                double[] Icq_VCC2 = new double[ActiveSites.Length];
                double[] Icq_VDD = new double[ActiveSites.Length];
                double[] Icq_GROSS_END = new double[ActiveSites.Length];

                double[] i_result;
                // VCC2@1
                PS.SetVoltage(2, 5.0);

                // VCC1@1
                DC.setVoltage(3, 5.0);

                // VDD@1
                PS.SetVoltage(1, 5.0);

                // PAEN@1
                //--------------------PAEN Leakage  ------------------------------------------------------------------
                SigGen1Hal.Level = SigGen1Hal.LevelMin;

                DC.setLPRange(1, LRange.R_1mA);//PAEN@1
                //DC.setLPRange(2, LRange.R_1mA);//PAEN@2

                DC.setVoltage(1, 3.3);//PA_EN@1
                //DC.setVoltage(2, 3.3);//PA_EN@2

                DC.Meas.setTriggerSource("None");
                DC.Meas.setSampleClockandSamples(50e3, 10);
                Util.WaitTime(2e-3);
                i_result = DC.Meas.getCurrentAverage();
                Idle_IIH_PA_EN[0] = i_result[0];
                //Idle_IIH_PA_EN[1] = i_result[1];

                DC.setVoltage(1, 0);//PA_EN@1
                //DC.setVoltage(2, 0);//PA_EN@2

                DC.setLPRange(1, LRange.R_10uA);//PA_EN@1
                //DC.setLPRange(2, LRange.R_10uA);//PA_EN@2

                Util.WaitTime(1e-3);
                i_result = DC.Meas.getCurrentAverage();
                Idle_IIL_PA_EN[0] = i_result[0];
                //Idle_IIL_PA_EN[1] = i_result[1];

                DC.setLPRange(1, LRange.R_200mA);//PA_EN@1  LOW POWER
                //DC.setLPRange(2, LRange.R_200mA);//PA_EN@2


                //----------------------------------Measure C0/C1 leakage-------------------------------------------
                for (int siteNum = 0; siteNum < ActiveSites.Length; siteNum++)
                {
                    if (siteNum == 0 && ActiveSites[siteNum])
                    {

                        //----------------------------IIH_C1------------------------------
                        Digital.cpu_df(1, MVPs["C1"][siteNum].digPin, 0, 0);

                        Digital.pmufv(1, 1, 3.3, 0.3);//board number, chip number, voltage, clampi
                        Digital.con_pmu(1, MVPs["C1"][siteNum].digPin, 1);
                        Util.WaitTime(2e-3);

                        Idle_IIH_C1[siteNum] = Digital.imeas(1, MVPs["C1"][siteNum].digPin) / 1e3;
                        //Util.WaitTime(2e-3);
                        //---------------------------IIL_C1------------------------------
                        Digital.pmufv(1, 1, 0, 0.1);
                        Digital.con_pmu(1, MVPs["C1"][siteNum].digPin, 1);
                        Util.WaitTime(2e-3);

                        Idle_IIL_C1[siteNum] = Digital.imeas(1, MVPs["C1"][siteNum].digPin) / 1e3;
                        Digital.con_pmu(1, MVPs["C1"][siteNum].digPin, 0);
                        Digital.cpu_df(1, MVPs["C1"][siteNum].digPin, 1, 0);

                        //----------------------------IIH_C0------------------------------
                        Digital.cpu_df(1, MVPs["C0"][siteNum].digPin, 0, 0);
                        Digital.pmufv(1, 1, 3.3, 0.5);//board number, chip number, voltage, clampi
                        Digital.con_pmu(1, MVPs["C0"][siteNum].digPin, 1);
                        Util.WaitTime(5e-3);
                        Idle_IIH_C0[siteNum] = Digital.imeas(1, MVPs["C0"][siteNum].digPin) / 1e3;
                        //---------------------------IIL_C0------------------------------
                        Digital.pmufv(1, 1, 0, 0.1);
                        Digital.con_pmu(1, MVPs["C0"][siteNum].digPin, 1);
                        Util.WaitTime(2e-3);

                        Idle_IIL_C0[siteNum] = Digital.imeas(1, MVPs["C0"][siteNum].digPin) / 1e3;
                        Digital.con_pmu(1, MVPs["C0"][siteNum].digPin, 0);
                        Digital.cpu_df(1, MVPs["C0"][siteNum].digPin, 1, 0);
                        //Util.WaitTime(2e-3);


                    }

                    //if (siteNum == 1 && ActiveSites[siteNum])
                    //Digital.cpu_df(1, MVPs["C1"][siteNum].digPin, 0, 0);

                    //Digital.pmufv(1, 1, 3.3, 0.3);//board number, chip number, voltage, clampi
                    //Digital.con_pmu(1, MVPs["C1"][siteNum].digPin, 1);
                    //Util.WaitTime(2e-3);

                    //Idle_IIH_C1[siteNum] = Digital.imeas(1, MVPs["C1"][siteNum].digPin) / 1e3;
                    //Util.WaitTime(2e-3);
                    //---------------------------IIL_C1------------------------------
                    //Digital.pmufv(1, 1, 0, 0.1);
                    //Digital.con_pmu(1, MVPs["C1"][siteNum].digPin, 1);
                    //Util.WaitTime(2e-3);

                    //Idle_IIL_C1[siteNum] = Digital.imeas(1, MVPs["C1"][siteNum].digPin) / 1e3;
                    //Digital.con_pmu(1, MVPs["C1"][siteNum].digPin, 0);
                    //Digital.cpu_df(1, MVPs["C1"][siteNum].digPin, 1, 0);

                    //----------------------------IIH_C0------------------------------
                    //Digital.cpu_df(1, MVPs["C0"][siteNum].digPin, 0, 0);
                    //Digital.pmufv(1, 1, 3.3, 0.3);//board number, chip number, voltage, clampi
                    //.con_pmu(1, MVPs["C0"][siteNum].digPin, 1);
                    //Util.WaitTime(2e-3);
                    //Idle_IIH_C0[siteNum] = Digital.imeas(1, MVPs["C0"][siteNum].digPin) / 1e3;
                    //---------------------------IIL_C0------------------------------
                    //Digital.pmufv(1, 1, 0, 0.1);
                    //Digital.con_pmu(1, MVPs["C0"][siteNum].digPin, 1);
                    //Util.WaitTime(2e-3);

                    //Idle_IIL_C0[siteNum] = Digital.imeas(1, MVPs["C0"][siteNum].digPin) / 1e3;
                    //Digital.con_pmu(1, MVPs["C0"][siteNum].digPin, 0);
                    //Digital.cpu_df(1, MVPs["C0"][siteNum].digPin, 1, 0);
                    //Util.WaitTime(2e-3);
                }

                //----------------------------IDLE VDD-------------------------------------------
                //----------------------------IDLE VCC-------------------------------------------

                DC.setHPRange(1, HRange.R_10mA);//VDD@1
                DC.setHPRange(2, HRange.R_10uA);//VCC2@1
                DC.setLPRange(3, LRange.R_10uA);//VCC1@1
                Util.WaitTime(10e-3);
                
                i_result = DC.Meas.getCurrentAverage();

                Icq_VDD[0] = i_result[4];
                Icq_VCC1[0] = i_result[2];
                Icq_VCC2[0] = i_result[5];

    
                Icq_GROSS_END[0] = Icq_VCC1[0] + Icq_VDD[0] + Icq_VCC2[0];



                //------------Set large range for RF test-------------------------
                DC.setHPRange(1, HRange.R_1A);//VDD@1
                DC.setHPRange(2, HRange.R_1A);//VCC2@1
                DC.setLPRange(3, LRange.R_200mA);//VCC1@1
                

                if (writeToDatalog)
                {

                    LogTestResult(testNumber++, Icq_VCC1);
                    LogTestResult(testNumber++, Icq_VCC2);
                    LogTestResult(testNumber++, Icq_VDD);
                    LogTestResult(testNumber++, Icq_GROSS_END);
                    LogTestResult(testNumber++, Idle_IIH_C1);
                    LogTestResult(testNumber++, Idle_IIH_PA_EN);
                    LogTestResult(testNumber++, Idle_IIH_C0);
                    LogTestResult(testNumber++, Idle_IIL_C1);
                    LogTestResult(testNumber++, Idle_IIL_PA_EN);
                    LogTestResult(testNumber++, Idle_IIL_C0);
                }
            }
            catch (Exception ex)
            {
                trace.TraceError(testNumber + " : " + ex.Message + "\n" + ex.StackTrace);
            }
        }

        public void Measure_End_Idle_Current(int testNumber, bool retest)
        {
            try
            {
                if (!isOSPass) return;

                trace.TraceInformation(testNumber + "  Measure_WLAN");
                int testNumberStart = testNumber;

                double[] TX_VCC1 = new double[ActiveSites.Length];
                double[] TX_VCC2 = new double[ActiveSites.Length];
                double[] TX_VDD = new double[ActiveSites.Length];
                double[] TX_ICQ = new double[ActiveSites.Length];//TX_VCC1+TX_VCC2+TX_VDD
                double[] LNA_VCC1 = new double[ActiveSites.Length];//LNA Mode
                double[] IOFF_VCC1 = new double[ActiveSites.Length];

                double[] IOFF_VCC2 = new double[ActiveSites.Length];
                double[] IOFF_VDD = new double[ActiveSites.Length];
                double[] IOFF_GROSS_END = new double[ActiveSites.Length];//IOFF_VCC1+IOFF_VCC2+IOFF_VDD


                double[] i_result;

                i_result = DC.Meas.getCurrentAverage();
                TX_VDD[0] = i_result[4];  //VDD@1
                TX_VCC2[0] = i_result[5];  //VCC2@1
                TX_VCC1[0] = i_result[2];  //VCC1@1
                TX_ICQ[0] = TX_VCC1[0] + TX_VCC2[0] + TX_VDD[0];//Total_ICC@1

                DeviceInit(GPIO.WLAN_GAIN_RX);
                i_result = DC.HPMeas.getCurrentAverage();
                LNA_VCC1[0] = i_result[0];  //VDD@1

                DeviceInit(GPIO.Power_off);
                DC.setHPRange(1, HRange.R_10mA);//VDD@1
                i_result = DC.Meas.getCurrentAverage();
                IOFF_VCC1[0] = i_result[2];  //VCC1@1
                IOFF_VCC2[0] = i_result[5];  //VCC2@1
                IOFF_VDD[0] = i_result[4];  //VDD@1
                IOFF_GROSS_END[0] = IOFF_VCC1[0] + IOFF_VCC2[0] + IOFF_VDD[0];//Total_ICC@1

                if (writeToDatalog)
                {

                    LogTestResult(testNumber++, TX_VCC1);
                    LogTestResult(testNumber++, TX_VCC2);
                    LogTestResult(testNumber++, TX_VDD);
                    LogTestResult(testNumber++, TX_ICQ);
                    LogTestResult(testNumber++, LNA_VCC1);
                    //LogTestResult(testNumber++, IOFF_VCC1);
                    //LogTestResult(testNumber++, IOFF_VCC2);
                    //LogTestResult(testNumber++, IOFF_VDD);
                    //LogTestResult(testNumber++, IOFF_GROSS_END);
                    }
            }
            catch (Exception ex)
            {
                trace.TraceError(testNumber + " : " + ex.Message + "\n" + ex.StackTrace);
            }
        }

        public void Measure_ISO(int testNumber, SETUP measureIdle, SETUP measureSetup, SETUP measureDC, SETUP harmonic, double startPin,
                                  double targetLevel, double tolerance, double expectedGain, double expectedP1dB, double expectedP3dB, SERVO servoType, double centerFreq, double settleTime, double digitizerLevel,
                                  MTRF100.SigGen srcSelect, string srcPin, MTRF100.SrcAmpEnable srcAmpSelect, string measPin1, string measPin2, MTRF100.AmpState measAmpSelect100, string modMode, double calLevel,
                                  MTRF200.MeasAmpEnable measAmpSelect200, MTRF200.MeasPathFilterAttenBypass measPathSelect, double attenuation, MTRF200.DownConverterState downConvState, SrcMeasAmpConfig measAmpCalPath)

        {
            try
            {
                if (!isOSPass) return;

                trace.TraceInformation(testNumber + "  Measure_ISO");
                srcCalFactor = 0;
                measCalFactor = 0;

                double[] measPower = new double[0];
                double[] PgainCal = new double[ActiveSites.Length];
                double[] PinCal = new double[ActiveSites.Length];
                double[] ISO = new double[ActiveSites.Length];
                double srcTraceCalFactor = 0;
                double measTraceCalFactor1 = 0;
                double measTraceCalFactor2 = 0;

                Pin[0] = 9999;
                Pin[1] = 9999;
                Pout[0] = 9999;
                Pout[1] = 9999;
                Pout1dB[0] = 9999;
                Pout1dB[1] = 9999;
                Pout3dB[0] = 9999;
                Pout3dB[1] = 9999;
                VCC1_ICC[0] = 9999;
                VCC1_ICC[1] = 9999;
                VCC2_ICC[0] = 9999;
                VCC2_ICC[1] = 9999;
                VDD_ICC[0] = 9999;
                VDD_ICC[1] = 9999;
                Total_ICC[0] = 9999;
                Total_ICC[1] = 9999;
                Vdet[0] = 9999;
                Vdet[1] = 9999;
                VdetOff[0] = 9999;
                VdetOff[1] = 9999;

                DigitizerHal.InputLevel = digitizerLevel;

                for (int siteNum = 0; siteNum < ActiveSites.Length; siteNum++)
                {
                    if (!ActiveSites[siteNum])
                        continue;

                    if (useTraceCalData)
                    {
                        srcTraceCalFactor = TraceCal.GetTraceLossFactor(centerFreq / 1e6, srcPin, siteNum + 1);
                        measTraceCalFactor1 = TraceCal.GetTraceLossFactor(centerFreq / 1e6, measPin1, siteNum + 1);
                        measTraceCalFactor2 = TraceCal.GetTraceLossFactor(centerFreq / 1e6, measPin2, siteNum + 1);
                    }
                    if (measPin1 == "ANT")
                    {
                        //rf100.ConnectSourcePort(srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect);
                        //rf100.ConnectMeasurePort(MVPs[measPin1][siteNum].srcMeasPort, measAmpSelect100);

                        rf100.ConnectSourceAndMeasurePort(srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect, MVPs[measPin1][siteNum].srcMeasPort, measAmpSelect100);
                        rf200.ConnectMeasurePort(MVPs[measPin1][siteNum].measPort, measAmpSelect200, measPathSelect, attenuation, downConvState, MTRF200.DigitizerState.Digitizer1);
                        srcCalFactor = PortModuleCal.GetSrcCalFactorNew(centerFreq, srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect, modMode, currentWaveform, calLevel);
                        measCalFactor = PortModuleCal.GetMeasCalFactorNew(centerFreq, MVPs[measPin1][siteNum].SystemPort, measPathSelect, measAmpCalPath, modMode, currentWaveform, calLevel);

                        SigGen1Hal.Frequency = centerFreq;
                        DigitizerHal.Frequency = centerFreq;

                        int servoStatus = -1;
                        
                        servoStatus = Servo(servoType, startPin - srcTraceCalFactor, targetLevel + measTraceCalFactor1, tolerance, expectedGain, expectedP1dB, expectedP3dB,
                                            settleTime, 10, centerFreq, ref Pin[siteNum], ref Pout[siteNum], ref Pout1dB[siteNum], ref Pout3dB[siteNum], ref gainDeltaX[siteNum]);

                        Pin[siteNum] = Pin[siteNum] + srcTraceCalFactor;
                        ANT_Pout[siteNum] = Pout[siteNum] - measTraceCalFactor1;


                        //rf100.ConnectSourcePort(srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect);
                        //rf100.ConnectMeasurePort(MVPs[measPin2][siteNum].srcMeasPort, measAmpSelect100);

                        rf100.ConnectSourceAndMeasurePort(srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect, MVPs[measPin2][siteNum].srcMeasPort, measAmpSelect100);
                        rf200.ConnectMeasurePort(MVPs[measPin2][siteNum].measPort, measAmpSelect200, measPathSelect, attenuation, downConvState, MTRF200.DigitizerState.Digitizer1);
                        srcCalFactor = PortModuleCal.GetSrcCalFactorNew(centerFreq, srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect, modMode, currentWaveform, calLevel);
                        measCalFactor = PortModuleCal.GetMeasCalFactorNew(centerFreq, MVPs[measPin2][siteNum].SystemPort, measPathSelect, measAmpCalPath, modMode, currentWaveform, calLevel);

                        SigGen1Hal.Frequency = centerFreq;
                        DigitizerHal.Frequency = centerFreq;

                  

                        servoStatus = Servo(servoType, startPin - srcTraceCalFactor, targetLevel + measTraceCalFactor2, tolerance, expectedGain, expectedP1dB, expectedP3dB,
                                            settleTime, 10, centerFreq, ref Pin[siteNum], ref Pout[siteNum], ref Pout1dB[siteNum], ref Pout3dB[siteNum], ref gainDeltaX[siteNum]);

                        Pin[siteNum] = Pin[siteNum] + srcTraceCalFactor;
                        RX_Pout[siteNum] = Pout[siteNum] - measTraceCalFactor2;
                        ISO[siteNum] = Math.Abs(RX_Pout[siteNum] - ANT_Pout[siteNum]);

                    }
                    else
                    {
                        //rf100.ConnectSourcePort(srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect);
                        //rf100.ConnectMeasurePort(MVPs[measPin1][siteNum].srcMeasPort, measAmpSelect100);

                        rf100.ConnectSourceAndMeasurePort(srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect, MVPs[measPin1][siteNum].srcMeasPort, measAmpSelect100);
                        rf200.ConnectMeasurePort(MVPs[measPin1][siteNum].measPort, measAmpSelect200, measPathSelect, attenuation, downConvState, MTRF200.DigitizerState.Digitizer1);
                        srcCalFactor = PortModuleCal.GetSrcCalFactorNew(centerFreq, srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect, modMode, currentWaveform, calLevel);
                        measCalFactor = PortModuleCal.GetMeasCalFactorNew(centerFreq, MVPs[measPin1][siteNum].SystemPort, measPathSelect, measAmpCalPath, modMode, currentWaveform, calLevel);

                        SigGen1Hal.Frequency = centerFreq;
                        DigitizerHal.Frequency = centerFreq;

                        int servoStatus = -1;

                        servoStatus = Servo(servoType, startPin - srcTraceCalFactor, targetLevel + measTraceCalFactor1, tolerance, expectedGain, expectedP1dB, expectedP3dB,
                                            settleTime, 10, centerFreq, ref Pin[siteNum], ref Pout[siteNum], ref Pout1dB[siteNum], ref Pout3dB[siteNum], ref gainDeltaX[siteNum]);

                        Pin[siteNum] = Pin[siteNum] + srcTraceCalFactor;
                        RX_Pout[siteNum] = Pout[siteNum] - measTraceCalFactor1;
                        ISO[siteNum] = Math.Abs(RX_Pout[siteNum] - Pin[siteNum]);

                    }
                    if (writeToDatalog)
                    {

                        LogTestResult(testNumber++, ISO);

                    }

                    SigGen1Hal.Level = SigGen1Hal.LevelMin;
                }
            }
            catch (Exception ex)
            {
                trace.TraceError(testNumber + " : " + ex.Message + "\n" + ex.StackTrace);
            }

        }

        public void Measure_CW_Tests(int testNumber, GPIO mode, SETUP measureIdle, SETUP measureSetup, SETUP measureDC, SETUP harmonic, double startPin,
                                   double targetLevel, double tolerance, double expectedGain, double expectedP1dB, double expectedP3dB, SERVO servoType, double centerFreq, double settleTime, double digitizerLevel,
                                   MTRF100.SigGen srcSelect, string srcPin, MTRF100.SrcAmpEnable srcAmpSelect, string measPin, MTRF100.AmpState measAmpSelect100, string modMode, double calLevel,
                                   MTRF200.MeasAmpEnable measAmpSelect200, MTRF200.MeasPathFilterAttenBypass measPathSelect, double attenuation, MTRF200.DownConverterState downConvState, SrcMeasAmpConfig measAmpCalPath)

        {
            try
            {
                if (!isOSPass) return;

                trace.TraceInformation(testNumber + "  Measure_CW_Tests");
                srcCalFactor = 0;
                measCalFactor = 0;

                double[] measPower = new double[0];
                double[] PgainCal = new double[ActiveSites.Length];
                double[] PinCal = new double[ActiveSites.Length];

                double srcTraceCalFactor = 0;
                double measTraceCalFactor = 0;

                Pin[0] = 9999;
                Pin[1] = 9999;
                Pout[0] = 9999;
                Pout[1] = 9999;
                Pout1dB[0] = 9999;
                Pout1dB[1] = 9999;
                Pout3dB[0] = 9999;
                Pout3dB[1] = 9999;
                VCC1_ICC[0] = 9999;
                VCC1_ICC[1] = 9999;
                VCC2_ICC[0] = 0;
                VCC2_ICC[1] = 0;
                VDD_ICC[0] = 9999;
                VDD_ICC[1] = 9999;
                Total_ICC[0] = 9999;
                Total_ICC[1] = 9999;
                Vdet[0] = 9999;
                Vdet[1] = 9999;
                VdetOff[0] = 9999;
                VdetOff[1] = 9999;
                GainPout1dB[0] = 9999;
                GainPout1dB[1] = 9999;
                GainPout3dB[0] = 9999;
                GainPout3dB[1] = 9999;
                Pinoffset[0] = 9999;
                Pinoffset[1] = 9999;
                Poutoffset[0] = 9999;
                Poutoffset[1] = 9999;


                //DigitizerHal.InputLevel = -20;
                DigitizerHal.InputLevel = digitizerLevel;

                for (int siteNum = 0; siteNum < ActiveSites.Length; siteNum++)
                {
                    if (!ActiveSites[siteNum])
                        continue;

                    if (SETUP.DC_IDLE == measureIdle)
                    {
                        MeasureIdle(testNumber, siteNum);
                    }

                    if (SETUP.DC_SETUP == measureSetup)
                    {
                        DcSetup(testNumber);
                    }

                    if (useTraceCalData)
                    {
                        srcTraceCalFactor = TraceCal.GetTraceLossFactor(centerFreq / 1e6, srcPin, siteNum + 1);
                        measTraceCalFactor = TraceCal.GetTraceLossFactor(centerFreq / 1e6, measPin, siteNum + 1);
                    }

                    //rf100.ConnectSourcePort(srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect);
                    //rf100.ConnectMeasurePort(MVPs[measPin][siteNum].srcMeasPort, measAmpSelect100);

                    rf100.ConnectSourceAndMeasurePort(srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect, MVPs[measPin][siteNum].srcMeasPort, measAmpSelect100);
                    rf200.ConnectMeasurePort(MVPs[measPin][siteNum].measPort, measAmpSelect200, measPathSelect, attenuation, downConvState, MTRF200.DigitizerState.Digitizer1);
                    srcCalFactor = PortModuleCal.GetSrcCalFactorNew(centerFreq, srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect, modMode, currentWaveform, calLevel);
                    measCalFactor = PortModuleCal.GetMeasCalFactorNew(centerFreq, MVPs[measPin][siteNum].SystemPort, measPathSelect, measAmpCalPath, modMode, currentWaveform, calLevel);

                    SigGen1Hal.Frequency = centerFreq;
                    DigitizerHal.Frequency = centerFreq;

                    int servoStatus = -1;

                    servoStatus = Servo(servoType, startPin - srcTraceCalFactor, targetLevel + measTraceCalFactor, tolerance, expectedGain, expectedP1dB, expectedP3dB,
                                        settleTime, 20, centerFreq, ref Pin[siteNum], ref Pout[siteNum], ref Pout1dB[siteNum], ref Pout3dB[siteNum], ref gainDelta[siteNum]);

                    Pin[siteNum] = Pin[siteNum] + srcTraceCalFactor;
                    Pout[siteNum] = Pout[siteNum] - measTraceCalFactor;
                    if (centerFreq == 2442000000 && mode == GPIO.WLAN_GAIN_RX)
                    { Gain_2442_RX[siteNum] = Pout[siteNum] - Pin[siteNum]; }
                    if (centerFreq == 2442000000 && mode == GPIO.WLAN_GAIN_TX)
                    { Gain_2442_TX[siteNum] = Pout[siteNum] - Pin[siteNum]; }
                    if (centerFreq == 2442000000 && mode == GPIO.WLAN_GAIN_RX_BYPASS)
                    { Gain_2442_RXBP[siteNum] = Pout[siteNum] - Pin[siteNum]; }

                    Pout1dB[siteNum] = Pout1dB[siteNum] - measTraceCalFactor;
                    Pout3dB[siteNum] = Pout3dB[siteNum] - measTraceCalFactor;

                    gain[siteNum] = Pout[siteNum] - Pin[siteNum];

                    if (mode == GPIO.WLAN_GAIN_RX && centerFreq == 5150e6 && servoType == SERVO.SERVO_NONE)
                    {
                        Gainoffset[siteNum] = RetriveOffset(testNumber+1, siteNum + 1);
                        gain[siteNum] = gain[siteNum] + Gainoffset[siteNum];
                        RX_5150[siteNum] = gain[siteNum];

                    }
                    else if (mode == GPIO.WLAN_GAIN_RX && centerFreq == 5500e6 && servoType == SERVO.SERVO_NONE)
                    {
                        Gainoffset[siteNum] = RetriveOffset(testNumber, siteNum + 1);
                        gain[siteNum] = gain[siteNum] + Gainoffset[siteNum];
                        RX_5500[siteNum] = gain[siteNum];
                        RX_5500_Pout[siteNum] = Pout[siteNum];
                    }
                    else if (mode == GPIO.WLAN_GAIN_RX && centerFreq == 5850e6 && servoType == SERVO.SERVO_NONE)
                    {
                        Gainoffset[siteNum] = RetriveOffset(testNumber, siteNum + 1);
                        gain[siteNum] = gain[siteNum] + Gainoffset[siteNum];
                        RX_5850[siteNum] = gain[siteNum];

                    }
                    else if (mode == GPIO.WLAN_GAIN_RX && centerFreq == 5925e6 && servoType == SERVO.SERVO_NONE)
                    {
                        Gainoffset[siteNum] = RetriveOffset(testNumber, siteNum + 1);
                        gain[siteNum] = gain[siteNum] + Gainoffset[siteNum];
                        RX_5925[siteNum] = gain[siteNum];
                        RX_Flatness[siteNum] = Math.Max(Math.Max(Math.Max(RX_5150[siteNum], RX_5500[siteNum]), RX_5850[siteNum]), RX_5925[siteNum]) - Math.Min(Math.Min(Math.Min(RX_5150[siteNum], RX_5500[siteNum]), RX_5850[siteNum]), RX_5925[siteNum]);


                    }
                    else if (mode == GPIO.WLAN_GAIN_RX && centerFreq == 2410e6 && servoType == SERVO.SERVO_NONE)
                    {
                        Gainoffset[siteNum] = RetriveOffset(testNumber, siteNum + 1);
                        gain[siteNum] = gain[siteNum] + Gainoffset[siteNum];
                        RX_2410[siteNum] = gain[siteNum];

                    }
                    else if (mode == GPIO.WLAN_GAIN_RX && centerFreq == 5500e6 && servoType == SERVO.SERVO_P1DB)
                    {

                        GainPout1dB[siteNum] = gainDelta[siteNum];

                    }
                    else if (mode == GPIO.WLAN_GAIN_RX_BYPASS && centerFreq == 5150e6 && servoType == SERVO.SERVO_NONE)
                    {
                        Gainoffset[siteNum] = RetriveOffset(testNumber+1, siteNum + 1);
                        gain[siteNum] = gain[siteNum] + Gainoffset[siteNum];
                        RX_BYPASS_5150[siteNum] = gain[siteNum];

                    }
                    else if (mode == GPIO.WLAN_GAIN_RX_BYPASS && centerFreq == 5500e6 && servoType == SERVO.SERVO_NONE)
                    {
                        Gainoffset[siteNum] = RetriveOffset(testNumber, siteNum + 1);
                        gain[siteNum] = gain[siteNum] + Gainoffset[siteNum];
                        RX_BYPASS_5500[siteNum] = gain[siteNum];
                        RX_BYPASS_5500_Pout[siteNum] = Pout[siteNum];

                    }
                    else if (mode == GPIO.WLAN_GAIN_RX_BYPASS && centerFreq == 5850e6 && servoType == SERVO.SERVO_NONE)
                    {
                        Gainoffset[siteNum] = RetriveOffset(testNumber, siteNum + 1);
                        gain[siteNum] = gain[siteNum] + Gainoffset[siteNum];
                        RX_BYPASS_5850[siteNum] = gain[siteNum];

                    }
                    else if (mode == GPIO.WLAN_GAIN_RX_BYPASS && centerFreq == 5925e6 && servoType == SERVO.SERVO_NONE)
                    {
                        Gainoffset[siteNum] = RetriveOffset(testNumber, siteNum + 1);
                        gain[siteNum] = gain[siteNum] + Gainoffset[siteNum];
                        RX_BYPASS_5925[siteNum] = gain[siteNum];

                    }
                    else if (mode == GPIO.WLAN_GAIN_TX && centerFreq == 5150e6 && servoType == SERVO.SERVO_NONE)
                    {


                        Gainoffset[siteNum] = RetriveOffset(36, siteNum + 1);
                        gain[siteNum] = gain[siteNum] + Gainoffset[siteNum];
                        TX_5150[siteNum] = TX_5150[siteNum] + Gainoffset[siteNum];
                        TX_5150[siteNum] = gain[siteNum];
                        
                    }
                    else if (mode == GPIO.WLAN_GAIN_TX && centerFreq == 5500e6 && servoType == SERVO.SERVO_NONE)
                    {
                        Gainoffset[siteNum] = RetriveOffset(testNumber, siteNum + 1);
                        gain[siteNum] = gain[siteNum] + Gainoffset[siteNum];
                        TX_5500[siteNum] = gain[siteNum];
                        TX_5500_Pout[siteNum] = Pout[siteNum];
                    }
                    else if (mode == GPIO.WLAN_GAIN_TX && centerFreq == 5850e6 && servoType == SERVO.SERVO_NONE)
                    {

                        Gainoffset[siteNum] = RetriveOffset(testNumber, siteNum + 1);
                        gain[siteNum] = gain[siteNum] + Gainoffset[siteNum];
                        TX_5850[siteNum] = gain[siteNum];
                       
                    }
                    else if (mode == GPIO.WLAN_GAIN_TX && centerFreq == 3800e6 && servoType == SERVO.SERVO_NONE)
                    {

                        Gainoffset[siteNum] = RetriveOffset(testNumber, siteNum + 1);
                        gain[siteNum] = gain[siteNum] + Gainoffset[siteNum];
                        TX_3800[siteNum] = gain[siteNum];
                       
                    }
                    else if (mode == GPIO.WLAN_GAIN_TX && centerFreq == 2600e6 && servoType == SERVO.SERVO_NONE)
                    {
                        Gainoffset[siteNum] = RetriveOffset(testNumber, siteNum + 1);
                        gain[siteNum] = gain[siteNum] + Gainoffset[siteNum];
                        TX_2600[siteNum] = gain[siteNum];
                    }
                    else if (mode == GPIO.WLAN_GAIN_TX && centerFreq == 5925e6 && servoType == SERVO.SERVO_NONE)
                    {


                        Gainoffset[siteNum] = RetriveOffset(testNumber, siteNum + 1);
                        gain[siteNum] = gain[siteNum] + Gainoffset[siteNum];
                        TX_5925[siteNum] = gain[siteNum];
                        TX_Flatness[siteNum] = Math.Max(Math.Max(Math.Max(TX_5150[siteNum], TX_5500[siteNum]), TX_5850[siteNum]), TX_5925[siteNum]) - Math.Min(Math.Min(Math.Min(TX_5150[siteNum], TX_5500[siteNum]), TX_5850[siteNum]), TX_5925[siteNum]);
                    }
                    else if (mode == GPIO.WLAN_GAIN_TX && centerFreq == 5500e6 && servoType == SERVO.SERVO_P1DB)
                    {
                        GainPout1dB[siteNum] = gainDelta[siteNum];
                    }
                    else if (mode == GPIO.WLAN_GAIN_TX && centerFreq == 5500e6 && servoType == SERVO.SERVO_P3DB)
                    {
                        GainPout3dB[siteNum] = gainDelta[siteNum];
                    }
                    else
                    {
                        trace.TraceInformation(testNumber + "  Measure_CW_Tests has invalid mode!");
                    }
                }

                if (writeToDatalog)
                {
                    if (mode == GPIO.WLAN_GAIN_RX && centerFreq == 5150e6 && servoType == SERVO.SERVO_NONE)
                    {

                        LogTestResult(testNumber++, VDD_ICC);
                        LogTestResult(testNumber++, gain);

                    }
                    else if (mode == GPIO.WLAN_GAIN_RX && centerFreq == 5500e6 && servoType == SERVO.SERVO_NONE)
                    {

                        LogTestResult(testNumber++, gain);

                    }
                    else if (mode == GPIO.WLAN_GAIN_RX && centerFreq == 5850e6 && servoType == SERVO.SERVO_NONE)
                    {

                        LogTestResult(testNumber++, gain);

                    }
                    else if (mode == GPIO.WLAN_GAIN_RX && centerFreq == 5925e6 && servoType == SERVO.SERVO_NONE)
                    {


                        LogTestResult(testNumber++, gain);
                        LogTestResult(testNumber++, RX_Flatness);

                    }
                    else if (mode == GPIO.WLAN_GAIN_RX && centerFreq == 2410e6 && servoType == SERVO.SERVO_NONE)
                    {

                        LogTestResult(testNumber++, gain);

                    }
                    else if (mode == GPIO.WLAN_GAIN_RX && centerFreq == 5500e6 && servoType == SERVO.SERVO_P1DB)
                    {

                        LogTestResult(testNumber++, Pin);
                        LogTestResult(testNumber++, GainPout1dB);

                    }
                    else if (mode == GPIO.WLAN_GAIN_RX_BYPASS && centerFreq == 5150e6 && servoType == SERVO.SERVO_NONE)
                    {
                        LogTestResult(testNumber++, VDD_ICC);
                        LogTestResult(testNumber++, gain);

                    }
                    else if (mode == GPIO.WLAN_GAIN_RX_BYPASS && centerFreq == 5500e6 && servoType == SERVO.SERVO_NONE)
                    {
                        LogTestResult(testNumber++, gain);

                    }
                    else if (mode == GPIO.WLAN_GAIN_RX_BYPASS && centerFreq == 5850e6 && servoType == SERVO.SERVO_NONE)
                    {
                        LogTestResult(testNumber++, gain);

                    }
                    else if (mode == GPIO.WLAN_GAIN_RX_BYPASS && centerFreq == 5925e6 && servoType == SERVO.SERVO_NONE)
                    {
                        LogTestResult(testNumber++, gain);

                    }
                    else if (mode == GPIO.WLAN_GAIN_TX && centerFreq == 5150e6 && servoType == SERVO.SERVO_NONE)
                    {

                        LogTestResult(testNumber++, VCC1_ICC);
                        LogTestResult(testNumber++, VCC2_ICC);
                        LogTestResult(testNumber++, VDD_ICC);
                        LogTestResult(testNumber++, Total_ICC);
                        LogTestResult(testNumber++, VDET_NORF);
                        //todo:
                        testNumber = 36;
                        LogTestResult(testNumber++, gain);

                    }
                    else if (mode == GPIO.WLAN_GAIN_TX && centerFreq == 5500e6 && servoType == SERVO.SERVO_NONE)
                    {
                        LogTestResult(testNumber++, gain);
                    }
                    else if (mode == GPIO.WLAN_GAIN_TX && centerFreq == 5850e6 && servoType == SERVO.SERVO_NONE)
                    {
                        LogTestResult(testNumber++, gain);
                    }
                    else if (mode == GPIO.WLAN_GAIN_TX && centerFreq == 5925e6 && servoType == SERVO.SERVO_NONE)
                    {
                        LogTestResult(testNumber++, gain);
                        testNumber = 42;
                        LogTestResult(testNumber++, TX_Flatness);
                    }
                    else if (mode == GPIO.WLAN_GAIN_TX && centerFreq == 3800e6 && servoType == SERVO.SERVO_NONE)
                    {
                        LogTestResult(testNumber++, gain);
                    }
                    else if (mode == GPIO.WLAN_GAIN_TX && centerFreq == 2600e6 && servoType == SERVO.SERVO_NONE)
                    {
                        LogTestResult(testNumber++, gain);
                    }
                    else if (mode == GPIO.WLAN_GAIN_TX && centerFreq == 5500e6 && servoType == SERVO.SERVO_P1DB)
                    {
                        LogTestResult(testNumber++, Pout1dB);
                        LogTestResult(testNumber++, GainPout1dB);
                        //LogTestResult(testNumber++, Pout3dB);
                        //LogTestResult(testNumber++, GainPout3dB);
                    }
                    else if (mode == GPIO.WLAN_GAIN_TX && centerFreq == 5500e6 && servoType == SERVO.SERVO_P3DB)
                    {

                        LogTestResult(testNumber++, Pout3dB);
                        LogTestResult(testNumber++, GainPout3dB);
                    }
                    else
                    {
                        trace.TraceInformation(testNumber + "  Measure_CW_Tests has invalid mode!");
                    }
                }

                SigGen1Hal.Level = SigGen1Hal.LevelMin;
            }


            catch (Exception ex)
            {
                trace.TraceError(testNumber + " : " + ex.Message + "\n" + ex.StackTrace);
            }
        }

        public void Measure_VDET_ICC(int testNumber, SETUP measureIdle, SETUP measureSetup, SETUP measureRfOn, double startPin, double targetLevel,
                                              double tolerance, double expectedGain, SERVO ServoType, double centerFreq, double settleTime, double digitizerLevel, double sampleRate,
                                              MTRF100.SigGen srcSelect, string srcPin, MTRF100.SrcAmpEnable srcAmpSelect, string measPin, MTRF100.AmpState measAmpSelect100, string modMode, double calLevel,
                                              MTRF200.MeasAmpEnable measAmpSelect200, MTRF200.MeasPathFilterAttenBypass measPathSelect, double attenuation, MTRF200.DownConverterState downConvState, SrcMeasAmpConfig measAmpCalPath, double VDET_TARGET)
        {
            try
            {
                if (!isOSPass) return;

                trace.TraceInformation(testNumber + "  Measure_VDET_ICC");
                int testNumberStart = testNumber;
                srcCalFactor = 0;
                measCalFactor = 0;

                double expectedP1dB = 0;
                double expectedP3dB = 0;

                double srcTraceCalFactor = 0;
                double measTraceCalFactor = 0;

                Pin[0] = 9999;
                Pin[1] = 9999;
                Pout[0] = 9999;
                Pout[1] = 9999;
                evm[0] = 9999;
                evm[1] = 9999;
                evmdB[0] = 9999;
                evmdB[1] = 9999;
                Pout[0] = 9999;
                Pout[1] = 9999;
                VDET[0] = 9999;
                VDET[1] = 9999;


                VDET_IDEALPOUT_5[0] = 9999;
                VDET_IDEALPOUT_5[1] = 9999;

                VDET_IDEALPOUT_7[0] = 9999;
                VDET_IDEALPOUT_7[1] = 9999;


                MEASURE_IDEAL_POUT_5[0] = 9999;
                MEASURE_IDEAL_POUT_5[1] = 9999;

                MEASURE_IDEAL_POUT_7[0] = 9999;
                MEASURE_IDEAL_POUT_7[1] = 9999;


                double[] measPower = new double[0];

                SigGen1Hal.Frequency = centerFreq;
                DigitizerHal.Frequency = centerFreq;

                DigitizerHal.InputLevel = digitizerLevel;

                for (int siteNum = 0; siteNum < ActiveSites.Length; siteNum++)
                {

                    if (!ActiveSites[siteNum])
                        continue;

                    if (SETUP.DC_IDLE == measureIdle)
                    {
                        MeasureIdle(testNumber, siteNum);
                    }

                    if (SETUP.DC_SETUP == measureSetup)
                    {
                        DcSetup(testNumber);
                    }

                    if (useTraceCalData)
                    {
                        srcTraceCalFactor = TraceCal.GetTraceLossFactor(centerFreq / 1e6, srcPin, siteNum + 1);
                        measTraceCalFactor = TraceCal.GetTraceLossFactor(centerFreq / 1e6, measPin, siteNum + 1);
                    }


                    //rf100.ConnectSourcePort(srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect);
                    //rf100.ConnectMeasurePort(MVPs[measPin][siteNum].srcMeasPort, measAmpSelect100);

                    rf100.ConnectSourceAndMeasurePort(srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect, MVPs[measPin][siteNum].srcMeasPort, measAmpSelect100);
                    rf200.ConnectMeasurePort(MVPs[measPin][siteNum].measPort, measAmpSelect200, measPathSelect, attenuation, downConvState, MTRF200.DigitizerState.Digitizer1);
                    srcCalFactor = PortModuleCal.GetSrcCalFactorNew(centerFreq, srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect, modMode, currentWaveform, calLevel);
                    measCalFactor = PortModuleCal.GetMeasCalFactorNew(centerFreq, MVPs[measPin][siteNum].SystemPort, measPathSelect, measAmpCalPath, modMode, currentWaveform, calLevel);

                    Poutoffset[siteNum] = RetriveOffset(testNumber, siteNum + 1);

                    int servoStatus = -1;

                    targetLevel = targetLevel + Poutoffset[siteNum];

                    // Temporarily adding a fixed trigger delay ( positive ) of 30us to get around a Rfmx WLAN issue ( it doesn't work with large delays before  burst )
                    //DigitizerHal.TriggerDelay = 30e-6;
                    servoStatus = Servo(ServoType, startPin - srcTraceCalFactor, targetLevel + measTraceCalFactor, tolerance, expectedGain, expectedP1dB, expectedP3dB,
                                        settleTime, 10, centerFreq, ref Pin[siteNum], ref Pout[siteNum], ref Pout1dB[siteNum], ref Pout3dB[siteNum], ref gainDeltaX[siteNum]);

                    Pin[siteNum] = Pin[siteNum] + srcTraceCalFactor;
                    Pout[siteNum] = Pout[siteNum] - measTraceCalFactor- Poutoffset[siteNum];
                    gain[siteNum] = Pout[siteNum] - Pin[siteNum];


                    //-------------------Measure ICC-------------------

                    DC.Meas.stopAI();
                    DC.Meas.setTriggerSource("None");
                    DC.Meas.setSampleClockandSamples(50e3, 10);
                    //Util.WaitTime(2e-3);
                    //DC.Meas.setArm();


                    if (SETUP.DC_MEAS == measureRfOn && (targetLevel == 10 || targetLevel == 18 || targetLevel == 22 || targetLevel == 25))
                    {
                        MeasureDC(testNumber, siteNum);
                    }



                    //-------------------Measure VDET-------------------
                    Digital.con_pmu(1, 0, 0);

                    if (targetLevel == 10)
                    {

                        if (siteNum == 0 && ActiveSites[siteNum])
                        {
                            //Digital.cpu_df(1, 1, 0, 0);
                            Digital.cpu_df(1, MVPs["VDET"][siteNum].digPin, 0, 0);
                            Digital.con_pmu(1, MVPs["VDET"][siteNum].digPin, 1);
                            Digital.pmufi(1, 1, 0, 1.0, 0.0);//board number, chip number, double ri, cvh,cvl ;                       
                            Util.WaitTime(2.0e-3);
                            VDET_10[0] = Digital.vmeas(1, MVPs["VDET"][siteNum].digPin);
                            //Util.WaitTime(2e-3);
                            Digital.con_pmu(1, MVPs["VDET"][siteNum].digPin, 0);

                        }
                        if (siteNum == 1 && ActiveSites[siteNum])
                        {
                            Digital.cpu_df(1, MVPs["VDET"][siteNum].digPin, 0, 0);
                            Digital.con_pmu(1, MVPs["VDET"][siteNum].digPin, 1);
                            Digital.pmufi(1, 1, 0, 1.0, 0.0);//board number, chip number, double ri, cvh,cvl ;                       
                            Util.WaitTime(2.0e-3);
                            VDET_10[0] = Digital.vmeas(1, MVPs["VDET"][siteNum].digPin);
                            //Util.WaitTime(2e-3);
                            Digital.con_pmu(1, MVPs["VDET"][siteNum].digPin, 0);
                        }


                    }


                    if (targetLevel == 18)
                    {

                        if (siteNum == 0 && ActiveSites[siteNum])
                        {
                            Digital.cpu_df(1, MVPs["VDET"][siteNum].digPin, 0, 0);
                            Digital.pmufi(1, 1, 0, 1.2, 0.01);//board number, chip number, double ri, cvh,cvl ;
                            Digital.con_pmu(1, MVPs["VDET"][siteNum].digPin, 1);
                            Util.WaitTime(2.0e-3);
                            VDET_18[siteNum] = Digital.vmeas(1, MVPs["VDET"][siteNum].digPin);
                            //Util.WaitTime(2e-3);
                            Digital.con_pmu(1, MVPs["VDET"][siteNum].digPin, 0);
                        }
                        if (siteNum == 1 && ActiveSites[siteNum])
                        {
                            Digital.cpu_df(1, MVPs["VDET"][siteNum].digPin, 0, 0);
                            Digital.pmufi(1, 1, 0, 1.2, 0.01);//board number, chip number, double ri, cvh,cvl ;
                            Digital.con_pmu(1, MVPs["VDET"][siteNum].digPin, 1);
                            Util.WaitTime(2.0e-3);
                            VDET_18[siteNum] = Digital.vmeas(1, MVPs["VDET"][siteNum].digPin);
                            //Util.WaitTime(2e-3);
                            Digital.con_pmu(1, MVPs["VDET"][siteNum].digPin, 0);
                        }


                    }


                    if (targetLevel == 22)
                    {

                        if (siteNum == 0 && ActiveSites[siteNum])
                        {
                            Digital.cpu_df(1, MVPs["VDET"][siteNum].digPin, 0, 0);
                            Digital.pmufi(1, 1, 0, 1.2, 0.01);//board number, chip number, double ri, cvh,cvl ;
                            Digital.con_pmu(1, MVPs["VDET"][siteNum].digPin, 1);
                            Util.WaitTime(2.0e-3);
                            VDET_22[siteNum] = Digital.vmeas(1, MVPs["VDET"][siteNum].digPin);
                            //Util.WaitTime(2e-3);
                            Digital.con_pmu(1, MVPs["VDET"][siteNum].digPin, 0);
                        }
                        if (siteNum == 1 && ActiveSites[siteNum])
                        {
                            Digital.cpu_df(1, MVPs["VDET"][siteNum].digPin, 0, 0);
                            Digital.pmufi(1, 1, 0, 1.2, 0.01);//board number, chip number, double ri, cvh,cvl ;
                            Digital.con_pmu(1, MVPs["VDET"][siteNum].digPin, 1);
                            Util.WaitTime(2.0e-3);
                            VDET_22[siteNum] = Digital.vmeas(1, MVPs["VDET"][siteNum].digPin);
                            //Util.WaitTime(2e-3);
                            Digital.con_pmu(1, MVPs["VDET"][siteNum].digPin, 0);
                        }


                    }


                    if (targetLevel == 25)
                    {

                        if (siteNum == 0 && ActiveSites[siteNum])
                        {
                            Digital.cpu_df(1, MVPs["VDET"][siteNum].digPin, 0, 0);
                            Digital.pmufi(1, 1, 0, 3.0, 0.01);//board number, chip number, double ri, cvh,cvl ;
                            Digital.con_pmu(1, MVPs["VDET"][siteNum].digPin, 1);
                            Util.WaitTime(2.0e-3);
                            VDET_25[siteNum] = Digital.vmeas(1, MVPs["VDET"][siteNum].digPin);
                            //Util.WaitTime(2e-3);
                            Digital.con_pmu(1, MVPs["VDET"][siteNum].digPin, 0);
                        }
                        if (siteNum == 1 && ActiveSites[siteNum])
                        {
                            Digital.cpu_df(1, MVPs["VDET"][siteNum].digPin, 0, 0);
                            Digital.pmufi(1, 1, 0, 3.0, 0.01);//board number, chip number, double ri, cvh,cvl ;
                            Digital.con_pmu(1, MVPs["VDET"][siteNum].digPin, 1);
                            Util.WaitTime(2.0e-3);
                            VDET_25[siteNum] = Digital.vmeas(1, MVPs["VDET"][siteNum].digPin);
                            //Util.WaitTime(2e-3);
                            Digital.con_pmu(1, MVPs["VDET"][siteNum].digPin, 0);
                        }


                        SigGen1Hal.Level = SigGen1Hal.LevelMin;

                        VDET_SLOPE[siteNum] = (VDET_22[siteNum] - VDET_10[siteNum]) / 15;

                        //VDET_INTERCEPT[siteNum] = VDET_25[siteNum] - (VDET_SLOPE[siteNum] * 25);
                    }


                    if (writeToDatalog)
                    {
                        testNumber = testNumberStart;
                        if (targetLevel == 10)
                        {
                            LogTestResult(testNumber++, Pout);
                            LogTestResult(testNumber++, Total_ICC);
                            LogTestResult(testNumber++, VDET_10);
                        }

                        if (targetLevel == 18)
                        {
                            LogTestResult(testNumber++, Pout);
                            LogTestResult(testNumber++, Total_ICC);
                            LogTestResult(testNumber++, VDET_18);
                        }

                        if (targetLevel == 22)
                        {
                            LogTestResult(testNumber++, Pout);
                            LogTestResult(testNumber++, Total_ICC);
                            LogTestResult(testNumber++, VDET_22);
                        }

                        if (targetLevel == 25)
                        {
                            LogTestResult(testNumber++, Pout);
                            LogTestResult(testNumber++, Total_ICC);
                            LogTestResult(testNumber++, VDET_25);
                            LogTestResult(testNumber++, VDET_SLOPE);
                            //LogTestResult(testNumber++, VDET_INTERCEPT);
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                trace.TraceError(testNumber + " : " + ex.Message + "\n" + ex.StackTrace);
            }
        }

        public void Measure_Noise_Figure(int testNumber, double centerFreq, double settleTime, double digitizerLevel,
                                         MTRF100.SigGen srcSelect, string srcPin, MTRF100.SrcAmpEnable srcAmpSelect, string measPin, MTRF100.AmpState measAmpSelect100,
                                         MTRF200.MeasAmpEnable measAmpSelect200, MTRF200.MeasPathFilterAttenBypass measPathSelect, double attenuation, MTRF200.DownConverterState downConvState)

        {
            try
            {
                if (!isOSPass) return;

                trace.TraceInformation(testNumber + "  Measure_Noise_Figure");
                int testNumberStart = testNumber;

                noiseGain[0] = double.NaN;
                noiseGain[1] = double.NaN;
                noiseFigure[0] = double.NaN;
                noiseFigure[1] = double.NaN;

                for (int siteNum = 0; siteNum < ActiveSites.Length; siteNum++)
                {
                    if (!ActiveSites[siteNum])
                        continue;

                    rf100.ConnectSourcePort(srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect);
                    rf100.ConnectMeasurePort(MVPs[measPin][siteNum].srcMeasPort, measAmpSelect100);


                    rf100.ConnectSourceAndMeasurePort(srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect, MVPs[measPin][siteNum].srcMeasPort, measAmpSelect100);
                    rf200.ConnectMeasurePort(MVPs[measPin][siteNum].measPort, measAmpSelect200, measPathSelect, attenuation, downConvState, MTRF200.DigitizerState.Digitizer1);

                    DigitizerHal.Frequency = centerFreq;

                    DigitizerHal.InputLevel = digitizerLevel;

                    ////-----------------------------------------------Measure Noise Figure--------------------------------------------------------------------

                    SigGen1Hal.Level = SigGen1Hal.LevelMin;

                    Util.WaitTime(settleTime);

                    nf.SetupMeasureNoiseFigure(centerFreq, MVPs[srcPin][siteNum].srcPort, MVPs[measPin][siteNum].srcMeasPort, false, MeasAmpEnable.A1);
                    nf.MeasureNoiseFigure(centerFreq, out noiseFigure[siteNum], out noiseGain[siteNum]);

                    if (double.IsInfinity(noiseFigure[siteNum]) || noiseFigure[siteNum].Equals(double.NaN))
                    {
                        noiseFigure[siteNum] = -9999;
                    }
                    rf200.ConnectMeasurePort(MVPs[measPin][siteNum].measPort, MTRF200.MeasAmpEnable.None, measPathSelect, attenuation, downConvState, MTRF200.DigitizerState.Digitizer1);

                    
                }
                if (writeToDatalog)
                {

                    testNumber = testNumberStart;
                    LogTestResult(testNumber++, noiseFigure);
                }
            }

            catch (Exception ex)
            {
                trace.TraceError(testNumber + " : " + ex.Message + "\n" + ex.StackTrace);
            }
        }
        
        
        public void Measure_WLAN(int testNumber, SETUP measureIdle, SETUP measureSetup, SETUP measureRfOn, double startPin, double targetLevel,
                                 double tolerance, double expectedGain, SERVO ServoType, double centerFreq, double settleTime, double digitizerLevel, double sampleRate,
                                 MTRF100.SigGen srcSelect, string srcPin, MTRF100.SrcAmpEnable srcAmpSelect, string measPin, MTRF100.AmpState measAmpSelect100, string modMode, double calLevel,
                                 MTRF200.MeasAmpEnable measAmpSelect200, MTRF200.MeasPathFilterAttenBypass measPathSelect, double attenuation, MTRF200.DownConverterState downConvState, SrcMeasAmpConfig measAmpCalPath,
                                 WlanStandard AnalysisType, WlanBandwidth ChanBandwidth, ChannelEstimationType ReferenceType)
        {
            try
            {
                if (!isOSPass) return;

                trace.TraceInformation(testNumber + "  Measure_WLAN");
                int testNumberStart = testNumber;
                srcCalFactor = 0;
                measCalFactor = 0;

                double expectedP1dB = 0;
                double expectedP3dB = 0;

                double srcTraceCalFactor = 0;
                double measTraceCalFactor = 0;

                Pin[0] = 9999;
                Pin[1] = 9999;
                Pout[0] = 9999;
                Pout[1] = 9999;
                evm[0] = 9999;
                evm[1] = 9999;
                evmdB[0] = 9999;
                evmdB[1] = 9999;
                Pout[0] = 9999;
                Pout[1] = 9999;
                double[] measPower = new double[0];

                SetupWlanAnalysis(testNumber, centerFreq, "WLAN", sampleRate, WlanMeasurementTypes.OfdmModAcc, ChanBandwidth, AnalysisType, ReferenceType);

                //////-------------DEVM(Vramp arm)--------------------------------------------
                //DC.Meas.setTriggerSource("PXI_Trig1");
                //DC.Meas.setSampleClockandSamples(100e3, 10);

                SigGen1Hal.Frequency = centerFreq;
                DigitizerHal.Frequency = centerFreq;

                DigitizerHal.InputLevel = digitizerLevel;

                for (int siteNum = 0; siteNum < ActiveSites.Length; siteNum++)
                {

                    if (!ActiveSites[siteNum])
                        continue;

                    if (SETUP.DC_IDLE == measureIdle)
                    {
                        MeasureIdle(testNumber, siteNum);
                    }

                    if (SETUP.DC_SETUP == measureSetup)
                    {
                        DcSetup(testNumber);
                    }

                    if (useTraceCalData)
                    {
                        srcTraceCalFactor = TraceCal.GetTraceLossFactor(centerFreq / 1e6, srcPin, siteNum + 1);
                        measTraceCalFactor = TraceCal.GetTraceLossFactor(centerFreq / 1e6, measPin, siteNum + 1);
                    }


                    //rf100.ConnectSourcePort(srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect);
                    //rf100.ConnectMeasurePort(MVPs[measPin][siteNum].srcMeasPort, measAmpSelect100);

                    rf100.ConnectSourceAndMeasurePort(srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect, MVPs[measPin][siteNum].srcMeasPort, measAmpSelect100);
                    rf200.ConnectMeasurePort(MVPs[measPin][siteNum].measPort, measAmpSelect200, measPathSelect, attenuation, downConvState, MTRF200.DigitizerState.Digitizer1);


                    srcCalFactor = PortModuleCal.GetSrcCalFactorNew(centerFreq, srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect, modMode, currentWaveform, calLevel);
                    measCalFactor = PortModuleCal.GetMeasCalFactorNew(centerFreq, MVPs[measPin][siteNum].SystemPort, measPathSelect, measAmpCalPath, modMode, currentWaveform, calLevel);

                    Poutoffset[siteNum] = RetriveOffset(testNumber, siteNum + 1);
                    int servoStatus = -1;

                    // Temporarily adding a fixed trigger delay ( positive ) of 30us to get around a Rfmx WLAN issue ( it doesn't work with large delays before  burst )
                    //DigitizerHal.TriggerDelay = 30e-6;

                    targetLevel = targetLevel + Poutoffset[siteNum];

                    servoStatus = Servo(ServoType, startPin - srcTraceCalFactor, targetLevel + measTraceCalFactor, tolerance, expectedGain, expectedP1dB, expectedP3dB,
                                            settleTime, 10, centerFreq, ref Pin[siteNum], ref Pout[siteNum], ref Pout1dB[siteNum], ref Pout3dB[siteNum], ref gainDeltaX[siteNum]);

                    Pin[siteNum] = Pin[siteNum] + srcTraceCalFactor;
                    Pout[siteNum] = Pout[siteNum] - measTraceCalFactor-Poutoffset[siteNum];
                    gain[siteNum] = Pout[siteNum] - Pin[siteNum];

                    //Util.WaitTime(20e-3);
                    //-------------DEVM(Set Vramp Waveform)--------------------------------
                    SigGen1Hal.StopWaveform();
                    DC.setVoltage(siteNum + 1, 0); //Set TX-enable@1 to 0;
                    Util.WaitTime(2e-3);
                    
                    if(targetLevel == 18)
                    {

                        SetupVramp(0, 4030, 4700, 30, 30, 3.3, 3.3, 0, 0, siteNum + 1);
                    }
                    else 
                    { SetupVramp(0, 330, 1000, 30, 30, 3.3, 3.3, 0, 0, siteNum + 1); }
                    //SetupVramp(0, 380, 1000, 60, 30, 3.3, 3.3, 0, 0, siteNum + 1);
                    DC.LPSource.setArm();


                    //-------------DEVM(Set digitizer arm - playwaveform - analysis IQ)---- -

                    DigitizerHal.CaptureIQAsync();
                    SigGen1Hal.PlayWaveform();

                    DigitizerHal.FetchIQ(ref wlanAnalysis.IData, ref wlanAnalysis.QData);

                    wlanAnalysis.AnalyzeIQ();
                    evmdB[siteNum] = wlanAnalysis.RmsEvmMeanDb;
                    if (double.IsInfinity(evmdB[siteNum]) || evmdB[siteNum].Equals(double.NaN))
                    {
                        evmdB[siteNum] = -9999;
                    }
                }

                if (ServoType == SERVO.SERVO_NONE)
                {
                    LogTestResult(testNumber++, evmdB);
                }
                else
                {
                    LogTestResult(testNumber++, Pout);
                    LogTestResult(testNumber++, evmdB);
                }

                SigGen1Hal.Level = SigGen1Hal.LevelMin;
            }
            catch (Exception ex)
            {
                trace.TraceError(testNumber + " : " + ex.Message + "\n" + ex.StackTrace);
            }
        }

        public void Measure_RX_WLAN(int testNumber, SETUP measureIdle, SETUP measureSetup, SETUP measureRfOn, double startPin, double targetLevel,
                                double tolerance, double expectedGain, SERVO ServoType, double centerFreq, double settleTime, double digitizerLevel, double sampleRate,
                                MTRF100.SigGen srcSelect, string srcPin, MTRF100.SrcAmpEnable srcAmpSelect, string measPin, MTRF100.AmpState measAmpSelect100, string modMode, double calLevel,
                                MTRF200.MeasAmpEnable measAmpSelect200, MTRF200.MeasPathFilterAttenBypass measPathSelect, double attenuation, MTRF200.DownConverterState downConvState, SrcMeasAmpConfig measAmpCalPath,
                                WlanStandard AnalysisType, WlanBandwidth ChanBandwidth, ChannelEstimationType ReferenceType)
        {
            try
            {
                if (!isOSPass) return;

                trace.TraceInformation(testNumber + "  Measure_WLAN");
                trace.TraceInformation("Memory in Measure_WLAN header: " + Process.GetCurrentProcess().PrivateMemorySize64 / 1024 / 1024 + "MB");
                int testNumberStart = testNumber;
                srcCalFactor = 0;
                measCalFactor = 0;

                double expectedP1dB = 0;
                double expectedP3dB = 0;

                double srcTraceCalFactor = 0;
                double measTraceCalFactor = 0;

                Pin[0] = 9999;
                Pin[1] = 9999;
                Pout[0] = 9999;
                Pout[1] = 9999;
                evm[0] = 9999;
                evm[1] = 9999;
                evmdB[0] = 9999;
                evmdB[1] = 9999;
                Pout[0] = 9999;
                Pout[1] = 9999;
                double[] measPower = new double[0];

                SetupWlanAnalysis(testNumber, centerFreq, "WLAN", sampleRate, WlanMeasurementTypes.OfdmModAcc, ChanBandwidth, AnalysisType, ReferenceType);

                //////-------------DEVM(Vramp arm)--------------------------------------------
                //DC.Meas.setTriggerSource("PXI_Trig1");
                //DC.Meas.setSampleClockandSamples(100e3, 10);

                SigGen1Hal.Frequency = centerFreq;
                DigitizerHal.Frequency = centerFreq;

                DigitizerHal.InputLevel = digitizerLevel;

                for (int siteNum = 0; siteNum < ActiveSites.Length; siteNum++)
                {

                    if (!ActiveSites[siteNum])
                        continue;

                    if (SETUP.DC_IDLE == measureIdle)
                    {
                        MeasureIdle(testNumber, siteNum);
                    }

                    if (SETUP.DC_SETUP == measureSetup)
                    {
                        DcSetup(testNumber);
                    }

                    if (useTraceCalData)
                    {
                        srcTraceCalFactor = TraceCal.GetTraceLossFactor(centerFreq / 1e6, srcPin, siteNum + 1);
                        measTraceCalFactor = TraceCal.GetTraceLossFactor(centerFreq / 1e6, measPin, siteNum + 1);
                    }


                    //rf100.ConnectSourcePort(srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect);
                    //rf100.ConnectMeasurePort(MVPs[measPin][siteNum].srcMeasPort, measAmpSelect100);

                    rf100.ConnectSourceAndMeasurePort(srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect, MVPs[measPin][siteNum].srcMeasPort, measAmpSelect100);
                    rf200.ConnectMeasurePort(MVPs[measPin][siteNum].measPort, measAmpSelect200, measPathSelect, attenuation, downConvState, MTRF200.DigitizerState.Digitizer1);
                    srcCalFactor = PortModuleCal.GetSrcCalFactorNew(centerFreq, srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect, modMode, currentWaveform, calLevel);
                    measCalFactor = PortModuleCal.GetMeasCalFactorNew(centerFreq, MVPs[measPin][siteNum].SystemPort, measPathSelect, measAmpCalPath, modMode, currentWaveform, calLevel);


                    Poutoffset[siteNum] = RetriveOffset(testNumber, siteNum + 1);

                    int servoStatus = -1;

                    // Temporarily adding a fixed trigger delay ( positive ) of 30us to get around a Rfmx WLAN issue ( it doesn't work with large delays before  burst )
                    //DigitizerHal.TriggerDelay = 30e-6;
                    targetLevel = targetLevel + Poutoffset[siteNum];
                    servoStatus = Servo(ServoType, startPin - srcTraceCalFactor, targetLevel + measTraceCalFactor, tolerance, expectedGain, expectedP1dB, expectedP3dB,
                                            settleTime, 10, centerFreq, ref Pin[siteNum], ref Pout[siteNum], ref Pout1dB[siteNum], ref Pout3dB[siteNum], ref gainDeltaX[siteNum]);

                    Pin[siteNum] = Pin[siteNum] + srcTraceCalFactor;
                    Pout[siteNum] = Pout[siteNum] - measTraceCalFactor-Poutoffset[siteNum];
                    gain[siteNum] = Pout[siteNum] - Pin[siteNum];

                    //Util.WaitTime(20e-3);

                    DigitizerHal.FetchIQ(ref wlanAnalysis.IData, ref wlanAnalysis.QData);

                    wlanAnalysis.AnalyzeIQ();
                    evmdB[siteNum] = wlanAnalysis.RmsEvmMeanDb;
                    if (double.IsInfinity(evmdB[siteNum]) || evmdB[siteNum].Equals(double.NaN))
                    {
                        evmdB[siteNum] = -9999;
                    }
                }

                LogTestResult(testNumber++, evmdB);

                SigGen1Hal.Level = SigGen1Hal.LevelMin;
            }
            catch (Exception ex)
            {
                trace.TraceError(testNumber + " : " + ex.Message + "\n" + ex.StackTrace);
            }
        }

        // ================================== Calibration Configuration Data ==================================
        private Dictionary<string, Dictionary<int, PathFactorVer3>> MVPs;
        private void BuildCalDatasVer3MVP()
        {
            MVPs = new Dictionary<string, Dictionary<int, PathFactorVer3>>();
            MVPs.Add("TX", new Dictionary<int, PathFactorVer3>());
            MVPs["TX"].Add(0, new PathFactorVer3() { srcPort = MTRF100.SrcPort.P2, measPort = MTRF200.MeasPort.NOP, SystemPort = SystemPort.NOP });

            MVPs.Add("ANT", new Dictionary<int, PathFactorVer3>());
            MVPs["ANT"].Add(0, new PathFactorVer3() { srcPort = MTRF100.SrcPort.P3, srcMeasPort = MTRF100.MeasPort.P3, measPort = MTRF200.MeasPort.SRC, SystemPort = SystemPort.P3 });

            MVPs.Add("RX", new Dictionary<int, PathFactorVer3>());
            MVPs["RX"].Add(0, new PathFactorVer3() { srcMeasPort = MTRF100.MeasPort.P1, measPort = MTRF200.MeasPort.SRC, SystemPort = SystemPort.P1 });

            MVPs.Add("VDET", new Dictionary<int, PathFactorVer3>());
            MVPs["VDET"].Add(0, new PathFactorVer3() { digPin = 1 });
            MVPs["VDET"].Add(1, new PathFactorVer3() { digPin = 6 });

            MVPs.Add("C1", new Dictionary<int, PathFactorVer3>());
            MVPs["C1"].Add(0, new PathFactorVer3() { digPin = 2 });
            MVPs["C1"].Add(1, new PathFactorVer3() { digPin = 7 });

            MVPs.Add("C0", new Dictionary<int, PathFactorVer3>());
            MVPs["C0"].Add(0, new PathFactorVer3() { digPin = 3 });
            MVPs["C0"].Add(1, new PathFactorVer3() { digPin = 8 });
        }

        private void SetupWlanAnalysis(int testNumber, double centerFreq, string modulationType, double sampleRate,
            WlanMeasurementTypes measurementType, WlanBandwidth ChanBandwidth, WlanStandard AnalysisType, ChannelEstimationType ReferenceType)
        {
            wlanAnalysis.CenterFrequency = centerFreq;
            wlanAnalysis.ChannelBandwidth = ChanBandwidth;
            wlanAnalysis.Measurments = measurementType;
            wlanAnalysis.Standard = AnalysisType;
            wlanAnalysis.AmplitudeTrackingEnabled = false;
            wlanAnalysis.PhaseTrackingEnabled = true;
            wlanAnalysis.TimeTrackingEnabled = false;
            wlanAnalysis.ChannelEstimationType = ReferenceType;
            wlanAnalysis.SampleRate = sampleRate;
        }

        //=================================== Application Setup Functions ====================================

        public int ApplicationLoad() //ApplicationLoad(string pathUserCal, bool isCal)
        {

            try
            {
                trace.TraceInformation("ApplicationLoad");
                int status = -1;

                //-------------------- Read User Calibration Data File --------------------
                status = PortModuleCal.ReadCalDataFile(@"C:\MerlinTest\Production\UserCal\KCT8526HW_RF_Cal_Data.csv");

                if (useTraceCalData)
                {
                    string traceCalTarget = Path.Combine(ProgramDir, "Trace", "RF_Trace_Data.csv");
                    status = TraceCal.ReadCalDataFile(traceCalTarget);
                    if (status != 0) ProgramExit();
                }

                BuildCalDatasVer3MVP();

                if (status != 0) ProgramExit();

                if (status != 0) ProgramExit();

                //-------------------- Initialize Tester --------------------

                DC = ATE.DC;
                PS = ATE.io488PS;
                Digital = ATE.Digital;

                rf100 = new MTRF100("MT-RF100");
                rf100.Initialize();

                rf200 = new MTRF200("MT-RF200");
                rf200.Initialize();

                Matrix = ATE.Matrix;

                if (ATE.SigGen1HAL != null) SigGen1Hal = ATE.SigGen1HAL;
                if (ATE.SigGen2HAL != null) SigGen2Hal = ATE.SigGen2HAL;
                if (ATE.DigitizerHAL1 != null) DigitizerHal = ATE.DigitizerHAL1;

                if (ATE.SigGen1HAL != null)
                {
                    // Set some defaults
                    SigGen1Hal.RfState = true;

                    ATE.SigGen1HAL.DeleteAllWaveforms();

                    // Load some default waveforms.



                    string waveform = Path.Combine(ProgramDir, "Waveforms", "11ac_MCS7_HT40_300us_50%.tdms");
                    string markerFile30usDelay_MCS7 = Path.Combine(ProgramDir, "Waveforms", "Marker1_30usDelay_MCS7.xml");
                    ATE.SigGen1HAL.LoadWaveforms(waveform, markerFile30usDelay_MCS7);


                    waveform = Path.Combine(ProgramDir, "Waveforms", "11ac_MCS9_HT80_300us_50%.tdms");
                    string markerFile30usDelay_MCS9 = Path.Combine(ProgramDir, "Waveforms", "Marker1_30usDelay_MCS9.xml");
                    ATE.SigGen1HAL.LoadWaveforms(waveform, markerFile30usDelay_MCS9);

                    //waveform = Path.Combine(ProgramDir, "Waveforms", "11ac_MCS9_HE80_4msec_50%.tdms");
                    //string markerFile_MCS0 = Path.Combine(ProgramDir, "Waveforms", "Marker1_30usDelay_MCS9.xml");
                    //ATE.SigGen1HAL.LoadWaveforms(waveform, markerFile_MCS0);



                    // Configure shared LO's for VST
                    SigGen1Hal.LOOutEnabled = true;
                    SigGen1Hal.ExternalLO = false;

                    DigitizerHal.LOOutEnabled = false;
                    DigitizerHal.ExternalLO = true;
                }

                //-------------------- Read User Calibration Data File --------------------

                nf.DC = DC;
                nf.Rf100 = rf100;
                nf.Rf200 = rf200;
                nf.dig_HAL = DigitizerHal;
                nf.NoiseSoureCalFile = @"C:\MerlinTest\Production\UserCal\KCT8526HW_NF_Cal_Data.csv";
                nf.ReadNoiseCalDataFile();

                Util.WaitTime(10e-3);

                VNA = new mtVNA();
                VNA.ate = ATE;
                VNA.initialize();
                //VNA.initialize("KCT8526HW");
                double[] calFreqs = new double[2] { 5500e6,5500e6};
                VNA.setFreqRangePoints(calFreqs);

                //------------Low Power avoid spur------------------------------------
                DC.LPSource.stepMode = true;
                DC.LPSource.stepModeStepSize = 0.1;
                //-----------Make sure HP LP can be compensated-----------------------
                DC.Meas.PS = PS;
                DC.Meas.usePSMHPCal = true;
                DC.Meas.usePSMLPCal = true;
                //-------------------- Check 10 MHz Reference Lock --------------------

                bool refLockStatus = false;

                if (SigGen1Hal != null) refLockStatus = SigGen1Hal.Locked;
                if (refLockStatus == false) ProgramExit();

                if (SigGen2Hal != null) refLockStatus = SigGen2Hal.Locked;
                if (refLockStatus == false) ProgramExit();

                if (DigitizerHal != null) refLockStatus = DigitizerHal.Locked;
                if (refLockStatus == false) ProgramExit();

                //-------------------- Configure Digital Instrument --------------------

                string digitalPattern = Path.Combine(ProgramDir, "Digital", "MIPI.pez");
                status = Digital.lmload(1, 1, 0, digitalPattern);


                if (status == -1)
                {
                    throw new Exception("ERROR: Digital pattern does not exist!");
                }

                Digital.set_rz(1, 1, 0x00000000);
                Digital.set_ro(1, 1, 0x00000000);// FS1 RZ:RO=00 =>NF(no format), RZ:RO=10 =>RTZ

                // set TP=10*5ns for all TS///unit is 5ns
                Digital.set_addif(1, 0);//when LMADD<= LMIF compare result will be ignored,no LMADD ignored 

                int TSW = 200;// mipi write 1MHz

                Digital.set_tp(1, 1, TSW);

                int edge1 = 1 * TSW / 10;
                int edge2 = 9 * TSW / 10;
                int edge3 = 6 * TSW / 10;
                int edgeStrobe = 9 * TSW / 10;

                //TS1 write, tstart and tstop should not be 0 or TS.

                Digital.set_tstart(1, 2, 1, edge1);
                Digital.set_tstop(1, 2, 1, edge2);//--C1--

                Digital.set_tstart(1, 3, 1, edge1);
                Digital.set_tstop(1, 3, 1, edge2);//--C0--

                //-----------------------SITE1---------------------------------------
                DC.Meas.enableHP(1); // VDD@1
                DC.Meas.enableHP(2); // VCC2@1
                DC.Meas.enableHP(3); // VDD@2
                DC.Meas.enableHP(4); // VCC2@2

                DC.Meas.enableLP(1); // PAEN@1
                DC.Meas.enableLP(2); // PAEN@2
                DC.Meas.enableLP(3); // VCC1@1
                DC.Meas.enableLP(4); // VCC1@2

                PS.SetOutputEnable(1, true);//VDD@1
                PS.SetOutputEnable(3, true);//VCC2@2
                PS.SetOutputEnable(2, true);//VDD@1
                PS.SetOutputEnable(4, true);//VCC2@2

                SigGen1Hal.SetConnection(RoutingMatrixVsg.ARB_MARKER_2, RoutingMatrixVsg.PXI_TRIG_1);
                SigGen1Hal.SetConnection(RoutingMatrixVsg.ARB_MARKER_1, RoutingMatrixVsg.PXI_TRIG_0);

                DigitizerHal.ExternalTrigger = ExternalTriggerSourceVsa.PXI_TRIG_0;

                //-------------------- Configure Digital Channels--------------------

                //-----------------------SITE1---------------------------------------

                // DET@1
                Digital.set_vil(1, 1, 0.0);
                Digital.set_vih(1, 1, 3.3);


                // C1@1
                Digital.set_vil(1, 2, 0.0);
                Digital.set_vih(1, 2, 3.3);

                // C0@1
                Digital.set_vil(1, 3, 0.0);
                Digital.set_vih(1, 3, 3.3);

                //-----------------------SITE2---------------------------------------

                // C1@1
                Digital.set_vil(1, 6, 0.0);
                Digital.set_vih(1, 6, 3.3);

                // C1@1
                Digital.set_vil(1, 7, 0.0);
                Digital.set_vih(1, 7, 3.3);

                // C0@1
                Digital.set_vil(1, 8, 0.0);
                Digital.set_vih(1, 8, 3.3);


                return status;

            }
            catch (Exception ex)
            {
                trace.TraceError(ex.Message + "\n" + ex.StackTrace);
                return -1;
            }


        }

        public void ApplicationStart()
        {
            try
            {
                trace.TraceInformation("ApplicationLoad");
                // Start program timer
                programTimer.Reset();
                programTimer.Start();

                if (SigGen1Hal != null) SigGen1Hal.Level = SigGen1Hal.LevelMin;
                if (SigGen2Hal != null) SigGen2Hal.Level = SigGen2Hal.LevelMin;

                //-------------------------------Site1-------------------------------------------

                // VDD@1
                PS.ClearProtectionFault(1);
                PS.SetVoltage(1, 0.0);
                PS.SetCurrent(1, 1.0);
                DC.Meas.setHPRange(1, HRange.R_1A);

                // VCC2@2
                PS.ClearProtectionFault(2);
                PS.SetVoltage(2, 0.0);
                PS.SetCurrent(2, 1.0);
                DC.Meas.setHPRange(2, HRange.R_1A);

                // VCC1@1
                DC.LPMeas.enable(3);
                DC.setVoltage(3, 0.0);
                DC.setLPRange(3, LRange.R_200mA);
                DC.Meas.setLPRange(3, LRange.R_200mA);
                
                // PAEN@1
                DC.LPMeas.enable(1);
                DC.setVoltage(1, 0.0);
                DC.setLPRange(1, LRange.R_200mA);


                //-------------------------------Site2-------------------------------------------
                //// VCC@2
                ///PS.ClearProtectionFault(3);
                //PS.SetVoltage(3, 5.0);
                //PS.SetCurrent(3, 1.0);
                //DC.Meas.setHPRange(3, HRange.R_1A);

                // VDD@2
                //DC.LPMeas.enable(4);
                //DC.setVoltage(4, 5.0);
                //DC.setLPRange(4, LRange.R_200mA);
                //DC.Meas.setLPRange(4, LRange.R_200mA);


                //// PAEN@2
                //DC.LPMeas.enable(2);
                //DC.setVoltage(2, 0.0);
                //DC.setLPRange(2, LRange.R_200mA);

                //Util.WaitTime(10e-3);

                isOSPass = true;

                if (writeToConsole)
                {
                    Console.WriteLine("\n==================== Device Number {0} ====================\n", dutNumber);
                }
            }
            catch (Exception ex)
            {
                trace.TraceError(ex.Message + "\n" + ex.StackTrace);

            }
        }

        public void ApplicationEnd()
        {
            try
            {
                trace.TraceInformation("ApplicationLoad");
                if (SigGen1Hal != null) SigGen1Hal.Level = SigGen1Hal.LevelMin;
                if (SigGen2Hal != null) SigGen2Hal.Level = SigGen2Hal.LevelMin;

                double[] testTime = new double[ActiveSites.Length];
                testTime[0] = testTime[1] = 0;

                DC.setVoltage(1, 0.0);//PA_EN@1
                DC.setVoltage(2, 0.0);//PA_EN@2
                DC.setVoltage(3, 0.0);//VCC1@1
                DC.setVoltage(4, 0.0);//VCC1@2

                Digital.cpu_df(1, 2, 1, 0);//C1@1
                Digital.cpu_df(1, 3, 1, 0);//C0@1
                Digital.cpu_df(1, 5, 1, 0);//C1@2
                Digital.cpu_df(1, 6, 1, 0);//C0@2

                PS.SetVoltage(1, 0.0);//VDD@1
                PS.SetVoltage(3, 0.0);//VDD@2
                PS.SetVoltage(2, 0.0);//VCC2@1
                PS.SetVoltage(4, 0.0);//VCC2@2

                programTimer.Stop();
                testTime[0] = testTime[1] = 1e3 * programTimer.ElapsedTicks / (Stopwatch.Frequency * 1e3);
                LogTestResult(87, testTime.ToList());
            }
            catch (Exception ex)
            {
                trace.TraceError(ex.Message + "\n" + ex.StackTrace);

            }
        }

        public int ApplicationUnload()
        {
            int status = -1;

            return status;
        }

        public void Measure_CW_Aging(double startPin, double centerFreq, double settleTime, double digitizerLevel,
            MTRF100.SigGen srcSelect, string srcPin, MTRF100.SrcAmpEnable srcAmpSelect, string measPin, MTRF100.AmpState measAmpSelect100, string modMode, double calLevel,
            MTRF200.MeasAmpEnable measAmpSelect200, MTRF200.MeasPathFilterAttenBypass measPathSelect, double attenuation, MTRF200.DownConverterState downConvState, SrcMeasAmpConfig measAmpCalPath)
        {
            try
            {
                if (!isOSPass) return;

                SigGen1Hal.StopWaveform();
                SigGen1Hal.Mode = VsgMode.CW;

                srcCalFactor = 0;
                double srcTraceCalFactor = 0;

                DigitizerHal.InputLevel = digitizerLevel;

                for (int siteNum = 0; siteNum < ActiveSites.Length; siteNum++)
                {
                    if (!ActiveSites[siteNum])
                        continue;

                    if (useTraceCalData)
                    {
                        srcTraceCalFactor = TraceCal.GetTraceLossFactor(centerFreq / 1e6, srcPin, siteNum + 1);
                    }

                    rf100.ConnectSourceAndMeasurePort(srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect, MVPs[measPin][siteNum].srcMeasPort, measAmpSelect100);
                    rf200.ConnectMeasurePort(MVPs[measPin][siteNum].measPort, measAmpSelect200, measPathSelect, attenuation, downConvState, MTRF200.DigitizerState.Digitizer1);
                    srcCalFactor = PortModuleCal.GetSrcCalFactorNew(centerFreq, srcSelect, MVPs[srcPin][siteNum].srcPort, srcAmpSelect, modMode, "", calLevel);

                    SigGen1Hal.Frequency = centerFreq;
                    DigitizerHal.Frequency = centerFreq;
                    
                    SigGen1Hal.Level = (startPin - srcTraceCalFactor - srcCalFactor);
                    Util.WaitTime(settleTime);
                    Util.WaitTime(50E-3);
                }

                SigGen1Hal.Level = SigGen1Hal.LevelMin;
            }


            catch (Exception ex)
            {
                trace.TraceError("Measure_CW_Aging : " + ex.Message + "\n" + ex.StackTrace);
            }
        }
    }

    public class MVP
    {
        public string Pin;
        public string Port;
        public int Site;
    }

    public class PathFactorVer3
    {
        public MTRF100.SrcPort srcPort;
        public MTRF100.MeasPort srcMeasPort;
        public int digPin;
        public MTRF200.MeasPort measPort;
        public SystemPort SystemPort;
        public Dictionary<SrcAmpEnable, double> srcCalFactors;
        public Dictionary<MeasAmpEnable, double> measCalFactors;
    }

    class Program : ApplicationBaseMrk2
    {
        //======================================== Main Test Program =========================================

        static void Main(string[] args)
        {
            KCT8526HW program = new KCT8526HW();
            program.ATE = new Tester(false);

            int status = program.ATE.Initialize();
            if (status != 0)
            {

                Console.Write("\nprogram.ATE.Initialize() != 0. ");
                Console.ReadKey();
                return;
            }

            program.ActiveSites = new bool[2] { true, false };
            program.ProgramDir = @"C:\MerlinTest\Project\KCT8526HW";
            uint numberOfDeviceRuns = 10;

            program.ApplicationLoad();

            try
            {
                for (uint device = 0; device < numberOfDeviceRuns; device++)
                {
                    program.ApplicationStart();


                    string WLAN_11ac_MCS7_40MHz = @"C:\MerlinTest\Project\KCT8526HW\Waveforms\11ac_MCS7_HT40_300us_50%.tdms";
                    string WLAN_11ac_MCS9_80MHz = @"C:\MerlinTest\Project\KCT8526HW\Waveforms\11ac_MCS9_HT80_300us_50%.tdms";
                    string WLAN_11ac_MCS9_80MHz_4msec = @"C:\MerlinTest\Project\KCT8526HW\Waveforms\11ac_MCS9_HE80_4msec_50%.tdms";

                    program.Measure_OS(1,false);
                    program.Measure_Idle_Current(3,false);
                    program.LoSharing(false);
                    program.DeviceInit(GPIO.WLAN_GAIN_TX);
                    program.Measure_CW_Aging(5,5500e6,2e-3,10, MTRF100.SigGen.SG1,"TX", MTRF100.SrcAmpEnable.A2_A3,"ANT", MTRF100.AmpState.Bypass,"",-20, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.Bypass);
                    program.DeviceInit(GPIO.WLAN_GAIN_RX);
                    program.VsgVsaSetup(1e6,1e-3,1e-3,1e-3,0, TriggerTypeEnum.FreeRun, VsgMode.CW,"");
                    program.Measure_CW_Tests(18, GPIO.WLAN_GAIN_RX, SETUP.DC_IDLE, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF,-25,0,0,0,0,0, SERVO.SERVO_NONE,5150e6,2e-3,-10, MTRF100.SigGen.SG1,"ANT", MTRF100.SrcAmpEnable.A2_A3,"RX", MTRF100.AmpState.Enable,"",-5, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.A4_MEAS);
                    program.Measure_CW_Tests(20, GPIO.WLAN_GAIN_RX, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF,-25,0,0,0,0,0, SERVO.SERVO_NONE,5500e6,2e-3,-10, MTRF100.SigGen.SG1,"ANT", MTRF100.SrcAmpEnable.A2_A3,"RX", MTRF100.AmpState.Enable,"",-5, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.A4_MEAS);
                    program.Measure_CW_Tests(21, GPIO.WLAN_GAIN_RX, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF,-25,0,0,0,0,0, SERVO.SERVO_NONE,5850e6,2e-3,-10, MTRF100.SigGen.SG1,"ANT", MTRF100.SrcAmpEnable.A2_A3,"RX", MTRF100.AmpState.Enable,"",-5, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.A4_MEAS);
                    program.Measure_CW_Tests(22, GPIO.WLAN_GAIN_RX, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF,-25,0,0,0,0,0, SERVO.SERVO_NONE,5925e6,2e-3,-10, MTRF100.SigGen.SG1,"ANT", MTRF100.SrcAmpEnable.A2_A3,"RX", MTRF100.AmpState.Enable,"",-5, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.A4_MEAS);
                    program.Measure_CW_Tests(24, GPIO.WLAN_GAIN_RX, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF,-25,0,0,0,0,0, SERVO.SERVO_NONE,2410e6,2e-3,-10, MTRF100.SigGen.SG1,"ANT", MTRF100.SrcAmpEnable.A2_A3,"RX", MTRF100.AmpState.Enable,"",-20, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.A4_MEAS);
                    program.Measure_CW_Tests(30, GPIO.WLAN_GAIN_RX, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF,-30,0,0.1,17,17,0, SERVO.SERVO_P1DB,5500e6,2e-3,10, MTRF100.SigGen.SG1,"ANT", MTRF100.SrcAmpEnable.A1_SRC1_A2_A3,"RX", MTRF100.AmpState.Enable,"",-5, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.A4_MEAS);
                    program.VsgVsaSetup(200e6,1e-3,1e-3,1e-3,0, TriggerTypeEnum.FreeRun, VsgMode.CW,"");
                    program.Measure_Noise_Figure(32,5500e6,5e-3,-10, MTRF100.SigGen.SG1,"ANT", MTRF100.SrcAmpEnable.None,"RX", MTRF100.AmpState.Bypass, MTRF200.MeasAmpEnable.A1, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP);
                    program.LoSharing(true);
                    program.VsgVsaSetup(160e6,320e-6,300e-6,320e-6,0, TriggerTypeEnum.External, VsgMode.ARB, WLAN_11ac_MCS9_80MHz);
                    program.Measure_RX_WLAN(33, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF,-30,20,0.1,25, SERVO.SERVO_NONE,5500e6,2e-3,-27,160e6, MTRF100.SigGen.SG1,"ANT", MTRF100.SrcAmpEnable.A2,"RX", MTRF100.AmpState.Bypass,"WLAN",-25, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.Bypass, WlanStandard.Wlan802_11ac, WlanBandwidth.Bandwidth80MHz, ChannelEstimationType.Reference);
                    program.LoSharing(false);
                    program.DeviceInit(GPIO.WLAN_GAIN_RX_BYPASS);
                    program.VsgVsaSetup(1e6,1e-3,1e-3,1e-3,0, TriggerTypeEnum.FreeRun, VsgMode.CW,"");
                    program.Measure_CW_Tests(25, GPIO.WLAN_GAIN_RX_BYPASS, SETUP.DC_IDLE, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF,-7,0,0,0,0,0, SERVO.SERVO_NONE,5150e6,2e-3,-10, MTRF100.SigGen.SG1,"ANT", MTRF100.SrcAmpEnable.A2_A3,"RX", MTRF100.AmpState.Enable,"",-5, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.A4_MEAS);
                    program.Measure_CW_Tests(27, GPIO.WLAN_GAIN_RX_BYPASS, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF,-7,0,0,0,0,0, SERVO.SERVO_NONE,5500e6,2e-3,-10, MTRF100.SigGen.SG1,"ANT", MTRF100.SrcAmpEnable.A2_A3,"RX", MTRF100.AmpState.Enable,"",-5, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.A4_MEAS);
                    program.Measure_CW_Tests(28, GPIO.WLAN_GAIN_RX_BYPASS, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF,-7,0,0,0,0,0, SERVO.SERVO_NONE,5850e6,2e-3,-10, MTRF100.SigGen.SG1,"ANT", MTRF100.SrcAmpEnable.A2_A3,"RX", MTRF100.AmpState.Enable,"",-5, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.A4_MEAS);
                    program.Measure_CW_Tests(29, GPIO.WLAN_GAIN_RX_BYPASS, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF,-7,0,0,0,0,0, SERVO.SERVO_NONE,5925e6,2e-3,-10, MTRF100.SigGen.SG1,"ANT", MTRF100.SrcAmpEnable.A2_A3,"RX", MTRF100.AmpState.Enable,"",-5, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.A4_MEAS);
                    program.DeviceInit(GPIO.WLAN_GAIN_TX);
                    program.VsgVsaSetup(1e6,1e-3,1e-3,1e-3,0, TriggerTypeEnum.FreeRun, VsgMode.CW,"");
                    program.Measure_ISO(34, SETUP.DC_IDLE, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF,-15,0,0,0,0,0, SERVO.SERVO_NONE,5500e6,2e-3,-15, MTRF100.SigGen.SG1,"TX", MTRF100.SrcAmpEnable.A2_A3,"ANT","RX", MTRF100.AmpState.Bypass,"",-20, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.Bypass);
                    program.Measure_ISO(35, SETUP.DC_IDLE, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF,-25,0,0,0,0,0, SERVO.SERVO_NONE,5500e6,2e-3,-15, MTRF100.SigGen.SG1,"TX", MTRF100.SrcAmpEnable.A2_A3,"RX","RX", MTRF100.AmpState.Bypass,"",-20, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.Bypass);
                    program.Measure_CW_Tests(13, GPIO.WLAN_GAIN_TX, SETUP.DC_IDLE, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF,-25,5,0.1,29,29,29, SERVO.SERVO_NONE,5150e6,2e-3,-10, MTRF100.SigGen.SG1,"TX", MTRF100.SrcAmpEnable.A2_A3,"ANT", MTRF100.AmpState.Bypass,"",-20, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.Bypass);
                    program.Measure_CW_Tests(37, GPIO.WLAN_GAIN_TX, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF,-25,5,0.1,31,31,31, SERVO.SERVO_NONE,5500e6,2e-3,-10, MTRF100.SigGen.SG1,"TX", MTRF100.SrcAmpEnable.A2_A3,"ANT", MTRF100.AmpState.Bypass,"",-20, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.Bypass);
                    program.Measure_CW_Tests(38, GPIO.WLAN_GAIN_TX, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF,-25,5,0.1,29,29,29, SERVO.SERVO_NONE,5850e6,2e-3,-10, MTRF100.SigGen.SG1,"TX", MTRF100.SrcAmpEnable.A2_A3,"ANT", MTRF100.AmpState.Bypass,"",-20, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.Bypass);
                    program.Measure_CW_Tests(39, GPIO.WLAN_GAIN_TX, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF,-25,5,0.1,29,29,29, SERVO.SERVO_NONE,5925e6,2e-3,-10, MTRF100.SigGen.SG1,"TX", MTRF100.SrcAmpEnable.A2_A3,"ANT", MTRF100.AmpState.Bypass,"",-20, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.Bypass);
                    program.Measure_CW_Tests(40, GPIO.WLAN_GAIN_TX, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF,-25,5,0.1,29,29,29, SERVO.SERVO_NONE,3800e6,2e-3,-10, MTRF100.SigGen.SG1,"TX", MTRF100.SrcAmpEnable.A2_A3,"ANT", MTRF100.AmpState.Bypass,"",-20, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.Bypass);
                    program.Measure_CW_Tests(41, GPIO.WLAN_GAIN_TX, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF,-25,5,0.1,29,29,29, SERVO.SERVO_NONE,2600e6,2e-3,-10, MTRF100.SigGen.SG1,"TX", MTRF100.SrcAmpEnable.A2_A3,"ANT", MTRF100.AmpState.Bypass,"",-20, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.Bypass);
                    program.Measure_CW_Tests(43, GPIO.WLAN_GAIN_TX, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS, SETUP.HARM_OFF,-26,0,0.1,26,26,26, SERVO.SERVO_P1DB,5500e6,5e-3,6, MTRF100.SigGen.SG1,"TX", MTRF100.SrcAmpEnable.A2_A3,"ANT", MTRF100.AmpState.Bypass,"",-20, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.Bypass);
                    program.Measure_CW_Tests(45, GPIO.WLAN_GAIN_TX, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS, SETUP.HARM_OFF,-25,0,0.1,26,26,26, SERVO.SERVO_P3DB,5500e6,5e-3,6, MTRF100.SigGen.SG1,"TX", MTRF100.SrcAmpEnable.A1_SRC1_A2_A3,"ANT", MTRF100.AmpState.Bypass,"",-20, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.Bypass);
                    program.Measure_VDET_ICC(47, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS,-25,10,0.1,25, SERVO.SERVO_POUT,5500e6,2e-3,-15,40e6, MTRF100.SigGen.SG1,"TX", MTRF100.SrcAmpEnable.A2,"ANT", MTRF100.AmpState.Bypass,"",-20, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.Bypass,0.5);
                    program.Measure_VDET_ICC(50, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS,-25,18,0.1,25, SERVO.SERVO_POUT,5500e6,2e-3,3,40e6, MTRF100.SigGen.SG1,"TX", MTRF100.SrcAmpEnable.A2,"ANT", MTRF100.AmpState.Bypass,"",-20, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.Bypass,0.5);
                    program.Measure_VDET_ICC(53, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS,-25,22,0.1,25, SERVO.SERVO_POUT,5500e6,2e-3,3,40e6, MTRF100.SigGen.SG1,"TX", MTRF100.SrcAmpEnable.A2,"ANT", MTRF100.AmpState.Bypass,"",-20, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.Bypass,0.5);
                    program.Measure_VDET_ICC(56, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS,-25,25,0.1,25, SERVO.SERVO_POUT,5500e6,2e-3,8,40e6, MTRF100.SigGen.SG1,"TX", MTRF100.SrcAmpEnable.A2,"ANT", MTRF100.AmpState.Bypass,"",-20, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.Bypass,0.5);
                    program.LoSharing(true);
                    program.VsgVsaSetup(80e6,320e-6,300e-6,320e-6,0, TriggerTypeEnum.External, VsgMode.ARB, WLAN_11ac_MCS7_40MHz);
                    program.Measure_WLAN(60, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF,-25,21,0.1,28, SERVO.SERVO_POUT,5190e6,2e-3,5,80e6, MTRF100.SigGen.SG1,"TX", MTRF100.SrcAmpEnable.A2,"ANT", MTRF100.AmpState.Bypass,"WLAN",-10, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.Bypass, WlanStandard.Wlan802_11ac, WlanBandwidth.Bandwidth40MHz, ChannelEstimationType.Reference);
                    program.Measure_WLAN(62, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF,-15,21,0.1,23, SERVO.SERVO_NONE,5510e6,2e-3,-5,80e6, MTRF100.SigGen.SG1,"TX", MTRF100.SrcAmpEnable.A2,"ANT", MTRF100.AmpState.Bypass,"WLAN",-10, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.Bypass, WlanStandard.Wlan802_11ac, WlanBandwidth.Bandwidth40MHz, ChannelEstimationType.Reference);
                    program.Measure_WLAN(63, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF,-25,21,0.1,28, SERVO.SERVO_POUT,5795e6,2e-3,5,80e6, MTRF100.SigGen.SG1,"TX", MTRF100.SrcAmpEnable.A2,"ANT", MTRF100.AmpState.Bypass,"WLAN",-10, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.Bypass, WlanStandard.Wlan802_11ac, WlanBandwidth.Bandwidth40MHz, ChannelEstimationType.Reference);
                    program.LoSharing(true);
                    program.VsgVsaSetup(160e6,320e-6,300e-6,320e-6,0, TriggerTypeEnum.External, VsgMode.ARB, WLAN_11ac_MCS9_80MHz);
                    program.Measure_WLAN(65, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF,-25,20,0.1,25, SERVO.SERVO_POUT,5210e6,2e-3,3,160e6, MTRF100.SigGen.SG1,"TX", MTRF100.SrcAmpEnable.A2,"ANT", MTRF100.AmpState.Bypass,"WLAN",-10, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.Bypass, WlanStandard.Wlan802_11ac, WlanBandwidth.Bandwidth80MHz, ChannelEstimationType.Reference);
                    program.Measure_WLAN(67, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF,-15,20,0.1,25, SERVO.SERVO_NONE,5530e6,2e-3,-5,160e6, MTRF100.SigGen.SG1,"TX", MTRF100.SrcAmpEnable.A2,"ANT", MTRF100.AmpState.Bypass,"WLAN",-10, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.Bypass, WlanStandard.Wlan802_11ac, WlanBandwidth.Bandwidth80MHz, ChannelEstimationType.Reference);
                    program.Measure_WLAN(68, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF,-25,20,0.1,25, SERVO.SERVO_POUT,5775e6,2e-3,3,160e6, MTRF100.SigGen.SG1,"TX", MTRF100.SrcAmpEnable.A2,"ANT", MTRF100.AmpState.Bypass,"WLAN",-10, MTRF200.MeasAmpEnable.None, MTRF200.MeasPathFilterAttenBypass.Bypass,0, MTRF200.DownConverterState.NOP, SrcMeasAmpConfig.Bypass, WlanStandard.Wlan802_11ac, WlanBandwidth.Bandwidth80MHz, ChannelEstimationType.Reference);
                    program.Measure_End_Idle_Current(70,true);
                    program.Measure_Idle_Current(75,false);
                    program.Measure_OS(85,true);

                    program.ApplicationEnd();

                    if (device < numberOfDeviceRuns - 1) Util.WaitTime(500e-6); // Wait time between consecutive runs (device cool-down)
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message + "\n" + ex.StackTrace);
            }

            program.ApplicationUnload();

            Console.Write("\nPress any key to quit. ");
            Console.ReadKey();
        }
    }
}
