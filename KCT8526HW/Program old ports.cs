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
//using MT.TesterDriver.VNA;
using MT.Tools.HwResourceManager.Manager;
using MerlinTest.Tools.DataLogging;
using MerlinTest.Tools.DataLogging.Enums;
using MT.TesterDriver.Enums;
using NationalInstruments;
using MerlinTest.Tools.Diagnostic.Logging;

namespace KCT8526HW
{
    public enum GPIO { WLAN_GAIN_TX, WLAN_GAIN_RX, WLAN_GAIN_RX_BYPASS, WLAN_IDLE_RX, WLAN_IDLE_VCC };
    public enum SERVO { SERVO_POUT, SERVO_P1DB, SERVO_P3DB, SERVO_P1DB_P3DB, SERVO_NONE }
    public enum SETUP { TEST_INIT, TEST_INIT_OFF, DEVICE_INIT, DEVICE_INIT_OFF, DC_IDLE, DC_IDLE_OFF,
        DC_SETUP, DC_SETUP_OFF, DC_MEAS, DC_MEAS_OFF, HARM_2ND, HARM_3RD, HARM_2ND_3RD, HARM_OFF
    }

    public class KCT8526HW : ApplicationBase
    {
        /// <summary>
        /// Gets / Sets the diagnostic logging object.
        /// </summary>
        private NetworkTraceSource trace { get; set; }

        public KCT8526HW()
        {
            // Initialize the network trace object.
            this.trace = new NetworkTraceSource("Application Code", "KCT8526HW");

            // Define the level of logging to display
            SourceSwitch myTraceSourceSwitch = new SourceSwitch("Data Logging Prototype");
            myTraceSourceSwitch.Level = SourceLevels.Information;
            trace.Switch = myTraceSourceSwitch;
        }
        //private const bool IsEVB = true;          // Switch between EVB and production test fixture
        private const bool useMultithread = false;  // Enable multithread calculations
        private const bool writeToDatalog = true;  // Write test results to datalog
        private const bool writeToConsole = true; // Print test results to console window
        private const bool writeToFile = false;    // Write test results to file
        private bool writeInstrumentStatusToConsole = false; // Print instrument state to console window

        private const bool getlocalTime = false;

        private NoiseFigurePortModule nf = new NoiseFigurePortModule();
        PortModuleUserCal PortModuleCal = new PortModuleUserCal();
        private Dictionary<int, double> LNA_Noise_Figure_cached = new Dictionary<int, double>();

        private Dictionary<int, WlanRfmxAnalysis> Wlan = new Dictionary<int, WlanRfmxAnalysis>();

        private List<int> listTestNumbers = new List<int>(); // For multithreaded analysis
        private Dictionary<int, SpectrumAnalysis> Spectrum = new Dictionary<int, SpectrumAnalysis>();

        private PSM DC;
        private PowerSupply PS;
        private pe32h Digital;
        private MTRF100 rf100;
        private MTRF200 rf200;
        private SwitchMatrix Matrix;

        private ISignalGenerator SigGen1Hal;
        private ISignalGenerator SigGen2Hal;
        private IDigitizer DigitizerHal;

        private List<DynamicKey> dynamicPool;
        private List<CalDataNew> calSetup = new List<CalDataNew>();
        private Info calInfo = new Info();
        private PowerMeter calMeter = new PowerMeter();
        private Amplifier calAmp = new Amplifier();
        private InternalAttenuation calAttenInt = new InternalAttenuation();
        private ExternalAttenuation calAttenExt = new ExternalAttenuation();
        private InternalCalibration calDirectPath = new InternalCalibration();

        static CalSettings calSettings = new CalSettings();
        static Noise NoiseData = new Noise();

        List<CalDataVer3> calSetupVer3;
        Info calInfoVer3;
        CalSettings calSettingVer3;
        ExternalAttenuation calAttenExtVer3;
        private Dictionary<CalDataKey, Dictionary<string, List<PathFactorVer3>>> calDatasVer3;

        private Dictionary<CalDataKey, Dictionary<string, List<PathFactor>>> calDatas;

        private int PE32 = 0;

        private double indexTime = 0;
        private double[] localTime = new double[20];

        private Stopwatch indexTimer = new Stopwatch();   // Performance timer (index)
        //private Stopwatch testTimer = new Stopwatch();    // Performance timer (per test)
        private Stopwatch localTimer = new Stopwatch();   // Performance timer (per section)
        private Stopwatch programTimer = new Stopwatch(); // Performance timer (test program)

        private double[] VCC_ICC = new double[2];
        private double[] VDD_ICC = new double[2];
        private double[] Total_ICC = new double[2];

        private double[] gain = new double[2];
        private double[] gainMin = new double[2] {10000, 10000};
        private double[] gainMax = new double[2] { -10000, -10000};

        private double[] TX_OP1dB_2442 = new double[2];
        private double[] TX_OP3dB_2442 = new double[2];

        private double[] Pin = new double[2];
        private double[] Pout = new double[2];

        private double[] Pout1dB = new double[2];
        private double[] Pout3dB = new double[2];

        private double[] Harmonic_2nd = new double[2];
        private double[] Harmonic_3rd = new double[2];

        private double[] evm = new double[2];
        private double[] evmdB = new double[2];

        private double[] Vdet = new double[2];

        private double[] noiseGain = new double[2];
        private double[] noiseFigure = new double[2];

        private int testNumGlobal = 0;

        private double srcCalFactor = 0;
        private double measCalFactor = 0;
        private double calCalFactor = 0;

        private Dictionary<int, List<WlanAnalysis>> dictWlan;
        private Dictionary<int, List<SpectrumAnalysis>> dictSpectrum;

        private const string testDataFile = "APS100_KCT8526HW_Test_Data.csv"; // Test results file
        private static string testDataTarget;
        public StreamWriter datalog;

        private uint dutNumber = 1;

        public double testTime = 0;
        private static uint MaxSite = 2;

        private List<CalKey> listCalKey = new List<CalKey>();

        public class CalKey
        {
            public int testNumber;
            public string srcSelect;
            public string measPathMatrix;
            public string srcPathMatrix;
            public double srcFreq;
            public string modulationType;
        }

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

        private int RunDigitalPattern(int bdn, int lbeg, int lend)
        {
            int rst = 0;

            Digital.set_checkmode(bdn, 0);
            Digital.set_addbeg(bdn, lbeg);
            Digital.set_addend(bdn, lend);
            Digital.cycle(bdn, 0);
            Digital.fstart(bdn, 1);

            // Wait for sequencer to stop
            while (Digital.check_tprun(bdn) != 0) Util.WaitTime(1e-6);

            rst = Digital.check_tpass(bdn); // Return 1 is pass, else fail
            Digital.fstart(bdn, 0);

            //mts3_msg("FCCNT = %d", pe32_rd_fccnt(1));

            return rst;
        }
        private int RunDigitalPatternPXITrigger(int bdn, int lbeg, int lend)
        {
            int rst = 0;

            Digital.set_checkmode(bdn, 0);
            Digital.set_addbeg(bdn, lbeg);
            Digital.set_addend(bdn, lend);
            Digital.cycle(bdn, 0);
            //Digital.fstart(bdn, 1);
            Digital.pxi_fstart(1, 6, 1);
            // Wait for sequencer to stop
            while (Digital.check_tprun(bdn) != 0) Util.WaitTime(1e-6);

            rst = Digital.check_tpass(bdn); // Return 1 is pass, else fail
            //Digital.fstart(bdn, 0);
            //Digital.pxi_fstart(1, 0, 1);

            //mts3_msg("FCCNT = %d", pe32_rd_fccnt(1));

            return rst;
        }

        public int LoSharing(bool share)
        {
            //VSG_5840 vsg5840obj = SigGen1Hal as VSG_5840;
            //VSA_5840 vsa5840obj = DigitizerHal as VSA_5840;

            //if (vsg5840obj != null || vsa5840obj != null)
            //{
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
            //}

            return 1;
        }

        private int DeviceSetup(GPIO mode, int siteNum)
        {
            if (siteNum == 0 && ActiveSites[siteNum])
            {
                if (mode == GPIO.WLAN_GAIN_TX)
                {
                    Digital.cpu_df(1, 1, 1, 1);//PA_EN
                    Digital.cpu_df(1, 2, 1, 1);//C1
                    Digital.cpu_df(1, 3, 1, 0);//C0

                    DC.setHPRange(1, HRange.R_200mA);//VCC@1
                    DC.setHPRange(2, HRange.R_5mA_1K);//VDD@1
                }

                else if (mode == GPIO.WLAN_GAIN_RX)
                {
                    Digital.cpu_df(1, 1, 1, 0);//PA_EN
                    Digital.cpu_df(1, 2, 1, 0);//C1
                    Digital.cpu_df(1, 3, 1, 1);//C0

                    DC.setHPRange(1, HRange.R_5mA_1K);//VCC@1
                    DC.setHPRange(2, HRange.R_50mA);//VDD@1
                }

                else if (mode == GPIO.WLAN_GAIN_RX_BYPASS)
                {
                    DC.setVoltage(1, 5.0);//VCC
                    DC.setVoltage(2, 5.0);//VDD
                    Digital.cpu_df(1, 1, 1, 0);//PA_EN
                    Digital.cpu_df(1, 2, 1, 1);//C1
                    Digital.cpu_df(1, 3, 1, 1);//C0

                    DC.setHPRange(1, HRange.R_5mA_1K);//VCC@1
                    DC.setHPRange(2, HRange.R_5mA_1K);//VDD@1
                }
            }

            return 1;
        }

        private int MeasureIdle(int testnumber)
        {
            double[] i_result;

            i_result = DC.HPMeas.getCurrentAverage();

            VCC_ICC[0] = i_result[0];  //VCC
            VDD_ICC[0] = i_result[1];  //VDD
            Total_ICC[0] = VCC_ICC[0] + VDD_ICC[0];

            return 1;
        }

        private int DcSetup(int testnumber)
        {

            return 1;
        }

        private int MeasureDC(int testnumber)
        {
            double[] i_result;

            i_result = DC.HPMeas.getCurrentAverage();

            VCC_ICC[0] = i_result[0];  //VCC
            VDD_ICC[0] = i_result[1];  //VDD
            Total_ICC[0] = VCC_ICC[0] + VDD_ICC[0];

            return 1;
        }

        private int PortModuleSetup(int testnumber, int siteNum, string sourcePort, string measurePort)
        {
            if (siteNum == 0 && ActiveSites[siteNum])
            {
                if (testnumber == 14 || testnumber == 16 || testnumber == 17 || testnumber == 21 || testnumber == 23 || testnumber == 24 || testnumber == 28 || testnumber == 71)
                {
                    rf100.ConnectSourcePort(MTRF100.SigGen.SG1, MVPs[sourcePort][siteNum].srcPort, MTRF100.SrcAmpEnable.A2_A3);
                    rf200.ConnectMeasurePort(MVPs[measurePort][siteNum].measPort, MTRF200.MeasAmpEnable.A1);
                }

                if (testnumber == 31 || testnumber == 35 || testnumber == 36 || testnumber == 58 || testnumber == 61 || testnumber == 64 || testnumber == 67)
                {
                    rf100.ConnectSourcePort(MTRF100.SigGen.SG1, MVPs[sourcePort][siteNum].srcPort, MTRF100.SrcAmpEnable.A2_A3);
                    rf100.ConnectMeasurePort(MVPs[measurePort][siteNum].srcMeasPort, MTRF100.AmpState.Bypass);
                    rf200.ConnectMeasurePort(MVPs[measurePort][siteNum].measPort, MTRF200.MeasAmpEnable.None);
                }

                if (testnumber == 40 || testnumber == 41 || testnumber == 42 || testnumber == 43 || testnumber == 44 || testnumber == 45 || testnumber == 46)
                {
                    rf100.ConnectSourcePort(MTRF100.SigGen.SG1, MVPs[sourcePort][siteNum].srcPort, MTRF100.SrcAmpEnable.A2);
                    rf100.ConnectMeasurePort(MVPs[measurePort][siteNum].srcMeasPort, MTRF100.AmpState.Bypass);
                    rf200.ConnectMeasurePort(MTRF200.MeasPort.SRC, MTRF200.MeasAmpEnable.None);
                }

                if (testnumber == 46 || testnumber == 48 || testnumber == 50 || testnumber == 52)
                {
                    rf100.ConnectSourcePort(MTRF100.SigGen.SG1, MVPs[sourcePort][siteNum].srcPort, MTRF100.SrcAmpEnable.A2_A3);
                    rf100.ConnectMeasurePort(MVPs[measurePort][siteNum].srcMeasPort, MTRF100.AmpState.Bypass);
                    rf200.ConnectMeasurePort(MTRF200.MeasPort.SRC, MTRF200.MeasAmpEnable.None);
                }

                if (testnumber == 54 || testnumber == 56)
                {
                    rf100.ConnectSourcePort(MTRF100.SigGen.SG1, MVPs[sourcePort][siteNum].srcPort, MTRF100.SrcAmpEnable.All);
                    rf100.ConnectMeasurePort(MVPs[measurePort][siteNum].srcMeasPort, MTRF100.AmpState.Bypass);
                    rf200.ConnectMeasurePort(MTRF200.MeasPort.SRC, MTRF200.MeasAmpEnable.None);
                }

                if (testnumber == 70)
                {
                    rf200.FilterSelect(MTRF200.MeasFilt.FILT3);
                }

                if (testnumber == 72)
                {
                    rf100.ConnectSourcePort(MTRF100.SigGen.SG1, MVPs[sourcePort][siteNum].srcPort, MTRF100.SrcAmpEnable.A2_A3);
                    rf200.ConnectMeasurePort(MVPs[measurePort][siteNum].measPort, MTRF200.MeasAmpEnable.None);
                }

                if (testnumber == -1)
                {
                    rf200.FilterSelect(MTRF200.MeasFilt.Bypass);
                }
            }

            return 1;
        }

        private int RetrieveCalFactors(int testnumber, int siteNum, string sourcePort, string measurePort, double freq)
        {
            if (siteNum == 0 && ActiveSites[siteNum])
            {
                if (testnumber == 14 || testnumber == 16 || testnumber == 17 || testnumber == 21 || testnumber == 23 || testnumber == 24 || testnumber == 28 || testnumber == 71)
                {
                    srcCalFactor = PortModuleCal.GetSrcCalFactor(freq, MTRF100.SigGen.SG1, MVPs[sourcePort][siteNum].srcPort, SrcAmpConfig.A2_A3, "");
                    measCalFactor = PortModuleCal.GetMeasCalFactor(freq, MVPs[measurePort][siteNum].measPort, MTRF200.MeasFilt.Bypass, SrcMeasAmpConfig.A1, "");
                    calCalFactor = PortModuleCal.GetDirCalFactor(freq, MTRF100.SigGen.SG1, MVPs[sourcePort][siteNum].srcPort, SrcAmpConfig.A2_A3, MeasAmpConfig.Bypass, "");
                }

                if (testnumber == 31 || testnumber == 35 || testnumber == 36 || testnumber == 58 || testnumber == 61 || testnumber == 64 || testnumber == 67)
                {
                    srcCalFactor = PortModuleCal.GetSrcCalFactor(freq, MTRF100.SigGen.SG1, MVPs[sourcePort][siteNum].srcPort, SrcAmpConfig.A2_A3, "");
                    measCalFactor = PortModuleCal.GetMeasCalFactor(freq, MVPs[measurePort][siteNum].srcMeasPort, MTRF200.MeasFilt.Bypass, SrcMeasAmpConfig.Bypass, "");
                    calCalFactor = PortModuleCal.GetDirCalFactor(freq, MTRF100.SigGen.SG1, MVPs[sourcePort][siteNum].srcPort, SrcAmpConfig.A2_A3, MeasAmpConfig.Bypass, "");
                }

                if (testnumber == 40 || testnumber == 41 || testnumber == 42 || testnumber == 43 || testnumber == 44 || testnumber == 45 || testnumber == 46)
                {
                    srcCalFactor = PortModuleCal.GetSrcCalFactor(freq, MTRF100.SigGen.SG1, MVPs[sourcePort][siteNum].srcPort, SrcAmpConfig.A2, "WLAN");
                    measCalFactor = PortModuleCal.GetMeasCalFactor(freq, MVPs[measurePort][siteNum].srcMeasPort, MTRF200.MeasFilt.Bypass, SrcMeasAmpConfig.Bypass, "WLAN");
                    calCalFactor = PortModuleCal.GetDirCalFactor(freq, MTRF100.SigGen.SG1, MVPs[sourcePort][siteNum].srcPort, SrcAmpConfig.A2, MeasAmpConfig.Bypass, "WLAN");
                }

                if (testnumber == 46 || testnumber == 48 || testnumber == 50 || testnumber == 52)
                {
                    srcCalFactor = PortModuleCal.GetSrcCalFactor(freq, MTRF100.SigGen.SG1, MVPs[sourcePort][siteNum].srcPort, SrcAmpConfig.A2_A3, "WLAN");
                    measCalFactor = PortModuleCal.GetMeasCalFactor(freq, MVPs[measurePort][siteNum].srcMeasPort, MTRF200.MeasFilt.Bypass, SrcMeasAmpConfig.Bypass, "WLAN");
                    calCalFactor = PortModuleCal.GetDirCalFactor(freq, MTRF100.SigGen.SG1, MVPs[sourcePort][siteNum].srcPort, SrcAmpConfig.A2_A3, MeasAmpConfig.Bypass, "WLAN");
                }

                if (testnumber == 54 || testnumber == 56)
                {
                    srcCalFactor = PortModuleCal.GetSrcCalFactor(freq, MTRF100.SigGen.SG1, MVPs[sourcePort][siteNum].srcPort, SrcAmpConfig.A1_A2_A3, "WLAN");
                    measCalFactor = PortModuleCal.GetMeasCalFactor(freq, MVPs[measurePort][siteNum].srcMeasPort, MTRF200.MeasFilt.Bypass, SrcMeasAmpConfig.Bypass, "WLAN");
                    calCalFactor = PortModuleCal.GetDirCalFactor(freq, MTRF100.SigGen.SG1, MVPs[sourcePort][siteNum].srcPort, SrcAmpConfig.A1_A2_A3, MeasAmpConfig.Bypass, "WLAN");
                }

                if (testnumber == 70)
                {
                    measCalFactor = PortModuleCal.GetMeasCalFactor(freq, MVPs[measurePort][siteNum].srcMeasPort, MTRF200.MeasFilt.FILT3, SrcMeasAmpConfig.Bypass, "");
                }

                if (testnumber == 72)
                {
                    srcCalFactor = PortModuleCal.GetSrcCalFactor(freq, MTRF100.SigGen.SG1, MVPs[sourcePort][siteNum].srcPort, SrcAmpConfig.A2_A3, "");
                    measCalFactor = PortModuleCal.GetMeasCalFactor(freq, MVPs[measurePort][siteNum].measPort, MTRF200.MeasFilt.Bypass, SrcMeasAmpConfig.Bypass, "", 0);
                    calCalFactor = PortModuleCal.GetDirCalFactor(freq, MTRF100.SigGen.SG1, MVPs[sourcePort][siteNum].srcPort, SrcAmpConfig.A2_A3, MeasAmpConfig.Bypass, "");
                }
            }

            return 1;
        }

        private int StoreResult(int testNumber, int siteNum)
        {

            if (writeToDatalog)
            {
                if (siteNum == 0 && ActiveSites[siteNum])
                {
                    if (testNumber == 14 || testNumber == 21)
                    {
                        LogTestResult(testNumber++, Total_ICC[siteNum]);
                    }

                    if (testNumber == 31)
                    {
                        LogTestResult(testNumber++, VCC_ICC[siteNum]);
                        LogTestResult(testNumber++, VDD_ICC[siteNum]);
                        LogTestResult(testNumber++, Total_ICC[siteNum]);
                    }

                    if ((testNumber >= 15 && testNumber <= 17) || (testNumber >= 22 && testNumber <= 24) || (testNumber >= 34 && testNumber <= 36))
                    {
                        if (gain[siteNum] > gainMax[siteNum]) gainMax[siteNum] = gain[siteNum];
                        if (gain[siteNum] < gainMin[siteNum]) gainMin[siteNum] = gain[siteNum];

                        LogTestResult(testNumber++, gain[siteNum]);

                        if (testNumber == 18 || testNumber == 25 || testNumber == 37)
                        {
                            LogTestResult(testNumber++, (gainMax[siteNum] - gainMin[siteNum]));
                            gainMax[siteNum] = -10000;
                            gainMin[siteNum] = 10000;
                        }
                    }

                    if (testNumber == 28)
                    {
                        LogTestResult(testNumber, Pin[siteNum]);
                    }

                    if (testNumber == 29)
                    {
                        LogTestResult(testNumber, noiseFigure[siteNum]);
                    }

                    if (testNumber >= 40 && testNumber <= 45)
                    {
                        LogTestResult(testNumber, evmdB[siteNum]);
                    }

                    if (testNumber >= 46 && testNumber <= 56)
                    {
                        LogTestResult(testNumber++, Pout[siteNum]);
                        LogTestResult(testNumber, evm[siteNum]);
                    }

                    if (testNumber == 58)
                    {
                        LogTestResult(testNumber++, Pout1dB[siteNum]);
                        LogTestResult(testNumber, Pout3dB[siteNum]);
                    }

                    if (testNumber == 61 || testNumber == 64|| testNumber == 67)
                    {
                        LogTestResult(testNumber++, Pout[siteNum]);
                        LogTestResult(testNumber++, Pout[siteNum]);
                        LogTestResult(testNumber, Pout[siteNum]);
                        testNumGlobal = testNumber + 1;
                    }

                    if (testNumber == 70)
                    {
                        LogTestResult(testNumber, Harmonic_2nd[siteNum]);
                    }

                    if (testNumber == 71 || testNumber == 72)
                    {
                        LogTestResult(testNumber, gain[siteNum]);
                    }
                }
            }

            return 1;
        }

        private int Servo(SERVO servoType, double startLevel, double poutTarget, double targetTolerance, double expectedGain, double limitP1dB, double limitP3dB,
                          double settlingTime, int maxLoopIterations, double centerFreq, double calFactorIn, double calFactorOut, uint numOfSamples, ref double Pin,
                          ref double Pout, ref double Pout1dB, ref double Pout3dB)
        {
            double[] measPower = new double[0];
            double error = 100;
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

            SigGen1Hal.Level = (startLevel - calFactorIn);
            Util.WaitTime(settlingTime);
            measPower = DigitizerHal.CapturePower();
            Pout = measPower[0] - calFactorOut;

            switch (servoType)
            {
                case SERVO.SERVO_P1DB:
                case SERVO.SERVO_P1DB_P3DB:
                    poutTarget = limitP1dB;
                    DigitizerHal.InputLevel = measPower[0] + 3;
                    Util.WaitTime(settlingTime);
                    measPower = DigitizerHal.CapturePower();
                    Pout = measPower[0] - calFactorOut;
                    DigitizerHal.InputLevel = limitP1dB + calFactorOut + 5;
                    Util.WaitTime(settlingTime);
                    break;
                case SERVO.SERVO_P3DB:
                    poutTarget = limitP3dB;
                    poutTarget = limitP1dB;
                    DigitizerHal.InputLevel = measPower[0] + 3;
                    Util.WaitTime(settlingTime);
                    measPower = DigitizerHal.CapturePower();
                    Pout = measPower[0] - calFactorOut;
                    DigitizerHal.InputLevel = limitP1dB + calFactorOut + 5;
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
                                Console.WriteLine("WARNING: Pin {0:F2} dBm exceeds maximum safe level of 8 dBm on servo loop iteration {1}.", newPin - calFactorIn, loopTries + 1);
                            }
                        }
                    }
                    if (loopDone == false)
                    {
                        SigGen1Hal.Level = (newPin - calFactorIn);
                        Util.WaitTime(settlingTime);

                        Pin = newPin;
                        measPower = DigitizerHal.CapturePower();
                        Pout = measPower[0] - calFactorOut;
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

                    trace.TraceInformation("Gain Init = {0}\tGain = {1} Gain Delta = {2} Loop tries {3}", gainInit,gain,gainDelta, loopTries);

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
                            Console.WriteLine("WARNING: Pin {0:F2} dBm exceeds maximum safe level of 8 dBm on servo loop iteration {1}.", newPin - calFactorIn, loopTries + 1);
                        }
                    }
                    if (loopDone == false)
                    {
                        SigGen1Hal.Level = (newPin - calFactorIn);
                        Util.WaitTime(settlingTime);

                        Pin = newPin;
                        measPower = DigitizerHal.CapturePower();
                        Pout = measPower[0] - calFactorOut;
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
                            Console.WriteLine("WARNING: Pin {0:F2} dBm exceeds maximum safe level of 8 dBm on servo loop iteration {1}.", newPin - calFactorIn, loopTries + 1);
                        }
                    }
                    if (loopDone == false)
                    {
                        SigGen1Hal.Level = (newPin - calFactorIn);
                        Util.WaitTime(settlingTime);

                        Pin = newPin;
                        measPower = DigitizerHal.CapturePower();
                        Pout = measPower[0] - calFactorOut;
                        gain = Pout - Pin;
                    }

                    loopTries++;

                } while (loopDone == false && loopTries < maxLoopIterations);
            }

            if (wasSuccessful) return loopTries;
            else return -1;
        }

        //================================== Multithread Analysis Functions ==================================
        private List<NoiseData> noiseDatas = new List<NoiseData>();
        private List<AppResult> results;

        private void ProcessTestPointWlan(object WlanObj)
        {
            WlanThreadObject WlanThreadObj = WlanObj as WlanThreadObject;

            lock (WlanThreadObj.wlanLock)
            {
                try
                {
                    //dictWlan[(int)WlanThreadObj.index][WlanThreadObj.SiteNum].Analyze();
                    //Wlan[(int)WlanThreadObj.index].AnalyzeIQ();
                    //Interlocked.Decrement(ref MeasurmentsActiveCounter);
                }
                
                catch (Exception ex)
                {
                    Console.WriteLine("WLAN exception: " + ex.Message);
                }

            //    dictWlan[(int)WlanThreadObj.index][WlanThreadObj.SiteNum].threadComplete = true;
            }
        }

        public int Measure_OS(int testNumber)

        {
            int testNumberStart = testNumber;

            double[] OS_DET = new double[ActiveSites.Length];//pin1
            double[] OS_LNAEN = new double[ActiveSites.Length];//pin2
            double[] OS_RXEN = new double[ActiveSites.Length];//pin3

            Digital.cpu_df(1, 1, 1, 0);//DET@1
            Digital.cpu_df(1, 2, 1, 0);//LNAEN@1
            Digital.cpu_df(1, 3, 1, 0);//RXEN@1

            Digital.cpu_df(1, 6, 1, 0);//DET@2
            Digital.cpu_df(1, 7, 1, 0);//LNAEN@2
            Digital.cpu_df(1, 8, 1, 0);//RXEN@2

            Digital.con_pmu(1, 0, 0);

            //------------------Measure OS DET@1----------------------------------------
            //Digital.cpu_df(1, 1, 0, 0);//DET
            //Digital.pmufi(1, 1, 0, 2, -1);
            //Digital.con_pmu(1, 5, 1);//connect CH8 to PMU to avoid spur          
            //Util.WaitTime(2.0e-3);
            //Digital.con_pmu(1, 1, 1);//DET
            //Digital.con_pmu(1, 5, 0);
            //Util.WaitTime(1e-3);
            //Digital.pmufi(1, 1, -0.1, 2, -1);
            //Util.WaitTime(5e-3);
            //OS_DET[0] = Digital.vmeas(1, 1);

            //Digital.con_pmu(1, 1, 0);

            //------------------Measure OS LNAEN@1----------------------------------------
            Digital.cpu_df(1, 2, 0, 0);//LNAEN
            Digital.pmufi(1, 1, 0, 2, -1);
            Digital.con_pmu(1, 5, 1);//connect CH8 to PMU to avoid spur          
            Util.WaitTime(2.0e-3);
            Digital.con_pmu(1, 2, 1);//LNAEN
            Digital.con_pmu(1, 5, 0);
            Util.WaitTime(1e-3);
            Digital.pmufi(1, 1, -0.1, 2, -1);
            Util.WaitTime(5e-3);
            OS_LNAEN[0] = Digital.vmeas(1, 2);

            Digital.con_pmu(1, 2, 0);

            //------------------Measure OS RXEN@1----------------------------------------
            Digital.cpu_df(1, 3, 0, 0);//DET
            Digital.pmufi(1, 1, 0, 2, -1);
            Digital.con_pmu(1, 5, 1);//connect CH8 to PMU to avoid spur          
            Util.WaitTime(2.0e-3);
            Digital.con_pmu(1, 3, 1);//DET
            Digital.con_pmu(1, 5, 0);
            Util.WaitTime(1e-3);
            Digital.pmufi(1, 1, -0.1, 2, -1);
            Util.WaitTime(5e-3);
            OS_RXEN[0] = Digital.vmeas(1, 3);

            Digital.con_pmu(1, 3, 0);


            ////------------------Measure OS DET@2----------------------------------------
            //Digital.cpu_df(1, 6, 0, 0);//DET
            //Digital.pmufi(1, 1, 0, 2, -1);
            //Digital.con_pmu(1, 5, 1);//connect CH8 to PMU to avoid spur          
            //Util.WaitTime(2.0e-3);
            //Digital.con_pmu(1, 6, 1);//DET
            //Digital.con_pmu(1, 5, 0);
            //Util.WaitTime(1e-3);
            //Digital.pmufi(1, 1, -0.1, 2, -1);
            //Util.WaitTime(25e-3);
            //OS_DET[1] = Digital.vmeas(1, 6);

            //Digital.con_pmu(1,6, 0);

            ////------------------Measure OS LNAEN@2----------------------------------------
            //Digital.cpu_df(1, 7, 0, 0);//LNAEN
            //Digital.pmufi(1, 1, 0, 2, -1);
            //Digital.con_pmu(1, 5, 1);//connect CH8 to PMU to avoid spur          
            //Util.WaitTime(2.0e-3);
            //Digital.con_pmu(1, 7, 1);//LNAEN
            //Digital.con_pmu(1, 5, 0);
            //Util.WaitTime(1e-3);
            //Digital.pmufi(1, 1, -0.1, 2, -1);
            //Util.WaitTime(25e-3);
            //OS_LNAEN[1] = Digital.vmeas(1, 7);

            //Digital.con_pmu(1, 7, 0);

            ////------------------Measure OS RXEN@2----------------------------------------
            //Digital.cpu_df(1, 8, 0, 0);//DET
            //Digital.pmufi(1, 1, 0, 2, -1);
            //Digital.con_pmu(1, 5, 1);//connect CH8 to PMU to avoid spur          
            //Util.WaitTime(2.0e-3);
            //Digital.con_pmu(1, 8, 1);//DET
            //Digital.con_pmu(1, 5, 0);
            //Util.WaitTime(1e-3);
            //Digital.pmufi(1, 1, -0.1, 2, -1);
            //Util.WaitTime(25e-3);
            //OS_RXEN[1] = Digital.vmeas(1, 8);

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
                //LogTestResult(testNumber++, OS_DET);
                LogTestResult(testNumber++, OS_LNAEN);
                LogTestResult(testNumber++, OS_RXEN);

            }
            return 0;
        }

        public void Measure_DC_Current_retest(int testNumber)
        {

            int testNumberStart = testNumber;

            double[] DC_RX_VCC = new double[ActiveSites.Length];
            double[] DC_TX_IDD_VCC1 = new double[ActiveSites.Length];
            double[] DC_TX_IDD_VCC2 = new double[ActiveSites.Length];
            double[] DC_TX_IDD_TOTAL = new double[ActiveSites.Length];

            double[] i_result;
            //-------------------- Leakage ------------------------------------------------------------------
            SigGen1Hal.Level = SigGen1Hal.LevelMin;

            for (int siteNum = 0; siteNum < ActiveSites.Length; siteNum++)
            {
                if (siteNum == 0 && ActiveSites[siteNum])
                {
                    DC.setHPRange(1, HRange.R_500mA);//VDD1@1
                    DC.setHPRange(2, HRange.R_500mA);//VDD@1

                    DC.setLPRange(1, LRange.R_100mA);//TXEN@1

                    Util.WaitTime(2e-3);
    
                    DC.HPMeas.setTriggerSource("None");
                    DC.HPMeas.setSampleClockandSamples(50e3, 30);
                    Util.WaitTime(2e-3);
                    i_result = DC.HPMeas.getCurrentAverage();
                    DC_RX_VCC[siteNum] = i_result[0] + i_result[1];

                }
                if (siteNum == 1 && ActiveSites[siteNum])
                {

                    DC.setHPRange(3, HRange.R_500mA);//VDD1@2
                    DC.setHPRange(4, HRange.R_500mA);//VDD@2

                    DC.setLPRange(2, LRange.R_100mA);//TXEN@2

                    Util.WaitTime(2e-3);
      
                    DC.HPMeas.setTriggerSource("None");
                    DC.HPMeas.setSampleClockandSamples(50e3, 30);
                    Util.WaitTime(2e-3);
                    i_result = DC.HPMeas.getCurrentAverage();
                    DC_RX_VCC[siteNum] = i_result[2] + i_result[3];
                }
            }

            for (int siteNum = 0; siteNum < ActiveSites.Length; siteNum++)
            {
                if (siteNum == 0 && ActiveSites[siteNum])
                {
                    DC.setHPRange(1, HRange.R_500mA);//VDD1@1
                    DC.setHPRange(2, HRange.R_500mA);//VDD@1

                    DC.setLPRange(1, LRange.R_100mA);//TXEN@1

                    Util.WaitTime(2e-3);

                    DC.HPMeas.setTriggerSource("None");
                    DC.HPMeas.setSampleClockandSamples(50e3, 30);
                    Util.WaitTime(2e-3);
                    i_result = DC.HPMeas.getCurrentAverage();
                    DC_TX_IDD_VCC1[siteNum] = i_result[0];
                    DC_TX_IDD_VCC2[siteNum] = i_result[1];
                    DC_TX_IDD_TOTAL[siteNum] = i_result[0] + i_result[1];


                }
                if (siteNum == 1 && ActiveSites[siteNum])
                {

                    DC.setHPRange(3, HRange.R_500mA);//VDD1@2
                    DC.setHPRange(4, HRange.R_500mA);//VDD@2

                    DC.setLPRange(2, LRange.R_100mA);//TXEN@2

                    Util.WaitTime(2e-3);

                    DC.HPMeas.setTriggerSource("None");
                    DC.HPMeas.setSampleClockandSamples(50e3, 30);
                    Util.WaitTime(2e-3);
                    i_result = DC.HPMeas.getCurrentAverage();
                    DC_TX_IDD_VCC1[siteNum] = i_result[2];
                    DC_TX_IDD_VCC2[siteNum] = i_result[3];
                    DC_TX_IDD_TOTAL[siteNum] = i_result[2] + i_result[3];
                }
            }

            if (writeToDatalog)
            {
                LogTestResult(testNumber++, DC_RX_VCC);
                LogTestResult(testNumber++, DC_TX_IDD_VCC1);
                LogTestResult(testNumber++, DC_TX_IDD_VCC2);
                LogTestResult(testNumber++, DC_TX_IDD_TOTAL);
            }

        }

        public void Measure_CW_Tests(int testNumber, GPIO mode, SETUP testInit, SETUP deviceInit, SETUP measureIdle, SETUP measureSetup, SETUP measureDC, SETUP harmonic, double startPin,
                                      double targetLevel, double tolerance, double expectedGain, double expectedP1dB, double expectedP3dB, SERVO servoType, double centerFreq,
                                      double settleTime, double digitizerLevel, double captureTime, double sampleRate, string srcSelect, string srcPin, string measPin)
        {
            srcCalFactor = 0;
            measCalFactor = 0;
            calCalFactor = 0;

            uint numOfSamples = (uint)(sampleRate * captureTime);
            double[] measPower = new double[0];
            double[] PgainCal = new double[ActiveSites.Length];
            double[] PinCal = new double[ActiveSites.Length];

            Pin[0] = double.NaN;
            Pin[1] = double.NaN;
            Pout[0] = double.NaN;
            Pout[1] = double.NaN;
            Pout1dB[0] = double.NaN;
            Pout1dB[1] = double.NaN;
            Pout3dB[0] = double.NaN;
            Pout3dB[1] = double.NaN;
            Vdet[0] = double.NaN;
            Vdet[1] = double.NaN;
            VCC_ICC[0] = double.NaN;
            VCC_ICC[1] = double.NaN;
            VDD_ICC[0] = double.NaN;
            VDD_ICC[1] = double.NaN;
            Total_ICC[0] = double.NaN;
            Total_ICC[1] = double.NaN;

            if (SETUP.TEST_INIT == testInit)
            {
                double measLength = captureTime;
                double measStepLength = captureTime;

                SigGen1Hal.StopWaveform();
                SigGen1Hal.Mode = VsgMode.CW;

                DigitizerHal.SamplingFreq = sampleRate;
                DigitizerHal.DwellTime = captureTime;
                DigitizerHal.NumberOfPowerMeasurements = 1;
                DigitizerHal.StepLength = captureTime;
                DigitizerHal.MeasurementOffset = 0;
                DigitizerHal.MeasurementLength = captureTime;
                DigitizerHal.TriggerType = TriggerTypeEnum.FreeRun;
            }

            if (SETUP.DEVICE_INIT == deviceInit)
            {
                DeviceSetup(mode, 0);
            }

            if (SETUP.DC_IDLE == measureIdle)
            {
                MeasureIdle(testNumber);
            }

            DigitizerHal.InputLevel = digitizerLevel;

            for (int siteNum = 0; siteNum < ActiveSites.Length; siteNum++)
            {
                if (!ActiveSites[siteNum])
                    continue;

                if (SETUP.DC_SETUP == measureSetup)
                {
                    DcSetup(testNumber);
                }

                PortModuleSetup(testNumber, siteNum, srcPin, measPin);
                RetrieveCalFactors(testNumber, siteNum, srcPin, measPin, centerFreq);

                //--------------------Measure TX Gain & RX Gain at CW mode--------------------------------------------------------------------------------------------------------------------------------

                SigGen1Hal.Frequency = centerFreq;
                DigitizerHal.Frequency = centerFreq;

                int servoStatus = -1;                

                servoStatus = Servo(servoType, startPin, targetLevel, tolerance, expectedGain, expectedP1dB, expectedP3dB, settleTime, 10, centerFreq,
                                    srcCalFactor, measCalFactor, numOfSamples, ref Pin[siteNum], ref Pout[siteNum], ref Pout1dB[siteNum], ref Pout3dB[siteNum]);

                gain[siteNum] = Pout[siteNum] - Pin[siteNum];

                if (SETUP.DC_MEAS == measureDC)
                {
                    MeasureDC(testNumber);
                }

                StoreResult(testNumber, siteNum);

                if (SETUP.HARM_OFF != harmonic)
                {
                    testNumber = testNumGlobal;
                    if(SETUP.HARM_2ND == harmonic || SETUP.HARM_2ND_3RD == harmonic)
                    {
                        PortModuleSetup(testNumber, siteNum, srcPin, measPin);
                        RetrieveCalFactors(testNumber, siteNum, srcPin, measPin, centerFreq * 2);
                        DigitizerHal.Frequency = centerFreq * 2;
                        Util.WaitTime(settleTime);
                        measPower = DigitizerHal.CapturePower();
                        Harmonic_2nd[siteNum] = measPower[0] - measCalFactor;
                    }

                    if (SETUP.HARM_3RD == harmonic || SETUP.HARM_2ND_3RD == harmonic)
                    {
                        PortModuleSetup(testNumber, siteNum, srcPin, measPin);
                        RetrieveCalFactors(testNumber, siteNum, srcPin, measPin, centerFreq * 3);
                        DigitizerHal.Frequency = centerFreq * 3;
                        Util.WaitTime(settleTime);
                        measPower = DigitizerHal.CapturePower();
                        Harmonic_3rd[siteNum] = measPower[0] - measCalFactor;
                    }

                    StoreResult(testNumber, siteNum);

                    testNumber = -1;

                    PortModuleSetup(testNumber, siteNum, srcPin, measPin);
                }
                //-----------------------------------------------------Do Internal Calibration-------------------------------------------------------------

                ////rf100.ConnectSourcePort(MTRF100.SigGen.SG1, MTRF100.SrcPort.CAL, MTRF100.SrcAmpEnable.A2_A3);
                ////rf200.ConnectMeasurePort(MTRF200.MeasPort.CAL, MTRF200.MeasAmpEnable.None);

                ////Util.WaitTime(1e-3);

                ////PinCal = DigitizerHal.CapturePower();

                ////PinCal[siteNum] -= calCalFactor;
                ////PgainCal[siteNum] = Pout[siteNum] - PinCal[siteNum];

                ////rf100.ConnectSourcePort(MTRF100.SigGen.SG1, MVPs[srcPin][siteNum].srcPort, MTRF100.SrcAmpEnable.A2_A3);
                ////rf200.ConnectMeasurePort(MVPs[measPin][siteNum].measPort, MTRF200.MeasAmpEnable.None);

                SigGen1Hal.Level = SigGen1Hal.LevelMin;
            }
        }


        public void Measure_NOISE_FIGURE(int testNumber, GPIO mode, double centerFreq, double settleTime, double digitizerLevel,
                                         double captureTime, double sampleRate, string srcPin, string measPin)

        {
            int testNumberStart = testNumber;

            noiseGain[0] = double.NaN;
            noiseGain[1] = double.NaN;
            noiseFigure[0] = double.NaN;
            noiseFigure[1] = double.NaN;

            uint numOfSamples = (uint)(sampleRate * captureTime);

            // Configure analysis library
            if (!Spectrum.Keys.Contains(testNumberStart))
            {
                SpectrumAnalysis spectrumAnalysis = new SpectrumAnalysis() { sampleFreq = 100e3, centerFreq = centerFreq, numOfSamples = 4096, measSpan = 100e3, digSpan = 36e6, rbw = 100, vbw = 10e3, analysisMode = SpectrumAnalysisMode.AllIQ, configurationModeType = SpectrumConfigurationModeType.FFTAnalyzer, detectorModeType = SpectrumDetectorModeType.MaxPeak, measurements = SpectrumMeasurement.PowerVsFreq };
                spectrumAnalysis.AnalysisSetup();
                spectrumAnalysis.SampleSize();
                Spectrum.Add(testNumberStart, spectrumAnalysis);
            }

            double measLength = captureTime;
            double measStepLength = captureTime;

            DigitizerHal.SamplingFreq = sampleRate;
            DigitizerHal.DwellTime = captureTime;
            DigitizerHal.NumberOfPowerMeasurements = 1;
            DigitizerHal.StepLength = captureTime;
            DigitizerHal.MeasurementOffset = 0;
            DigitizerHal.MeasurementLength = captureTime;
            DigitizerHal.TriggerType = TriggerTypeEnum.FreeRun;

            for (int siteNum = 0; siteNum < ActiveSites.Length; siteNum++)
            {
                if (!ActiveSites[siteNum])
                    continue;

                rf100.ConnectSourcePort(MTRF100.SigGen.SG1, MVPs[srcPin][siteNum].srcPort, MTRF100.SrcAmpEnable.None);
                rf200.ConnectMeasurePort(MVPs[measPin][siteNum].measPort, MTRF200.MeasAmpEnable.A1);

                DigitizerHal.Frequency = centerFreq;

                DigitizerHal.InputLevel = digitizerLevel;

                ////-----------------------------------------------Measure Noise Figure--------------------------------------------------------------------

                SigGen1Hal.Level = SigGen1Hal.LevelMin;

                nf.SetupMeasureNoiseFigure(centerFreq, MVPs[srcPin][siteNum].srcPort, MVPs[measPin][siteNum].measPort, false, MeasAmpEnable.A1);

                Util.WaitTime(settleTime);

                nf.MeasureNoiseFigure(centerFreq, out noiseFigure[siteNum], out noiseGain[siteNum]);

                if (double.IsInfinity(noiseFigure[siteNum]) || noiseFigure[siteNum].Equals(double.NaN))
                {
                    noiseFigure[siteNum] = -9999;
                }

                StoreResult(testNumber, siteNum);
            }
        }

        public void Measure_WLAN(int testNumber, GPIO mode, SETUP testInit, SETUP deviceInit, SETUP measureIdle, SETUP measureSetup, SETUP measureRfOn, double startPin,
                                double targetLevel, double tolerance, double expectedGain,
                                SERVO ServoType, double centerFreq, double settleTime, double digitizerLevel, double captureTime,
                                double measLength, double measStepLength, double measOffset, double sampleRate, string srcSelect, string srcPin, string measPin, 
                                WlanStandard AnalysisType, WlanBandwidth ChanBandwidth, string modWaveform)
        {
            int testNumberStart = testNumber;
            srcCalFactor = 0;
            measCalFactor = 0;
            calCalFactor = 0;
            double expectedP1dB = 0;
            double expectedP3dB = 0;

            uint numOfSamples = (uint)(sampleRate * captureTime);

            uint preTrigSamples = (uint)(sampleRate * measOffset);

            Pin[0] = double.NaN;
            Pin[1] = double.NaN;
            Pout[0] = double.NaN;
            Pout[1] = double.NaN;
            evm[0] = double.NaN;
            evm[1] = double.NaN;
            evmdB[0] = double.NaN;
            evmdB[1] = double.NaN;
            Pout[0] = double.NaN;
            Pout[1] = double.NaN;

            double[] PinCal = new double[ActiveSites.Length];
            double[] PgainCal = new double[ActiveSites.Length];
            double[] evmDynamic = new double[ActiveSites.Length];
            double[] evmdBDynamic = new double[ActiveSites.Length];
            double[] PAE = new double[ActiveSites.Length];

            double[] measPower = new double[0];

            if (SETUP.DEVICE_INIT == deviceInit)
            {
                DeviceSetup(mode, 0);
            }

            if (SETUP.DC_IDLE == measureIdle)
            {
                MeasureIdle(testNumber);
            }

            if (SETUP.TEST_INIT == testInit)
            {
                SigGen1Hal.Waveform = modWaveform;
                SigGen1Hal.Mode = VsgMode.ARB;

                SigGen1Hal.StopWaveform();
                SigGen1Hal.PlayWaveform();

                DigitizerHal.SamplingFreq = sampleRate;
                DigitizerHal.DwellTime = captureTime;
                DigitizerHal.NumberOfPowerMeasurements = 1;
                DigitizerHal.StepLength = measStepLength;
                DigitizerHal.MeasurementOffset = measOffset;
                DigitizerHal.MeasurementLength = captureTime;
                DigitizerHal.TriggerType = TriggerTypeEnum.External;

                //DC.Meas.setTriggerSource("PXI_Trig0");
                //DC.Meas.setSampleClockandSamples(100e3, 10);
            }

            SigGen1Hal.Frequency = centerFreq;
            DigitizerHal.Frequency = centerFreq;

            DigitizerHal.InputLevel = digitizerLevel;

            for (int siteNum = 0; siteNum < ActiveSites.Length; siteNum++)
            {

                if (!ActiveSites[siteNum])
                    continue;

                if (SETUP.DC_SETUP == measureSetup)
                {
                    DcSetup(testNumber);
                }

                PortModuleSetup(testNumber, siteNum, srcPin, measPin);
                RetrieveCalFactors(testNumber, siteNum, srcPin, measPin, centerFreq);

                int servoStatus = -1;

                servoStatus = Servo(ServoType, startPin, targetLevel, tolerance, expectedGain, expectedP1dB, expectedP3dB, settleTime, 10, centerFreq,
                              srcCalFactor, measCalFactor, numOfSamples, ref Pin[siteNum], ref Pout[siteNum], ref Pout1dB[siteNum], ref Pout3dB[siteNum]);

                gain[siteNum] = Pout[siteNum] - Pin[siteNum];

                if (SETUP.DC_MEAS == measureRfOn)
                {
                    MeasureDC(testNumber);
                }

                DigitizerHal.FetchIQ();
                
                // Configure analysis library
                if (!Wlan.Keys.Contains(testNumberStart))
                {
                    WlanRfmxAnalysis wlanAnalysis = new WlanRfmxAnalysis();
                    wlanAnalysis.CenterFrequency = centerFreq;
                    wlanAnalysis.ChannelBandwidth = ChanBandwidth;
                    wlanAnalysis.Measurments = WlanMeasurementTypes.OfdmModAcc;
                    wlanAnalysis.Standard = AnalysisType;
                    wlanAnalysis.AmplitudeTrackingEnabled = false;
                    wlanAnalysis.PhaseTrackingEnabled = true;
                    wlanAnalysis.TimeTrackingEnabled = false;
                    wlanAnalysis.SampleRate = sampleRate;
                    Wlan.Add(testNumberStart, wlanAnalysis);

                }

                if (useMultithread)
                {
                    //Wlan[testNumberStart].threadComplete = false;
                    WlanThreadObject threadObject = new WlanThreadObject();
                    threadObject.index = (uint)testNumberStart;
                    ThreadPool.QueueUserWorkItem(new WaitCallback(ProcessTestPointWlan), threadObject);
                }
                else
                {
                    Wlan[testNumberStart].AnalyzeIQ(DigitizerHal.IBuffer, DigitizerHal.QBuffer);

                    evmdB[siteNum] = Wlan[testNumberStart].RmsEvmMeanDb;
                    evm[siteNum] = Wlan[testNumberStart].RmsEVMMeanPct;

                    if (double.IsInfinity(evmdB[siteNum]) || evmdB[siteNum].Equals(double.NaN))
                    {
                        evmdB[siteNum] = -9999;
                    }

                    StoreResult(testNumber, siteNum);

                    SigGen1Hal.Level = SigGen1Hal.LevelMin;
                }
            }
        }

        // ================================== Calibration Configuration Data ==================================
        private Dictionary<string, Dictionary<int, PathFactorVer3>> MVPs;
        private void BuildCalDatasVer3MVP()
        {
            MVPs = new Dictionary<string, Dictionary<int, PathFactorVer3>>();
            MVPs.Add("TX", new Dictionary<int, PathFactorVer3>());
            MVPs["TX"].Add(0, new PathFactorVer3() {srcPort = MTRF100.SrcPort.P10});
            MVPs.Add("ANT", new Dictionary<int, PathFactorVer3>());
            MVPs["ANT"].Add(0, new PathFactorVer3() {srcPort = MTRF100.SrcPort.P9, srcMeasPort = MTRF100.MeasPort.P9, measPort = MTRF200.MeasPort.SRC});
         
            MVPs.Add("RX", new Dictionary<int, PathFactorVer3>());
            MVPs["RX"].Add(0, new PathFactorVer3() {measPort = MTRF200.MeasPort.M1 });
        }

        private int ReadUserCalDataFileV2(string calDataFile)
        {
            if (writeInstrumentStatusToConsole) Console.WriteLine("Loading RF user calibration data file: {0}\n", Path.GetFileName(calDataFile));

            //string calDataTarget = Path.Combine(Directory.GetParent(Directory.GetCurrentDirectory()).FullName, "Calibration\\", calDataFile);

            CalImport Importer = new CalImport();
            var ConfigData = Importer.ImportCalDataVer2(calDataFile);
            calSetup = ConfigData.Item1;
            calInfo = ConfigData.Item2;
            calMeter = ConfigData.Item3;
            calAmp = ConfigData.Item4;
            calAttenInt = ConfigData.Item5;
            calAttenExt = ConfigData.Item6;
            calDirectPath = ConfigData.Item7;

            if (calSetup.Count() <= 0) // No calibration configuration settings defined
            {
                if (writeInstrumentStatusToConsole) Console.WriteLine("ERROR: No calibration data defined\n");
                return -1;
            }

            // Add record number to cal data entries (save original order)
            for (int calIndex = 0; calIndex < calSetup.Count; calIndex++)
            {
                calSetup[calIndex].record = calIndex + 1;
            }

            //BuildCalDatas();

            return 0;
        }

        private void ResetVarible(int testNumber, double centerFreq, string modulationType)
        {
            DynamicKey dynamicKey = new DynamicKey() { testNumber = testNumber, modulationType = modulationType };

            if (!dynamicPool.Any(x => (x.testNumber == dynamicKey.testNumber && x.modulationType == dynamicKey.modulationType)))
            {
                dynamicPool.Add(dynamicKey);
                
                if (modulationType == "WLAN")
                {
                    dictWlan.Add(dynamicKey.testNumber, new List<WlanAnalysis>());
                    for (int siteNum = 0; siteNum < MaxSite; siteNum++)
                    {
                        WlanAnalysis wlanAnalysis = new WlanAnalysis() { sampleFreq = 160e6, centerFreq = centerFreq,
                                                                        numOfSamples = 39840,
                                                                        bandwidth = WlanChannelBandwidth.ChannelBandwidth80MHz,
                                                                        digSpan = 90e6,
                                                                        measurements = WlanMeasurement.LocateBurst | WlanMeasurement.ModAccuracy,
                                                                        mode = WlanAnalysisMode.AnalysisMode11acVHT,
                                                                        ofdmMode = WlanOfdmEqMode.OfdmEqPreamble,
                                                                        VHTdataLength = WlanVHTDataLengthSource.All };
                        wlanAnalysis.AnalysisSetup();
                        dictWlan[dynamicKey.testNumber].Add(wlanAnalysis);
                    }
                }

                else if (modulationType == "CW")
                {
                    //dictSpectrum.Add(dynamicKey.testNumber, new List<SpectrumAnalysis>());
                    //for (int siteNum = 0; siteNum < MaxSite; siteNum++)
                    //{
                    //    SpectrumAnalysis spectrumAnalysis = new SpectrumAnalysis() { sampleFreq = 100e6, centerFreq = centerFreq, numOfSamples = 4096, measSpan = 100e3, digSpan = 36e6, rbw = 100, vbw = 10e3, analysisMode = SpectrumAnalysisMode.AllIQ, configurationModeType = SpectrumConfigurationModeType.FFTAnalyzer, detectorModeType = SpectrumDetectorModeType.MaxPeak, measurements = SpectrumMeasurement.PowerVsFreq };
                    //    spectrumAnalysis.AnalysisSetup();
                    //    spectrumAnalysis.SampleSize();
                    //    dictSpectrum[dynamicKey.testNumber].Add(spectrumAnalysis);
                    //}
                }
            }
            
        }        

        private CalDataVer3 GetCalDataVer3(SigGen srcSelect, double centerFreq, SrcPort srcPort, MeasPort measPort, string modulationType)
        {
            CalDataVer3 calData;
            if (calSetupVer3.Any(x => x.srcSelect == srcSelect && x.srcFreq == centerFreq && x.srcPort == srcPort && x.measPort == measPort && x.modulationType == modulationType))
            {
                calData = calSetupVer3.First(x => x.srcSelect == srcSelect && x.srcFreq == centerFreq && x.srcPort == srcPort && x.measPort == measPort && x.modulationType == modulationType);
            }
            else
            {
                throw new Exception(string.Format("CalData for {0}/{1}/{2} is not available, please process the calibration again!", srcSelect, centerFreq, modulationType));
            }
            return calData;
        }

        //=================================== Application Setup Functions ====================================

        public int ApplicationLoad() //ApplicationLoad(string pathUserCal, bool isCal)
        {

            int status = -1;

            //-------------------- Read User Calibration Data File --------------------
            status = PortModuleCal.ReadCalDataFile(@"C:\MerlinTest\Production\UserCal\KCT8526HW_RF_Cal_Data.csv");

            BuildCalDatasVer3MVP();

            if (status != 0) ProgramExit();

            System.Diagnostics.Process.GetCurrentProcess().PriorityClass = System.Diagnostics.ProcessPriorityClass.BelowNormal;
            dynamicPool = new List<DynamicKey>();
            dictWlan = new Dictionary<int, List<WlanAnalysis>>();
            dictSpectrum = new Dictionary<int, List<SpectrumAnalysis>>();
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

            if ( ATE.SigGen1HAL != null)
            {
                // Set some defaults
                SigGen1Hal.RfState = true;

                ATE.SigGen1HAL.DeleteAllWaveforms();

                // Load some default waveforms.
                string waveform = Path.Combine(ProgramDir, "Waveforms", "wlan11acMCS940MHz320us50pct.tdms");
                string markerFile = Path.Combine(ProgramDir, "Waveforms", "Marker1AtStart.xml");


                ATE.SigGen1HAL.LoadWaveforms(waveform, markerFile);

                waveform = Path.Combine(ProgramDir, "Waveforms", "wlan11acMCS940MHz400us50pct.tdms");
                ATE.SigGen1HAL.LoadWaveforms(waveform, markerFile);

                waveform = Path.Combine(ProgramDir, "Waveforms", "wlan11axMCS1140MHz400us50pct.tdms");
                ATE.SigGen1HAL.LoadWaveforms(waveform, markerFile);

                waveform = Path.Combine(ProgramDir, "Waveforms", "wlan11nMCS720MHz320us50pct.tdms");
                ATE.SigGen1HAL.LoadWaveforms(waveform, markerFile);

                // Configure the RFSG to export it's LO and the RFSA to import it.

                VSG_5840 vsg5840obj = SigGen1Hal as VSG_5840;

                // Check the cast wasn't NULL.
                if (vsg5840obj != null)
                {
                    vsg5840obj.LOOutEnabled = true;
                    vsg5840obj.ExternalLO = false;
                }

                VSA_5840 vsa5840obj = DigitizerHal as VSA_5840;

                // Check the cast wasn't NULL.
                if (vsa5840obj != null)
                {
                    vsa5840obj.LOOutEnabled = false;
                    vsa5840obj.ExternalLO = true;
                }

            }

            //-------------------- Read User Calibration Data File --------------------
            nf.DC = DC;
            nf.Rf100 = rf100;
            nf.Rf200 = rf200;
            nf.dig_HAL = DigitizerHal;
            nf.NoiseSoureCalFile = @"C:\MerlinTest\Production\UserCal\KCT8526HW_NF_Cal_Data.csv";
            nf.ReadNoiseCalDataFile();

            Util.WaitTime(10e-3);

            //-------------------- Check 10 MHz Reference Lock --------------------

            bool refLockStatus = false;

            if (SigGen1Hal != null) refLockStatus = SigGen1Hal.Locked;
            if (refLockStatus == false) ProgramExit();

            if (SigGen2Hal != null) refLockStatus = SigGen2Hal.Locked;
            if (refLockStatus == false) ProgramExit();

            if (DigitizerHal != null) refLockStatus = DigitizerHal.Locked;
            if (refLockStatus == false) ProgramExit();

            //-------------------- Configure DC Instrument ----------------------
            DC.Meas.PS = PS;
            DC.Meas.usePSMHPCal = true;
            DC.Meas.usePSMLPCal = true;
            //-----------------------SITE1---------------------------------------

            DC.Meas.enableHP(1); // VCC@1
            DC.Meas.enableHP(2); // VDD@1

            DC.Meas.enableFront(1); // Vdet

            DC.Meas.setSampleClockandSamples(100e3, 10);

            PS.SetOutputEnable(1, true);//VCC@1
            PS.SetOutputEnable(2, true);//VDD@1

            SigGen1Hal.SetConnection(RoutingMatrixVsg.ARB_MARKER_2, RoutingMatrixVsg.PXI_TRIG_1);
            SigGen1Hal.SetConnection(RoutingMatrixVsg.ARB_MARKER_1, RoutingMatrixVsg.PXI_TRIG_0);

            DigitizerHal.ExternalTrigger = ExternalTriggerSourceVsa.PXI_TRIG_0;

            //-------------------- Configure Digital Channels--------------------

            //-----------------------SITE1---------------------------------------
            // PA_EN@1
            Digital.set_vil(1, 1, 0.0);
            Digital.set_vih(1, 1, 3.3);

            // C1@1
            Digital.set_vil(1, 2, 0.0);
            Digital.set_vih(1, 2, 3.3);

            // C0@1
            Digital.set_vil(1, 3, 0.0);
            Digital.set_vih(1, 3, 3.3);

            return status;
        }

        public void ApplicationStart()
        {

            // Start program timer
            programTimer.Reset();
            programTimer.Start();

            if (SigGen1Hal != null) SigGen1Hal.Level = SigGen1Hal.LevelMin;
            if (SigGen2Hal != null) SigGen2Hal.Level = SigGen2Hal.LevelMin;

            //-------------------------------Site1-------------------------------------------

            // VCC@1
            PS.SetVoltage(1, 5.0);
            PS.SetCurrent(1, 0.3);
            DC.Meas.setHPRange(1, HRange.R_500mA);

            // VDD@1
            PS.SetVoltage(2, 5.0);
            PS.SetCurrent(2, 0.3);
            DC.Meas.setHPRange(2, HRange.R_500mA);
            
            //-------------------------------Site2-------------------------------------------



            //Util.WaitTime(10e-3);
            if (writeToConsole)
            {
                Console.WriteLine("\n==================== Device Number {0} ====================\n", dutNumber);
            }
          
        }

        public void ApplicationEnd()
        {
            int testNumber=89;
            if (SigGen1Hal != null) SigGen1Hal.Level = SigGen1Hal.LevelMin;
            if (SigGen2Hal != null) SigGen2Hal.Level = SigGen2Hal.LevelMin;

            double[] testTime = new double[MaxSite];
            testTime[0] = testTime[1] = 0;
            for (int siteNum = 0; siteNum < ActiveSites.Length; siteNum++)

            Digital.cpu_df(1, 1, 1, 0);//PA_EN
            Digital.cpu_df(1, 2, 1, 0);//C1
            Digital.cpu_df(1, 3, 1, 0);//C0
            PS.SetVoltage(1, 0.0);//VCC@1
            PS.SetVoltage(2, 0.0);//VDD@1

            if (useMultithread)
            {
                int counter = 0;
            }

            if (getlocalTime)
            {
                localTimer.Stop();
                localTime[9] = 1e3 * localTimer.ElapsedTicks / (Stopwatch.Frequency * 1e3);
            }

            programTimer.Stop();
            testTime[0] = testTime[1] = 1e3 * programTimer.ElapsedTicks / (Stopwatch.Frequency * 1e3);
            LogTestResult(testNumber++, testTime.ToList());
        }

        public int ApplicationUnload()
        {
            int status = -1;
            dynamicPool.Clear();
            listCalKey.Clear();
          
            return status;
        }
    }

    public class WlanThreadObject_M : WlanThreadObject
    {
        public int SiteNum { get; set; }
    }

    public class DynamicKey
    {
        public int testNumber;
        public string modulationType;
    }

    public class MVP
    {
        public string Pin;
        public string Port;
        public int Site;
    }

    public class CalDataKey
    {
        public string srcSelect;
        public double srcFreq;
        public SrcPort srcPort;
        public MeasPort measPort;
        public string modulationType;
        public double srcLevel;
        public double dutyCycle;
    }

    public class PathFactor
    {
        public string PathMatrix;
        public double CalFactor;
    }

    public class PathFactorVer3
    {
        public MTRF100.SrcPort srcPort;
        public MTRF100.MeasPort srcMeasPort;
        public MTRF200.MeasPort measPort;
        public Dictionary<SrcAmpEnable, double> srcCalFactors;
        public Dictionary<MeasAmpEnable, double> measCalFactors;
    }

    class program
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

            program.ActiveSites = new bool[2] { true ,false};
            program.ProgramDir = @"C:\MerlinTest\Project\KCT8526HW";
            uint numberOfDeviceRuns = 1;

            program.ApplicationLoad();

            for (uint device = 0; device < numberOfDeviceRuns; device++)
            {               
                program.ApplicationStart();

                string wlanAcmcs940Mhz320 = @"C:\MerlinTest\Project\KCT8526HW\Waveforms\wlan11acMCS940MHz320us50pct.tdms";
                string wlan11acMCS940MHz400 = @"C:\MerlinTest\Project\KCT8526HW\Waveforms\wlan11acMCS940MHz400us50pct.tdms";
                string wlan11axMCS1140MHz400 = @"C:\MerlinTest\Project\KCT8526HW\Waveforms\wlan11axMCS1140MHz400us50pct.tdms";
                string wlan11nMCS720MHz320 = @"C:\MerlinTest\Project\KCT8526HW\Waveforms\wlan11nMCS720MHz320us50pct.tdms";

                //program.Measure_OS(1);
                //program.Measure_Idle_Current(1);
                program.LoSharing(false);

                program.Measure_CW_Tests(14, GPIO.WLAN_GAIN_RX_BYPASS, SETUP.TEST_INIT, SETUP.DEVICE_INIT, SETUP.DC_IDLE, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF,-25, 0, 0, 0, 0, 0, SERVO.SERVO_NONE, 2400e6, 2e-3, -20, 1e-3, 1e6, "SG1", "ANT", "RX");
                program.Measure_CW_Tests(16, GPIO.WLAN_GAIN_RX_BYPASS, SETUP.TEST_INIT_OFF, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF, -25, 0, 0, 0, 0, 0, SERVO.SERVO_NONE, 2450e6, 2e-3, -20, 1e-3, 1e6, "SG1", "ANT", "RX");
                program.Measure_CW_Tests(17, GPIO.WLAN_GAIN_RX_BYPASS, SETUP.TEST_INIT_OFF, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF, -25, 0, 0, 0, 0, 0, SERVO.SERVO_NONE, 2500e6, 2e-3, -20, 1e-3, 1e6, "SG1", "ANT", "RX");

                program.Measure_CW_Tests(21, GPIO.WLAN_GAIN_RX, SETUP.TEST_INIT_OFF, SETUP.DEVICE_INIT, SETUP.DC_IDLE, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF, -25, 0, 0, 0, 0, 0, SERVO.SERVO_NONE, 2400e6, 2e-3, 0, 1e-3, 1e6, "SG1", "ANT", "RX");
                program.Measure_CW_Tests(23, GPIO.WLAN_GAIN_RX, SETUP.TEST_INIT_OFF, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF, -25, 0, 0, 0, 0, 0, SERVO.SERVO_NONE, 2450e6, 2e-3, 0, 1e-3, 1e6, "SG1", "ANT", "RX");
                program.Measure_CW_Tests(24, GPIO.WLAN_GAIN_RX, SETUP.TEST_INIT_OFF, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF, -25, 0, 0, 0, 0, 0, SERVO.SERVO_NONE, 2500e6, 2e-3, 0, 1e-3, 1e6, "SG1", "ANT", "RX");

                program.Measure_CW_Tests(28, GPIO.WLAN_GAIN_RX, SETUP.TEST_INIT_OFF, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF, -35, 0, 0.1, 17, 7, 0, SERVO.SERVO_P1DB, 2442e6, 2e-3, -10, 1e-3, 1e6, "SG1", "ANT", "RX");

                program.Measure_NOISE_FIGURE(29, GPIO.WLAN_GAIN_RX, 2442e6, 5e-3, -10, 1e-3, 200e6, "ANT", "RX");

                program.Measure_CW_Tests(31, GPIO.WLAN_GAIN_TX, SETUP.TEST_INIT, SETUP.DEVICE_INIT, SETUP.DC_IDLE, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF, -25, 0, 0, 0, 0, 0, SERVO.SERVO_NONE, 2400e6, 2e-3, -15, 1e-3, 1e6, "SG1", "TX", "ANT");
                program.Measure_CW_Tests(35, GPIO.WLAN_GAIN_TX, SETUP.TEST_INIT_OFF, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF, -25, 0, 0, 0, 0, 0, SERVO.SERVO_NONE, 2450e6, 2e-3, -15, 1e-3, 1e6, "SG1", "TX", "ANT");
                program.Measure_CW_Tests(36, GPIO.WLAN_GAIN_TX, SETUP.TEST_INIT_OFF, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF, -25, 0, 0, 0, 0, 0, SERVO.SERVO_NONE, 2500e6, 2e-3, -15, 1e-3, 1e6, "SG1", "TX", "ANT");

                program.Measure_CW_Tests(61, GPIO.WLAN_GAIN_TX, SETUP.TEST_INIT_OFF, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF, -15, 17, 0.1, 25, 0, 0, SERVO.SERVO_POUT, 2442e6, 2e-3, -1, 1e-3, 1e6, "SG1", "TX", "ANT");
                program.Measure_CW_Tests(64, GPIO.WLAN_GAIN_TX, SETUP.TEST_INIT_OFF, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF, -15, 20, 0.1, 25, 0, 0, SERVO.SERVO_POUT, 2442e6, 2e-3, 2, 1e-3, 1e6, "SG1", "TX", "ANT");
                program.Measure_CW_Tests(67, GPIO.WLAN_GAIN_TX, SETUP.TEST_INIT_OFF, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_2ND, -15, 24, 0.1, 25, 0, 0, SERVO.SERVO_POUT, 2442e6, 2e-3, 6, 1e-3, 1e6, "SG1", "TX", "ANT");

                program.Measure_CW_Tests(71, GPIO.WLAN_GAIN_TX, SETUP.TEST_INIT, SETUP.DEVICE_INIT, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF, 0, 0, 0, 0, 0, 0, SERVO.SERVO_NONE, 2442e6, 2e-3, -20, 1e-3, 1e6, "SG1", "ANT", "RX");

                program.Measure_CW_Tests(72, GPIO.WLAN_GAIN_TX, SETUP.TEST_INIT_OFF, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF, -15, 24, 0.1, 25, 0, 0, SERVO.SERVO_NONE, 2442e6, 2e-3, 6, 1e-3, 1e6, "SG1", "TX", "RX");

                program.Measure_CW_Tests(58, GPIO.WLAN_GAIN_TX, SETUP.TEST_INIT_OFF, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, SETUP.HARM_OFF, -30, 0, 0.1, 25, 26, 26, SERVO.SERVO_P1DB_P3DB, 2442e6, 2e-3, 10, 1e-3, 1e6, "SG1", "TX", "ANT");

                program.LoSharing(true);

                program.Measure_WLAN(40, GPIO.WLAN_GAIN_TX, SETUP.TEST_INIT, SETUP.DEVICE_INIT, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, -20, 0, 0, 25, SERVO.SERVO_NONE, 2422e6, 2e-3, -3,
                     210e-6, 200e-6, 210e-6, 0, 80e6, "SG1", "TX", "ANT", WlanStandard.Wlan802_11ax, WlanBandwidth.Bandwidth40MHz, wlan11axMCS1140MHz400);
                program.Measure_WLAN(41, GPIO.WLAN_GAIN_TX, SETUP.TEST_INIT, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, -20, 0, 0, 25, SERVO.SERVO_NONE, 2462e6, 2e-3, -3,
                     210e-6, 200e-6, 210e-6, 0, 80e6, "SG1", "TX", "ANT", WlanStandard.Wlan802_11ax, WlanBandwidth.Bandwidth40MHz, wlan11axMCS1140MHz400);
                program.Measure_WLAN(42, GPIO.WLAN_GAIN_TX, SETUP.TEST_INIT, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, -20, 0, 0, 25, SERVO.SERVO_NONE, 2422e6, 2e-3, -3,
                     210e-6, 200e-6, 210e-6, 0, 80e6, "SG1", "TX", "ANT", WlanStandard.Wlan802_11ac, WlanBandwidth.Bandwidth40MHz, wlan11acMCS940MHz400);
                program.Measure_WLAN(43, GPIO.WLAN_GAIN_TX, SETUP.TEST_INIT, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, -20, 0, 0, 25, SERVO.SERVO_NONE, 2462e6, 2e-3, -3,
                     170e-6, 160e-6, 170e-6, 0, 80e6, "SG1", "TX", "ANT", WlanStandard.Wlan802_11ac, WlanBandwidth.Bandwidth40MHz, wlanAcmcs940Mhz320);
                program.Measure_WLAN(44, GPIO.WLAN_GAIN_TX, SETUP.TEST_INIT, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, -20, 0, 0, 25, SERVO.SERVO_NONE, 2412e6, 2e-3, -3,
                     170e-6, 160e-6, 170e-6, 0, 40e6, "SG1", "TX", "ANT", WlanStandard.Wlan802_11n, WlanBandwidth.Bandwidth20MHz, wlan11nMCS720MHz320);
                program.Measure_WLAN(45, GPIO.WLAN_GAIN_TX, SETUP.TEST_INIT, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, -20, 0, 0, 25, SERVO.SERVO_NONE, 2472e6, 2e-3, -3,
                     170e-6, 160e-6, 170e-6, 0, 40e6, "SG1", "TX", "ANT", WlanStandard.Wlan802_11n, WlanBandwidth.Bandwidth20MHz, wlan11nMCS720MHz320);
                program.Measure_WLAN(46, GPIO.WLAN_GAIN_TX, SETUP.TEST_INIT, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, -15, 18, 0.1, 25, SERVO.SERVO_POUT, 2462e6, 2e-3, 3,
                     210e-6, 200e-6, 210e-6, 0, 80e6, "SG1", "TX", "ANT", WlanStandard.Wlan802_11ax, WlanBandwidth.Bandwidth40MHz, wlan11axMCS1140MHz400);
                program.Measure_WLAN(48, GPIO.WLAN_GAIN_TX, SETUP.TEST_INIT, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, -15, 18, 0.1, 25, SERVO.SERVO_POUT, 2422e6, 2e-3, 3,
                     210e-6, 200e-6, 210e-6, 0, 80e6, "SG1", "TX", "ANT", WlanStandard.Wlan802_11ax, WlanBandwidth.Bandwidth40MHz, wlan11axMCS1140MHz400);
                program.Measure_WLAN(50, GPIO.WLAN_GAIN_TX, SETUP.TEST_INIT, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, -15, 22, 0.1, 25, SERVO.SERVO_POUT, 2422e6, 2e-3, 7,
                     170e-6, 160e-6, 170e-6, 0, 80e6, "SG1", "TX", "ANT", WlanStandard.Wlan802_11ac, WlanBandwidth.Bandwidth40MHz, wlanAcmcs940Mhz320);
                program.Measure_WLAN(52, GPIO.WLAN_GAIN_TX, SETUP.TEST_INIT, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, -15, 22, 0.1, 25, SERVO.SERVO_POUT, 2462e6, 2e-3, 7,
                     170e-6, 160e-6, 170e-6, 0, 80e6, "SG1", "TX", "ANT", WlanStandard.Wlan802_11ac, WlanBandwidth.Bandwidth40MHz, wlanAcmcs940Mhz320);
                program.Measure_WLAN(54, GPIO.WLAN_GAIN_TX, SETUP.TEST_INIT, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, -15, 23, 0.1, 25, SERVO.SERVO_POUT, 2412e6, 2e-3, 8,
                     170e-6, 160e-6, 170e-6, 0, 40e6, "SG1", "TX", "ANT", WlanStandard.Wlan802_11n, WlanBandwidth.Bandwidth20MHz, wlan11nMCS720MHz320);
                program.Measure_WLAN(56, GPIO.WLAN_GAIN_TX, SETUP.TEST_INIT, SETUP.DEVICE_INIT_OFF, SETUP.DC_IDLE_OFF, SETUP.DC_SETUP_OFF, SETUP.DC_MEAS_OFF, -15, 23, 0.1, 25, SERVO.SERVO_POUT, 2472e6, 2e-3, 8,
                     170e-6, 160e-6, 170e-6, 0, 40e6, "SG1", "TX", "ANT", WlanStandard.Wlan802_11n, WlanBandwidth.Bandwidth20MHz, wlan11nMCS720MHz320);

                ////////program.Measure_DC_Current_retest(51);
                ////////program.Measure_Idle_Current(55);
                program.ApplicationEnd();

                if (device < numberOfDeviceRuns - 1) Util.WaitTime(500e-6); // Wait time between consecutive runs (device cool-down)
            }

            program.ApplicationUnload();

            Console.Write("\nPress any key to quit. ");
            Console.ReadKey();
        }
    }
}






















