using MerlinTest.Tools.Diagnostic.Logging;
using MT.APS100.Model;
using MT.TesterDriver;
using MT.TesterDriver.VNA;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace KCT8526HW
{
    public class ApplicationBaseMrk2 : ApplicationBase
    {

        protected TraceLoss TraceCal = new TraceLoss();

        /// <summary>
        /// Gets / Sets the diagnostic logging object.
        /// </summary>
        protected NetworkTraceSource trace { get; set; }

        protected const bool writeToDatalog = true;  // Write test results to datalog
        protected const bool writeToConsole = true; // Print test results to console window
        protected const bool writeToFile = false;    // Write test results to file
        protected const bool useTraceCalData = true; // Apply RF test fixture trace loss
        protected bool writeInstrumentStatusToConsole = false; // Print instrument state to console window

        protected MerlinTest.TesterDriver.CalImport.NoiseFigurePortModule nf = new MerlinTest.TesterDriver.CalImport.NoiseFigurePortModule();
        protected MerlinTest.TesterDriver.CalImport.PortModuleUserCal PortModuleCal = new MerlinTest.TesterDriver.CalImport.PortModuleUserCal();

        protected PSM DC;
        protected PowerSupply PS;
        protected pe32h Digital;
        protected MTRF100 rf100;
        protected MTRF200 rf200;
        protected SwitchMatrix Matrix;
        protected mtVNA VNA;

        protected ISignalGenerator SigGen1Hal;
        protected ISignalGenerator SigGen2Hal;
        protected IDigitizer DigitizerHal;

        protected Stopwatch programTimer = new Stopwatch(); // Performance timer (test program)

        public enum SERVO { SERVO_POUT, SERVO_P1DB, SERVO_P3DB, SERVO_P1DB_P3DB, SERVO_NONE }

        protected double[] VCC1_ICC = new double[2];
        protected double[] VCC2_ICC = new double[2];
        protected double[] VDD_ICC = new double[2];
        protected double[] Total_ICC = new double[2];

        protected double[] gain = new double[2];
        protected double[] gainMin = new double[2] { 10000, 10000 };
        protected double[] gainMax = new double[2] { -10000, -10000 };

        protected double[] Pin = new double[2];
        protected double[] Pout = new double[2];
        protected double[] Pinoffset = new double[2];
        protected double[] Poutoffset = new double[2];
        protected double[] Gainoffset = new double[2];


        protected double[] ANT_Pout = new double[2];
        protected double[] RX_Pout = new double[2];
        protected double[] TX_2442 = new double[2];
        protected double[] Gain_2442_RXBP = new double[2];
        protected double[] Gain_2442_RX = new double[2];
        protected double[] Gain_2442_TX = new double[2];
        protected double[] VDET = new double[2];
        protected double[] VDET_TARGET = new double[2];

        protected double[] Pout1dB = new double[2];
        protected double[] Pout3dB = new double[2];
        protected double[] gainDelta = new double[2];
        protected double[] gainDeltaX = new double[2];
        //protected double[] DeltaGain = new double[2];


        protected double[] Harmonic_2nd = new double[2];
        protected double[] Harmonic_3rd = new double[2];

        protected double[] evm = new double[2];
        protected double[] evmdB = new double[2];
        protected double[] masklevel = new double[2];
        protected bool[] maskpass = new bool[2];

        protected double[] Vdet = new double[2];
        protected double[] VdetOff = new double[2];
        protected double[] GainPout1dB = new double[2];
        protected double[] GainPout3dB = new double[2];

        protected double[] noiseGain = new double[2];
        protected double[] noiseFigure = new double[2];

        protected double[] S11Mag = new double[2];
        protected double[] S11Phase = new double[2];
        protected double[] S21Mag = new double[2];
        protected double[] S21Phase = new double[2];
        protected double[] S12Mag = new double[2];
        protected double[] S12Phase = new double[2];
        protected double[] S22Mag = new double[2];
        protected double[] S22Phase = new double[2];


        protected string currentWaveform;
        protected double srcCalFactor = 0;
        protected double measCalFactor = 0;
        protected uint dutNumber = 1;
        protected double testTime = 0;

        protected bool isOSPass = true;

        protected WlanRfmxAnalysis wlanAnalysis = new WlanRfmxAnalysis();
    }
}


