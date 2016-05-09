﻿using System;
using Windows.Foundation;
using System.Diagnostics;
using Windows.Devices.Gpio;
using System.Threading;
using Windows.System.Threading;

namespace RobotApp
{
    /// <summary>
    /// **** Motor Control Class ****
    /// Handles pulse timings to motors of robot
    /// </summary>
    class MotorCtrl
    {
        public static void MotorsInit()
        {
            DebounceInit();
            GpioInit();

            ticksPerMs = (ulong)(Stopwatch.Frequency) / 1000;

            workItemThread = Windows.System.Threading.ThreadPool.RunAsync(
                 (source) =>
                 {
                     // setup, ensure pins initialized
                     ManualResetEvent mre = new ManualResetEvent(false);
                     mre.WaitOne(1000);
                     while (!GpioInitialized)
                     {
                         CheckSystem();
                     }

                     waitTime = PulseMs.stop;

                     // main motor timing loop
                     while (true)
                     {
                         //PulseMotor(MotorIds.Left);
                         //mre.WaitOne(2);
                         PulseMotor();
                         mre.WaitOne(2);

                         CheckSystem();
                     }
                 }, WorkItemPriority.High);

        }

        private static IAsyncAction workItemThread;
        private static ulong ticksPerMs;

        const int PWM_PIN = 26;
        //const int ACT_LED_PIN = 47; // rpi2-its-pin47, rpi-its-pin16
        private static GpioController gpioController = null;
        private static GpioPin pwmPin = null;

        public enum PulseMs { stop = -1, ms1 = 0, ms2 = 2 } // values selected for thread-safety
        public static PulseMs waitTime = PulseMs.stop;

        public static int speedValue = 10000;


        /// <summary>
        /// Generate a single motor pulse wave for given servo motor (High for 1 to 2ms, duration for 20ms).
        ///   motor value denotes which moter to send pulse to.
        /// </summary>
        /// <param name="motor"></param>
        private static void PulseMotor()
        {
            speedValue = 10000;

            // Begin pulse (setup for simple Single Speed)
            ulong pulseTicks = ticksPerMs;
            
            if (waitTime == PulseMs.stop) return;
            if (waitTime == PulseMs.ms2) pulseTicks = ticksPerMs * 2;
            pwmPin.Write(GpioPinValue.High);

            // Timing
            ulong delta;
            ulong starttick = (ulong)(MainPage.stopwatch.ElapsedTicks);
            while (true)
            {
                delta = (ulong)(MainPage.stopwatch.ElapsedTicks) - starttick;
                if (delta > (20 * ticksPerMs)) break;
                if (delta > pulseTicks) break;
            }

            // End of pulse
            pwmPin.Write(GpioPinValue.Low);
        }

        /// <summary>
        /// CheckSystem - monitor for priority robot motion conditions (dead stick, or contact with object, etc.)
        /// </summary>
        private static void CheckSystem()
        {
            long msCurTime = MainPage.stopwatch.ElapsedMilliseconds;

            //--- Safety stop robot if no directions for awhile
            if ((msCurTime - Controllers.msLastDirectionTime) > 15000)
            {
                Debug.WriteLine("Safety Stop (CurTime={0}, LastDirTime={1})", msCurTime, Controllers.msLastDirectionTime);
                Controllers.SetRobotDirection(Controllers.CtrlCmds.Stop, (int)Controllers.CtrlSpeeds.Max);
                Controllers.FoundLocalControlsWorking = false;
                if ((msCurTime - Controllers.msLastMessageInTime) > 12000)
                {
                    NetworkCmd.NetworkInit(MainPage.serverHostName);
                }

                Controllers.XboxJoystickCheck();
            }
        }

        private static void MoveMotorsForTime(uint ms)
        {
            if (!GpioInitialized) return;

            ManualResetEvent mre = new ManualResetEvent(false);
            ulong stick = (ulong)MainPage.stopwatch.ElapsedTicks;
            while (true)
            {
                ulong delta = (ulong)(MainPage.stopwatch.ElapsedTicks) - stick;
                if (delta > (ms * ticksPerMs)) break;  // stop motion after given time

                PulseMotor();
                mre.WaitOne(2);
            }
        }

        private static void BackupRobotSequence()
        {
            // stop the robot
            Controllers.SetRobotDirection(Controllers.CtrlCmds.Stop, (int)Controllers.CtrlSpeeds.Max);
            MoveMotorsForTime(200);

            // back away from the obstruction
            Controllers.SetRobotDirection(Controllers.CtrlCmds.Backward, (int)Controllers.CtrlSpeeds.Max);
            MoveMotorsForTime(300);

            // spin 180 degress
            Controllers.SetRobotDirection(Controllers.CtrlCmds.Right, (int)Controllers.CtrlSpeeds.Max);
            MoveMotorsForTime(800);

            // leave in stopped condition
            Controllers.SetRobotDirection(Controllers.CtrlCmds.Stop, (int)Controllers.CtrlSpeeds.Max);
        }

        private static bool GpioInitialized = false;
        private static void GpioInit()
        {
            try
            {
                gpioController = GpioController.GetDefault();
                if (null != gpioController)
                {
                    pwmPin = gpioController.OpenPin(PWM_PIN);
                    pwmPin.SetDriveMode(GpioPinDriveMode.Output);

                    //statusLedPin = gpioController.OpenPin(ACT_LED_PIN);
                    //statusLedPin.SetDriveMode(GpioPinDriveMode.Output);
                    //statusLedPin.Write(GpioPinValue.Low);

                    GpioInitialized = true;
                }
            }
            catch (Exception ex)
            {
                Debug.WriteLine("ERROR: GpioInit failed - " + ex.Message);
            }
        }

        private const int MaxDebs = 10;
        private static int[] debounceValues;
        private static int[] debounceCounts;
        private static int[] debounceLast;
        private static void DebounceInit()
        {
            debounceValues = new int[MaxDebs];
            debounceCounts = new int[MaxDebs];
            debounceLast = new int[MaxDebs];
        }

        /// <summary>
        /// DebounceValue - returns a smoothened, un-rippled, value from a run of given, possibly transient, pin values.  
        ///   curValue = raw pin input value
        ///   ix = an index for a unique pin, or purpose, to locate in array
        ///   run = the maximum number of values, which signify the signal value is un-rippled or solid
        /// </summary>
        /// <param name="curValue"></param>
        /// <param name="ix"></param>
        /// <param name="run"></param>
        /// <returns></returns>
        private static int DebounceValue(int curValue, int ix, int run)
        {
            if (ix < 0 || ix > debounceValues.Length) return 0;
            if (curValue == debounceValues[ix])
            {
                debounceCounts[ix] = 0;
                return curValue;
            }

            if (curValue == debounceLast[ix]) debounceCounts[ix] += 1;
            else debounceCounts[ix] = 0;

            if (debounceCounts[ix] >= run)
            {
                debounceCounts[ix] = run;
                debounceValues[ix] = curValue;
            }
            debounceLast[ix] = curValue;
            return debounceValues[ix];
        }

    }
}

