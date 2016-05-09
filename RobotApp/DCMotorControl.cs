using System;
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
    class DCMotorCtrl
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

                     Controllers.SetRobotDirection(Controllers.CtrlCmds.Stop, (int)Controllers.CtrlSpeeds.Max);

                     // settle period - to dismiss transient startup conditions, as things are starting up
                     for (int x = 0; x < 10; ++x)
                     {
                         mre.WaitOne(100);
                     }

                     // main motor timing loop
                     while (true)
                     {
                         // left motor
                         if (waitTimeLeft == PulseMs.stop)// && leftRunning)
                         {
                             leftPwmPin.Write(GpioPinValue.Low);
                             mre.WaitOne(2);
                         }
                         else if ((waitTimeLeft == PulseMs.ms1 || waitTimeLeft == PulseMs.ms2))// && !leftRunning)
                         {
                             leftPwmPin.Write(GpioPinValue.High);
                             mre.WaitOne(2);
                         }
                         else
                         {
                             leftPwmPin.Write(GpioPinValue.Low);
                             mre.WaitOne(2);
                         }

                         // right motor
                         if (waitTimeRight == PulseMs.stop)// && rightRunning)
                         {
                             rightPwmPin.Write(GpioPinValue.Low);
                             mre.WaitOne(2);
                         }
                         else if ((waitTimeRight == PulseMs.ms1 || waitTimeRight == PulseMs.ms2))// && !rightRunning)
                         {
                             rightPwmPin.Write(GpioPinValue.High);
                             mre.WaitOne(2);
                         }
                         else
                         {
                             rightPwmPin.Write(GpioPinValue.Low);
                             mre.WaitOne(2);
                         }
                         CheckSystem();
                     }
                 }, WorkItemPriority.High);

            fireBall = Windows.System.Threading.ThreadPool.RunAsync(
                 (source) =>
                 {
                     // setup, ensure pins initialized
                     ManualResetEvent mre = new ManualResetEvent(false);
                     mre.WaitOne(1000);
                     while (!GpioInitialized)
                     {
                         CheckSystem();
                     }

                     // main motor timing loop
                     while (true)
                     {
                         if (firing)
                         {
                             leftFirePin.Write(GpioPinValue.High);
                             rightFirePin.Write(GpioPinValue.High);
                             MotorCtrl.waitTime = MotorCtrl.PulseMs.ms1;
                             mre.WaitOne(650);
                             MotorCtrl.waitTime = MotorCtrl.PulseMs.ms2;
                             mre.WaitOne(440);
                             leftFirePin.Write(GpioPinValue.Low);
                             rightFirePin.Write(GpioPinValue.Low);
                             MotorCtrl.waitTime = MotorCtrl.PulseMs.stop;
                             mre.WaitOne(1000);  // wait for the ball to reload
                             firing = false;
                         }
                     }
                 }, WorkItemPriority.High);
            
        }

        private static IAsyncAction workItemThread;
        private static IAsyncAction fireBall;
        private static ulong ticksPerMs;

        const int LEFT_PWM_PIN = 5;
        const int RIGHT_PWM_PIN = 6;
        const int LEFT_FIRE_PIN = 13;
        const int RIGHT_FIRE_PIN = 19;
        private static GpioController gpioController = null;
        private static GpioPin leftPwmPin = null;
        private static GpioPin rightPwmPin = null;
        private static GpioPin leftFirePin = null;
        private static GpioPin rightFirePin = null;

        private enum MotorIds { Left, Right };
        public enum PulseMs { stop = -1, ms1 = 0, ms2 = 2 } // values selected for thread-safety
        public static PulseMs waitTimeLeft = PulseMs.stop;
        public static PulseMs waitTimeRight = PulseMs.stop;

        public static int speedValue = 10000;
        public static bool firing = false;

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
                    leftPwmPin = gpioController.OpenPin(LEFT_PWM_PIN);
                    leftPwmPin.SetDriveMode(GpioPinDriveMode.Output);
                    leftPwmPin.Write(GpioPinValue.Low);

                    rightPwmPin = gpioController.OpenPin(RIGHT_PWM_PIN);
                    rightPwmPin.SetDriveMode(GpioPinDriveMode.Output);
                    rightPwmPin.Write(GpioPinValue.Low);

                    leftFirePin = gpioController.OpenPin(LEFT_FIRE_PIN);
                    leftFirePin.SetDriveMode(GpioPinDriveMode.Output);
                    leftFirePin.Write(GpioPinValue.Low);

                    rightFirePin = gpioController.OpenPin(RIGHT_FIRE_PIN);
                    rightFirePin.SetDriveMode(GpioPinDriveMode.Output);
                    rightFirePin.Write(GpioPinValue.Low);

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
