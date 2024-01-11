using System;
using System.Diagnostics;
using System.IO;
using System.IO.MemoryMappedFiles;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using FreePIE.Core.Contracts;
using Newtonsoft.Json;
using BetterJoyForCemu;
using JoyconPlugin.Fusion;
using static JoyconPlugin.Fusion.FusionMath;
using static JoyconPlugin.Fusion.FusionCalibration;
using WindowsInput.Native;


namespace JoyconPlugin
{

    public static class Settings
    {
        internal static string DriverLocation = @"plugins\JoyconPlugin\";
        internal static int MapSize = 1024;
    }

    //FreePIE Plugin conversion by linkoid
    [GlobalType(IsIndexed = true, Type = typeof(JoyconPluginGlobal))]
    public class JoyconPlugin : IPlugin
    {
        internal Process JoyconDriver;
        internal MemoryMappedFile JoyconMapFile;
        internal MemoryMappedViewStream JoyconMapStream;
        internal MemoryMappedViewAccessor JoyconMapAccessor;
        internal bool isUpdated = false;

        static FusionMatrix gyroscopeMisalignment = new FusionMatrix( 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f);
        static FusionVector gyroscopeSensitivity = new FusionVector(1.0f, 1.0f, 1.0f);
        static FusionVector gyroscopeOffset = new FusionVector( 0.0f, 0.0f, 0.0f);
        static FusionMatrix accelerometerMisalignment = new FusionMatrix(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f);
        static FusionVector accelerometerSensitivity = new FusionVector(1.0f, 1.0f, 1.0f);
        static FusionVector accelerometerOffset = new FusionVector(0.0f, 0.0f, 0.0f);
        static FusionMatrix softIronMatrix = new FusionMatrix( 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f);
        static FusionVector hardIronOffset = new FusionVector( 0.0f, 0.0f, 0.0f);

        public static joyconInputs[] joyconInputList = new joyconInputs[4];
        public static JoyconPluginGlobal[] joyconInputListGlobal = new JoyconPluginGlobal[4];

        public void onUpdate() { }

        public static void onJoyconUpdate(Joycon jc)
        {
            try
            {
                FusionVector gyroscope = new FusionVector()
                {
                    axis = new FusionVectorAxis()
                    {
                        x = (float)jc.GetGyro().X,
                        y = (float)jc.GetGyro().Y,
                        z = (float)jc.GetGyro().Z
                    }
                };
                double g = 9.80665;
                FusionVector accelerometer = new FusionVector()
                {
                    axis = new FusionVectorAxis()
                    {
                        x = (float)(jc.GetAccel().X * 10 / g),
                        y = (float)(jc.GetAccel().Y * 10 / g),
                        z = (float)(jc.GetAccel().Z * 10 / g)
                    }
                };

                // Apply calibration
                gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
                accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);

                // Update gyroscope offset correction algorithm
                gyroscope = joyconInputList[jc.PadId].offset.FusionOffsetUpdate(gyroscope);

                joyconInputList[jc.PadId].ahrs.FusionAhrsUpdateNoMagnetometer(gyroscope, accelerometer, 0.005f);
                
                FusionEuler eul = FusionQuaternionToEuler(joyconInputList[jc.PadId].ahrs.FusionAhrsGetQuaternion());
                joyconInputList[jc.PadId].acceleration = new SmallVector3()
                {
                    x = accelerometer.axis.x,
                    y = accelerometer.axis.y,
                    z = accelerometer.axis.z
                };
                joyconInputList[jc.PadId].gyro = new Angles()
                {
                    yaw = gyroscope.axis.z,
                    pitch = gyroscope.axis.y,
                    roll = gyroscope.axis.x,
                };
                joyconInputList[jc.PadId].rotation = new Angles()
                {
                    yaw = eul.angle.yaw,
                    pitch = eul.angle.pitch,
                    roll = eul.angle.roll,
                };
                joyconInputList[jc.PadId].a = jc.GetButton(Joycon.Button.A);
                joyconInputList[jc.PadId].b = jc.GetButton(Joycon.Button.B);
                joyconInputList[jc.PadId].x = jc.GetButton(Joycon.Button.X);
                joyconInputList[jc.PadId].y = jc.GetButton(Joycon.Button.Y);
                joyconInputList[jc.PadId].shoulder1 = jc.GetButton(Joycon.Button.SHOULDER_1);
                joyconInputList[jc.PadId].shoulder2 = jc.GetButton(Joycon.Button.SHOULDER_2);
                joyconInputList[jc.PadId].home = jc.GetButton(Joycon.Button.HOME);
                joyconInputList[jc.PadId].capture = jc.GetButton(Joycon.Button.CAPTURE);
                joyconInputList[jc.PadId].plus = jc.GetButton(Joycon.Button.PLUS);
                joyconInputList[jc.PadId].minus = jc.GetButton(Joycon.Button.MINUS);
                joyconInputList[jc.PadId].sl = jc.GetButton(Joycon.Button.SL);
                joyconInputList[jc.PadId].sr = jc.GetButton(Joycon.Button.SR);
                joyconInputList[jc.PadId].down = jc.GetButton(Joycon.Button.DPAD_DOWN);
                joyconInputList[jc.PadId].left = jc.GetButton(Joycon.Button.DPAD_LEFT);
                joyconInputList[jc.PadId].right = jc.GetButton(Joycon.Button.DPAD_RIGHT);
                joyconInputList[jc.PadId].up = jc.GetButton(Joycon.Button.DPAD_UP);
                joyconInputList[jc.PadId].stickBtn = jc.GetButton(Joycon.Button.STICK);
                joyconInputList[jc.PadId].stick.x = jc.GetStick()[0];
                joyconInputList[jc.PadId].stick.y = jc.GetStick()[1];
                if(jc.isPro) joyconInputList[jc.PadId].type = 2;
                if(jc.isLeft) joyconInputList[jc.PadId].type = 0;
                else joyconInputList[jc.PadId].type = 1;
                joyconInputList[jc.PadId].loaded = !joyconInputList[jc.PadId].ahrs.FusionAhrsGetFlags().initialising;
            }
            catch (NullReferenceException ex) { Trace.WriteLine("\nJoyconFile NullException : " + ex + " = "); }
        }

        public object CreateGlobal()
        {
            for (int i = 0; i < 4; i++)
            {
                joyconInputList.SetValue(new joyconInputs(), i);
                joyconInputListGlobal.SetValue(new JoyconPluginGlobal(this, i), i);
            }

            return joyconInputListGlobal;
        }

        public Action Start()
        {

            Program.Main(null);

            return null;

        }

        public void Stop()
        {
            Program.form.BeginInvoke(new Action(() =>
            {
                Program.form.Close();
            }));
        }

        public event EventHandler Started;

        public string FriendlyName
        {
            get { return "Joycon Plugin"; }
        }

        public bool GetProperty(int index, IPluginProperty property)
        {
            return false;
        }

        public bool SetProperties(Dictionary<string, object> properties)
        {
            return false;
        }

        public void DoBeforeNextExecute()
        {
            //This method will be executed each iteration of the script
            //onJoyconUpdate();
            //for (int i = 0; i < 16; i++)
            //{
            //    joyconInputs input = joyconInputList[i];
            //    input.accelUpdate.Invoke(this,new EventArgs());
            //    input.buttonUpdate.Invoke(this, new EventArgs());
            //    input.gyroUpdate.Invoke(this, new EventArgs());
            //    input.leftStickUpdate.Invoke(this, new EventArgs());
            //    input.rightStickUpdate.Invoke(this, new EventArgs());
            //    input.rotationUpdate.Invoke(this, new EventArgs());
            //}

            for (int i = 0; i < 4; i++)
            {
                joyconInputList[i].Updated = false;
            }
        }
    }

    public class JoyconObjectLists
    {
        public List<JoyconPluginGlobal> GlobalObjects;
    }

    [Global(Name = "joycon")]
    public class JoyconPluginGlobal
    {
        private readonly JoyconPlugin myPlugin;
        private readonly int myId;
        private readonly joyconInputs myInputs;

        private readonly GyroGlobal _gyro;
        private readonly RotationGlobal _rotation;
        private readonly AccelerationGlobal _acceleration;
        private readonly StickGlobal _stick;
        private readonly ButtonsGlobal _buttons;

        public JoyconPluginGlobal(JoyconPlugin plug, int id)
        {
            this.myPlugin = plug;
            this.myId = id;
            this.myInputs = JoyconPlugin.joyconInputList[id];

            _gyro = new GyroGlobal(this.myInputs);
            _rotation = new RotationGlobal(this.myInputs);
            _acceleration = new AccelerationGlobal(this.myInputs);
            _stick = new StickGlobal(this.myInputs);
            _buttons = new ButtonsGlobal(this.myInputs);
        }

        public GyroGlobal gyro { get { return _gyro; } }
        public RotationGlobal rotation { get { return _rotation; } }
        public AccelerationGlobal acceleration { get { return _acceleration; } }
        public StickGlobal stick { get { return _stick; } }
        public ButtonsGlobal buttons { get { return _buttons; } }

        public int type { get { return myInputs.type; } }
        public int updates { get { return myInputs.updates; } }
        public bool loaded { get { return myInputs.loaded; } }
    }

    [Global(Name = "gyro")]
    public class GyroGlobal
    {
        private readonly joyconInputs input;
        public event EventHandler update;
        public GyroGlobal(joyconInputs input)
        {
            this.input = input;
            update = input.gyroUpdate;
        }

        public double yaw { get { return input.gyro.yaw; } }
        public double pitch { get { return input.gyro.pitch; } }
        public double roll { get { return input.gyro.roll; } }
    }

    [Global(Name = "rotation")]
    public class RotationGlobal
    {
        private readonly joyconInputs input;
        public event EventHandler update;
        public RotationGlobal(joyconInputs input)
        {
            this.input = input;
            update = input.rotationUpdate;
        }

        public double yaw { get { return input.rotation.yaw; } }
        public double pitch { get { return input.rotation.pitch; } }
        public double roll { get { return input.rotation.roll; } }
    }

    [Global(Name = "acceleration")]
    public class AccelerationGlobal
    {
        private readonly joyconInputs input;
        public event EventHandler update;
        public AccelerationGlobal(joyconInputs input)
        {
            this.input = input;
            update = input.accelUpdate;
        }

        public double x { get { return input.acceleration.x; } }
        public double y { get { return input.acceleration.y; } }
        public double z { get { return input.acceleration.z; } }
    }

    [Global(Name = "stick")]
    public class StickGlobal
    {
        private readonly joyconInputs input;
        public event EventHandler update;
        public StickGlobal(joyconInputs input)
        {
            this.input = input;
            update = input.stickUpdate;
        }

        public double x { get { return input.stick.x; } }
        public double y { get { return input.stick.y; } }
    }


    [Global(Name = "buttons")]
    public class ButtonsGlobal
    {
        private readonly joyconInputs input;
        public event EventHandler update;
        public ButtonsGlobal(joyconInputs input)
        {
            this.input = input;
            update = input.buttonUpdate;
        }

        public bool stick {  get { return input.stickBtn;  } }

        public bool shoulder1 { get { return input.shoulder1; } }
        public bool shoulder2 { get { return input.shoulder2; } }

        public bool up { get { return input.up; } }
        public bool down { get { return input.down; } }
        public bool left { get { return input.left; } }
        public bool right { get { return input.right; } }

        public bool a { get { return input.a; } }
        public bool b { get { return input.b; } }
        public bool x { get { return input.x; } }
        public bool y { get { return input.y; } }

        public bool home { get { return input.home; } }
        public bool capture { get { return input.capture; } }
        public bool minus { get { return input.minus; } }
        public bool plus { get { return input.plus; } }

        public bool sl { get { return input.sl; } }
        public bool sr { get { return input.sr; } }
    }




    public class joyconInputs
    {
        public bool Updated = false;
        public int type = 0;

        public Angles gyro = new Angles();
        public Angles rotation = new Angles();
        public SmallVector3 acceleration = new SmallVector3();

        public FusionAhrs ahrs = new FusionAhrs();
        public FusionOffset offset = new FusionOffset(200);

        public int updates = 0;

        public Vector2 stick = new Vector2();

        public bool stickBtn;

        public bool shoulder1;
        public bool shoulder2;

        public bool up;
        public bool down;
        public bool left;
        public bool right;

        public bool a;
        public bool b;
        public bool x;
        public bool y;

        public bool home;
        public bool capture;
        public bool plus;
        public bool minus;

        public bool loaded;

        public bool sl;
        public bool sr;

        public EventHandler stickUpdate;
        public EventHandler buttonUpdate;
        public EventHandler rotationUpdate;
        public EventHandler gyroUpdate;
        public EventHandler accelUpdate;

        public joyconInputs()
        {
            //rightStickUpdate = new EventHandler(rightStickUpdate);
            //leftStickUpdate  = new EventHandler(leftStickUpdate);
            //buttonUpdate     = new EventHandler(buttonUpdate);
            //rotationUpdate   = new EventHandler(rotationUpdate);
            //gyroUpdate       = new EventHandler(gyroUpdate);
            //accelUpdate      = new EventHandler(accelUpdate);
            ahrs.FusionAhrsSetSettings(new FusionAhrs.FusionAhrsSettings()
            {
                convention = FusionConvention.FusionConventionNwu,
                gain = 0.5f,
                gyroscopeRange = 2000.0f, // replace this with actual gyroscope range in degrees/s 
                accelerationRejection = 30.0f,
                magneticRejection = 0,
                recoveryTriggerPeriod = 0, // 5 seconds = 5 * (1/0.005s = 200Hz)
            });
        }
    }


    public partial class Angles
    {
        public double _yaw = 0;
        public double _pitch = 0;
        public double _roll = 0;

        public double yaw { get { return _yaw; } set { _yaw = value; } }
        public double pitch;
        public double roll;
    }

    public partial class Vector2
    {
        public double x;
        public double y;
    }

    public partial class SmallVector3
    {
        public double x;
        public double y;
        public double z;
    }
}