﻿using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Configuration;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Http;
using System.Net.NetworkInformation;
using System.Runtime.InteropServices;
using System.ServiceProcess;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Timers;
using System.Windows.Forms;
using BetterJoyForCemu.Collections;
using static BetterJoyForCemu._3rdPartyControllers;
using static BetterJoyForCemu.HIDapi;

namespace BetterJoyForCemu {
    public class JoyconManager {
        public bool EnableIMU = true;
        public bool EnableLocalize = false;

        private const ushort vendor_id = 0x57e;
        private const ushort product_l = 0x2006;
        private const ushort product_r = 0x2007;
        private const ushort product_pro = 0x2009;
        private const ushort product_snes = 0x2017;

        public ConcurrentList<Joycon> j { get; private set; } // Array of all connected Joy-Cons
        static JoyconManager instance;

        public MainForm form;

        System.Timers.Timer controllerCheck;

        public static JoyconManager Instance {
            get { return instance; }
        }

        public void Awake() {
            instance = this;
            j = new ConcurrentList<Joycon>();
            HIDapi.hid_init();
        }

        public void Start() {
            controllerCheck = new System.Timers.Timer(2000); // check for new controllers every 2 seconds
            controllerCheck.Elapsed += CheckForNewControllersTime;
            controllerCheck.Start();
        }

        bool ControllerAlreadyAdded(string path) {
            foreach (Joycon v in j)
                if (v.path == path)
                    return true;
            return false;
        }

        void CleanUp() { // removes dropped controllers from list
            List<Joycon> rem = new List<Joycon>();
            foreach (Joycon joycon in j) {
                if (joycon.state == Joycon.state_.DROPPED) {
                    if (joycon.other != null)
                        joycon.other.other = null; // The other of the other is the joycon itself

                    joycon.Detach(true);
                    rem.Add(joycon);

                    foreach (Button b in form.con) {
                        if (b.Enabled & b.Tag == joycon) {
                            b.Invoke(new MethodInvoker(delegate {
                                b.BackColor = System.Drawing.Color.FromArgb(0x00, System.Drawing.SystemColors.Control);
                                b.Enabled = false;
                                b.BackgroundImage = JoyconPlugin.Properties.Resources.cross;
                            }));
                            break;
                        }
                    }

                    form.AppendTextBox("Removed dropped controller. Can be reconnected.\r\n");
                }
            }

            foreach (Joycon v in rem)
                j.Remove(v);
        }

        void CheckForNewControllersTime(Object source, ElapsedEventArgs e) {
            CleanUp();
            if (Config.IntValue("ProgressiveScan") == 1) {
                CheckForNewControllers();
            }
        }

        private ushort TypeToProdId(byte type) {
            switch (type) {
                case 1:
                    return product_pro;
                case 2:
                    return product_l;
                case 3:
                    return product_r;
            }
            return 0;
        }

        public void CheckForNewControllers() {
            // move all code for initializing devices here and well as the initial code from Start()
            bool isLeft = false;
            IntPtr ptr = HIDapi.hid_enumerate(0x0, 0x0);
            IntPtr top_ptr = ptr;

            hid_device_info enumerate; // Add device to list
            bool foundNew = false;
            while (ptr != IntPtr.Zero) {
                SController thirdParty = null;
                enumerate = (hid_device_info)Marshal.PtrToStructure(ptr, typeof(hid_device_info));

                if (enumerate.serial_number == null) {
                    ptr = enumerate.next; // can't believe it took me this long to figure out why USB connections used up so much CPU.
                                          // it was getting stuck in an inf loop here!
                    continue;
                }

                bool validController = (enumerate.product_id == product_l || enumerate.product_id == product_r ||
                                        enumerate.product_id == product_pro || enumerate.product_id == product_snes) && enumerate.vendor_id == vendor_id;
                // check list of custom controllers specified
                foreach (SController v in Program.thirdPartyCons) {
                    if (enumerate.vendor_id == v.vendor_id && enumerate.product_id == v.product_id && enumerate.serial_number == v.serial_number) {
                        validController = true;
                        thirdParty = v;
                        break;
                    }
                }

                ushort prod_id = thirdParty == null ? enumerate.product_id : TypeToProdId(thirdParty.type);
                if (prod_id == 0) {
                    ptr = enumerate.next; // controller was not assigned a type, but advance ptr anyway
                    continue;
                }

                if (validController && !ControllerAlreadyAdded(enumerate.path)) {
                    switch (prod_id) {
                        case product_l:
                            isLeft = true;
                            form.AppendTextBox("Left Joy-Con connected.\r\n"); break;
                        case product_r:
                            isLeft = false;
                            form.AppendTextBox("Right Joy-Con connected.\r\n"); break;
                        case product_pro:
                            isLeft = true;
                            form.AppendTextBox("Pro controller connected.\r\n"); break;
                        case product_snes:
                            isLeft = true;
                            form.AppendTextBox("SNES controller connected.\r\n"); break;
                        default:
                            form.AppendTextBox("Non Joy-Con Nintendo input device skipped.\r\n"); break;
                    }

                    // Add controller to block-list for HidGuardian
                    if (Program.useHIDG) {
                        HttpWebRequest request = (HttpWebRequest)WebRequest.Create(@"http://localhost:26762/api/v1/hidguardian/affected/add/");
                        string postData = @"hwids=HID\" + enumerate.path.Split('#')[1].ToUpper();
                        var data = Encoding.UTF8.GetBytes(postData);

                        request.Method = "POST";
                        request.ContentType = "application/x-www-form-urlencoded; charset=UTF-8";
                        request.ContentLength = data.Length;

                        using (var stream = request.GetRequestStream())
                            stream.Write(data, 0, data.Length);

                        try {
                            var response = (HttpWebResponse)request.GetResponse();
                            var responseString = new StreamReader(response.GetResponseStream()).ReadToEnd();
                        } catch {
                            form.AppendTextBox("Unable to add controller to block-list.\r\n");
                        }
                    }
                    // -------------------- //

                    IntPtr handle = HIDapi.hid_open_path(enumerate.path);
                    try {
                        HIDapi.hid_set_nonblocking(handle, 1);
                    } catch {
                        form.AppendTextBox("Unable to open path to device - are you using the correct (64 vs 32-bit) version for your PC?\r\n");
                        break;
                    }

                    bool isPro = prod_id == product_pro;
                    bool isSnes = prod_id == product_snes;
                    j.Add(new Joycon(handle, EnableIMU, EnableLocalize & EnableIMU, 0.05f, isLeft, enumerate.path, enumerate.serial_number, j.Count, isPro, isSnes, thirdParty != null));

                    foundNew = true;
                    j.Last().form = form;

                    if (j.Count < 5) {
                        int ii = -1;
                        foreach (Button v in form.con) {
                            ii++;
                            if (!v.Enabled) {
                                System.Drawing.Bitmap temp;
                                switch (prod_id) {
                                    case (product_l):
                                        temp = JoyconPlugin.Properties.Resources.jc_left_s; break;
                                    case (product_r):
                                        temp = JoyconPlugin.Properties.Resources.jc_right_s; break;
                                    case (product_pro):
                                        temp = JoyconPlugin.Properties.Resources.pro; break;
                                    case (product_snes):
                                        temp = JoyconPlugin.Properties.Resources.snes; break;
                                    default:
                                        temp = JoyconPlugin.Properties.Resources.cross; break;
                                }

                                v.Invoke(new MethodInvoker(delegate {
                                    v.Tag = j.Last(); // assign controller to button
                                    v.Enabled = true;
                                    v.Click += new EventHandler(form.conBtnClick);
                                    v.BackgroundImage = temp;
                                }));

                                form.loc[ii].Invoke(new MethodInvoker(delegate {
                                    form.loc[ii].Tag = v;
                                    form.loc[ii].Click += new EventHandler(form.locBtnClickAsync);
                                }));

                                break;
                            }
                        }
                    }

                    byte[] mac = new byte[6];
                    try {
                        for (int n = 0; n < 6; n++)
                            mac[n] = byte.Parse(enumerate.serial_number.Substring(n * 2, 2), System.Globalization.NumberStyles.HexNumber);
                    } catch (Exception e) {
                        // could not parse mac address
                    }
                    j[j.Count - 1].PadMacAddress = new PhysicalAddress(mac);
                }

                ptr = enumerate.next;
            }

            if (foundNew) { // attempt to auto join-up joycons on connection
                Joycon temp = null;
                foreach (Joycon v in j) {
                    // Do not attach two controllers if they are either:
                    // - Not a Joycon
                    // - Already attached to another Joycon (that isn't itself)
                    if (v.isPro || (v.other != null && v.other != v)) {
                        continue;
                    }

                    // Otherwise, iterate through and find the Joycon with the lowest
                    // id that has not been attached already (Does not include self)
                    if (temp == null)
                        temp = v;
                    else if (temp.isLeft != v.isLeft && v.other == null) {
                        temp.other = v;
                        v.other = temp;

                        foreach (Button b in form.con)
                            if (b.Tag == v || b.Tag == temp) {
                                Joycon tt = (b.Tag == v) ? v : (b.Tag == temp) ? temp : v;
                                b.BackgroundImage = tt.isLeft ? JoyconPlugin.Properties.Resources.jc_left : JoyconPlugin.Properties.Resources.jc_right;
                            }

                        temp = null;    // repeat
                    }
                }
            }

            HIDapi.hid_free_enumeration(top_ptr);

            bool on = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None).AppSettings.Settings["HomeLEDOn"].Value.ToLower() == "true";
            foreach (Joycon jc in j) { // Connect device straight away
                if (jc.state == Joycon.state_.NOT_ATTACHED) {

                    try {
                        jc.Attach();
                    } catch (Exception e) {
                        jc.state = Joycon.state_.DROPPED;
                        continue;
                    }

                    jc.SetHomeLight(on);

                    jc.Begin();
                    if (form.allowCalibration) {
                        jc.getActiveData();
                    }
                }
            }
        }

        public void OnApplicationQuit() {
            foreach (Joycon v in j) {
                if (Boolean.Parse(ConfigurationManager.AppSettings["AutoPowerOff"]))
                    v.PowerOff();

                v.Detach();

            }

            controllerCheck.Stop();
            HIDapi.hid_exit();
        }
    }

    class Program {
        public static PhysicalAddress btMAC = new PhysicalAddress(new byte[] { 0, 0, 0, 0, 0, 0 });
        public static UdpServer server;

        private static readonly HttpClient client = new HttpClient();

        public static JoyconManager mgr;
        static string pid;

        public static MainForm form;

        static public bool useHIDG = Boolean.Parse(ConfigurationManager.AppSettings["UseHIDG"]);

        public static List<SController> thirdPartyCons = new List<SController>();

        private static WindowsInput.Events.Sources.IKeyboardEventSource keyboard;
        private static WindowsInput.Events.Sources.IMouseEventSource mouse;

        public static void Start() {
            pid = Process.GetCurrentProcess().Id.ToString(); // get current process id for HidCerberus.Srv

            if (useHIDG) {
                form.console.AppendText("HidGuardian is enabled.\r\n");
                try {
                    var HidCerberusService = new ServiceController("HidCerberus Service");
                    if (HidCerberusService.Status == ServiceControllerStatus.Stopped) {
                        form.console.AppendText("HidGuardian was stopped. Starting...\r\n");

                        try {
                            HidCerberusService.Start();
                        } catch (Exception e) {
                            form.console.AppendText("Unable to start HidGuardian - everything should work fine without it, but if you need it, run the app again as an admin.\r\n");
                            useHIDG = false;
                        }
                    }
                } catch (Exception e) {
                    form.console.AppendText("Unable to start HidGuardian - everything should work fine without it, but if you need it, install it properly as admin.\r\n");
                    useHIDG = false;
                }

                HttpWebResponse response;
                if (Boolean.Parse(ConfigurationManager.AppSettings["PurgeWhitelist"])) {
                    try {
                        response = (HttpWebResponse)WebRequest.Create(@"http://localhost:26762/api/v1/hidguardian/whitelist/purge/").GetResponse(); // remove all programs allowed to see controller
                    } catch (Exception e) {
                        form.console.AppendText("Unable to purge whitelist.\r\n");
                        useHIDG = false;
                    }
                }

                try {
                    response = (HttpWebResponse)WebRequest.Create(@"http://localhost:26762/api/v1/hidguardian/whitelist/add/" + pid).GetResponse(); // add BetterJoyForCemu to allowed processes 
                } catch (Exception e) {
                    form.console.AppendText("Unable to add program to whitelist.\r\n");
                    useHIDG = false;
                }
            }

            foreach (NetworkInterface nic in NetworkInterface.GetAllNetworkInterfaces()) {
                // Get local BT host MAC
                if (nic.NetworkInterfaceType != NetworkInterfaceType.FastEthernetFx && nic.NetworkInterfaceType != NetworkInterfaceType.Wireless80211) {
                    if (nic.Name.Split()[0] == "Bluetooth") {
                        btMAC = nic.GetPhysicalAddress();
                    }
                }
            }

            // a bit hacky
            _3rdPartyControllers partyForm = new _3rdPartyControllers();
            partyForm.CopyCustomControllers();

            mgr = new JoyconManager();
            mgr.form = form;
            mgr.Awake();
            mgr.CheckForNewControllers();
            mgr.Start();

            server = new UdpServer(mgr.j);
            server.form = form;

            server.Start(IPAddress.Parse(ConfigurationManager.AppSettings["IP"]), Int32.Parse(ConfigurationManager.AppSettings["Port"]));

            // Capture keyboard + mouse events for binding's sake
            keyboard = WindowsInput.Capture.Global.KeyboardAsync();
            keyboard.KeyEvent += Keyboard_KeyEvent;
            mouse = WindowsInput.Capture.Global.MouseAsync();
            mouse.MouseEvent += Mouse_MouseEvent;

            form.console.AppendText("All systems go\r\n");
        }

        private static void Mouse_MouseEvent(object sender, WindowsInput.Events.Sources.EventSourceEventArgs<WindowsInput.Events.Sources.MouseEvent> e) {
            if (e.Data.ButtonDown != null) {
                string res_val = Config.Value("reset_mouse");
                if (res_val.StartsWith("mse_"))
                    if ((int)e.Data.ButtonDown.Button == Int32.Parse(res_val.Substring(4)))
                        WindowsInput.Simulate.Events().MoveTo(Screen.PrimaryScreen.Bounds.Width / 2, Screen.PrimaryScreen.Bounds.Height / 2).Invoke();

                res_val = Config.Value("active_gyro");
                if (res_val.StartsWith("mse_"))
                    if ((int)e.Data.ButtonDown.Button == Int32.Parse(res_val.Substring(4)))
                        foreach (var i in mgr.j)
                            i.active_gyro = true;
            }

            if (e.Data.ButtonUp != null) {
                string res_val = Config.Value("active_gyro");
                if (res_val.StartsWith("mse_"))
                    if ((int)e.Data.ButtonUp.Button == Int32.Parse(res_val.Substring(4)))
                        foreach (var i in mgr.j)
                            i.active_gyro = false;
            }
        }

        private static void Keyboard_KeyEvent(object sender, WindowsInput.Events.Sources.EventSourceEventArgs<WindowsInput.Events.Sources.KeyboardEvent> e) {
            if (e.Data.KeyDown != null) {
                string res_val = Config.Value("reset_mouse");
                if (res_val.StartsWith("key_"))
                    if ((int)e.Data.KeyDown.Key == Int32.Parse(res_val.Substring(4)))
                        WindowsInput.Simulate.Events().MoveTo(Screen.PrimaryScreen.Bounds.Width / 2, Screen.PrimaryScreen.Bounds.Height / 2).Invoke();

                res_val = Config.Value("active_gyro");
                if (res_val.StartsWith("key_"))
                    if ((int)e.Data.KeyDown.Key == Int32.Parse(res_val.Substring(4)))
                        foreach (var i in mgr.j)
                            i.active_gyro = true;
            }

            if (e.Data.KeyUp != null) {
                string res_val = Config.Value("active_gyro");
                if (res_val.StartsWith("key_"))
                    if ((int)e.Data.KeyUp.Key == Int32.Parse(res_val.Substring(4)))
                        foreach (var i in mgr.j)
                            i.active_gyro = false;
            }
        }

        public static void Stop() {
            if (Program.useHIDG) {
                try {
                    HttpWebResponse response = (HttpWebResponse)WebRequest.Create(@"http://localhost:26762/api/v1/hidguardian/whitelist/remove/" + pid).GetResponse();
                } catch (Exception e) {
                    form.console.AppendText("Unable to remove program from whitelist.\r\n");
                }
            }

            if (Boolean.Parse(ConfigurationManager.AppSettings["PurgeAffectedDevices"]) && Program.useHIDG) {
                try {
                    HttpWebResponse r1 = (HttpWebResponse)WebRequest.Create(@"http://localhost:26762/api/v1/hidguardian/affected/purge/").GetResponse();
                } catch { }
            }

            keyboard.Dispose(); mouse.Dispose();
            server.Stop();
            mgr.OnApplicationQuit();
        }

        private static string appGuid = "1bf709e9-c133-41df-933a-c9ff3f664c7b"; // randomly-generated
        public static void Main(string[] args) {

            // Setting the culturesettings so float gets parsed correctly
            CultureInfo.CurrentCulture = new CultureInfo("en-US", false);

            // Set the correct DLL for the current OS
            SetupDlls();

            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            
            form = new MainForm();
            Task.Run(() =>
            {
                form.ShowDialog();
            });
            //Application.Run(form);
        }

        static void SetupDlls() {
            string archPath = $"{AppDomain.CurrentDomain.BaseDirectory}{(Environment.Is64BitProcess ? "x64" : "x86")}\\";
            string pathVariable = Environment.GetEnvironmentVariable("PATH");
            pathVariable = $"{archPath};{pathVariable}";
            Environment.SetEnvironmentVariable("PATH", pathVariable);
        }

        // Helper funtions to set the hidapi dll location acording to the system instruction set.
        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        static extern bool SetDefaultDllDirectories(int directoryFlags);
        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        static extern void AddDllDirectory(string lpPathName);
    }
}
