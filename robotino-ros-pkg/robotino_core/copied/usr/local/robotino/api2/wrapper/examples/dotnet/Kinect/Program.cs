using System;
using System.Collections.Generic;
using System.Windows.Forms;
using System.Threading;

namespace rec.robotino.api2.examples.kinect
{
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new MainForm(new Robot()));
        }
    }
}