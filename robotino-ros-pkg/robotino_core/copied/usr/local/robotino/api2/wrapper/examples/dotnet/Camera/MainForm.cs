using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;

namespace rec.robotino.api2.examples.camera
{
    /// <summary>
    /// This is the main form of this example.
    /// It provides an user interface to enter the IP and connect to the robot,
    /// a basic interface to drive Robotino, it shows the camera image and
    /// displays the standard text output.
    /// </summary>
    public partial class MainForm : Form
    {
        public MainForm(Robot robot)
        {
            InitializeComponent();

            ConnectControl connectControl = new ConnectControl(robot);
            connectControl.Dock = DockStyle.Fill;
            DriveControl driveControl = new DriveControl(robot);
            ConsoleControl consoleControl = new ConsoleControl();
            consoleControl.Dock = DockStyle.Fill;
            CameraControl cameraControl = new CameraControl(robot);
            cameraControl.Dock = DockStyle.Fill;

            tableLayoutPanel2.Controls.Add(driveControl);
            tableLayoutPanel2.Controls.Add(cameraControl);

            tableLayoutPanel1.Controls.Add(connectControl);
            tableLayoutPanel1.Controls.Add(tableLayoutPanel2);
            tableLayoutPanel1.Controls.Add(consoleControl);
        }
    }
}