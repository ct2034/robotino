using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Text;
using System.Windows.Forms;
using rec.robotino.api2;

namespace rec.robotino.api2.examples.camera
{
    /// <summary>
    /// This control provides a graphical user interface to connect to Robotino.
    /// </summary>
    public partial class ConnectControl : UserControl
    {
        protected readonly Robot robot;

        public ConnectControl(Robot robot)
        {
            this.robot = robot;
            robot.Connected += new Robot.ConnectedEventHandler(robot_Connected);
            robot.Disconnected += new Robot.DisconnectedEventHandler(robot_Disconnected);

            InitializeComponent();
        }

        void robot_Disconnected(Robot sender)
        {
            if (this.InvokeRequired)
                Invoke(new MethodInvoker(Connected));
        }

        void robot_Connected(Robot sender)
        {
            if (this.InvokeRequired)
                Invoke(new MethodInvoker(Disconnected));
        }

        private void Connected()
        {
            buttonConnect.Enabled = true;
        }

        private void Disconnected()
        {
            buttonConnect.Enabled = false;
        }

        private void buttonConnect_Click(object sender, EventArgs e)
        {
            robot.Connect(textBoxHostname.Text, false);
        }
    }
}
