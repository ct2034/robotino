using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Text;
using System.Windows.Forms;

namespace rec.robotino.api2.examples.camera
{
    /// <summary>
    /// This control provides basic controls to drive Robotino.
    /// </summary>
    public partial class DriveControl : UserControl
    {
        protected readonly Robot robot;
        protected volatile float speed = 0.2f;
        protected volatile float rotSpeed = 0.4f;

        public DriveControl(Robot robot)
        {
            this.robot = robot;

            InitializeComponent();
        }

        private void buttonUp_Click(object sender, EventArgs e)
        {
            robot.SetVelocity(speed, 0.0f, 0.0f);
        }

        private void buttonDown_Click(object sender, EventArgs e)
        {
            robot.SetVelocity(-speed, 0.0f, 0.0f);
        }

        private void buttonLeft_Click(object sender, EventArgs e)
        {
            robot.SetVelocity(0.0f, speed, 0.0f);
        }

        private void buttonRight_Click(object sender, EventArgs e)
        {
            robot.SetVelocity(0.0f, -speed, 0.0f);
        }

        private void buttonRotRight_Click(object sender, EventArgs e)
        {
            robot.SetVelocity(0.0f, 0.0f, rotSpeed);
        }

        private void buttonRotLeft_Click(object sender, EventArgs e)
        {
            robot.SetVelocity(0.0f, 0.0f, -rotSpeed);
        }

        private void buttonMiddle_Click(object sender, EventArgs e)
        {
            robot.SetVelocity(0.0f, 0.0f, 0.0f);
        }
    }
}
