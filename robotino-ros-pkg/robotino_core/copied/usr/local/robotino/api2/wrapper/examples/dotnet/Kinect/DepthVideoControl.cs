using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace rec.robotino.api2.examples.kinect
{
    public partial class DepthVideoControl : UserControl
    {
        protected readonly KinectControl kinectcontrol;

        public DepthVideoControl(KinectControl kinectcontrol)
        {
            this.kinectcontrol = kinectcontrol;
            InitializeComponent();
        }

        private void radioButton1_CheckedChanged(object sender, EventArgs e)
        {
            kinectcontrol.setDepth(true);
        }

        private void radioButton2_CheckedChanged(object sender, EventArgs e)
        {
            kinectcontrol.setDepth(false);
        }
    }
}
