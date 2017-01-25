using System;
using System.Collections.Generic;
using System.Text;
using rec.robotino.api2;
using System.Collections;
using System.Threading;
using System.Drawing;

namespace rec.robotino.api2.examples.camera
{
    /// <summary>
    /// The class Robot demonstrates the usage of the most common robot component classes.
    /// Furthermore it shows how to handle events and receive incoming camera images.
    /// </summary>
    public class Robot
    {
        public delegate void ConnectedEventHandler(Robot sender);
        public delegate void DisconnectedEventHandler(Robot sender);
        public delegate void ErrorEventHandler(Robot sender, String error);
        public delegate void ImageReceivedEventHandler(Robot sender, Image img);

        protected readonly Com com;
	    protected readonly OmniDrive omniDrive;
        protected readonly Bumper bumper;
        protected readonly Camera camera;

        private volatile bool isConnected;

        public Robot()
        {
            com = new MyCom(this);
            omniDrive = new OmniDrive();
            bumper = new Bumper();
            camera = new MyCamera(this);

            omniDrive.setComId(com.id());
            bumper.setComId(com.id());
            camera.setComId(com.id());
        }

        public event ConnectedEventHandler Connected;
        public event DisconnectedEventHandler Disconnected;
        public event ErrorEventHandler Error;
        public event ImageReceivedEventHandler ImageReceived;

        public bool IsConnected
        {
            get
            {
                return isConnected;
            }
        }

        public virtual void Connect(String hostname, bool blockUntilConnected)
        {
            com.setAddress(hostname);
            com.connectToServer(blockUntilConnected);
            Console.WriteLine("Connecting...");
        }

        public virtual void SetVelocity(float vx, float vy, float omega)
        {
            omniDrive.setVelocity(vx, vy, omega);
        }

        private class MyCom : Com
        {
            Robot robot;
            System.Timers.Timer spinTimer;

            public MyCom(Robot robot)
            {
                this.robot = robot;
                spinTimer = new System.Timers.Timer();
                spinTimer.Elapsed += new System.Timers.ElapsedEventHandler(onSpinTimerTimeout);
                spinTimer.Interval = 10;
                spinTimer.Enabled = true;
            }

            public void onSpinTimerTimeout(object obj, System.Timers.ElapsedEventArgs e)
            {
                processEvents();
            }

            public override void connectedEvent()
            {
                Console.WriteLine("Connected");
                robot.isConnected = true;
                if (robot.Connected != null)
                    robot.Connected.BeginInvoke(robot, null, null);
            }

            public override void connectionClosedEvent()
            {
                Console.WriteLine("Disconnected");
                robot.isConnected = false;
                if (robot.Disconnected != null)
                    robot.Disconnected.BeginInvoke(robot, null, null);
            }

            public override void errorEvent(String errorStr)
            {
                Console.WriteLine("Error occured: " + errorStr);
                if (robot.Error != null)
                    robot.Error.BeginInvoke(robot, errorStr, null, null);
            }
        }

        private class MyCamera : Camera
        {
            Robot robot;

            public MyCamera(Robot robot)
            {
                this.robot = robot;
                setCameraNumber(0);
                setBGREnabled(true);
            }

            public override void imageReceivedEvent(Image data, uint dataSize, uint width, uint height, uint step)
            {
                /*
                 * we could pass the Image directly to the CameraControl, because this function is called from the main thread from Com::processEvents
                 */
                if (robot.ImageReceived != null)
                    robot.ImageReceived.BeginInvoke(robot, data, null, null);
            }
        }
    }
}
