using System;
using System.Collections.Generic;
using System.Text;
using rec.robotino.api2;
using System.Collections;
using System.Threading;
using System.Drawing;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;

namespace rec.robotino.api2.examples.kinect
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
        public delegate void DepthReceivedEventHandler(Robot sender, Image img);
        public delegate void AccelReceivedEventHandler(Robot sender, double x, double y, double z);
        public delegate void TiltReceivedEventHandler(Robot sender, double angleDeg);

        protected readonly Com com;
	    protected readonly OmniDrive omniDrive;
        protected readonly Bumper bumper;
        protected readonly Kinect kinect;

        private volatile bool isConnected;

        public Robot()
        {
            com = new MyCom(this);
            omniDrive = new OmniDrive();
            bumper = new Bumper();
            kinect = new MyKinect(this);

            omniDrive.setComId(com.id());
            bumper.setComId(com.id());
            kinect.setComId(com.id());
        }

        public event ConnectedEventHandler Connected;
        public event DisconnectedEventHandler Disconnected;
        public event ErrorEventHandler Error;
        public event ImageReceivedEventHandler ImageReceived;
        public event DepthReceivedEventHandler DepthReceived;
        public event AccelReceivedEventHandler AccelReceived;
        public event TiltReceivedEventHandler TiltReceived;

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

        private class MyKinect : Kinect
        {
            Robot robot;
            Int16[] _gamma = new Int16[2048];

            public MyKinect(Robot robot)
            {
                this.robot = robot;
                for (int i = 0; i < 2048; i++)
                {
                    float v = i / 2048.0f;
                    v = (float)Math.Pow(v, 3) * 6;
                    _gamma[i] = (Int16)(v * 6 * 256);
                }
            }

            public override void videoEvent(Image data, uint dataSize, uint width, uint height, uint step, uint format, uint stamp)
            {
                /*
                 * we could pass the Image directly to the KinectControl, because this function is called from the main thread from Com::processEvents
                 */
                //if (robot.ImageReceived != null)
                //    robot.ImageReceived.BeginInvoke(robot, data, null, null);
            }

            public override void depthEvent(Int16[] data, uint dataSize, uint width, uint height, uint format, uint stamp)
            {
                Bitmap bitmap = new Bitmap((int)width, (int)height, PixelFormat.Format24bppRgb);

                // Create BitmapData and Lock all pixels to be written
                BitmapData bmpData = bitmap.LockBits(
                    new Rectangle(0, 0, bitmap.Width, bitmap.Height),
                    ImageLockMode.WriteOnly, bitmap.PixelFormat);

                IntPtr ptr = bmpData.Scan0;
                int bytes = Math.Abs(bmpData.Stride) * bitmap.Height;
                byte[] rgbValues = new byte[bytes];

                for (int i = 0; i < height; ++i)
                {
                    for (int j = 0; j < width; ++j)
                    {
                        int pval = _gamma[data[i * width + j]];
                        int lb = pval & 0xff;
                        switch (pval >> 8)
                        {
                            case 0:
                                rgbValues[(j + i * width) * 3 + 0] = (byte)255;
                                rgbValues[(j + i * width) * 3 + 1] = (byte)(255 - lb);
                                rgbValues[(j + i * width) * 3 + 2] = (byte)(255 - lb);
                                break;
                            case 1:
                                rgbValues[(j + i * width) * 3 + 0] = (byte)255;
                                rgbValues[(j + i * width) * 3 + 1] = (byte)(255 - lb);
                                rgbValues[(j + i * width) * 3 + 2] = (byte)(255 - lb);
                                break;
                            case 2:
                                rgbValues[(j + i * width) * 3 + 0] = (byte)(255 - lb);
                                rgbValues[(j + i * width) * 3 + 1] = (byte)255;
                                rgbValues[(j + i * width) * 3 + 2] = 0;
                                break;
                            case 3:
                                rgbValues[(j + i * width) * 3 + 0] = 0;
                                rgbValues[(j + i * width) * 3 + 1] = 255;
                                rgbValues[(j + i * width) * 3 + 2] = (byte)lb;
                                break;
                            case 4:
                                rgbValues[(j + i * width) * 3 + 0] = 0;
                                rgbValues[(j + i * width) * 3 + 1] = (byte)(255 - lb);
                                rgbValues[(j + i * width) * 3 + 2] = 255;
                                break;
                            case 5:
                                rgbValues[(j + i * width) * 3 + 0] = 0;
                                rgbValues[(j + i * width) * 3 + 1] = 0;
                                rgbValues[(j + i * width) * 3 + 2] = (byte)(255 - lb);
                                break;
                            default:
                                rgbValues[(j + i * width) * 3 + 0] = 0;
                                rgbValues[(j + i * width) * 3 + 1] = 0;
                                rgbValues[(j + i * width) * 3 + 2] = 0;
                                break;
                        }
                    }
                }

                Marshal.Copy(rgbValues, 0, ptr, bytes);

                // Unlock the pixels
                bitmap.UnlockBits(bmpData);

                if (robot.DepthReceived != null)
                    robot.DepthReceived.BeginInvoke(robot, bitmap, null, null);
            }

            public override void accelEvent(double x, double y, double z)
            {
                if (robot.AccelReceived != null)
                    robot.AccelReceived.BeginInvoke(robot, x, y , z, null, null);
            }

            public override void tiltEvent(double angleDeg)
            {
                if (robot.TiltReceived != null)
                    robot.TiltReceived.BeginInvoke(robot, angleDeg, null, null);
            }
        }
    }
}
