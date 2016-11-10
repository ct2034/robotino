using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using rec.robotino.api2;

namespace rec.robotino.api2.examples.driveincircles
{
    public class MyCom : Com
    {
        System.Timers.Timer spinTimer;

        public MyCom()
        {
            spinTimer = new System.Timers.Timer();
            spinTimer.Elapsed += new System.Timers.ElapsedEventHandler(onSpinTimerTimeout);
            spinTimer.Interval = 10;
            spinTimer.Enabled = true;
        }

        public void onSpinTimerTimeout(object obj, System.Timers.ElapsedEventArgs e)
        {
            processEvents();
        }

        override public void errorEvent(string errorString)
        {
            Console.WriteLine("Error: " + errorString);
        }

        override public void connectedEvent()
        {
            Console.WriteLine("Connected.");
        }

        override public void connectionClosedEvent()
        {
            Console.WriteLine("Connection closed.");
        }

        public void modeChangeEvent(bool isPassiveMode)
        {
            if (isPassiveMode)
                Console.WriteLine("Connected in passive mode.");
        }
    }

    class MyBumper : Bumper
    {
        bool bumped;
	    public MyBumper( bool val = false)
	    {
            bumped = val;
	    }

	    override public void bumperEvent( bool hasContact )
	    {
		    bumped |= hasContact;
	    }
    }
    public class Program
    {
        MyCom com;
        OmniDrive omniDrive;
        MyBumper bumper;

        Program()
        {
            com = new MyCom();
            omniDrive = new OmniDrive();
            bumper = new MyBumper();

            omniDrive.setComId(com.id());
            bumper.setComId(com.id());
        }

        public void init(string hostname)
        {
            Console.WriteLine("Connecting ... ");
            com.setAddress(hostname);
            com.connectToServer();

            Console.WriteLine("Connected.");
        }

        public void destroy()
        {
            com.disconnectFromServer();
        }

        public void rotate(float[] inArray, float[] outArray, float deg)
        {
            float rad = 2 * (float)Math.PI / 360.0f * deg;
            outArray[0] = (float)Math.Cos(rad) * inArray[0] - (float)Math.Sin(rad) * inArray[1];
            outArray[1] = (float)Math.Sin(rad) * inArray[0] + (float)Math.Cos(rad) * inArray[1];
        }

        public void drive()
        {
            Console.Write("Driving .. ");
            float[] startVector = new float[] {0.2f, 0.0f };
            float[] dir = new float[2];
            float a = 0.0f;

            while (com.isConnected() && false == bumper.value() )
            {
                //rotate 360degrees in 5s
		        rotate( startVector, dir, a );
		        a = 360.0f * com.msecsElapsed() / 5000;

		        omniDrive.setVelocity( dir[0], dir[1], 0 );

                System.Threading.Thread.Sleep(100);
            }
        }
        static void Main(string[] args)
        {
            string hostname = "127.0.0.1";
            if (1 == args.GetLength(0))
            {
                hostname = args.GetValue(0).ToString();
            }

            Program prg = new Program();
            try
            {
                prg.init(hostname);
                prg.drive();
                prg.destroy();
            }
            catch (Exception e)
            {
                Console.WriteLine(e.ToString());
            }
        }
    }
}
