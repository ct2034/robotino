using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using rec.robotino.api2;

namespace rec.robotino.api2.examples.com
{
    public class MyCom : Com
    {
        public MyCom()
        {
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
    public class Program
    {
        MyCom com;
        Program()
        {
            com = new MyCom();
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

        static void Main(string[] args)
        {
            string hostname = "127.0.0.1";
            Program prg = new Program();
            try
            {
                prg.init(hostname);
                prg.destroy();
            }
            catch (Exception e)
            {
                Console.WriteLine(e.ToString());
            }
        }
    }
}
