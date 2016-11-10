import java.util.TimerTask;
import java.util.Timer;

import rec.robotino.api2.Bumper;
import rec.robotino.api2.Com;
import rec.robotino.api2.OmniDrive;


/**
 * The class Robot demonstrates the usage of the most common robot component classes.
 * Furthermore it shows how to handle events and receive incoming camera images.
 */
public class Robot
{
	protected final Com _com;
	protected final OmniDrive _omniDrive;
	protected final Bumper _bumper;

	public Robot()
	{
		_com = new MyCom();
		_omniDrive = new OmniDrive();
		_bumper = new Bumper();
		
		_omniDrive.setComId(_com.id());
		_bumper.setComId(_com.id());
	}
	
	public boolean isConnected()
	{
		return _com.isConnected();
	}

	public void connect(String hostname, boolean block)
	{
		System.out.println("Connecting...");
		_com.setAddress( hostname );
		_com.connectToServer(block);
	}

	public void disconnect()
	{
		_com.disconnectFromServer();
	}
	
	public void setVelocity(float vx, float vy, float omega)
	{
		_omniDrive.setVelocity( vx, vy, omega );
	}
	
    public void rotate(float[] inArray, float[] outArray, float deg)
    {
        float rad = 2 * (float)Math.PI / 360.0f * deg;
        outArray[0] = (float)Math.cos(rad) * inArray[0] - (float)Math.sin(rad) * inArray[1];
        outArray[1] = (float)Math.sin(rad) * inArray[0] + (float)Math.cos(rad) * inArray[1];
    }
    
    public void drive() throws InterruptedException
    {
    	System.out.println("Driving...");
        float[] startVector = new float[] {0.2f, 0.0f };
        float[] dir = new float[2];
        float a = 0.0f;
        while (_com.isConnected() && false == _bumper.value() )
        {
            //rotate 360degrees in 5s
	        rotate( startVector, dir, a );
	        a = 360.0f * _com.msecsElapsed() / 5000;

	        _omniDrive.setVelocity( dir[0], dir[1], 0 );

            Thread.sleep(100);
        }
    }
	
	/**
	 * The class MyCom derives from rec.robotino.api2.Com and implements some of the virtual event handling methods.
	 * This is the standard approach for handling these Events.
	 */
	class MyCom extends Com
	{
		Timer _timer;
		
		public MyCom()
		{
			_timer = new Timer();
			_timer.scheduleAtFixedRate(new OnTimeOut(), 0, 20);
		}
		
		class OnTimeOut extends TimerTask
		{
			public void run()
			{
				processEvents();
			}
		}

		@Override
		public void connectedEvent()
		{
			System.out.println( "Connected" );
		}

		@Override
		public void errorEvent(String errorStr)
		{
			System.err.println( "Error: " + errorStr );
		}

		@Override
		public void connectionClosedEvent()
		{
			System.out.println( "Disconnected" );
		}
	}
	
	public static void main(String args[])
	{
		String hostname = "192.168.5.5";
		if( args.length == 1)
		{
			hostname = args[0].toString();
		}
		
		Robot robotino = new Robot();
		
		try
		{
			robotino.connect(hostname, true);
			robotino.drive();
			robotino.disconnect();
		}
		catch (Exception e)
		{
			System.out.println(e.toString());
		}
	}
}