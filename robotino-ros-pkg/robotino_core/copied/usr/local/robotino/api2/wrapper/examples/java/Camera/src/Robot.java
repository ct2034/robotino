import java.awt.Image;
import java.util.Collection;
import java.util.TimerTask;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.Timer;

import rec.robotino.api2.Bumper;
import rec.robotino.api2.Com;
import rec.robotino.api2.Motor;
import rec.robotino.api2.OmniDrive;
import rec.robotino.api2.Camera;
import rec.robotino.api2.Odometry;


/**
 * The class Robot demonstrates the usage of the most common robot component classes.
 * Furthermore it shows how to handle events and receive incoming camera images.
 */
public class Robot
{
	protected final Com _com;
	protected final Motor _motor1;
	protected final Motor _motor2;
	protected final Motor _motor3;
	protected final OmniDrive _omniDrive;
	protected final Bumper _bumper;
	protected final Camera _camera;
	protected final Odometry _odometry;
	
	protected final Collection<RobotListener> _listeners;

	public Robot()
	{
		_com = new MyCom();
		_motor1 = new Motor();
		_motor2 = new Motor();
		_motor3 = new Motor();
		_omniDrive = new OmniDrive();
		_bumper = new Bumper();
		_camera = new MyCamera();
		_odometry = new MyOdometry();
		
		_listeners = new CopyOnWriteArrayList<RobotListener>();
		
		init();
	}

	protected void init()
	{
		_motor1.setComId( _com.id() );
		_motor1.setMotorNumber( 0 );

		_motor2.setComId( _com.id() );
		_motor2.setMotorNumber( 1 );

		_motor3.setComId( _com.id() );
		_motor3.setMotorNumber( 2 );

		_omniDrive.setComId( _com.id() );

		_bumper.setComId( _com.id() );

		_camera.setComId( _com.id() );
		
		_odometry.setComId( _com.id() );
	}
	
	public void addListener(RobotListener listener)
	{
		_listeners.add( listener );
	}
	
	public void removeListener(RobotListener listener)
	{
		_listeners.remove( listener );
	}
	
	public boolean isConnected()
	{
		return _com.isConnected();
	}

	public void connect(String hostname, boolean block)
	{
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
	
	/**
	 * The class MyCom derives from rec.robotino.com.Com and implements some of the virtual event handling methods.
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
			for(RobotListener listener : _listeners)
				listener.onConnected();
		}

		@Override
		public void errorEvent(String errorStr)
		{
			System.err.println( "Error: " + errorStr );
			for( RobotListener listener : _listeners)
				listener.onError(errorStr);
		}

		@Override
		public void connectionClosedEvent()
		{
			System.out.println( "Disconnected" );
			for(RobotListener listener : _listeners)
				listener.onDisconnected();
		}
	}

	/**
	 * Class MyCamera inherits from rec.robotino.com.Camera and implements some of the virtual event handling methods.
	 * This is the standard approach for handling these Events.
	 */
	class MyCamera extends Camera
	{
		public MyCamera()
		{
			setCameraNumber(0);
			setBGREnabled(true);
		}
		
		@Override
		public void imageReceivedEvent(Image img, long dataSize, long width, long height, long step)
		{
			for(RobotListener listener : _listeners)
				listener.onImageReceived( img );
		}
	}
	
	class MyOdometry extends Odometry
	{
		public MyOdometry()
		{
			super();
		}
		
		@Override
		public void readingsEvent(double x, double y, double phi, float vx, float vy, float omega, long sequence)
		{
			for(RobotListener listener : _listeners)
				listener.onOdometryReceived(x, y, phi);
		}
	}
}
