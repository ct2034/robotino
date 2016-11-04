import java.awt.Dimension;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.Icon;
import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JLabel;

/**
 * This widget provides basic controls to drive Robotino.
 */
public class DriveWidget extends JComponent
{
	protected static final float speed = 0.2f;
	protected static final float rotSpeed = 2f;
	
	protected final Robot robot;

	Timer _timer;
	private float vx;
	private float vy;
	private float omega;
	
	public DriveWidget(Robot robot)
	{
		this.robot = robot;
		
		setLayout(new GridLayout(3, 5));
		
		JButton buttonUp = new JButton(getIcon("n"));
		JButton buttonDown = new JButton(getIcon("s"));
		JButton buttonLeft = new JButton(getIcon("o"));
		JButton buttonRight = new JButton(getIcon("w"));
		JButton buttonCL = new JButton(getIcon("cl"));
		JButton buttonCCL = new JButton(getIcon("ccl"));
		JButton buttonStop = new JButton(getIcon("stop"));
		
		buttonUp.addActionListener( new ButtonListener(speed, 0.0f, 0.0f, this) );
		buttonDown.addActionListener( new ButtonListener(-speed, 0.0f, 0.0f, this) );
		buttonLeft.addActionListener( new ButtonListener(0.0f, -speed, 0.0f, this) );
		buttonRight.addActionListener( new ButtonListener(0.0f, speed, 0.0f, this) );
		buttonCL.addActionListener( new ButtonListener(0.0f, 0.0f, -rotSpeed, this) );
		buttonCCL.addActionListener( new ButtonListener(0.0f, 0.0f, rotSpeed, this) );
		buttonStop.addActionListener( new ButtonListener(0.0f, 0.0f, 0.0f, this) );

		add(new JLabel());
		add(new JLabel());
		add(buttonUp);
		add(new JLabel());
		add(new JLabel());
		add(buttonCCL);
		add(buttonRight);
		add(buttonStop);
		add(buttonLeft);
		add(buttonCL);
		add(new JLabel());
		add(new JLabel());
		add(buttonDown);
		
		setMinimumSize( new Dimension(60, 30) );
		setPreferredSize( new Dimension(200, 120) );
		setMaximumSize( new Dimension(Short.MAX_VALUE, Short.MAX_VALUE) );

		_timer = new Timer();
		_timer.scheduleAtFixedRate(new OnTimeOut(), 0, 20);
	}
	
	public void setVelocity(float vx, float vy, float omega)
	{
		this.vx = vx;
		this.vy = vy;
		this.omega = omega;		
		robot.setVelocity( vx, vy, omega );
	}
	
	public void setVelocity_i()
	{	
		robot.setVelocity( this.vx, this.vy, this.omega );
	}
	
	class OnTimeOut extends TimerTask
	{
		public void run()
		{
			setVelocity_i();
		}
	}
	
	private Icon getIcon(String name)
	{
		return new ImageIcon(getClass().getResource( "icons/" + name + ".png"));
	}
	
	private class ButtonListener implements ActionListener
	{
		private final float vx;
		private final float vy;
		private final float omega;
		
		private final DriveWidget driveWidget;
		
		public ButtonListener(float vx, float vy, float omega, DriveWidget p)
		{
			this.vx = vx;
			this.vy = vy;
			this.omega = omega;
			this.driveWidget = p;
		}
			
		@Override
		public void actionPerformed(ActionEvent e) 
		{
			this.driveWidget.setVelocity( vx, vy, omega );
		}
	}
}