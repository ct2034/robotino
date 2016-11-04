import javax.swing.*;

import java.awt.*;

/**
 * This is the main frame of this example.
 * It provides an user interface to enter the IP and connect to the robot,
 * a basic interface to drive Robotino, it shows the camera image and
 * displays the standard text output.
 */
public class MainFrame extends JFrame
{
	protected final ConnectWidget connectComponent;
	protected final DriveWidget driveComponent;
	protected final CameraWidget cameraComponent;
	protected final ConsoleWidget consoleComponent;
	
	protected final JPanel centerPanel;

	public MainFrame(Robot robot)
	{
		connectComponent = new ConnectWidget(robot);
		driveComponent = new DriveWidget(robot);
		cameraComponent = new CameraWidget(robot);
		consoleComponent = new ConsoleWidget();
		
		centerPanel = new JPanel();
		centerPanel.setLayout( new BoxLayout(centerPanel, BoxLayout.X_AXIS) );
		centerPanel.add( driveComponent );
		centerPanel.add(Box.createRigidArea(new Dimension(15, 0)));
		centerPanel.add( cameraComponent );

		Container content = getContentPane();
		content.setLayout( new BoxLayout(content, BoxLayout.Y_AXIS) );
		content.add( connectComponent );
		content.add(Box.createRigidArea(new Dimension(0, 15)));
		content.add( centerPanel );
		content.add(Box.createRigidArea(new Dimension(0, 15)));
		content.add( consoleComponent);

		//Place this component in the middle of the screen
		Dimension d = Toolkit.getDefaultToolkit().getScreenSize();
		Dimension dGuiSize = new Dimension( 640, 480 );
		setSize( dGuiSize );
		setLocation( (d.width - dGuiSize.width) / 2, (d.height - dGuiSize.height) / 2 );
		setDefaultCloseOperation( JFrame.EXIT_ON_CLOSE );

		setVisible( true );
	}
}