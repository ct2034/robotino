import java.awt.Image;

/**
 * This class is used to listen for incoming events in the Robot class.
 */
public interface RobotListener
{
	void onConnected();
	void onDisconnected();
	void onError(String error);
	void onImageReceived(Image img);
	void onOdometryReceived(double x, double y, double phi);
}
