import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.EventQueue;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import javax.swing.JComponent;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;

/**
 *	Console widget.
 *  All standard text output (e.g. through calling System.out.print()) is diverted to this text area.
 */
public class ConsoleWidget extends JComponent
{
	protected final JTextArea textArea;
	
	public ConsoleWidget()
	{
		textArea = new JTextArea();
		textArea.setEditable( false );
		
		JScrollPane scrollPane = new JScrollPane(textArea);
		
		setLayout( new BorderLayout() );
		add( scrollPane, BorderLayout.CENTER );
		
		setMinimumSize( new Dimension(100, 30) );
		setPreferredSize( new Dimension(800, 200) );
		setMaximumSize( new Dimension(Short.MAX_VALUE, Short.MAX_VALUE) );
		
		PrintStream stdOut = new PrintStream(new TextAreaOutputStream());
		System.setOut( stdOut );
		System.setErr( stdOut );
	}
	
	private class TextAreaOutputStream extends OutputStream
	{
		private final StringBuffer buffer = new StringBuffer();
		
		@Override
		public void write(int b) throws IOException
		{
			buffer.append((char)b);
			flush();
		}
		
		@Override
		public void write(byte[] b) throws IOException
		{
			write(b, 0, b.length);
			flush();
		}
		
		@Override
		public void write(byte[] b, int off, int len) throws IOException
		{
			if (b == null)
			    throw new NullPointerException();
			else if ((off < 0) || (off > b.length) || (len < 0) || ((off + len) > b.length) || ((off + len) < 0))
			    throw new IndexOutOfBoundsException();
			else if (len == 0)
			    return;
			
			for (int i = 0 ; i < len ; i++)
				buffer.append((char)b[off + i]);
			
			flush();
		}
		
		@Override
		public void flush() throws IOException
		{
			final String str;
			
			synchronized(buffer)
			{
				str = buffer.toString();
				buffer.delete(0, buffer.length());
			}
			
			EventQueue.invokeLater(new Runnable()
			{
				@Override
				public void run()
				{
					textArea.append(str);
				}
			} );
		}
	}
}