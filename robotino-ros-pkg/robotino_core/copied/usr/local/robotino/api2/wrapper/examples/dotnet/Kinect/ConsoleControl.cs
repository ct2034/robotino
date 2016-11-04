using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Windows.Forms;
using System.IO;
using System.Text;

namespace rec.robotino.api2.examples.kinect
{
    /// <summary>
    ///	Console control.
    /// All standard text output (e.g. through calling System.out.print()) is diverted to this text area.
    /// </summary>
    public partial class ConsoleControl : UserControl
    {
        public ConsoleControl()
        {
            InitializeComponent();

            TextBoxWriter writer = new TextBoxWriter(textBoxConsole);

            Console.SetOut(writer);
            Console.SetError(writer);
        }

        private class TextBoxWriter : TextWriter
        {
            private delegate void WriteTextDelegate(string text);

            private readonly TextBox textbox;

            private readonly WriteTextDelegate writeTextDelegate;

            public TextBoxWriter(TextBox textbox)
            {
                this.textbox = textbox;
                this.writeTextDelegate = new WriteTextDelegate(WriteToTextBox);
            }

            public override void Write(char value)
            {
                Write(value.ToString());
            }

            public override void Write(string value)
            {
                if (textbox.InvokeRequired)
                    textbox.Invoke(writeTextDelegate, value);
            }

            private void WriteToTextBox(string value)
            {
                textbox.AppendText(value);
            }

            public override System.Text.Encoding Encoding
            {
                get { return new UTF8Encoding(); }
            }
        }
    }
}
