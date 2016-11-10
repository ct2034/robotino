using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace rec.robotino.api2.examples.wallfollow
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

    public class MyDistanceSensor : DistanceSensor
    {
        private float _distance;
        private uint _sensorNumber;

        // Default Constructor
        public MyDistanceSensor(uint sensorNumber)
        {
            this._sensorNumber = sensorNumber;
            setSensorNumber(_sensorNumber);
        }

        public float Distance
        {
            get { return _distance; }
            set { _distance = value; }
        }

        public uint SensorNumber
        {
            get { return _sensorNumber; }
            set { _sensorNumber = value; }
        }

        public override void distanceChangedEvent(float distance)
        {
            _distance = distance;
        }
    }

    public class MyBumper : Bumper
    {
        bool _bumped;
        public MyBumper(bool val = false)
        {
            _bumped = val;
        }

        public bool Value
        {
            get { return _bumped; }
        }
        override public void bumperEvent(bool hasContact)
        {
            _bumped |= hasContact;
        }
    }

    class Robotino
    {
        MyCom _com;
        MyBumper _bumper;
        OmniDrive _omniDrive;
        List<MyDistanceSensor> _distanceSensors;

        const float SLOW_VELOCITY = 0.08f;
        const float MEDIUM_VELOCITY = 0.16f;
        const float VELOCITY = 0.24f;
        const float FAST_VELOCITY = 0.32f;
        const float ANGULARVELOCITY = 0.02f;

        Robotino()
        {
            _com = new MyCom();
            _omniDrive = new OmniDrive();
            _bumper = new MyBumper();

            _distanceSensors = new List<MyDistanceSensor>();

            for (uint i = 0; i < 9; i++)
            {
                _distanceSensors.Add(new MyDistanceSensor(i));
                _distanceSensors[(int)i].setComId(_com.id());
            }

            _omniDrive.setComId(_com.id());
            _bumper.setComId(_com.id());
        }

        public void init(string hostname)
        {
            Console.WriteLine("Connecting ... ");
            _com.setAddress(hostname);
            _com.connectToServer();

            Console.WriteLine("Connected.");
        }

        public void destroy()
        {
            _com.disconnectFromServer();
        }

        public void followWalls()
        {
            float[] ev = new float[] { 1.0f, 0.0f };

            // escape vector for distance sensor
            float[][] escapeVector = new float[][]
            {
                new float[]{0.0f, 0.0f},
                new float[]{0.0f, 0.0f},
                new float[]{0.0f, 0.0f},
                new float[]{0.0f, 0.0f},
                new float[]{0.0f, 0.0f},
                new float[]{0.0f, 0.0f},
                new float[]{0.0f, 0.0f},
                new float[]{0.0f, 0.0f},
                new float[]{0.0f, 0.0f}
            };

            for (int i = 0; i < 9; i++)
            {
                rotate(ev, escapeVector[i], 40.0f * i);
            }
            const float ESCAPE_DISTANCE = 0.10f;
            const float WALL_LOST_DISTANCE = 0.35f;
            const float WALL_FOUND_DISTANCE = 0.30f;
            const float WALL_FOLLOW_DISTANCE = 0.15f;
            const float NEW_WALL_FOUND_DISTANCE = 0.12f;
            float[] escape = new float[] { 0.0f, 0.0f };
            int curWallSensor = -1;
            float[] dir = new float[] { 1.0f, 0.0f };
            float velocity = VELOCITY;
            float rotVelocity = ANGULARVELOCITY;

            while (_com.isConnected() && _bumper.Value != true)
            {
                velocity = VELOCITY;
                int minIndex = 0;
                float minDistance = 0.40f;
                int numEscape = 0;
                escape[0] = escape[1] = 0.0f;

                StringBuilder values = new StringBuilder();
                for (int i = 0; i < _distanceSensors.Count; ++i)
                {
                    float v = (float)_distanceSensors[i].Distance;
                    values.Append(v.ToString() + " ");
                    if (v < minDistance)
                    {
                        minDistance = v;
                        minIndex = i;
                    }
                    if (v < ESCAPE_DISTANCE)
                    {
                        ++numEscape;
                        addScaledVector(escape, escapeVector[i], v);
                    }
                }
                Console.WriteLine(values.ToString());
                if (numEscape >= 2)
                {
                    // close to walls with more than one sensor, try to escape
                    normalizeVector(escape);
                    rotate(escape, dir, 180);
                    velocity = SLOW_VELOCITY;
                }
                else
                {
                    if (curWallSensor != -1
                    && _distanceSensors[curWallSensor].Distance > WALL_LOST_DISTANCE)
                    {
                        curWallSensor = -1;
                        rotateInPlace(dir, -20);
                        rotVelocity = -0.20f;
                    }
                    if (curWallSensor == -1)
                    {
                        // wall not found yet
                        if (minDistance < WALL_FOUND_DISTANCE)
                        {
                            curWallSensor = minIndex;
                            velocity = SLOW_VELOCITY;
                        }
                    }
                    if (curWallSensor != -1)
                    {
                        float wallDist = (float)_distanceSensors[curWallSensor].Distance;

                        // check for global new wall
                        if (minIndex != curWallSensor && minDistance < NEW_WALL_FOUND_DISTANCE)
                        {
                            // new wall found, drive along this wall
                            curWallSensor = minIndex;
                            wallDist = (float)_distanceSensors[curWallSensor].Distance;
                            velocity = SLOW_VELOCITY;
                        }
                        else
                        {
                            if (_distanceSensors[(curWallSensor + 1) % 9].Distance < wallDist)
                            {
                                // switch walls
                                curWallSensor = (curWallSensor + 1) % 9;
                                velocity = MEDIUM_VELOCITY;
                            }
                            // check for new wall in direction
                            for (int i = 0; i < 2; ++i)
                            {
                                int tmpId = (curWallSensor + 2 + i) % 9;
                                if (_distanceSensors[tmpId].Distance < WALL_FOUND_DISTANCE)
                                {
                                    curWallSensor = tmpId;
                                    wallDist = (float)_distanceSensors[tmpId].Distance;
                                    velocity = SLOW_VELOCITY;
                                    break;
                                }
                            }
                        }
                        // try to keep neighbor distance sensors in balance
                        float vr = (float)_distanceSensors[(curWallSensor + 1) % 9].Distance;
                        float vl = (float)_distanceSensors[(curWallSensor + 8) % 9].Distance;

                        rotVelocity = (vr - vl);
                        float followAngle = 95;
                        if (Math.Abs(rotVelocity) > 0.30)
                        {
                            velocity = SLOW_VELOCITY;
                            followAngle = rotVelocity >= 0.0 ? 140 : 80;
                        }
                        else if (Math.Abs(rotVelocity) > 0.40)
                        {
                            velocity = MEDIUM_VELOCITY;
                            followAngle = rotVelocity >= 0.0 ? 120 : 85;
                        }
                        rotVelocity *= 8 * ANGULARVELOCITY;
                        // follow the wall to the left
                        rotate(escapeVector[curWallSensor], dir, followAngle);
                        // keep distance to wall steady
                        float scale = wallDist - WALL_FOLLOW_DISTANCE;

                        scale *= 10f;

                        addScaledVector(dir, escapeVector[curWallSensor], scale);
                        normalizeVector(dir);
                    }
                }
                if (minDistance > 0.20f)
                {
                    velocity = FAST_VELOCITY;
                }
                _omniDrive.setVelocity(velocity * (float)dir[0], velocity * (float)dir[1], rotVelocity);
                System.Threading.Thread.Sleep(100);
            }
        }

        public void rotate(float[] inArray, float[] outArray, float deg)
        {
            float rad = 2 * (float)Math.PI / 360.0f * deg;
            outArray[0] = (float)Math.Cos(rad) * inArray[0] - (float)Math.Sin(rad) * inArray[1];
            outArray[1] = (float)Math.Sin(rad) * inArray[0] + (float)Math.Cos(rad) * inArray[1];
        }
        void rotateInPlace(float[] v, float deg)
        {
            float rad = 2 * (float)Math.PI / 360.0f * deg;
            float tmp = v[0];
            v[0] = (float)Math.Cos(rad) * v[0] - (float)Math.Sin(rad) * v[1];
            v[1] = (float)Math.Sin(rad) * tmp + (float)Math.Cos(rad) * v[1];
        }
        void addScaledVector(float[] srcDest, float[] uv, float scale)
        {
            srcDest[0] += uv[0] * scale;
            srcDest[1] += uv[1] * scale;
        }
        void normalizeVector(float[] v)
        {
            float len = (float)Math.Sqrt(v[0] * v[0] + v[1] * v[1]);
            v[0] /= len;
            v[1] /= len;
        }

        static void Main(string[] args)
        {
            string hostname = "127.0.0.1";
            if (1 == args.GetLength(0))
            {
                hostname = args.GetValue(0).ToString();
            }

            Robotino prg = new Robotino();
            try
            {
                prg.init(hostname);
                prg.followWalls();
                prg.destroy();
            }
            catch (Exception e)
            {
                Console.WriteLine(e.ToString());
            }
        }
    }
}
