//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using System.Collections;
    using System.Windows.Controls;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 0;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 255, 255, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        private int bodyIndex = 0;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;


        private Double[] jointAngles = new Double[11];

        IDictionary<JointType, int> jointIndexs = new Dictionary<JointType, int>();

        private Double[] originalAngles = new Double[11];

        StreamReader sr = new StreamReader(Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\..\..\")) + @"\DanceData.txt");

        ArrayList allAngles = new ArrayList();

        private int indexOfAngle = 0;

        //private int image = 1;

        //private BitmapImage snapshot;
        //private ImageSource snapshot;


        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));


            //joint indeces
            jointIndexs.Add(JointType.Neck, 10);
            jointIndexs.Add(JointType.SpineShoulder, 4);
            //jointIndexs.Add(JointType.SpineShoulder, 5);
            jointIndexs.Add(JointType.ShoulderRight, 2);
            jointIndexs.Add(JointType.ElbowRight, 0);
            jointIndexs.Add(JointType.ShoulderLeft, 3);
            jointIndexs.Add(JointType.ElbowLeft, 1);
            jointIndexs.Add(JointType.HipRight, 6);
            jointIndexs.Add(JointType.KneeRight, 8);
            jointIndexs.Add(JointType.HipLeft, 7);
            jointIndexs.Add(JointType.KneeLeft, 9);

            while (sr.Peek() > 0)
            {
                
                //string angle = (File.ReadAllLines(Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\..\..\")) + @"\DanceData.txt").CopyTo(originalAngles,0));
                string angle = sr.ReadLine();
                allAngles.Add(angle);
                Console.WriteLine(angle);
            }

            // get angle data for first move
            GetNextMove();
            // get image for first move
            //GetNextImage(0);


            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();

        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

       /* public ImageSource Snapshot
        {
            get
            {
                //maybe have an array of images
                return this.snapshot; // need to change this to image from video of person doing move
            }
            set
            {
                this.snapshot = value;
            }

        }*/

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {
                        bodyIndex = bodyIndex % 6;
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                        }
                        bodyIndex++;
                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {

                    CalcAngles();


                    // need to match jointPoints[jointType] to jointAngles[] somehow

                    if (jointIndexs.ContainsKey(jointType))
                    {
                        drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], GetJointThickness(jointIndexs[jointType]), GetJointThickness(jointIndexs[jointType]));
                    }
                    else
                    {
                        drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);

                    }
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        public void CalcAngles()
        {
            //Right Side
            CameraSpacePoint WristRightP = bodies[bodyIndex].Joints[JointType.WristRight].Position;
            CameraSpacePoint ElbowRightP = bodies[bodyIndex].Joints[JointType.ElbowRight].Position;
            CameraSpacePoint ShoulderRightP = bodies[bodyIndex].Joints[JointType.ShoulderRight].Position;
            CameraSpacePoint HandRightP = bodies[bodyIndex].Joints[JointType.HandRight].Position;
            CameraSpacePoint HandTipRightP = bodies[bodyIndex].Joints[JointType.HandTipRight].Position;
            CameraSpacePoint HipRightP = bodies[bodyIndex].Joints[JointType.HipRight].Position;
            CameraSpacePoint KneeRightP = bodies[bodyIndex].Joints[JointType.KneeRight].Position;
            CameraSpacePoint AnkleRightP = bodies[bodyIndex].Joints[JointType.AnkleRight].Position;
            CameraSpacePoint FootRightP = bodies[bodyIndex].Joints[JointType.FootRight].Position;

            //Center
            CameraSpacePoint HeadP = bodies[bodyIndex].Joints[JointType.Head].Position;
            CameraSpacePoint NeckP = bodies[bodyIndex].Joints[JointType.Neck].Position;
            CameraSpacePoint SpineShoulderP = bodies[bodyIndex].Joints[JointType.SpineShoulder].Position;
            CameraSpacePoint SpineMidP = bodies[bodyIndex].Joints[JointType.SpineMid].Position;
            CameraSpacePoint SpineBaseP = bodies[bodyIndex].Joints[JointType.SpineBase].Position;

            //Left Side
            CameraSpacePoint WristLeftP = bodies[bodyIndex].Joints[JointType.WristLeft].Position;
            CameraSpacePoint ElbowLeftP = bodies[bodyIndex].Joints[JointType.ElbowLeft].Position;
            CameraSpacePoint ShoulderLeftP = bodies[bodyIndex].Joints[JointType.ShoulderLeft].Position;
            CameraSpacePoint HandLeftP = bodies[bodyIndex].Joints[JointType.HandLeft].Position;
            CameraSpacePoint HandTipLeftP = bodies[bodyIndex].Joints[JointType.HandTipLeft].Position;
            CameraSpacePoint HipLeftP = bodies[bodyIndex].Joints[JointType.HipLeft].Position;
            CameraSpacePoint KneeLeftP = bodies[bodyIndex].Joints[JointType.KneeLeft].Position;
            CameraSpacePoint AnkleLeftP = bodies[bodyIndex].Joints[JointType.AnkleLeft].Position;
            CameraSpacePoint FootLeftP = bodies[bodyIndex].Joints[JointType.FootLeft].Position;


            // elbow r angle: 0
            Vector rElbowWrist = new Vector(Convert.ToDouble(ElbowRightP.X - WristRightP.X), Convert.ToDouble(ElbowRightP.Y - WristRightP.Y));
            Vector rElbowShoulder = new Vector(Convert.ToDouble(ElbowRightP.X - ShoulderRightP.X), Convert.ToDouble(ElbowRightP.Y - ShoulderRightP.Y));
            //Console.WriteLine(System.Windows.Vector.AngleBetween(rElbowWrist, rElbowShoulder));
            jointAngles[0] = System.Windows.Vector.AngleBetween(rElbowWrist, rElbowShoulder);
            //return System.Windows.Vector.AngleBetween(rElbowWrist, rElbowShoulder);
            
            // elbow l angle: 1
            Vector lElbowWrist = new Vector(Convert.ToDouble(ElbowLeftP.X - WristLeftP.X), Convert.ToDouble(ElbowLeftP.Y - WristLeftP.Y));
            Vector lElbowShoulder = new Vector(Convert.ToDouble(ElbowLeftP.X - ShoulderLeftP.X), Convert.ToDouble(ElbowLeftP.Y - ShoulderLeftP.Y));
            //Console.WriteLine(System.Windows.Vector.AngleBetween(lElbowWrist, lElbowShoulder));
            jointAngles[1] = System.Windows.Vector.AngleBetween(lElbowWrist, lElbowShoulder);

            // shoulder r angle: 2
            Vector rShoulderElbow = new Vector(Convert.ToDouble(ShoulderRightP.X - ElbowRightP.X), Convert.ToDouble(ShoulderRightP.Y - ElbowRightP.Y));
            Vector rShoulderSpineShoulder = new Vector(Convert.ToDouble(ShoulderRightP.X - SpineShoulderP.X), Convert.ToDouble(ShoulderRightP.Y - SpineShoulderP.Y));
            //Console.WriteLine(System.Windows.Vector.AngleBetween(rShoulderElbow, rShoulderSpineShoulder));
            jointAngles[2] = System.Windows.Vector.AngleBetween(rShoulderElbow, rShoulderSpineShoulder);

            // shoulder l angle: 3
            Vector LShoulderElbow = new Vector(Convert.ToDouble(ShoulderLeftP.X - ElbowLeftP.X), Convert.ToDouble(ShoulderLeftP.Y - ElbowLeftP.Y));
            Vector LShoulderSpineShoulder = new Vector(Convert.ToDouble(ShoulderLeftP.X - SpineShoulderP.X), Convert.ToDouble(ShoulderLeftP.Y - SpineShoulderP.Y));
            //Console.WriteLine(System.Windows.Vector.AngleBetween(LShoulderElbow, LShoulderSpineShoulder));
            jointAngles[3] = System.Windows.Vector.AngleBetween(LShoulderElbow, LShoulderSpineShoulder);

            // spine r angle: 4
            Vector RSpineShoulderShoulder = new Vector(Convert.ToDouble(SpineShoulderP.X - ShoulderRightP.X), Convert.ToDouble(SpineShoulderP.Y - ShoulderRightP.Y));
            Vector RSpineShoulderSpineMid = new Vector(Convert.ToDouble(SpineShoulderP.X - SpineMidP.X), Convert.ToDouble(SpineShoulderP.Y - SpineMidP.Y));
            //Console.WriteLine(System.Windows.Vector.AngleBetween(RSpineShoulderShoulder, RSpineShoulderSpineMid));
            jointAngles[4] = System.Windows.Vector.AngleBetween(RSpineShoulderShoulder, RSpineShoulderSpineMid);

            // spine l angle: 5
            Vector LSpineShoulderShoulder = new Vector(Convert.ToDouble(SpineShoulderP.X - ShoulderLeftP.X), Convert.ToDouble(SpineShoulderP.Y - ShoulderLeftP.Y));
            Vector LSpineShoulderSpineMid = new Vector(Convert.ToDouble(SpineShoulderP.X - SpineMidP.X), Convert.ToDouble(SpineShoulderP.Y - SpineMidP.Y));
            //Console.WriteLine(System.Windows.Vector.AngleBetween(LSpineShoulderShoulder, LSpineShoulderSpineMid));
            jointAngles[5] = System.Windows.Vector.AngleBetween(LSpineShoulderShoulder, LSpineShoulderSpineMid);

            // hip r angle: 6
            Vector HipSpineBaseR = new Vector(Convert.ToDouble(HipRightP.X - SpineBaseP.X), Convert.ToDouble(HipRightP.Y - SpineBaseP.Y));
            Vector HipKneeR = new Vector(Convert.ToDouble(HipRightP.X - KneeRightP.X), Convert.ToDouble(HipRightP.Y - KneeRightP.Y));
            //Console.WriteLine(System.Windows.Vector.AngleBetween(HipSpineBaseR, HipKneeR));
            jointAngles[6] = System.Windows.Vector.AngleBetween(HipSpineBaseR, HipKneeR);

            // hip l angle: 7
            Vector lHipSpineBase = new Vector(Convert.ToDouble(HipLeftP.X - SpineBaseP.X), Convert.ToDouble(HipLeftP.Y - SpineBaseP.Y));
            Vector lHipKnee = new Vector(Convert.ToDouble(HipLeftP.X - KneeLeftP.X), Convert.ToDouble(HipLeftP.Y - KneeLeftP.Y));
            //Console.WriteLine(System.Windows.Vector.AngleBetween(lHipSpineBase, lHipKnee));
            jointAngles[7] = System.Windows.Vector.AngleBetween(lHipSpineBase, lHipKnee);

            // knee r angle: 8
            Vector KneeHipr = new Vector(Convert.ToDouble(KneeRightP.X - HipRightP.X), Convert.ToDouble(KneeRightP.Y - HipRightP.Y));
            Vector KneeAnkler = new Vector(Convert.ToDouble(KneeRightP.X - AnkleRightP.X), Convert.ToDouble(KneeRightP.Y - AnkleRightP.Y));
            //Console.WriteLine(System.Windows.Vector.AngleBetween(KneeHipr, KneeAnkler));
            jointAngles[8] = System.Windows.Vector.AngleBetween(KneeHipr, KneeAnkler);

            // knee l angle: 9
            Vector lKneeHip = new Vector(Convert.ToDouble(KneeLeftP.X - HipLeftP.X), Convert.ToDouble(KneeLeftP.Y - HipLeftP.Y));
            Vector lKneeAnkle = new Vector(Convert.ToDouble(KneeLeftP.X - AnkleLeftP.X), Convert.ToDouble(KneeLeftP.Y - AnkleLeftP.Y));
            //Console.WriteLine(System.Windows.Vector.AngleBetween(lKneeHip, lKneeAnkle));
            jointAngles[9] = System.Windows.Vector.AngleBetween(lKneeHip, lKneeAnkle);

            // neck angle: 10
            Vector NeckHead = new Vector(Convert.ToDouble(NeckP.X - HeadP.X), Convert.ToDouble(NeckP.Y - HeadP.Y));
            Vector NeckSpineShoulder = new Vector(Convert.ToDouble(NeckP.X - SpineShoulderP.X), Convert.ToDouble(NeckP.Y - SpineShoulderP.Y));
            //Console.WriteLine(System.Windows.Vector.AngleBetween(NeckHead, NeckSpineShoulder));
            jointAngles[10] = System.Windows.Vector.AngleBetween(NeckHead, NeckSpineShoulder);


        }

        public double CalculateAccuracy(int i) // what does this need to take in?
        {
            // if less than angle user/orignal
            // if greater than angle user-2*angleoff then divide user by original
            /*if (userAngle > orignalAngle)
            {
                double diff = userAngle - orignalAngle;
                userAngle = userAngle - 2 * diff;
            }

            return userAngle/orignalAngle;*/

            double userAngle = jointAngles[i];
            double originalAngle = originalAngles[i];
            userAngle = Math.Abs(userAngle) / 180;
            originalAngle = Math.Abs(originalAngle) /180;

            double diff = Math.Abs(userAngle - originalAngle);
            return 1 / (diff + 1);
        }

        private double GetJointThickness(int i) //will also need a way to identify the specific joint, then call CalcAccuracy and CalcAngle
        {
            double accuracy = CalculateAccuracy(i);

            double jt = -9 * accuracy + 12;
            return jt;
        }

        private void GetNextMove()
        {
            if (kinectSensor.IsAvailable)
            {
                for (int i = 0; i < 11; i++)
                {
                originalAngles[i] = Convert.ToDouble(allAngles[indexOfAngle]);
                Console.WriteLine(indexOfAngle);
                Console.WriteLine(originalAngles[i]);
                indexOfAngle++;

                }
            }
            else
            {
                Console.WriteLine("The Kinect is currently not being used");
            }
           
        }

        /*private void GetNextImage(int n)
        {
            // read image
            image = image + n;

            */
            /*Image snapshot = new Image();
            snapshot.Width = 100;

            BitmapImage bitmap = new BitmapImage();
            //this.snapshot = "Images\mmlogo.png";
            bitmap.BeginInit();
            bitmap.UriSource = new Uri(Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\..\..\")) + @"\" + image + ".jpg");
            bitmap.EndInit();

            snapshot.Source = bitmap;*/

            //var uri = new Uri(@"Images/" + Convert.ToString(image) + ".jpg", UriKind.Relative);
            //Console.WriteLine(uri.ToString());
            //snapshot.Image
            //snapshot.Source = new BitmapImage(new Uri(@"/Images/" + Convert.ToString(image) + ".jpg", UriKind.Relative));
            /*BitmapImage bitmap = new BitmapImage();
            this.snapshot = "Images\mmlogo.png";
            bitmap.BeginInit();
            bitmap.UriSource = new Uri(Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\..\..\")) + @"\" + image + ".jpg");
            bitmap.EndInit();
            this.snapshot = bitmap;*/

            /*using (var stream = await imageFile.OpenReadAsync())
            {
                await bitmap.SetSourceAsync(stream);
            }*/
            /*
            //this.snapshot
            BitmapImage bitmap = new BitmapImage();
            bitmap.BeginInit();
            bitmap.CacheOption = BitmapCacheOption.None;
            bitmap.CreateOptions = BitmapCreateOptions.IgnoreImageCache;
            bitmap.UriSource = new Uri(Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\..\..\")) + @"" + image + ".jpg");

            bitmap.EndInit();
            Snapshot = new BitmapImage(bitmap.UriSource);

            Console.WriteLine(new Uri(Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\..\..\..\")) + @"" + image + ".jpg"));
            //imagetochange.Source = Snapshot;
        }*/

        private void RightArrow_Click(object sender, RoutedEventArgs e)
        {
            //change image (and underlay) - no longer doing underlay
            //this.imageSource = new DrawingImage(this.drawingGroup) // need to define drawing group for underlay
            Console.WriteLine("right");
            Console.WriteLine(allAngles.Count);

            if(indexOfAngle >= allAngles.Count)
            {
                indexOfAngle = indexOfAngle - 11;
            }

            //0-10
            //11-21
            GetNextMove();
            //GetNextImage(1);
        }

        private void LeftArrow_Click(object sender, RoutedEventArgs e)
        {
            // change image (and underlay) - no longer doing underlay
            ////this.imageSource = new DrawingImage(this.drawingGroup) // need to define drawing group for underlay
            Console.WriteLine("left");

            indexOfAngle = indexOfAngle - 22;
            if(indexOfAngle < 0)
            {
                indexOfAngle = 0;
            }
            GetNextMove();
            //GetNextImage(-1);
        }
    }
}
