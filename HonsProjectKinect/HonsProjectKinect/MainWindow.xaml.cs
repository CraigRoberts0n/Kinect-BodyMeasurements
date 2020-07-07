using System;
using System.ComponentModel;
using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using System.Diagnostics;
//using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
//using System.Windows.Data;
//using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
//using System.Windows.Navigation;
//using System.Windows.Shapes;
using Microsoft.Kinect;

namespace HonsProjectKinect
{
    public partial class MainWindow : Window
    {
        //BodyIndex Colours
        private const int BytesPerPixel = 4;
        private static readonly uint[] BodyColor =
        {
            0x0000FF00,
            0x00FF0000,
            0xFFFF4000,
            0x40FFFF00,
            0xFF40FF00,
            0xFF808000,
        };

        //Skeleton
        private const double HandSize = 30;
        private const double JointThickness = 3;
        private const double ClipBoundsThickness = 10;
        private const float InferredZPositionClamp = 0.1f;
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));
        private readonly Brush inferredJointBrush = Brushes.Yellow;
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);
        private DrawingGroup drawingGroup;
        private DrawingImage imageSource;
        private KinectSensor kinectSensor = null;
        private CoordinateMapper coordinateMapper = null;
        private Body[] bodies = null;
        private List<Tuple<JointType, JointType>> bones;
        private int displayWidth;
        private int displayHeight;
        private List<Pen> bodyColors;

        //MultiFrame
        private MultiSourceFrameReader multiFrameSourceReader = null;

        //BodyIndex 
        private FrameDescription bodyIndexFrameDescription = null;
        private WriteableBitmap bodyIndexBitmap = null;
        private uint[] bodyIndexPixels = null;
        public string segmentationtitleTB = null;

        //Depth
        private WriteableBitmap depthBitmap = null;
        private byte[] depthPixels = null;
        private FrameDescription depthFrameDescription = null;
        private const int MapDepthToByte = 8000 / 256;

        public MainWindow()
        {
            this.kinectSensor = KinectSensor.GetDefault();
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;
            this.bones = new List<Tuple<JointType, JointType>>();

            //MultiFrame Reader
            this.multiFrameSourceReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Body | FrameSourceTypes.BodyIndex | FrameSourceTypes.Depth);
            this.multiFrameSourceReader.MultiSourceFrameArrived += this.Reader_MultiSourceFrameArrived;

            //BodyIndexDescription
            this.bodyIndexFrameDescription = this.kinectSensor.BodyIndexFrameSource.FrameDescription;

            //BodyIndex Pixels Array
            this.bodyIndexPixels = new uint[this.bodyIndexFrameDescription.Width * this.bodyIndexFrameDescription.Height];

            //BodyIndex Bitmap
            this.bodyIndexBitmap = new WriteableBitmap(this.bodyIndexFrameDescription.Width, this.bodyIndexFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

            //Depth
            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
            this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
            this.depthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);

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

            // open the sensor
            this.kinectSensor.Open();
            this.drawingGroup = new DrawingGroup();
            this.imageSource = new DrawingImage(this.drawingGroup);
            this.DataContext = this;

            InitializeComponent();
        }

        private unsafe void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            BodyIndexFrame bodyIndexFrame = null;
            BodyFrame bodyFrame = null;
            DepthFrame depthFrame = null;

            //Get frames
            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();

            //Check is frames are null
            if (multiSourceFrame == null)
            {
                return;
            }

            //Try acquiring frames
            try
            {
                bodyIndexFrame = multiSourceFrame.BodyIndexFrameReference.AcquireFrame();
                bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame();
                depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame();

                //Check if frames are null
                if ((bodyIndexFrame == null) || (bodyFrame == null) || (depthFrame == null))
                {
                    return;
                }

                bool bodyIndexFrameProcessed = false;

                //Populate the body index array + display the depth image
                using (KinectBuffer bodyIndexBuffer = bodyIndexFrame.LockImageBuffer())
                {
                    using (Microsoft.Kinect.KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                    {
                        if (((this.bodyIndexFrameDescription.Width * this.bodyIndexFrameDescription.Height) == bodyIndexBuffer.Size) &&
                             (this.bodyIndexFrameDescription.Width == this.bodyIndexBitmap.PixelWidth) && (this.bodyIndexFrameDescription.Height == this.bodyIndexBitmap.PixelHeight))
                        {
                            this.ProcessDepthBodyIndexFrameData(bodyIndexBuffer.UnderlyingBuffer, bodyIndexBuffer.Size, depthBuffer.UnderlyingBuffer, depthBuffer.Size, depthFrame.DepthMinReliableDistance);
                            bodyIndexFrameProcessed = true;
                        }
                    }
                }

                //BodyIndex render pixels
                if (bodyIndexFrameProcessed)
                {
                    this.RenderBodyIndexDepthPixels(); 
                }
                bodyIndexFrame.Dispose();
                bodyIndexFrame = null;

                //Add body to Array for skeleton 
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                }

                //Draw the skeleton on the display
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    dc.DrawRectangle(Brushes.Transparent, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                    int penIndex = 0;
                    //iterate through all skeleton models - dependant on people in view
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];
                        if (body.IsTracked)
                        {
                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }
                            this.DrawBody(joints, jointPoints, dc, drawPen);
                        }
                    }
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }

                //get the depth frames and perform the Height and Width calculations
                using (KinectBuffer depthFrameData = depthFrame.LockImageBuffer())
                {
                    ushort* frameDataDepth = (ushort*)depthFrameData.UnderlyingBuffer;

                    //iterate through all tracked bodies - uses skeleton to get amount of body indexes
                    for (int i = 0; i < bodyFrame.BodyCount; i++)
                    {
                        if (this.bodies[i].IsTracked)
                        {
                            //creates the textbloxk that appears over top individuals heads
                            TextBlock Overlaylabel = (TextBlock)segmentationView.FindName("heightOverlay" + i);
                            if (Overlaylabel != null)
                            {
                                UnregisterName(Overlaylabel.Name);
                                segmentationView.Children.Remove(Overlaylabel);
                            }

                            //Get the Segmentation Height of a individual depending on variable 'i'
                            Tuple<double, double, double> segmentationHeight = getHeightSegmentation(depthFrame.FrameDescription.Width, frameDataDepth, i);

                            //Get the Segmentation Width of a individual depending on variable 'i'
                            double segmentationWidth = getWidthSegmentation(depthFrame.FrameDescription.Width, frameDataDepth, i);

                            //Draw the height and width on the display using the textblocks
                            drawOnSegmentedDisplay(i, segmentationHeight.Item1, segmentationWidth, segmentationHeight.Item2, segmentationHeight.Item3);
                        }
                    }
                }
                depthFrame.Dispose();
                depthFrame = null;

            }
            finally
            {
                if (bodyIndexFrame != null)
                {
                    bodyIndexFrame.Dispose();
                }
                if (bodyFrame != null)
                {
                    bodyFrame.Dispose();
                }
                if (depthFrame != null)
                {
                    depthFrame.Dispose();
                }
            }
        }

        public unsafe double getWidthSegmentation(double frameWidth, ushort* frameDataDepth, int bodyIndexValue)
        {
            var tempYAxisValue = 0;
            var tempYAxisIndex = 0;

            //iterate through Body Index Pixels array 512 indexes at a time - display 512 pixels wide
            for (int i = 0; i < bodyIndexPixels.Length; i += 512)
            {
                var dest = new uint[512];
                //Grab 512 bits of data - equals Y axis = 1
                Array.Copy(bodyIndexPixels, i, dest, 0, 512);

                //Iterate through this Y = 1 and count how much body appears 
                int bodyIndexOccu = 0;
                foreach (uint s in dest)
                {
                    if (s.Equals(BodyColor[bodyIndexValue])) bodyIndexOccu++;
                }

                //Updates variables with most populated Y Axis - Logs Y Axis and occurrences 
                if (bodyIndexOccu > tempYAxisValue)
                {
                    tempYAxisValue = 0;
                    tempYAxisValue = tempYAxisValue + bodyIndexOccu;
                    tempYAxisIndex = 0;
                    tempYAxisIndex = tempYAxisIndex + i;
                }
            }

            //Get first & last point indexes of the Y Axis
            uint[] getYAxisRange = new uint[512];
            Array.Copy(bodyIndexPixels, tempYAxisIndex, getYAxisRange, 0, 512);

            var firstPoint = Array.FindIndex(getYAxisRange, val => val.Equals(BodyColor[bodyIndexValue])) + (tempYAxisIndex);
            var lastPoint = Array.FindLastIndex(getYAxisRange, val => val.Equals(BodyColor[bodyIndexValue])) + (tempYAxisIndex);

            if (firstPoint != -1)
            {
                double XCoor = (Math.Floor(firstPoint % frameWidth));
                double YCoor = (firstPoint / frameWidth);
                double ZCoor = frameDataDepth[firstPoint];

                double XCoor2 = (Math.Floor(lastPoint % frameWidth));
                double YCoor2 = (lastPoint / frameWidth);
                double ZCoor2 = frameDataDepth[lastPoint];

                //Translates X,Y,Z of first/last point into 3D space points that we can use to calculate distance
                CameraSpacePoint firstIndex = xyToCameraSpacePoint(Convert.ToSingle(XCoor), Convert.ToSingle(YCoor), (ushort)ZCoor);
                CameraSpacePoint lastIndex = xyToCameraSpacePoint(Convert.ToSingle(XCoor2), Convert.ToSingle(YCoor2), (ushort)ZCoor2);

                //gets length in metres between the two points
                double segmentationWidth = getLength(firstIndex, lastIndex);
                return segmentationWidth;
            }
            return 0.0;
        }

        public unsafe Tuple<double, double, double> getHeightSegmentation(double frameWidth, ushort* frameDataDepth, int bodyIndexValue)
        {
            //Gets first/last index of body index - because we're looking for height just first positive value index in array and last 
            var firstIndexOfBody = Array.FindIndex(bodyIndexPixels, val => val.Equals(BodyColor[bodyIndexValue]));
            var lastIndexOfBody = Array.FindLastIndex(bodyIndexPixels, val => val.Equals(BodyColor[bodyIndexValue]));

            if (firstIndexOfBody != -1)
            {
                double XCoor = Math.Floor(firstIndexOfBody % frameWidth);
                double YCoor = firstIndexOfBody / frameWidth;
                double ZCoor = frameDataDepth[firstIndexOfBody];

                double XCoor2 = Math.Floor(lastIndexOfBody % frameWidth);
                double YCoor2 = lastIndexOfBody / frameWidth;
                double ZCoor2 = frameDataDepth[lastIndexOfBody];

                //Translates X,Y,Z of first/last point into 3D space points that we can use to calculate distance
                CameraSpacePoint firstIndex = xyToCameraSpacePoint(Convert.ToSingle(XCoor), Convert.ToSingle(YCoor), (ushort)ZCoor);
                CameraSpacePoint lastIndex = xyToCameraSpacePoint(Convert.ToSingle(XCoor2), Convert.ToSingle(YCoor2), (ushort)ZCoor2);

                //gets length in metres between the two points
                double segmentationHeight = getLength(firstIndex, lastIndex);

                return Tuple.Create(segmentationHeight, XCoor, YCoor);
            }
            return Tuple.Create(0.0, 0.0, 0.0);
        }

        public void drawOnSegmentedDisplay(int bodyIndexValue, double height, double width, double X, double Y)
        {
            //Creates new textblock
            TextBlock textBlockOverlay = new TextBlock();
            textBlockOverlay.Foreground = Brushes.White;

            double myMaxHeight;
            double myMaxWidth;

            double myMinHeight;
            double myMinWidth;

            //Gets the Max height/width values from interface
            if (Double.TryParse(txtBoxHeight.Text, out myMaxHeight) | Double.TryParse(txtBoxWidth.Text, out myMaxWidth) | 
                Double.TryParse(txtBoxHeightMin.Text, out myMinHeight) | Double.TryParse(txtBoxWidthMin.Text, out myMinWidth))
            {
                if (height > myMaxHeight || width > myMaxWidth || height < myMinHeight || width < myMinWidth)
                {
                    textBlockOverlay.Foreground = Brushes.Red;
                }
            }

            textBlockOverlay.FontSize = 12;
            textBlockOverlay.Name = "heightOverlay" + bodyIndexValue;
            RegisterName(textBlockOverlay.Name, textBlockOverlay);

            textBlockOverlay.Text = "BodyIndex: " + bodyIndexValue + "\n" +
                                    "Height: " + height.ToString("0.###") + " m" + "\n" +
                                    "Width: " + width.ToString("0.###") + " m" + "\n";
            segmentationView.Children.Add(textBlockOverlay);

            //Place textblock above individuals head
            textBlockOverlay.RenderTransform = new TranslateTransform(X, Y - 65);
        }

        public CameraSpacePoint xyToCameraSpacePoint(float X, float Y, ushort Z)
        {
            DepthSpacePoint depthPoint = new DepthSpacePoint();
            depthPoint.X = X;
            depthPoint.Y = Y;

            //Maps the X,Y,Z to CameraSpace
            var CameraPoint = coordinateMapper.MapDepthPointToCameraSpace(depthPoint, Z);

            return CameraPoint;
        }

        public static double getLength(CameraSpacePoint p1, CameraSpacePoint p2)
        {
            //Calculates Distance between 2 3D points
            return Math.Sqrt(
                Math.Pow(p1.X - p2.X, 2) +
                Math.Pow(p1.Y - p2.Y, 2) +
                Math.Pow(p1.Z - p2.Z, 2));
        }

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
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

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

        private unsafe void ProcessDepthBodyIndexFrameData(IntPtr bodyIndexFrameData, uint bodyIndexFrameDataSize, IntPtr depthFrameData, uint depthFrameDataSize, ushort minDepth)
        {
            byte* frameData = (byte*)bodyIndexFrameData;

            ushort* frameDataDepth = (ushort*)depthFrameData;

            for (int i = 0; i < (int)bodyIndexFrameDataSize; ++i)
            {
                if (frameData[i] < 5)
                {
                    //Populates Body Index Array - body index are between 0-5 anything > is background
                    //Uses the Index Value (0-5) as the index for its colour - bodyIndexPixels gets populated with 8-bit uint that is index colour
                    this.bodyIndexPixels[i] = BodyColor[frameData[i]];
                }
                else
                {
                    this.bodyIndexPixels[i] = 0x00000000;
                }

                //Populates the depth array that is seen in Interface
                ushort depth = frameDataDepth[i];
                this.depthPixels[i] = (byte)(depth >= minDepth && depth <= ushort.MaxValue ? (depth / MapDepthToByte) : 0);
            }
        }

        private void RenderBodyIndexDepthPixels()
        {
            //Renders pixels on the interface
            this.bodyIndexBitmap.WritePixels(
                new Int32Rect(0, 0, this.bodyIndexBitmap.PixelWidth, this.bodyIndexBitmap.PixelHeight),
                this.bodyIndexPixels,
                this.bodyIndexBitmap.PixelWidth * (int)BytesPerPixel,
                0);

            this.depthBitmap.WritePixels(
                new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                this.depthPixels,
                this.depthBitmap.PixelWidth,
                0);
        }

        public ImageSource ImageSource
        {
            //getters for linking data to interface
            get
            {
                return this.imageSource;
            }
        }

        public ImageSource ImageSourceBodyIndex
        {
            //getters for linking data to interface
            get
            {
                return this.bodyIndexBitmap;
            }
        }

        public ImageSource ImageSourceDepth
        {
            //getters for linking data to interface
            get
            {
                return this.depthBitmap;
            }
        }

        //Check if window is loaded
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.multiFrameSourceReader != null)
            {
                this.multiFrameSourceReader.MultiSourceFrameArrived += this.Reader_MultiSourceFrameArrived;
                Console.WriteLine(":)");
            }
        }

        //Check if window is closed
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.multiFrameSourceReader != null)
            {
                // BodyFrameReader is IDisposable
                this.multiFrameSourceReader.Dispose();
                this.multiFrameSourceReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        private void Window_MouseDown(object sender, MouseButtonEventArgs e)
        {
            //Weird bug where if text box is selected/in focus it makes the height/width textblock flicker bad
            //Just clears focus away from textbox if window is clicked
            Keyboard.ClearFocus();
        }
    }
}
