

namespace myske
{
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using Microsoft.Kinect;
    using System.Diagnostics;
    using System.Runtime.InteropServices;

    using System.Windows.Media.Imaging;
   // using System.Windows.Forms.SystemInformation;
    using System.Windows.Forms.ComponentModel;
    

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private const float RenderWidth = 640.0f;
        private const float RenderHeight = 480.0f;
        private const double JointThickness = 3;
        private const double BodyCenterThickness=10;
        private const double ClipBoundsThickness=10;
        private readonly Brush centerPointBrush = Brushes.Blue;
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));
        private readonly Brush inferredJointBrush = Brushes.Yellow;
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        private KinectSensor sensor;
        private DrawingGroup drawingGroup;
        private DrawingImage imageSource;
        /// Bitmap that will hold color information
        /// </summary>
        private WriteableBitmap colorBitmap;

        /// <summary>
        /// Intermediate storage for the depth data received from the camera
        /// </summary>
        private DepthImagePixel[] depthPixels;

        /// <summary>
        /// Intermediate storage for the depth data converted to color
        /// </summary>
        private byte[] colorPixels;



        [DllImport("mouseControls.dll", CallingConvention = CallingConvention.Cdecl)]
        public extern static bool move_cursorTo(int x, int y);

        [DllImport("mouseControls.dll", CallingConvention = CallingConvention.Cdecl)]
        public extern static bool click_Left_down(int x, int y);

        [DllImport("mouseControls.dll", CallingConvention = CallingConvention.Cdecl)]
        public extern static bool click_Right_up(int x, int y);

        [DllImport("mouseControls.dll", CallingConvention = CallingConvention.Cdecl)]
        public extern static bool click_Left_up(int x, int y);

        [DllImport("mouseControls.dll", CallingConvention = CallingConvention.Cdecl)]
        public extern static bool click_Right_down(int x, int y);
        public MainWindow()
        {
            InitializeComponent();
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Display the drawing using our image control
            Image.Source = this.imageSource;

            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make your app robust against plug/unplug, 
            // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit (See components in Toolkit Browser).
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {
                this.sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);

                // Allocate space to put the depth pixels we'll receive
                this.depthPixels = new DepthImagePixel[this.sensor.DepthStream.FramePixelDataLength];

                // Allocate space to put the color pixels we'll create
                this.colorPixels = new byte[this.sensor.DepthStream.FramePixelDataLength * sizeof(int)];

                // This is the bitmap we'll display on-screen
                this.colorBitmap = new WriteableBitmap(this.sensor.DepthStream.FrameWidth, this.sensor.DepthStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);

                // Set the image we display to point to the bitmap where we'll put the image data
                this.Image.Source = this.colorBitmap;

                // Add an event handler to be called whenever there is new depth frame data
                this.sensor.DepthFrameReady += this.SensorDepthFrameReady;




                // Turn on the skeleton stream to receive skeleton frames
                this.sensor.SkeletonStream.Enable();

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

                // Start the sensor!
                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }

            if (null == this.sensor)
            {
                MessageBox.Show("No kinect Connected");
                System.Windows.Forms.Application.Exit();
            }
        }

        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
                //else
                //{
                //    MessageBox.Show("no person in view");
                //}
            }

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                //dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                if (skeletons.Length != 0)
                {
                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            this.DrawBonesAndJoints(skel, dc);
                        }
                        else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                        {
                            dc.DrawEllipse(
                            this.centerPointBrush,
                            null,
                            this.SkeletonPointToScreen(skel.Position),
                            BodyCenterThickness,
                            BodyCenterThickness);
                        }
                    }
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
        }

        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            
            double mapped_x = (depthPoint.X-100) / Image.Width * 1366.0;//static screen size
            double mapped_y = (depthPoint.Y-100) / Image.Height * 768.0;
            label.Content = (int)depthPoint.X + " , " + (int)depthPoint.Y;
            screen_coordinate.Content =(int) mapped_x+ " , " + (int)mapped_y;//+" , "+skelpoint.Z;
            
            bool a = move_cursorTo((int) mapped_x, (int ) mapped_y);//move the mouse cursor
            return new Point(depthPoint.X, depthPoint.Y);
        }

        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            // Render Torso
            //this.DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);
            //this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            //this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            //this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            //this.DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            //this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            //this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            // Left Arm
            //this.DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            //this.DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            //this.DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);

            // Right Arm
            //this.DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight);
            //this.DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            //th/is.DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);

            // Left Leg
            //this.DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft);
            //this.DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft);
            //this.DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft);

            // Right Leg
            //this.DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight);
            //this.DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight);
            //this.DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight);

            // Render Joints
            Joint left_joint = skeleton.Joints[JointType.HandLeft];
            //JointType joint_t=new JointType.HandLeft;
            Brush drawBrush = null;
            
            if (left_joint.TrackingState == JointTrackingState.Tracked)
            {
                drawBrush = this.trackedJointBrush;
            }
            else if (left_joint.TrackingState == JointTrackingState.Inferred)
            {
                drawBrush = this.inferredJointBrush;
            }
            if (drawBrush != null)
            {
                drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen(left_joint.Position), JointThickness, JointThickness);
                //Point a = left_joint.Position.ToString;
                //label.Content = left_joint.Position.X.ToString() + "   ,   " + left_joint.Position.Y;
                //Trace.WriteLine(a);
            }
            //foreach (Joint joint in skeleton.Joints)
            //{
              //  Brush drawBrush = null;

//                if (joint.TrackingState == JointTrackingState.Tracked)
 //               {
  //                  drawBrush = this.trackedJointBrush;
   //             }
    //            else if (joint.TrackingState == JointTrackingState.Inferred)
     //           {
      //              drawBrush = this.inferredJointBrush;
       //         }
        //        //if(JointType.AnkleLeft)
         //       if (drawBrush != null)
          //      {
           //         drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
            //        label.Content = this.SkeletonPointToScreen(joint.Position);
             //   }
            //}
        }

        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked)
            {
                drawPen = this.trackedBonePen;
            }

            drawingContext.DrawLine(drawPen, this.SkeletonPointToScreen(joint0.Position), this.SkeletonPointToScreen(joint1.Position));
            
        }

        private void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }
        private void SensorDepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            {
                if (depthFrame != null)
                {
                    // Copy the pixel data from the image to a temporary array
                    depthFrame.CopyDepthImagePixelDataTo(this.depthPixels);

                    // Get the min and max reliable depth for the current frame
                    int minDepth = depthFrame.MinDepth;
                    int maxDepth = depthFrame.MaxDepth;
                    mm.Content = minDepth+","+maxDepth;
                    // Convert the depth to RGB
                    int colorPixelIndex = 0;
                    for (int i = 0; i < this.depthPixels.Length; ++i)
                    {
                        // Get the depth for this pixel
                        short depth = depthPixels[i].Depth;

                        // To convert to a byte, we're discarding the most-significant
                        // rather than least-significant bits.
                        // We're preserving detail, although the intensity will "wrap."
                        // Values outside the reliable depth range are mapped to 0 (black).

                        // Note: Using conditionals in this loop could degrade performance.
                        // Consider using a lookup table instead when writing production code.
                        // See the KinectDepthViewer class used by the KinectExplorer sample
                        // for a lookup table example.
                        //byte intensity = (byte)(depth >= minDepth && depth <= maxDepth ? 1 : 0);
                        //int x = 10;
                        byte intensity;

                        if(depth>=maxDepth-1000 && depth<=maxDepth)
                        {
                            intensity =(byte) depth;    
                        }
                        else
                        {
                            intensity = 0;
                        }
                        //byte intensity = (byte)(depth >=maxDepth-1000 && depth <=maxDepth ? depth :0);

                        // Write out blue byte
                        this.colorPixels[colorPixelIndex++] = intensity;

                        // Write out green byte
                        this.colorPixels[colorPixelIndex++] = intensity;

                        // Write out red byte                        
                        this.colorPixels[colorPixelIndex++] = intensity;

                        // We're outputting BGR, the last byte in the 32 bits is unused so skip it
                        // If we were outputting BGRA, we would write alpha here.
                        ++colorPixelIndex;
                    }

                    // Write the pixel data into our bitmap
                    this.colorBitmap.WritePixels(
                        new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight),
                        this.colorPixels,
                        this.colorBitmap.PixelWidth * sizeof(int),
                        0);
                    Image.Source = colorBitmap;
                }
            }
        }
    }
}
