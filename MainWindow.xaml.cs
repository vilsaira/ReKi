using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

using Microsoft.Kinect;
using System.Drawing;
using System.IO;

namespace ReKi2 {
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    ///     

    public partial class MainWindow : Window, INotifyPropertyChanged{
        
        private KinectSensor _sensor = null;        
        private ColorFrameReader _readerColor = null;
        private DepthFrameReader _readerDepth = null;
        private ushort[] _depthMM = null; // this is used for depth measurements
        private TimeSpan? _firstFrameTime = new TimeSpan?();
        private string _statusText = null;
        private string _pathColor = null;
        private string _pathDepth = null;
        private string _subjectID = null;
        private bool _recording = false;
        private bool _processing = false;       
        
        public MainWindow( ) {                       
            this._sensor = KinectSensor.GetDefault();
            this._sensor.IsAvailableChanged += this.Sensor_Status;            
            if (this._sensor != null) {
                this._sensor.Open();
            }            
            this._readerColor = this._sensor.ColorFrameSource.OpenReader();
            this._readerDepth = this._sensor.DepthFrameSource.OpenReader();
            this._readerColor.FrameArrived += this.Reader_FrameArrived;
            this._readerDepth.FrameArrived += this.Reader_FrameArrived;
            InitializeComponent();
            this.Title = "Record Kinect 2.0";
            this._statusText = this._sensor.IsAvailable ? "Kinect v2 sensor OK" : "Kinect v2 sensor not found";
            this._subjectID = DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss");
            this.txtb_SubjectID.Text = this._subjectID;            
        }

        public event PropertyChangedEventHandler PropertyChanged;

        public string StatusText {
            get { return this._statusText; }
            set {
                if (this._statusText != value ) {
                    this._statusText = value;
                    if (this.PropertyChanged != null) {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }
        
        private void Sensor_Status( object sender,  IsAvailableChangedEventArgs e) {            
            if (!this._sensor.IsAvailable) {
                this.cameraColor.Source = null;
                this.cameraDepth.Source = null;
                this._statusText = "Kinect v2 sensor not found";
            } else {
                this._statusText = "Kinect v2 sensor OK";
            }
        }

        private void Reader_FrameArrived( object sender, ColorFrameArrivedEventArgs e ) {
            Task.Run(( ) => processColor(e, _recording, _firstFrameTime, _pathColor));
        }

        private void Reader_FrameArrived( object sender, DepthFrameArrivedEventArgs e ) {
            Task.Run(( ) => processDepth(e, _recording, _firstFrameTime, _depthMM, _pathDepth));
        }

        private async void processColor(ColorFrameArrivedEventArgs e, bool _recording, TimeSpan? _firstFrameTime, string _pathColor) {
            await Task.Run(( ) => {
                int width = 1920; int height = 1080;
                byte[] pixels = new byte[width * height * ((PixelFormats.Bgr32.BitsPerPixel + 7) / 8)]; // width(1920)*height(1080)*((PixelFormats.Bgr32.BitsPerPixel(=32)+7)/8)                
                int stride = width * PixelFormats.Bgr32.BitsPerPixel / 8; // width * PixelFormats.Bgr32.BitsPerPixel / 8;                
                bool gotdata = false;                
                string _frameTime = null;
                using (ColorFrame frame = e.FrameReference.AcquireFrame()) {
                    if (frame != null) {
                        _frameTime = Convert.ToString(frame.RelativeTime.TotalMilliseconds / 1000);
                        frame.CopyConvertedFrameDataToArray(pixels, ColorImageFormat.Bgra);
                        gotdata = true;
                    }
                }                
                if (gotdata) {
                    if (this._firstFrameTime == null) {
                        this._firstFrameTime = DateTime.Now.TimeOfDay;
                    }
                    TimeSpan? _frameSpan = DateTime.Now.TimeOfDay - this._firstFrameTime;

                    App.Current.Dispatcher.Invoke(new Action(( ) => {
                        this.cameraColor.Source = BitmapSource.Create(width, height, 96, 96, PixelFormats.Bgr32, null, pixels, stride);
                        this.txtb_colorTime.Text = "Color frame: " + _frameTime;
                        if (this._recording) {
                            this.txtb_stopwatch.Text = string.Format("{0:hh\\:mm\\:ss}", _frameSpan);                            
                        }
                    }));
                    if (this._recording) {                        
                        //BitmapEncoder encoder = new PngBitmapEncoder(); // 2000kt and slower
                        BitmapEncoder encoder = new JpegBitmapEncoder();  // 300kt     
                        encoder.Frames.Add(BitmapFrame.Create(BitmapSource.Create(width, height, 96, 96, PixelFormats.Bgr32, null, pixels, stride)));
                        string pathOut = System.IO.Path.Combine(this._pathColor + "\\K2V-color-" + _frameTime.Replace(',', '-') + "0000.jpg");
                        try {
                            using (FileStream fs = new FileStream(pathOut, FileMode.Create)) {
                                encoder.Save(fs);
                            }
                        } catch (IOException) {
                            // oops                                
                        }
                    }
                }
            });
        }

        private async void processDepth( DepthFrameArrivedEventArgs e, bool _recording, TimeSpan? _firstFrameTime, ushort[] _depthMM, string _pathDepth) {
            await Task.Run(( ) => {
                int width = 512; int height = 424;
                ushort[] depthData = new ushort[width * height];
                //byte[] pixelData = new byte[width * height * (PixelFormats.Gray8.BitsPerPixel + 7) / 8];
                byte[] pixelData = new byte[width * height * (PixelFormats.Bgr32.BitsPerPixel + 7) / 8];
                int stride = width * PixelFormats.Bgr32.BitsPerPixel / 8;
                //int stride8 = width * PixelFormats.Gray8.BitsPerPixel / 8;
                int stride16 = width * PixelFormats.Gray16.BitsPerPixel / 8;
                string _frameTime = null;
                bool gotdata = false;
                ushort maxDepth = 5000;
                ushort minDepth = 50;
                using (var frame = e.FrameReference.AcquireFrame()) {
                    if (frame != null) {
                        _frameTime = Convert.ToString(frame.RelativeTime.TotalMilliseconds / 1000);
                        minDepth = frame.DepthMinReliableDistance;
                        maxDepth = frame.DepthMaxReliableDistance;
                        frame.CopyFrameDataToArray(depthData);
                        gotdata = true;
                    }
                }
                if (!this._recording) {
                    this._depthMM = depthData;
                }
                if (gotdata) {
                    //for (int i = 0; i < depthData.Length; ++i) {
                    //    ushort depth = depthData[i];
                    //    byte intensity = (byte)(depth >= minDepth ? (depth <= maxDepth ? depth / 31.25 : 255) : 0);
                    //    pixelData[i] = intensity;
                    //}
                    //App.Current.Dispatcher.Invoke(new Action(( ) => {
                    //    this.cameraDepth.Source = BitmapSource.Create(width, height, 96, 96, PixelFormats.Gray8, null, pixelData, stride8);
                    //    this.txtb_depthTime.Text = _frameTime;
                    //}));
                    int colorIndex = 0;
                    for (int i = 0; i < depthData.Length; ++i) {
                        ushort depth = depthData[i];
                        byte r = (byte)(depth >= minDepth ? (depth <= maxDepth ? depth / 31.25 : 0) : 255);
                        byte bg = (byte)(depth >= minDepth ? (depth <= maxDepth ? depth / 31.25 : 255) : 0);                        
                        pixelData[colorIndex++] = bg;
                        pixelData[colorIndex++] = bg;
                        pixelData[colorIndex++] = r;
                        ++colorIndex;
                    }
                    App.Current.Dispatcher.Invoke(new Action(( ) => {
                        this.cameraDepth.Source = BitmapSource.Create(width, height, 96, 96, PixelFormats.Bgr32, null, pixelData, stride);
                        this.txtb_depthTime.Text = "Depth frame: " + _frameTime;                        
                    }));
                    if (this._recording) {                        
                        BitmapEncoder encoder = new PngBitmapEncoder();
                        encoder.Frames.Add(BitmapFrame.Create(BitmapSource.Create(width, height, 96, 96, PixelFormats.Gray16, null, depthData, stride16)));
                        string pathOut = System.IO.Path.Combine(this._pathDepth + "\\K2V-depth-" + _frameTime.Replace(',', '-') + "0000.png");                        
                        try {
                            using (FileStream fs = new FileStream(pathOut, FileMode.Create)) {
                                encoder.Save(fs);
                            }
                        } catch (IOException) {
                            // oops
                        }
                    }
                }
            });
        }
        
        private void Depth_MouseMove(object sender, MouseEventArgs e) {
            try {
                double x = Mouse.GetPosition(this.cameraDepth).X;
                double y = Mouse.GetPosition(this.cameraDepth).Y;               
                ushort depth = this._depthMM[(int)(y + x * y)];
                depth /= 10; // distance in cm
                if (depth != 0) {
                    this.txtb_distance.Text = "Distance: " + depth.ToString() + " cm";
                }
            }
            catch {

            }
        }
        
        private void Button_Record_Callback(object sender, RoutedEventArgs e) {
            if (!this._recording) {                                
                this.btn_Convert.IsEnabled = false;
                this.InputBox.Visibility = Visibility.Visible;                                
            }
            else {
                this._processing = true;
                this._recording = false;
                this.Processing.Visibility = Visibility.Visible;               
                this.recordingText.Visibility = Visibility.Collapsed;
                this.btn_RecordPause.Background = System.Windows.Media.Brushes.LightGreen;
                this.btn_RecordPause.Content = "Record";
                // Freeze UI and add blanks
                System.Threading.Thread.Sleep(500);
                string[] colorFiles = Directory.GetFiles(@_pathColor, "K2V-color*jpg");
                string[] depthFiles = Directory.GetFiles(@_pathDepth, "K2V-depth*png");
                string lastColorFile = colorFiles.Last();
                string lastDepthFile = depthFiles.Last();
                WriteableBitmap bmpC = new WriteableBitmap(1920, 1080, 96, 96, PixelFormats.Bgr32, null);
                WriteableBitmap bmpD = new WriteableBitmap(512, 424, 96, 96, PixelFormats.Gray16, null);
                bmpC.Freeze();
                bmpD.Freeze();                
                for (int i = 10; i < 70; ++i) {
                    string tmpPathC = lastColorFile.Replace("0000.jpg", "00" + i.ToString() + ".jpg");
                    string tmpPathD = lastDepthFile.Replace("0000.png", "00" + i.ToString() + ".png");
                    BitmapEncoder encoderJPEG = new JpegBitmapEncoder();
                    BitmapEncoder encoderPNG = new PngBitmapEncoder();
                    encoderJPEG.Frames.Add(BitmapFrame.Create(bmpC));
                    encoderPNG.Frames.Add(BitmapFrame.Create(bmpD));
                    try {
                        using (FileStream fs = new FileStream(tmpPathC, FileMode.Create)) { encoderJPEG.Save(fs); }
                        using (FileStream fs = new FileStream(tmpPathD, FileMode.Create)) { encoderPNG.Save(fs); }
                    } catch (IOException) {
                        // oops                                
                    }
                }                

                colorFiles = Directory.GetFiles(@_pathColor, "K2V-color*jpg");
                depthFiles = Directory.GetFiles(@_pathDepth, "K2V-depth*png");
                System.IO.File.WriteAllLines(@_subjectID + "\\log_color.txt", colorFiles);
                System.IO.File.WriteAllLines(@_subjectID + "\\log_depth.txt", depthFiles);
                this.btn_Convert.IsEnabled = true;
                this.Processing.Visibility = Visibility.Collapsed;
                this._processing = false;
            }
        }       

        private void Button_Continue_Callback(object sender, RoutedEventArgs e) {
            this.InputBox.Visibility = Visibility.Collapsed;
            this.recordingText.Visibility = Visibility.Visible;
            this._pathColor = System.IO.Directory.GetCurrentDirectory() + "\\" + this.txtb_SubjectID.Text + "\\Color";
            this._pathDepth = System.IO.Directory.GetCurrentDirectory() + "\\" + this.txtb_SubjectID.Text + "\\Depth";
            if (!System.IO.Directory.Exists(this._pathColor)) { System.IO.Directory.CreateDirectory(this._pathColor);  }
            if (!System.IO.Directory.Exists(this._pathDepth)) { System.IO.Directory.CreateDirectory(this._pathDepth); }
            // Get depth camera intrinsics and write them to a file at subject directory
            if (!System.IO.File.Exists(System.IO.Directory.GetCurrentDirectory() + "\\" + this.txtb_SubjectID.Text + "\\DepthCameraIntrinsics.txt")) {
                CameraIntrinsics tmp = this._sensor.CoordinateMapper.GetDepthCameraIntrinsics();
                string tmpstr = "FocalLengthX = " + tmp.FocalLengthX + Environment.NewLine;
                System.IO.File.WriteAllText(System.IO.Directory.GetCurrentDirectory() + "\\" + this.txtb_SubjectID.Text + "\\DepthCameraIntrinsics.txt", tmpstr);
                tmpstr = "FocalLengthY = " + tmp.FocalLengthY + Environment.NewLine;
                System.IO.File.AppendAllText(System.IO.Directory.GetCurrentDirectory() + "\\" + this.txtb_SubjectID.Text + "\\DepthCameraIntrinsics.txt", tmpstr);
                tmpstr = "PrincipalPointX = " + tmp.PrincipalPointX + Environment.NewLine;
                System.IO.File.AppendAllText(System.IO.Directory.GetCurrentDirectory() + "\\" + this.txtb_SubjectID.Text + "\\DepthCameraIntrinsics.txt", tmpstr);
                tmpstr = "PrincipalPointY = " + tmp.PrincipalPointY + Environment.NewLine;
                System.IO.File.AppendAllText(System.IO.Directory.GetCurrentDirectory() + "\\" + this.txtb_SubjectID.Text + "\\DepthCameraIntrinsics.txt", tmpstr);
                tmpstr = "RadialDistortionSecondOrder = " + tmp.RadialDistortionSecondOrder + Environment.NewLine;
                System.IO.File.AppendAllText(System.IO.Directory.GetCurrentDirectory() + "\\" + this.txtb_SubjectID.Text + "\\DepthCameraIntrinsics.txt", tmpstr);
                tmpstr = "RadialDistortionFourthOrder = " + tmp.RadialDistortionFourthOrder + Environment.NewLine;
                System.IO.File.AppendAllText(System.IO.Directory.GetCurrentDirectory() + "\\" + this.txtb_SubjectID.Text + "\\DepthCameraIntrinsics.txt", tmpstr);
                tmpstr = "RadialDistortionSixthOrder = " + tmp.RadialDistortionSixthOrder + Environment.NewLine;
                System.IO.File.AppendAllText(System.IO.Directory.GetCurrentDirectory() + "\\" + this.txtb_SubjectID.Text + "\\DepthCameraIntrinsics.txt", tmpstr);
            }
            this._recording = true;
            this._firstFrameTime = null;
            this.btn_RecordPause.Background = System.Windows.Media.Brushes.Red;
            this.btn_RecordPause.Content = "Pause/Stop";
        }

        private void Button_Cancel_Callback(object sender, RoutedEventArgs e ) {
            this.InputBox.Visibility = Visibility.Collapsed;
            this.btn_Convert.IsEnabled = true;
        }

        private void Button_Convert_Callback( object sender, RoutedEventArgs e ) {
            this.btn_Convert.IsEnabled = false;
            this.btn_RecordPause.IsEnabled = false;

            var dialog = new System.Windows.Forms.FolderBrowserDialog();
            dialog.SelectedPath = System.IO.Directory.GetCurrentDirectory();
            dialog.Description = "Select subject directory for color video conversion and depth data archiving.";
            if (dialog.ShowDialog() == System.Windows.Forms.DialogResult.OK) {
                string outDepthDir = dialog.SelectedPath + "\\Depth";
                string outColorDir = dialog.SelectedPath + "\\Color";
                string subjectID = dialog.SelectedPath.Substring(dialog.SelectedPath.LastIndexOf('\\') + 1);

                if (!System.IO.Directory.Exists(outColorDir)) {
                    MessageBox.Show(outColorDir + " was not found.");
                } else {
                    // run gstreamer
                    //Create config file for GStreamer script
                    Process process = new Process();
                    process.StartInfo.FileName = "C:\\gstreamer\\1.0\\x86_64\\bin\\gst-launch-1.0.exe";
                    process.StartInfo.WorkingDirectory = outColorDir;
                    process.StartInfo.Arguments = "tsfilesrc location=K2V-color-%f.jpg decimal-separator=- !jpegdec !videoconvert !x264enc !mp4mux !filesink location=" + dialog.SelectedPath.Replace("\\", "/") + "/" + subjectID + "_Color.mp4";
                    process.Start();
                    process.Dispose();
                }
                if (!System.IO.Directory.Exists(outDepthDir)) {
                    MessageBox.Show(outDepthDir + " was not found.");
                } else {
                    // run winzip                    
                    string tmp = dialog.SelectedPath + "\\" + subjectID + "_Depth.zip";
                    if (System.IO.File.Exists(tmp)) { System.IO.File.Delete(tmp); }
                    else { System.IO.Compression.ZipFile.CreateFromDirectory(@outDepthDir, @tmp); }
                }
            }

            this._processing = true;
            this.Processing.Visibility = Visibility.Visible;
            
            this.Processing.Visibility = Visibility.Collapsed;
            this._processing = false;
            this.btn_Convert.IsEnabled = true;
            this.btn_RecordPause.IsEnabled = true;
        }

        private void MainWindow_Closing( object sender, CancelEventArgs e ) {
            if (this._recording | this._processing) {
                // Do not allow user to shutdown the program if there is incomplete process
                e.Cancel = true;
            }
            if (this._readerColor != null) {
                // ColorFrameReder is IDisposable
                this._readerColor.Dispose();
                this._readerColor = null;
            }
            if (this._readerDepth != null) {
                // ColorFrameReder is IDisposable
                this._readerDepth.Dispose();
                this._readerDepth = null;
            }

            if (this._sensor != null) {
                this._sensor.Close();
                this._sensor = null;
            }
        }

    }

}
