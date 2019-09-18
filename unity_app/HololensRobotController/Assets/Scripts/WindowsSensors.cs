#if NETFX_CORE
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading.Tasks;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.Media.Capture;
using Windows.Media.Capture.Frames;
using Windows.Storage.Streams;
using RosSharp.RosBridgeClient;
using Windows.Graphics.Imaging;
#endif

namespace HololensRobotController.WindowsSensors
{
#if NETFX_CORE
    public abstract class DataSource : IEquatable<DataSource>
    {
        public string SourceName { get; set; }

        // IEquatable overrides
        public override int GetHashCode()
        {
            return SourceName.GetHashCode();
        }
        public override bool Equals(object obj)
        {
            if (obj == null && (obj as DataSource) == null)
            {
                return false;
            }
            else
            {
                return (this.SourceName.Equals((obj as DataSource).SourceName));
            }
        }
        public bool Equals(DataSource other)
        {
            if (other == null) return false;
            return (this.SourceName.Equals(other.SourceName));
        }
    }

    public class DataSourceGroup
    {
        public List<DataSource> DataSources { get; set; }
        private RosConnector RosConnector;

        public DataSourceGroup(ref RosConnector rosConnector)
        {
            RosConnector = rosConnector;
            DataSources = new List<DataSource>();
        }

        public async Task GetDataSources()
        {
            // If pose is supported and selected add pose source
            //var ver = Windows.System.Profile.AnalyticsInfo.VersionInfo.DeviceFamily;
            //if (ver == "Windows.Holographic")
            //{
            //    bool isSelected;
            //    Config.SourceSelectionDictionary.TryGetValue(Config.Pose, out isSelected);
            //    if (isSelected)
            //    {
            //        DataSources.Add(new PoseSource(ref RosConnector, ref SharedTimer)
            //        {
            //            SourceName = Config.Pose,
            //            PublishPeriod = 1 / Config.HololensPoseFPS
            //        });
            //    }
            //}

            // Check for any available cameras
            var possibleSourceKinds = new MediaFrameSourceKind[] { MediaFrameSourceKind.Depth, MediaFrameSourceKind.Infrared, MediaFrameSourceKind.Color };
            var groups = await MediaFrameSourceGroup.FindAllAsync();
            // Find the group that exposes all of the sensors for streaming
            foreach (var g in groups)
            {
                if (g.DisplayName == "Sensor Streaming")
                {
                    Debug.WriteLine("Found Sensor Streaming Source Group");
                    var mediaCapture = new MediaCapture();
                    await mediaCapture.InitializeAsync(
                        new MediaCaptureInitializationSettings()
                        {
                            SourceGroup = g,
                            MemoryPreference = MediaCaptureMemoryPreference.Cpu,
                            StreamingCaptureMode = StreamingCaptureMode.Video
                        }
                    );
                    var sources = mediaCapture.FrameSources.Where(fs => possibleSourceKinds.Contains(fs.Value.Info.SourceKind)).Select(fs => fs.Value);

                    foreach (var source in sources)
                    {
                        string originalSourceName = source.Info.Id.Substring(source.Info.Id.IndexOf("Source#"), 8);
                        string assignedSourceName;
                        Config.DataSourceDictionary.TryGetValue(originalSourceName, out assignedSourceName);

                        bool isSelected;
                        Config.SourceSelectionDictionary.TryGetValue(assignedSourceName, out isSelected);
                        if (isSelected)
                        {
                            double assignedFrameRate;
                            Config.FrameRateDictionary.TryGetValue(assignedSourceName, out assignedFrameRate);
                            double assignedPublishPeriod = 1.0 / (double)assignedFrameRate;
                            int originalFPS = (int)source.Info.VideoProfileMediaDescription[0].FrameRate;

                            CameraHandler handler = new CameraHandler(source.Info, mediaCapture, assignedPublishPeriod);
                            await handler.SetupReaderAsync();
                            DataSources.Add(new CameraSource(ref RosConnector, handler, assignedSourceName, assignedPublishPeriod)
                            {
                                Resolution = $"{ source.Info.VideoProfileMediaDescription[0].Width } x { source.Info.VideoProfileMediaDescription[0].Height }",
                                OriginalFPS = originalFPS,
                                SourceName = assignedSourceName
                            });
                        }
                    }
                    break;
                }
            }
        }
    }

    // CAMERA
    public class CameraHandler
    {
        private MediaFrameSourceInfo SourceInfo_;
        private MediaCapture MediaCapture_;
        private MediaFrameReader FrameReader_;
        private double PublishPeriod;
        private double NextPublishTime = Config.PublishingStartsAfter;
        public byte[] buffer;
        public uint Height;
        public uint Width;
        public uint RowSize;

        public event EventHandler<FrameEventArgs> FrameReadyToPublish;

        public CameraHandler(MediaFrameSourceInfo sourceInfo, MediaCapture mediaCapture, double publishPeriod)
        {
            PublishPeriod = publishPeriod;
            SourceInfo_ = sourceInfo;
            MediaCapture_ = mediaCapture;
            Height = SourceInfo_.VideoProfileMediaDescription[0].Height;
            Width = SourceInfo_.VideoProfileMediaDescription[0].Width;
            AllocateBuffer();
        }

        ~CameraHandler()
        {
            FrameReader_.FrameArrived -= FrameArrivedCallback;
            FrameReader_.Dispose();
        }

        public async Task SetupReaderAsync()
        {
            FrameReader_ = await MediaCapture_.CreateFrameReaderAsync(MediaCapture_.FrameSources[SourceInfo_.Id]);
            FrameReader_.AcquisitionMode = MediaFrameReaderAcquisitionMode.Realtime;
            FrameReader_.FrameArrived += FrameArrivedCallback;
        }

        public async Task StartReaderAsync()
        {
            await FrameReader_.StartAsync();
        }

        public async Task StopReaderAsync()
        {
            await FrameReader_.StopAsync();
        }

        private void AllocateBuffer()
        {
            var bytesPerPixel = GetBytesPerPixel(SourceInfo_.SourceKind);
            Int32 pixelBufferSize = (Int32)(Height * Width * bytesPerPixel);
            RowSize = Width * bytesPerPixel;
            buffer = new byte[pixelBufferSize];
        }

        private void FrameArrivedCallback(MediaFrameReader sender, MediaFrameArrivedEventArgs args)
        {
            TimeSpan sampledCurrentTime = Utilities.Timer.SampleCurrentStopwatch();
            double elapsedTotalSeconds = Utilities.Timer.GetElapsedTimeInSeconds(sampledCurrentTime);
            if (elapsedTotalSeconds >= NextPublishTime)
            {
                var frame = sender.TryAcquireLatestFrame();
                if (frame != null)
                {
                    IBuffer ibuffer = buffer.AsBuffer();
                    SoftwareBitmap originalSoftwareBitmap = frame.VideoMediaFrame.SoftwareBitmap;

                    SoftwareBitmap currentBitmap;
                    if (Config.convertColorToGrayscale)
                    {
                        currentBitmap = SoftwareBitmap.Convert(originalSoftwareBitmap, BitmapPixelFormat.Gray8);
                        originalSoftwareBitmap?.Dispose();
                    }
                    else
                    {
                        currentBitmap = originalSoftwareBitmap;
                    }

                    currentBitmap.CopyToBuffer(ibuffer);
                    OnFrameReadyToPublish(
                        new FrameEventArgs(Height,
                                           Width,
                                           GetCameraSourceEncoding(frame.SourceKind),
                                           RowSize,
                                           buffer,
                                           sampledCurrentTime + Utilities.Timer.GetOffsetUTC()));

                    NextPublishTime = NextPublishTime + PublishPeriod;
                }

            }
        }

        protected virtual void OnFrameReadyToPublish(FrameEventArgs e)
        {
            FrameReadyToPublish?.Invoke(this, e);
        }

        public static string GetCameraSourceEncoding(MediaFrameSourceKind kind)
        {
            string format = "bgra8";
            switch (kind)
            {
                case MediaFrameSourceKind.Depth:
                    format = "mono16";
                    break;
                case MediaFrameSourceKind.Infrared:
                    format = "mono8";
                    break;
                case MediaFrameSourceKind.Color:
                    if (Config.convertColorToGrayscale)
                        format = "mono8";
                    else
                        format = "bgra8";
                    break;
                default:
                    Debug.Assert(false);
                    break;
            }
            return format;
        }

        public static uint GetBytesPerPixel(MediaFrameSourceKind kind)
        {
            uint bytesPerPixel = 0;
            switch (kind)
            {
                case MediaFrameSourceKind.Depth:
                    bytesPerPixel = 2;
                    break;
                case MediaFrameSourceKind.Infrared:
                    bytesPerPixel = 1;
                    break;
                case MediaFrameSourceKind.Color:
                    if (Config.convertColorToGrayscale)
                        bytesPerPixel = 1;
                    else
                        bytesPerPixel = 4;
                    break;
                default:
                    Debug.Assert(false);
                    break;
            }
            return bytesPerPixel;
        }
    }

    public class CameraSource : DataSource
    {
        private CameraHandler CameraHandler_;
        private int FrameNo = 0;
        private RosSharp.RosBridgeClient.NonMono.Publisher<RosSharp.RosBridgeClient.Messages.Sensor.Image> publisher;

        public CameraSource(ref RosConnector rosConnector, CameraHandler handler, string name, double publishPeriod) : base()
        {
            string topic = "/hololens/" + name;
            publisher = new RosSharp.RosBridgeClient.NonMono.Publisher<RosSharp.RosBridgeClient.Messages.Sensor.Image>(ref rosConnector, topic);
            CameraHandler_ = handler;
            CameraHandler_.FrameReadyToPublish += PublishFrame;
            Debug.WriteLine("Publishing a new topic: " + topic);

            StartReader();
        }

        public async void StartReader()
        {
            await CameraHandler_.StartReaderAsync();
        }

        public async void StopReader()
        {
            await CameraHandler_.StopReaderAsync();
        }

        public string Resolution { get; set; }
        public double OriginalFPS { get; set; }

        private void PublishFrame(object sender, FrameEventArgs e)
        {
            int[] structuredTime = Utilities.Timer.GetSecondsNanosecondsStructure(e.TimeSpan);
            RosSharp.RosBridgeClient.Messages.Sensor.Image image = new RosSharp.RosBridgeClient.Messages.Sensor.Image();
            image.header.frame_id = SourceName;
            image.header.stamp.secs = structuredTime[0];
            image.header.stamp.nsecs = structuredTime[1];
            image.header.seq = FrameNo++;
            image.data = e.Buffer;
            image.encoding = e.Encoding;
            image.is_bigendian = 0; // TODO: try true
            image.height = (int)e.Height;
            image.width = (int)e.Width;
            image.step = (int)e.Step;

            publisher.Publish(image);
            Debug.WriteLine(SourceName + "sent frame-" + image.header.seq + " at time " + ((double)structuredTime[0] + ((double)structuredTime[1]) / 1e9).ToString());
        }
    }

    public class FrameEventArgs : EventArgs
    {
        public FrameEventArgs(uint h, uint w, string enc, uint rowsize, byte[] buff, TimeSpan time)
        {
            height = h;
            width = w;
            encoding = enc;
            step = rowsize;
            buffer = buff;
            timespan = time;
        }

        private uint height;
        public uint Height { get { return height; } }
        private uint width;
        public uint Width { get { return width; } }
        private string encoding;
        public string Encoding { get { return encoding; } }
        private uint step;
        public uint Step { get { return step; } }
        private byte[] buffer;
        public byte[] Buffer { get { return buffer; } }
        private TimeSpan timespan;
        public TimeSpan TimeSpan { get { return timespan; } }

    }

    //// POSE
    //public class PoseHandler
    //{
    //    private SpatialLocator SpatialLocator_;
    //    private SpatialStationaryFrameOfReference FrameOfReference_;

    //    public PoseHandler()
    //    {
    //        SpatialLocator_ = SpatialLocator.GetDefault();
    //        FrameOfReference_ = SpatialLocator_.CreateStationaryFrameOfReferenceAtCurrentLocation();
    //    }

    //    public SpatialLocation GetCurrentPose()
    //    {
    //        if (SpatialLocator_.Locatability == SpatialLocatability.PositionalTrackingActive)
    //        {
    //            // TODO: Figure out why FromSystemRelativeTargetTime doesn't work
    //            //PerceptionTimestamp timestamp = PerceptionTimestampHelper.FromSystemRelativeTargetTime(new TimeSpan());
    //            PerceptionTimestamp timestamp = PerceptionTimestampHelper.FromHistoricalTargetTime(new DateTimeOffset(DateTime.Now));
    //            SpatialLocation currentPose = SpatialLocator_.TryLocateAtTimestamp(timestamp, FrameOfReference_.CoordinateSystem);
    //            return currentPose;
    //        }
    //        else
    //            return null;
    //    }
    //}

    //public class PoseSource : DataSource
    //{
    //    private ThreadPoolTimer PollTimer_;
    //    private PoseHandler PoseHandler_;
    //    private RosSharp.RosBridgeClient.NonMono.Publisher<RosSharp.RosBridgeClient.Messages.Geometry.PoseStamped> publisher;

    //    public PoseSource(ref RosConnector rosConnector, ref Utilities.Timer sharedTimer)
    //        : base(ref sharedTimer)
    //    {
    //        string topic = "hololens/" + Config.Pose;
    //        publisher = new RosSharp.RosBridgeClient.NonMono.Publisher<RosSharp.RosBridgeClient.Messages.Geometry.PoseStamped>(ref rosConnector, topic);
    //        Debug.WriteLine("Publishing a new topic: " + SourceName);
    //        PoseHandler_ = new PoseHandler();
    //        PollTimer_ = ThreadPoolTimer.CreatePeriodicTimer(Poll, TimeSpan.FromSeconds(PublishPeriod));
    //    }

    //    ~PoseSource()
    //    {
    //        if (PollTimer_ != null)
    //        {
    //            PollTimer_.Cancel();
    //        }
    //    }

    //    public void Poll(ThreadPoolTimer t)
    //    {
    //        SpatialLocation currentLocation = PoseHandler_.GetCurrentPose();
    //        if (currentLocation != null)
    //        {
    //            TimeSpan sampledCurrentTime = SharedTimer.SampleCurrentTime();
    //            double elapsedTotalSeconds = Utilities.Timer.GetElapsedTimeInSeconds(sampledCurrentTime);
    //            if (elapsedTotalSeconds >= NextPublishTime)
    //            {
    //                int[] structuredTime = Utilities.Timer.GetSecondsNanosecondsStructure(sampledCurrentTime);
    //                Debug.WriteLine("CurrentLocation: (" + currentLocation.Position.X + ", " + currentLocation.Position.Y + ", " + currentLocation.Position.Z + ")");

    //                //TODO: Convert pose info to Message.Geometry.Pose and publish
    //                //publisher.Publish()
    //                NextPublishTime = NextPublishTime + PublishPeriod;
    //            }
    //        }

    //    }
    //}
#endif
}