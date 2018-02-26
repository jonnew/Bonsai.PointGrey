using System;
using OpenCV.Net;
using FlyCapture2Managed;
using System.Threading;
using System.Reactive.Linq;
using System.ComponentModel;
using System.Threading.Tasks;
using System.Drawing.Design;

namespace Bonsai.PointGrey
{
    public class FlyCapture : Source<FlyCaptureDataFrame>
    {
        [Description("The index of the camera from which to acquire images.")]
        public int Index { get; set; }

        [Description("Should image tearing errors be ignored, resulting in dropped frame?")]
        public bool IgnoreImageConsistencyError { get; set; }

        [Description("Enable or disable 3.3 V output on Blackfly GPIO pin 3.")]
        public bool EnableBlackflyOutputVoltage { get; set; }

        [Description("Automatic control of gain and shutter time.")]
        public bool AutoExposure { get; set; } = true;

        [Range(0, 1)]
        [Precision(2, 0.01)]
        [Description("Relative sensor gain.")]
        [Editor(DesignTypes.SliderEditor, typeof(UITypeEditor))]
        public float Gain { get; set; } = 0.5f;

        [Range(0, 1)]
        [Precision(2, 0.01)]
        [Description("Relative shutter open time.")]
        [Editor(DesignTypes.SliderEditor, typeof(UITypeEditor))]
        public float Shutter { get; set; } = 0.5f;

        [Range(0, 1000)]
        [Editor(DesignTypes.SliderEditor, typeof(UITypeEditor))]
        [Description("Frame rate in Hz.")]
        public float FramesPerSecond { get; set; } = 30;

        [Description("Embed frame count information into pixel data.")]
        public bool EnableEmbeddedFrameCounter { get; set; } = false;

        [Description("Embed frame timestamp information into pixel data.")]
        public bool EnableEmbeddedFrameTimeStamp { get; set; } = false;

        [Description("On camera color processing method for converting raw Bayer image.")]
        public ColorProcessingAlgorithm ColorProcessing { get; set; }

        IObservable<FlyCaptureDataFrame> source;
        readonly object captureLock = new object();

        public FlyCapture()
        {
            ColorProcessing = ColorProcessingAlgorithm.Default;
            source = Observable.Create<FlyCaptureDataFrame>((observer, cancellationToken) =>
            {
                return Task.Factory.StartNew(() =>
                {
                    lock (captureLock)
                    {
                        ManagedCamera camera;
                        using (var manager = new ManagedBusManager())
                        {
                            var guid = manager.GetCameraFromIndex((uint)Index);
                            camera = new ManagedCamera();
                            camera.Connect(guid);

                            // Power on the camera
                            const uint CameraPower = 0x610;
                            const uint CameraPowerValue = 0x80000000;
                            camera.WriteRegister(CameraPower, CameraPowerValue);

                            // Wait for camera to complete power-up
                            const Int32 MillisecondsToSleep = 100;
                            uint cameraPowerValueRead = 0;
                            do
                            {
                                System.Threading.Thread.Sleep(MillisecondsToSleep);
                                cameraPowerValueRead = camera.ReadRegister(CameraPower);
                            }
                            while ((cameraPowerValueRead & CameraPowerValue) == 0);

                        }

                        var capture = 0;
                        try
                        {
                            // Set frame rate
                            var prop = new CameraProperty(PropertyType.FrameRate);
                            prop.absControl = true;
                            prop.absValue = FramesPerSecond;
                            prop.autoManualMode = false;
                            prop.onOff = true;
                            camera.SetProperty(prop);

                            // Enable/disable blackfly pull up
                            const uint pullUp = 0x19D0;
                            if (EnableBlackflyOutputVoltage)
                                camera.WriteRegister(pullUp, 0x10000001);
                            else
                                camera.WriteRegister(pullUp, 0x10000000);

                            // Acquisition parameters
                            var colorProcessing = ColorProcessing;
                            var autoExposure = !AutoExposure; // Horrible hack to trigger update inititally
                            var shutter = Shutter;
                            var gain = Gain;

                            // Configure embedded info
                            const uint embeddedInfo = 0x12F8;
                            uint embeddedInfoState = camera.ReadRegister(embeddedInfo);
                            if (EnableEmbeddedFrameCounter)
                                embeddedInfoState |= (uint)1 << 6;
                            else
                                embeddedInfoState &= ~((uint)1 << 6);

                            if (EnableEmbeddedFrameTimeStamp)
                                embeddedInfoState |= (uint)1 << 0;
                            else
                                embeddedInfoState &= ~((uint)1 << 0);

                            camera.WriteRegister(embeddedInfo, embeddedInfoState);

                            using (var image = new ManagedImage())
                            using (var notification = cancellationToken.Register(() =>
                            {
                                Interlocked.Exchange(ref capture, 0);
                                camera.StopCapture();
                            }))
                            {
                                camera.StartCapture();
                                Interlocked.Exchange(ref capture, 1);
                                while (!cancellationToken.IsCancellationRequested)
                                {
                                    IplImage output;
                                    BayerTileFormat bayerTileFormat;

                                    if (autoExposure != AutoExposure && AutoExposure)
                                    {
                                        prop = new CameraProperty(PropertyType.AutoExposure);
                                        prop.autoManualMode = true;
                                        prop.onOff = true;
                                        camera.SetProperty(prop);
                                        autoExposure = AutoExposure;

                                        // Shutter
                                        prop = new CameraProperty(PropertyType.Shutter);
                                        prop.absControl = true;
                                        prop.autoManualMode = true;
                                        prop.onOff = true;
                                        camera.SetProperty(prop);

                                        // Shutter
                                        prop = new CameraProperty(PropertyType.Gain);
                                        prop.absControl = true;
                                        prop.autoManualMode = true;
                                        prop.onOff = true;
                                        camera.SetProperty(prop);

                                        autoExposure = AutoExposure;

                                    }
                                    else if (autoExposure != AutoExposure && !AutoExposure)
                                    {
                                        shutter = -0.1f; // Hack
                                        gain = -0.1f;

                                        autoExposure = AutoExposure;
                                    }

                                    if (shutter != Shutter && !AutoExposure)
                                    {

                                        // Figure out max shutter time given current frame rate
                                        var info = camera.GetPropertyInfo(PropertyType.Shutter);
                                        var delta = info.absMax - info.absMin;

                                        prop = new CameraProperty(PropertyType.Shutter);
                                        prop.absControl = true;
                                        prop.absValue = Shutter * delta + info.absMin;
                                        prop.autoManualMode = false;
                                        prop.onOff = true;
                                        camera.SetProperty(prop);

                                        shutter = Shutter;
                                    }

                                    if (gain != Gain && !AutoExposure)
                                    {

                                        // Figure out max shutter time given current frame rate
                                        var info = camera.GetPropertyInfo(PropertyType.Shutter);
                                        var delta = info.absMax - info.absMin;

                                        prop = new CameraProperty(PropertyType.Gain);
                                        prop.absControl = true;
                                        prop.absValue = Gain * delta + info.absMin; ;
                                        prop.autoManualMode = false;
                                        prop.onOff = true;
                                        camera.SetProperty(prop);

                                        gain = Gain;
                                    }

                                    try { camera.RetrieveBuffer(image); }
                                    catch (FC2Exception)
                                    {
                                        if (capture == 0) break;
                                        else throw;
                                    }

                                    if (image.pixelFormat == PixelFormat.PixelFormatMono8 ||
                                        image.pixelFormat == PixelFormat.PixelFormatMono16 ||
                                        (image.pixelFormat == PixelFormat.PixelFormatRaw8 &&
                                           (image.bayerTileFormat == BayerTileFormat.None ||
                                            colorProcessing == ColorProcessingAlgorithm.NoColorProcessing)))
                                    {
                                        unsafe
                                        {
                                            bayerTileFormat = image.bayerTileFormat;
                                            var depth = image.pixelFormat == PixelFormat.PixelFormatMono16 ? IplDepth.U16 : IplDepth.U8;
                                            var bitmapHeader = new IplImage(new Size((int)image.cols, (int)image.rows), depth, 1, new IntPtr(image.data));
                                            output = new IplImage(bitmapHeader.Size, bitmapHeader.Depth, bitmapHeader.Channels);
                                            CV.Copy(bitmapHeader, output);
                                        }
                                    }
                                    else
                                    {
                                        unsafe
                                        {
                                            bayerTileFormat = BayerTileFormat.None;
                                            output = new IplImage(new Size((int)image.cols, (int)image.rows), IplDepth.U8, 3);
                                            using (var convertedImage = new ManagedImage(
                                                (uint)output.Height,
                                                (uint)output.Width,
                                                (uint)output.WidthStep,
                                                (byte*)output.ImageData.ToPointer(),
                                                (uint)(output.WidthStep * output.Height),
                                                PixelFormat.PixelFormatBgr))
                                            {
                                                convertedImage.colorProcessingAlgorithm = colorProcessing;
                                                image.Convert(PixelFormat.PixelFormatBgr, convertedImage);
                                            }
                                        }
                                    }

                                    observer.OnNext(new FlyCaptureDataFrame(output, image.imageMetadata, bayerTileFormat));
                                }
                            }
                        }
                        finally
                        {

                            // Power off the camera
                            const uint CameraPower = 0x610;
                            const uint CameraPowerValue = 0x00000000;
                            camera.WriteRegister(CameraPower, CameraPowerValue);

                            if (capture != 0) camera.StopCapture();
                            camera.Disconnect();
                            camera.Dispose();
                        }
                    }
                },
                cancellationToken,
                TaskCreationOptions.LongRunning,
                TaskScheduler.Default);
            })
            .PublishReconnectable()
            .RefCount();
        }

        public override IObservable<FlyCaptureDataFrame> Generate()
        {
            return source;
        }
    }
}
