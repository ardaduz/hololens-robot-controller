using System.Collections.Generic;

public class Config
{
    // CHANGABLE SETTINGS
    public static string RosBridgeServerUrl = "ws://10.42.0.1:9090";

    public static bool convertColorToGrayscale = true;

    public static readonly float UnitySpatialMappingObserverTimeBetweenUpdates = (float) 3.5; // seconds
    public static readonly float UnitySpatialMappingObserverTrianglesPerCubicMeter = 50;
    public static readonly bool UnitySpatialMappingObserverDrawVisualMeshes = false;

    public static readonly string HololensWorldFrame = "hololens_world";
    public static readonly string MarkerPose = "marker_pose";
    public static readonly string HololensPose = "pose";
    public static readonly string PointCloud = "point_cloud";
    public static readonly string DepthNear = "depth_near";
    public static readonly string DepthFar = "depth_far";
    public static readonly string InfraredNarrow = "infrared_narrow";
    public static readonly string InfraredWide = "infrared_wide";
    public static readonly string StereoLeftLeft = "stereo_left_left";
    public static readonly string StereoLeft = "stereo_left";
    public static readonly string StereoRight = "stereo_right";
    public static readonly string StereoRightRight = "stereo_right_right";
    public static readonly string Mono = "mono";

    public static readonly double PublishingStartsAfter = 5; // seconds
    public static readonly double MonoFPS = 1; // max 15 or 30 depending on ARUWP choice
    public static readonly double StereoFPS = 5; // max 30
    public static readonly double DepthFPS = 15; // max 15
    public static readonly double InfraredFPS = 3; // max 3
    public static readonly double HololensPoseFPS = 15; // limits are unknown, but I would not push too much
    public static readonly double MarkerPoseFPS = 10; // 
    public static readonly double PointCloudFPS = 0.2; // between 0 and 2 is optimal

    public static readonly Dictionary<string, bool> SourceSelectionDictionary = new Dictionary<string, bool>()
    {
        { DepthNear, false},
        { InfraredNarrow, false},
        { DepthFar, false},
        { InfraredWide, false},
        { StereoLeftLeft, false},
        { StereoLeft, false},
        { StereoRight, false},
        { StereoRightRight, false},
        { Mono, false}
        
    };


    // DO NOT CHANGE THE FOLLOWING SETTINGS UNLESS YOU KNOW WHAT YOU ARE DOING
    public static readonly Dictionary<string, string> DataSourceDictionary = new Dictionary<string, string>()
    {
        { "Source#0", DepthNear},
        { "Source#1", InfraredNarrow},
        { "Source#2", DepthFar},
        { "Source#3", InfraredWide},
        { "Source#4", StereoLeftLeft},
        { "Source#5", StereoLeft},
        { "Source#6", StereoRight},
        { "Source#7", StereoRightRight}
    };

    public static readonly Dictionary<string, double> FrameRateDictionary = new Dictionary<string, double>()
    {
        { DepthNear, DepthFPS},
        { InfraredNarrow, InfraredFPS},
        { DepthFar, DepthFPS},
        { InfraredWide, InfraredFPS},
        { StereoLeftLeft, StereoFPS},
        { StereoLeft, StereoFPS},
        { StereoRight, StereoFPS},
        { StereoRightRight, StereoFPS},
        { Mono, MonoFPS}
    };

}
