using UnityEngine;
using System;
using HololensRobotController.Utilities;

#if NETFX_CORE
using Windows.System.Threading;
#endif

public class HololensPosePublisher
{
    private RosSharp.RosBridgeClient.RosConnector rosConnector;
    private RosSharp.RosBridgeClient.NonMono.Publisher<RosSharp.RosBridgeClient.Messages.Geometry.PoseStamped> publisher;
    private double nextPublishTime = Config.PublishingStartsAfter;
    private double publishPeriod = 1.0 / Config.HololensPoseFPS;
    private int frameIdx = 0;
        
    public void Init(ref RosSharp.RosBridgeClient.RosConnector rosConnector)
    {
        this.rosConnector = rosConnector;
        publisher = new RosSharp.RosBridgeClient.NonMono.Publisher<RosSharp.RosBridgeClient.Messages.Geometry.PoseStamped>(ref this.rosConnector, "/hololens/" + Config.HololensPose);
    }

    public void TryPublishing(TimeSpan currentTime, double elapsedTimeInSeconds)
    {
        if (elapsedTimeInSeconds >= nextPublishTime)
        {
            nextPublishTime = nextPublishTime + publishPeriod;
            Quaternion currentRotation = Camera.main.transform.rotation;
            Vector3 currentPosition = Camera.main.transform.position;
#if NETFX_CORE
            ThreadPool.RunAsync((PoseSendWork) => { SendPose(currentTime.Add(Timer.GetOffsetUTC()), currentRotation, currentPosition); });
#endif
        }
    }
    
    private void SendPose(TimeSpan currentTime, Quaternion currentRotation, Vector3 currentPosition)
    {
        int[] structuredTime = Timer.GetSecondsNanosecondsStructure(currentTime);
        RosSharp.RosBridgeClient.Messages.Geometry.PoseStamped message = new RosSharp.RosBridgeClient.Messages.Geometry.PoseStamped();

        message.header.frame_id = Config.HololensWorldFrame;
        message.header.seq = frameIdx++;
        message.header.stamp.secs = structuredTime[0];
        message.header.stamp.nsecs = structuredTime[1];

        CoordinateTransformations.ConvertPoseUnity2ROS(ref currentPosition, ref currentRotation);
        message.pose.orientation.x = currentRotation.x;
        message.pose.orientation.y = currentRotation.y;
        message.pose.orientation.z = currentRotation.z;
        message.pose.orientation.w = currentRotation.w;

        message.pose.position.x = currentPosition.x;
        message.pose.position.y = currentPosition.y;
        message.pose.position.z = currentPosition.z;

        publisher.Publish(message);
        System.Diagnostics.Debug.WriteLine("POSE at time: " + ((double)structuredTime[0] + ((double)structuredTime[1]) / 1e9).ToString() +
            " --- (" + currentPosition.x + ", " + currentPosition.y + ", " + currentPosition.z + ")");
    }

    public void Quit()
    {

    }
}
