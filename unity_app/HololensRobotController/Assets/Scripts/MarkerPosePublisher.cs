using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using HoloToolkit.Unity.SpatialMapping;
using HoloToolkit.Unity.InputModule;
using System;
using System.Linq;
using HololensRobotController.Utilities;

#if NETFX_CORE
using Windows.System.Threading;
#endif

public class MarkerPosePublisher
{
    private RosSharp.RosBridgeClient.RosConnector rosConnector;
    private RosSharp.RosBridgeClient.NonMono.Publisher<RosSharp.RosBridgeClient.Messages.Geometry.PoseStamped> publisher;
    private double nextPublishTime = Config.PublishingStartsAfter;
    private double publishPeriod = 1.0 / Config.MarkerPoseFPS;
    private int frameIdx = 0;
    private ARUWPMarker marker = null;
    
    public void Init(ref RosSharp.RosBridgeClient.RosConnector rosConnector)
    {
        this.rosConnector = rosConnector;
        publisher = new RosSharp.RosBridgeClient.NonMono.Publisher<RosSharp.RosBridgeClient.Messages.Geometry.PoseStamped>(ref this.rosConnector, "/hololens/" + Config.MarkerPose);

        GameObject aruwpController = GameObject.Find("ARUWPController");
        if (aruwpController == null)
        {
            Debug.Log("MarkerPosePublisher: GameObject ARUWPController could not be found!");
            Application.Quit();
        } else
        {
            marker = aruwpController.GetComponent<ARUWPMarker>();
        }

        if (marker == null)
        {
            Debug.Log("MarkerPosePublisher: GameObject ARUWPController has no ARUWPMarker component!");
            Application.Quit();
        }
    
    }

    public void TryPublishing(TimeSpan currentTime, double elapsedTimeInSeconds)
    {
        if (elapsedTimeInSeconds >= nextPublishTime && marker != null)
        {
            nextPublishTime = nextPublishTime + publishPeriod;
            
            if (marker.GetMarkerVisibility())
            {
                Matrix4x4 latestPoseMatrix = marker.GetMarkerPoseInWorldCoordinateFrame();
                Quaternion currentRotation = ARUWPUtils.QuaternionFromMatrix(latestPoseMatrix);
                Vector3 currentPosition = ARUWPUtils.PositionFromMatrix(latestPoseMatrix);

#if NETFX_CORE
                ThreadPool.RunAsync((MarkerPoseSendWork) => { SendPose(currentTime.Add(Timer.GetOffsetUTC()), currentRotation, currentPosition); });
#endif
            } else
            {
                Debug.Log("Marker not detected.");
            }
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
        System.Diagnostics.Debug.WriteLine("Marker pose at time: " + ((double)structuredTime[0] + ((double)structuredTime[1]) / 1e9).ToString() +
            " --- (" + currentPosition.x + ", " + currentPosition.y + ", " + currentPosition.z + ")");
    }

    public void Quit()
    {

    }
}
