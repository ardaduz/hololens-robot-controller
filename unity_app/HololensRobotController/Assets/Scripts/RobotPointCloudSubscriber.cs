using System;
using System.Collections.Concurrent;
using System.Linq;
using RosSharp.RosBridgeClient;
using System.Threading;
using RosSharp;
using UnityEngine;

public class RobotPointCloudSubscriber
{
    // ROS-Sharp stuff
    private RosConnector rosConnector;
    private float topicPollingPeriod; // in milliseconds

    // Communication with Unity main thread
    private ConcurrentQueue<Vector3[]> internalQueue;

    public RobotPointCloudSubscriber(string subscriberTopic, float topicPollingPeriod, 
        ref RosConnector rosConnector, ref ConcurrentQueue<Vector3[]> internalQueue)
    {
        this.internalQueue = internalQueue;

        this.rosConnector = rosConnector;
        this.rosConnector.RosSocket.Subscribe<RosSharp.RosBridgeClient.Messages.Sensor.PointCloud2>(subscriberTopic, ReceiveMessage, 
            (int)(topicPollingPeriod)); // the rate(in ms in between messages) at which to throttle the topics
    }

    public void ReceiveMessage(RosSharp.RosBridgeClient.Messages.Sensor.PointCloud2 message)
    {
        int oneAxisValueByteSize = 4; // 4(float32)
        int oneVertexByteSize = message.point_step;

        int numberOfVertices = message.data.Length / oneVertexByteSize;

        Vector3[] newPointCloudVertices = new Vector3[numberOfVertices];

        if(message.data != null)
        {
            int offset = 0;
            for (int i = 0; i < numberOfVertices; i++)
            {
                float x = BitConverter.ToSingle(message.data, offset);
                float y = BitConverter.ToSingle(message.data, offset + oneAxisValueByteSize);
                float z = BitConverter.ToSingle(message.data, offset + 2 * oneAxisValueByteSize);

                Vector3 pointInROSCoordinates = new Vector3(x, y, z);
                Vector3 pointInUnityCoordinates = pointInROSCoordinates.Ros2Unity();

                newPointCloudVertices[i] = pointInUnityCoordinates;

                offset = offset + oneVertexByteSize;
            }
            System.Diagnostics.Debug.WriteLine("Enqueued a new point cloud with " + newPointCloudVertices.Length.ToString() + " points");
            internalQueue.Enqueue(newPointCloudVertices);
        }
    }
}
