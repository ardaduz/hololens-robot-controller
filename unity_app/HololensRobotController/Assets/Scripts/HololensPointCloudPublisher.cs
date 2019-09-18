using System.Collections.Generic;
using UnityEngine;
using HoloToolkit.Unity.SpatialMapping;
using System;
using System.Linq;
using HololensRobotController.Utilities;

#if NETFX_CORE
using Windows.System.Threading;
#endif


public class HololensPointCloudPublisher
{
    private RosSharp.RosBridgeClient.RosConnector rosConnector;
    private RosSharp.RosBridgeClient.NonMono.Publisher<RosSharp.RosBridgeClient.Messages.Sensor.PointCloud2> publisher;
    private double nextPublishTime = Config.PublishingStartsAfter;
    private double publishPeriod = 1.0 / Config.PointCloudFPS;
    private int frameIdx = 0;
    private SpatialMappingManager spatialMappingManager;
    private RosSharp.RosBridgeClient.Messages.Sensor.PointField[] pointFields;
    
    public void Init(ref RosSharp.RosBridgeClient.RosConnector rosConnector)
    {
        this.rosConnector = rosConnector;
        publisher = new RosSharp.RosBridgeClient.NonMono.Publisher<RosSharp.RosBridgeClient.Messages.Sensor.PointCloud2>(ref this.rosConnector, "/hololens/" + Config.PointCloud);

        spatialMappingManager = SpatialMappingManager.Instance;
        spatialMappingManager.SurfaceObserver.TimeBetweenUpdates = Config.UnitySpatialMappingObserverTimeBetweenUpdates;
        spatialMappingManager.SurfaceObserver.TrianglesPerCubicMeter = Config.UnitySpatialMappingObserverTrianglesPerCubicMeter;
        spatialMappingManager.StartObserver(); // TODO: Check if offset is necessary, i.e. float startTime = SpatialMappingManager.Instance.StartTime;
        spatialMappingManager.DrawVisualMeshes = Config.UnitySpatialMappingObserverDrawVisualMeshes;

        CreatePointFieldArray();
    }

    public void TryPublishing(TimeSpan currentTime, double elapsedTimeInSeconds)
    {
        if (elapsedTimeInSeconds >= nextPublishTime)
        {
            // make meshes ready to be combined by the thread
            List<MeshFilter> meshFilters = spatialMappingManager.GetMeshFilters();
            if (meshFilters != null && meshFilters.Count != 0)
            {
                nextPublishTime = nextPublishTime + publishPeriod;

                List<Matrix4x4> transforms = new List<Matrix4x4>();
                List<Vector3[]> vertices = new List<Vector3[]>();
                for (int i = 0; i < meshFilters.Count; i++)
                {
                    vertices.Add(meshFilters[i].sharedMesh.vertices);
                    transforms.Add(meshFilters[i].transform.localToWorldMatrix);
                }

#if NETFX_CORE
                ThreadPool.RunAsync((PointCloudSendWork) => { SendPointCloud(currentTime.Add(Timer.GetOffsetUTC()), transforms, vertices); });
#endif
            }
            else
            {
                System.Diagnostics.Debug.WriteLine("COULD NOT FIND ANY MESH");
            }
        }
    }

    private void SendPointCloud(TimeSpan currentTime, List<Matrix4x4> transforms, List<Vector3[]> vertices)
    {
        int oneAxisValueByteSize = 4; // 4(float32)
        int oneVertexByteSize = 3 * oneAxisValueByteSize;

        int numberOfVertices = vertices.Sum(group => group.Length);
        byte[] data = new byte[numberOfVertices * oneVertexByteSize];

        int ngroups = transforms.Count;
        int copiedUntilNow = 0;
        for (int i = 0; i < ngroups; i++)
        {
            Matrix4x4 currentTransfrom = transforms[i];
            Vector3[] currentVertices = vertices[i];

            int subNumberOfVertices = currentVertices.Length;
            for (int j = 0; j < subNumberOfVertices; j++)
            {
                Vector3 vertex = currentVertices[j];
                Vector3 transformedVertex = currentTransfrom.MultiplyPoint3x4(vertex);

                transformedVertex = HololensRobotController.Utilities.CoordinateTransformations.ConvertPositionUnity2ROS(transformedVertex);
                Buffer.BlockCopy(BitConverter.GetBytes(transformedVertex.x), 0, data, copiedUntilNow + j * oneVertexByteSize, oneAxisValueByteSize);
                Buffer.BlockCopy(BitConverter.GetBytes(transformedVertex.y), 0, data, copiedUntilNow + j * oneVertexByteSize + oneAxisValueByteSize, oneAxisValueByteSize);
                Buffer.BlockCopy(BitConverter.GetBytes(transformedVertex.z), 0, data, copiedUntilNow + j * oneVertexByteSize + 2 * oneAxisValueByteSize, oneAxisValueByteSize);
            }

            copiedUntilNow = copiedUntilNow + subNumberOfVertices * oneVertexByteSize;
        }

        // pulish the message
        int[] structuredTime = Timer.GetSecondsNanosecondsStructure(currentTime);
        RosSharp.RosBridgeClient.Messages.Sensor.PointCloud2 message = new RosSharp.RosBridgeClient.Messages.Sensor.PointCloud2();
        message.header.frame_id = Config.HololensWorldFrame;
        message.header.seq = frameIdx++;
        message.header.stamp.secs = structuredTime[0];
        message.header.stamp.nsecs = structuredTime[1];
        message.height = 1;
        message.width = numberOfVertices;
        message.fields = pointFields;
        message.is_bigendian = false;
        message.is_dense = true;
        message.point_step = oneVertexByteSize;
        message.row_step = message.width * message.point_step;
        message.data = data;

        publisher.Publish(message);
    }

    private void CreatePointFieldArray()
    {
        byte datatype = 7;
        RosSharp.RosBridgeClient.Messages.Sensor.PointField pointFieldX = new RosSharp.RosBridgeClient.Messages.Sensor.PointField();
        RosSharp.RosBridgeClient.Messages.Sensor.PointField pointFieldY = new RosSharp.RosBridgeClient.Messages.Sensor.PointField();
        RosSharp.RosBridgeClient.Messages.Sensor.PointField pointFieldZ = new RosSharp.RosBridgeClient.Messages.Sensor.PointField();
        pointFieldX.name = "x";
        pointFieldY.name = "y";
        pointFieldZ.name = "z";
        pointFieldX.offset = 0; // Index or Byte offset?
        pointFieldY.offset = 4;
        pointFieldZ.offset = 8;
        pointFieldX.datatype = datatype; // corresponds to float32
        pointFieldY.datatype = datatype;
        pointFieldZ.datatype = datatype;
        pointFieldX.count = 1;
        pointFieldY.count = 1;
        pointFieldZ.count = 1;

        pointFields = new RosSharp.RosBridgeClient.Messages.Sensor.PointField[] { pointFieldX, pointFieldY, pointFieldZ };
    }

    public void Quit()
    {
        spatialMappingManager.StopObserver();
        spatialMappingManager.enabled = false;
    }
}
