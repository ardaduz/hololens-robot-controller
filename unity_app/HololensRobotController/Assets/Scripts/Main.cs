using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using HoloToolkit.Unity.SpatialMapping;
using HoloToolkit.Unity.InputModule;
using System;
using System.Collections.Concurrent;
using HololensRobotController.Utilities;
using RosSharp.RosBridgeClient.Messages.Geometry;
using RosSharp.RosBridgeClient.NonMono;
using Cursor = HoloToolkit.Unity.InputModule.Cursor;
using Quaternion = UnityEngine.Quaternion;
using Timer = HololensRobotController.Utilities.Timer;
using Vector3 = UnityEngine.Vector3;
#if NETFX_CORE
using Windows.System.Threading;
#endif

public class Main : MonoBehaviour, IInputClickHandler
{
    // Cursor for input commands - assigned in Unity
    public Cursor cursor;

    // Message box to show messages to the user
    public UserMessageManager userMessageManager;

    // A flag hologram to be placed when user air taps a target location
    public GameObject robotTargetFlag;

#if NETFX_CORE
    // the structure that holds all the sensors that are accessed via Windows libraries
    private HololensRobotController.WindowsSensors.DataSourceGroup dataSourceGroup;
#endif
    // ros# element that provides WebSocket connection and the object itself is acquired in Start() method
    private RosSharp.RosBridgeClient.RosConnector rosConnector;

    // Publishers, update time and update period ensures that messages are not sent more frequent than what is intended in provided Config.
    private HololensPointCloudPublisher hololensPointCloudPublisher;
    private HololensPosePublisher hololensPosePublisher;
    private MarkerPosePublisher markerPosePublisher;

    // Subscriber for point cloud data received from robot ROS
    // Queue that the messages are stored after processing
    // Stop signal to be sent to the callback method
    // GameObject that is used to render the point cloud
    private RobotPointCloudSubscriber robotPointCloudSubscriber;
    private ConcurrentQueue<Vector3[]> internalPointCloudQueue;
    private GameObject robotPointCloudGameObject;
    private TimeSpan lastTimeRobotPointCloudReceived;
    private bool isRobotPointCloudGameObjectActive = false;
    private bool isShowingRobotLostTrackingMessage = false;
    private double thresholdRobotLostTrackingMessage = 30;

    // Robot target pose publisher
    private RosSharp.RosBridgeClient.NonMono.Publisher<RosSharp.RosBridgeClient.Messages.Geometry.PoseStamped> robotTargetPosePublisher;
    private int robotTargetPoseFrameIdx = 0;


    // Start is called before the first frame update
    async void Start()
    {
        Timer.Init();

        // set period for fixedUpdate() to be called where point cloud and pose can be shared
        Time.fixedDeltaTime = (float)0.033333;

        rosConnector = (RosSharp.RosBridgeClient.RosConnector)GetComponent(typeof(RosSharp.RosBridgeClient.RosConnector));

        hololensPosePublisher = new HololensPosePublisher();
        hololensPosePublisher.Init(ref rosConnector);

        hololensPointCloudPublisher = new HololensPointCloudPublisher();
        hololensPointCloudPublisher.Init(ref rosConnector);

        markerPosePublisher = new MarkerPosePublisher();
        markerPosePublisher.Init(ref rosConnector);

        internalPointCloudQueue = new ConcurrentQueue<Vector3[]>();
        robotPointCloudSubscriber = new RobotPointCloudSubscriber("/trimbot/alignedmap", 0, ref rosConnector, ref internalPointCloudQueue);
        robotPointCloudGameObject = GameObject.Find("RobotPointCloud");
        lastTimeRobotPointCloudReceived = Timer.SampleCurrentStopwatch();

        robotTargetPosePublisher =new Publisher<PoseStamped>(ref rosConnector, "/trimbot/goal");

        InputManager.Instance.PushFallbackInputHandler(this.gameObject);

#if NETFX_CORE
        // initialize the camera data sources via Windows libraries
        dataSourceGroup = new HololensRobotController.WindowsSensors.DataSourceGroup(ref rosConnector);
        await dataSourceGroup.GetDataSources();
#endif
    }

    void FixedUpdate()
    {
        TimeSpan sampledCurrentTime = Timer.SampleCurrentStopwatch();
        double elapsedTimeInSeconds = Timer.GetElapsedTimeInSeconds(sampledCurrentTime);

        hololensPosePublisher.TryPublishing(sampledCurrentTime, elapsedTimeInSeconds);
        hololensPointCloudPublisher.TryPublishing(sampledCurrentTime, elapsedTimeInSeconds);
        markerPosePublisher.TryPublishing(sampledCurrentTime, elapsedTimeInSeconds);
    }

    // Update is called once per frame and is used to display the point cloud sent by the robot
    void Update()
    {
        // Update the point cloud mesh to display the most recent aligned point cloud sent by the robot side
        Vector3[] newPointCloudVertices;
        bool isAvailable = internalPointCloudQueue.TryDequeue(out newPointCloudVertices);

        TimeSpan currentTime = Timer.SampleCurrentStopwatch();
        TimeSpan elapsedRobotPointCloudReceived = currentTime - lastTimeRobotPointCloudReceived;
        if (isAvailable)
        {
            lastTimeRobotPointCloudReceived = currentTime;
            UpdatePointCloudMesh(ref newPointCloudVertices);
            isShowingRobotLostTrackingMessage = false;
        }
        else if (elapsedRobotPointCloudReceived.TotalSeconds > thresholdRobotLostTrackingMessage)
        {
            thresholdRobotLostTrackingMessage = 15;
            isRobotPointCloudGameObjectActive = false;
            robotTargetFlag.SetActive(false);
            robotPointCloudGameObject.SetActive(isRobotPointCloudGameObjectActive);
            if (!isShowingRobotLostTrackingMessage)
            {
                isShowingRobotLostTrackingMessage = true;
                string userMessage = "You are not receiving any new point clouds from the robot. Please try to move robot around using the virtual joystick!";
                userMessageManager.ShowUserMessage(userMessage, 999f);
            }
            
        }
    }

    public void OnInputClicked(InputClickedEventData eventData)
    {

        if (eventData.PressType == InteractionSourcePressInfo.Select && isRobotPointCloudGameObjectActive)
        {
            if (eventData.selectedObject != null)
            {
                if (eventData.selectedObject.layer == 31)
                {
                    Vector3 targetPosition = cursor.transform.position;

                    robotTargetFlag.transform.position = targetPosition;
                    robotTargetFlag.SetActive(true);

                    TimeSpan currentTime = Timer.SampleCurrentStopwatch();
#if NETFX_CORE
                    ThreadPool.RunAsync((PoseSendWork) => { SendRobotTargetPose(currentTime.Add(Timer.GetOffsetUTC()), Quaternion.identity, targetPosition); });
#endif

                    System.Diagnostics.Debug.WriteLine("Air tapped robot target position: " + targetPosition.ToString());
                }
            }
            else
            {
                string userMessage = "You air tapped on an unknown space. Try again!";
                userMessageManager.ShowUserMessage(userMessage, 2f);
                System.Diagnostics.Debug.WriteLine("You air tapped on an unknown space. Try again!");
            }
        }

    }

    void OnDestroy()
    {
        QuitGracefully();
        Debug.Log("Main.OnDestroy called");
    }

    void OnApplicationQuit()
    {
        QuitGracefully();
        Debug.Log("Application Quitted");
    }

    void OnApplicationPause(bool pauseStatus)
    {
        if (pauseStatus)
        {
            Debug.Log("Application Paused");
        }
    }

    void QuitGracefully()
    {
        hololensPosePublisher.Quit();
        hololensPointCloudPublisher.Quit();
        markerPosePublisher.Quit();

        SpatialMappingManager.Instance.StopObserver();

#if NETFX_CORE
        foreach (HololensRobotController.WindowsSensors.CameraSource cameraSource in dataSourceGroup.DataSources)
        {
            cameraSource.StopReader();
        }
#endif
    }

    void UpdatePointCloudMesh(ref Vector3[] newPointCloudVertices)
    {
        robotPointCloudGameObject.SetActive(isRobotPointCloudGameObjectActive);
        Mesh mesh = robotPointCloudGameObject.GetComponent<MeshFilter>().mesh;

        System.Diagnostics.Debug.WriteLine("Entered UpdatePointCloudMesh for " + newPointCloudVertices.Length.ToString() + " points");

        int[] indices = new int[newPointCloudVertices.Length];
        Color[] colors = new Color[newPointCloudVertices.Length];
        for (int i = 0; i < newPointCloudVertices.Length; i++)
        {
            indices[i] = i;
            colors[i] = new Color(0.0f, 1.0f, 0, 1.0f);
        }

        mesh.vertices = newPointCloudVertices;
        mesh.colors = colors;
        mesh.SetIndices(indices, MeshTopology.Points, 0);
        isRobotPointCloudGameObjectActive = true;
        robotPointCloudGameObject.SetActive(isRobotPointCloudGameObjectActive);
    }


    private void SendRobotTargetPose(TimeSpan currentTime, Quaternion currentRotation, Vector3 currentPosition)
    {
        int[] structuredTime = Timer.GetSecondsNanosecondsStructure(currentTime);
        PoseStamped message = new PoseStamped();

        message.header.frame_id = Config.HololensWorldFrame;
        message.header.seq = robotTargetPoseFrameIdx++;
        message.header.stamp.secs = structuredTime[0];
        message.header.stamp.nsecs = structuredTime[1];

        CoordinateTransformations.ConvertPoseUnity2ROS(ref currentPosition, ref currentRotation);
        message.pose.orientation.x = Quaternion.identity.x;
        message.pose.orientation.y = Quaternion.identity.y;
        message.pose.orientation.z = Quaternion.identity.z;
        message.pose.orientation.w = Quaternion.identity.w;

        message.pose.position.x = currentPosition.x;
        message.pose.position.y = currentPosition.y;
        message.pose.position.z = currentPosition.z;

        robotTargetPosePublisher.Publish(message);
        System.Diagnostics.Debug.WriteLine("ROBOT TARGET POSE at time: " + ((double)structuredTime[0] + ((double)structuredTime[1]) / 1e9).ToString() +
                                           " --- (" + currentPosition.x + ", " + currentPosition.y + ", " + currentPosition.z + ")");
    }
}
