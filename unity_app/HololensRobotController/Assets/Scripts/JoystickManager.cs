using System.Collections;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Messages.Geometry;
using UnityEngine;

public class JoystickManager : MonoBehaviour
{
    public Canvas joystickCanvas;
    public RosConnector rosConnector;

    private RosSharp.RosBridgeClient.NonMono.Publisher<RosSharp.RosBridgeClient.Messages.Geometry.Twist> cmdPublisher;
    private RosSharp.RosBridgeClient.Messages.Geometry.Vector3 forwardLinear;
    private RosSharp.RosBridgeClient.Messages.Geometry.Vector3 backwardLinear;
    private RosSharp.RosBridgeClient.Messages.Geometry.Vector3 stopLinear;
    private RosSharp.RosBridgeClient.Messages.Geometry.Vector3 counterClockwiseAngular;
    private RosSharp.RosBridgeClient.Messages.Geometry.Vector3 clockwiseAngular;
    private RosSharp.RosBridgeClient.Messages.Geometry.Vector3 stopAngular;

    float messageDelay = 0.1f;
    int backwardForwardLoop = 20;
    int othersLoop = 10;

    void Start()
    {
        joystickCanvas.enabled = false;
        cmdPublisher = new RosSharp.RosBridgeClient.NonMono.Publisher<Twist>(ref rosConnector, "/cmd_vel");

        forwardLinear =           new RosSharp.RosBridgeClient.Messages.Geometry.Vector3 {x =  0.4f, y = 0.0f, z =  0.0f};
        backwardLinear =          new RosSharp.RosBridgeClient.Messages.Geometry.Vector3 {x = -0.4f, y = 0.0f, z =  0.0f};
        stopLinear =              new RosSharp.RosBridgeClient.Messages.Geometry.Vector3 {x =  0.0f, y = 0.0f, z =  0.0f};
        counterClockwiseAngular = new RosSharp.RosBridgeClient.Messages.Geometry.Vector3 {x =  0.0f, y = 0.0f, z =  0.5f};
        clockwiseAngular =        new RosSharp.RosBridgeClient.Messages.Geometry.Vector3 {x =  0.0f, y = 0.0f, z = -0.5f};
        stopAngular =             new RosSharp.RosBridgeClient.Messages.Geometry.Vector3 {x =  0.0f, y = 0.0f, z =  0.0f};
    }

    // Update is called once per frame even if it is empty place it here
    void Update()
    {
    }

    public void OnUpClick()
    {
        StartCoroutine(Forward());
    }

    public void OnDownClick()
    {
        StartCoroutine(Backward());
    }

    public void OnLeftClick()
    {
        StartCoroutine(Left());
    }

    public void OnRightClick()
    {
        StartCoroutine(Right());
    }

    public void OnUpLeftClick()
    {
        StartCoroutine(ForwardLeft());
    }

    public void OnDownLeftClick()
    {
        StartCoroutine(BackwardLeft());
    }

    public void OnDownRightClick()
    {
        StartCoroutine(BackwardRight());
    }

    public void OnUpRightClick()
    {
        StartCoroutine(ForwardRight());
    }

    public void OnToggleClick()
    {
        joystickCanvas.enabled = !joystickCanvas.enabled;
    }

    private void SendStopMessage()
    {
        Twist message = new Twist();
        message.linear = stopLinear;
        message.angular = stopAngular;
        cmdPublisher.Publish(message);
    }

    private IEnumerator Forward()
    {
        for (int i = 0; i < backwardForwardLoop; i++)
        {
            Twist message = new Twist();
            message.linear = forwardLinear;
            message.angular = stopAngular;
            cmdPublisher.Publish(message);
            yield return new WaitForSecondsRealtime(messageDelay);
        }
        SendStopMessage();
    }

    private IEnumerator Backward()
    {
        for (int i = 0; i < backwardForwardLoop; i++)
        {
            Twist message = new Twist();
            message.linear = backwardLinear;
            message.angular = stopAngular;
            cmdPublisher.Publish(message);
            yield return new WaitForSecondsRealtime(messageDelay);
        }
        SendStopMessage();
    }

    private IEnumerator Left()
    {
        for (int i = 0; i < othersLoop; i++)
        {
            Twist message = new Twist();
            message.linear = stopLinear;
            message.angular = counterClockwiseAngular;
            cmdPublisher.Publish(message);
            yield return new WaitForSecondsRealtime(messageDelay);
        }
        SendStopMessage();
    }

    private IEnumerator Right()
    {
        for (int i = 0; i < othersLoop; i++)
        {
            Twist message = new Twist();
            message.linear = stopLinear;
            message.angular = clockwiseAngular;
            cmdPublisher.Publish(message);
            yield return new WaitForSecondsRealtime(messageDelay);
        }
    }

    private IEnumerator ForwardLeft()
    {
        for (int i = 0; i < othersLoop; i++)
        {
            Twist message = new Twist();
            message.linear = forwardLinear;
            message.angular = counterClockwiseAngular;
            cmdPublisher.Publish(message);
            yield return new WaitForSecondsRealtime(messageDelay);
        }
        SendStopMessage();
    }

    private IEnumerator ForwardRight()
    {
        for (int i = 0; i < othersLoop; i++)
        {
            Twist message = new Twist();
            message.linear = forwardLinear;
            message.angular = clockwiseAngular;
            cmdPublisher.Publish(message);
            yield return new WaitForSecondsRealtime(messageDelay);
        }
        SendStopMessage();
    }

    private IEnumerator BackwardLeft()
    {
        for (int i = 0; i < othersLoop; i++)
        {
            Twist message = new Twist();
            message.linear = backwardLinear;
            message.angular = clockwiseAngular;
            cmdPublisher.Publish(message);
            yield return new WaitForSecondsRealtime(messageDelay);
        }
        SendStopMessage();
    }

    private IEnumerator BackwardRight()
    {
        for (int i = 0; i < othersLoop; i++)
        {
            Twist message = new Twist();
            message.linear = backwardLinear;
            message.angular = counterClockwiseAngular;
            cmdPublisher.Publish(message);
            yield return new WaitForSecondsRealtime(messageDelay);
        }
        SendStopMessage();
    }

}
