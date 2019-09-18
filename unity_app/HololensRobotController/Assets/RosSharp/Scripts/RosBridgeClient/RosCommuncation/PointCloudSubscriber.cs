/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]
    public abstract class Subscriber<T> : MonoBehaviour where T: Message
    {
        public string Topic;
        public float TimeStep;

        protected virtual void Start()
        {
            RosConnector rosConnector = GetComponent<RosConnector>();
            rosConnector.RosSocket.Subscribe<T>(Topic, ReceiveMessage, (int)(TimeStep * 1000)); // the rate(in ms in between messages) at which to throttle the topics
        }

        protected abstract void ReceiveMessage(T message);

    }
}


namespace RosSharp.RosBridgeClient.NonMono
{
    public class PointCloudSubscriber<T> where T : Message
    {
        private RosSocket rosSocket;
        private string Topic;
        private float timeStep; // in milliseconds

        public PointCloudSubscriber(ref RosConnector rosConnector, string topic, float timeStep)
        {
            Topic = topic;
            rosSocket = rosConnector.RosSocket;
            rosSocket.Subscribe<Messages.Sensor.PointCloud2>(Topic, ReceiveMessage, (int)(timeStep)); // the rate(in ms in between messages) at which to throttle the topics
        }

        public void ReceiveMessage(Messages.Sensor.PointCloud2 message)
        {

        }

    }
}