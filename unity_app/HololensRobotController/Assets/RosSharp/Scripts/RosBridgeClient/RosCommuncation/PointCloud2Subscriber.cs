/*
© Siemens AG, 2018
Author: Berkay Alp Cakal (berkay_alp.cakal.ct@siemens.com)

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
    public class PointCloud2Subscriber : Subscriber<Messages.Sensor.PointCloud2>
    {
        public PointCloud2Writer pointCloudWriter;
        bool isMessageReceived = false;

        protected override void Start()
        {
            base.Start();
        }

        private void Update()
        {
            if (isMessageReceived)
                System.Diagnostics.Debug.WriteLine("Received point cloud!");
                ProcessMessage();
        }

        protected override void ReceiveMessage(Messages.Sensor.PointCloud2 pointCloud)
        {
            isMessageReceived = true;
            
        }

        private void ProcessMessage()
        {
        }


    }
}