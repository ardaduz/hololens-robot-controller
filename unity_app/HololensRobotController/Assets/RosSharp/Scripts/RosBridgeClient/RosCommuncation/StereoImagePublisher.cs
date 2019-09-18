/*
© CentraleSupelec, 2017
Author: Dr. Jeremy Fix (jeremy.fix@centralesupelec.fr)

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

// Adjustments to new Publication Timing and Execution Framework 
// © Siemens AG, 2018, Dr. Martin Bischoff (martin.bischoff@siemens.com)

using UnityEngine;
using System;

namespace RosSharp.RosBridgeClient
{
    public class StereoImagePublisher : Publisher<Messages.Sensor.Image>
    {
        private Main connectedComponent;
        public int id = 0;
        public int resolutionWidth = 160;
        public int resolutionHeight = 480;
        private Messages.Sensor.Image message;

        protected override void Start()
        {
            base.Start();
            Time.fixedDeltaTime = (float)0.02;
            message = new Messages.Sensor.Image();
            connectedComponent = (Main)GetComponent(typeof(Main));
        }

        void FixedUpdate()
        {
//#if NETFX_CORE
//            if (connectedComponent != null && connectedComponent.stereoCameraSources != null && id < connectedComponent.stereoCameraSources.Count)
//            {
//                bool isAvailable = connectedComponent.stereoCameraSources[id].images.TryPop(out message);
//                if (isAvailable)
//                {
//                    UpdateMessage();
//                    connectedComponent.stereoCameraSources[id].images.Clear();
//                }
//            }
//#endif
        }

        private void UpdateMessage()
        {
            message.header.frame_id = "stereo" + id.ToString();
            Publish(message);

            System.Diagnostics.Debug.WriteLine(message.header.frame_id + "- frame" +
                message.header.seq + " is PUBLISHED at time " + message.header.stamp.secs + message.header.stamp.nsecs/1e9);
        }
    }
}