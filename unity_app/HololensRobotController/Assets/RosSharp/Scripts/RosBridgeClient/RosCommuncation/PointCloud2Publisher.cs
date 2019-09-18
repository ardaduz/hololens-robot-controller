﻿/*
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

using System.Collections.Concurrent;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class PointCloud2Publisher : Publisher<Messages.Sensor.PointCloud2>
    {
        private Main connectedComponent;
        private ConcurrentStack<Messages.Sensor.PointCloud2> pointCloudStack;
        public string FrameId = "Unity";

        private Messages.Sensor.PointCloud2 message;
        private float scanPeriod;
        private float previousScanTime = 0;

        protected override void Start()
        {
            base.Start();
            connectedComponent = (Main)GetComponent(typeof(Main));
        }

        private void Update()
        { 
        }

        private void UpdateMessage()
        {
            /*
            message = new Messages.Sensor.PointCloud2
            {
                header = new Messages.Standard.Header { frame_id = FrameId },
                height = 1,
                width = pointCloudReader.width,
                fields = pointCloudReader.fields,
                is_bigendian = pointCloudReader.is_bigendian,
                point_step = pointCloudReader.point_step,
                row_step = pointCloudReader.row_step,
                data = pointCloudReader.data,
                is_dense = false
            };
            
            message.header.Update();
            pointCloudReader.Scan();
            Publish(message);
            */
        }
    }
}