using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using HoloToolkit.Unity.SpatialMapping;
using HoloToolkit.Unity.InputModule;
using System;
using System.Linq;

#if NETFX_CORE
using Windows.System.Threading;
#endif

namespace HololensRobotController.Utilities
{
    public static class CoordinateTransformations
    {
        public static void ConvertPoseUnity2ROS(ref Vector3 position, ref Quaternion orientation)
        {
            Vector3 convertedPosition = ConvertPositionUnity2ROS(position);
            Quaternion convertedOrientation = ConvertOrientationUnity2ROS(orientation);
            position = convertedPosition;
            orientation = convertedOrientation;
        }

        public static Vector3 ConvertPositionUnity2ROS(Vector3 position)
        {
            Vector3 convertedPosition = new Vector3();
            convertedPosition.x = position.z;
            convertedPosition.y = -position.x;
            convertedPosition.z = position.y;
            return convertedPosition;
        }

        public static Quaternion ConvertOrientationUnity2ROS(Quaternion orientation)
        {
            Quaternion convertedOrientation = new Quaternion();
            convertedOrientation.x = -orientation.z;
            convertedOrientation.y = orientation.x;
            convertedOrientation.z = -orientation.y;
            convertedOrientation.w = orientation.w;
            return convertedOrientation;
        }
    }
}
