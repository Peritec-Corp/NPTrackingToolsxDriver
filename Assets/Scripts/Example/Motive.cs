using System.Threading.Tasks;
using UnityEngine;

namespace NPTrackingToolsx64.Example
{
    public class Motive : MonoBehaviour
    {
        [SerializeField]
        Transform camera1;

        [SerializeField]
        Transform camera2;

        [SerializeField]
        Transform camera3;

        [SerializeField]
        Transform rigidBody;

        // Start is called before the first frame update
        void Start()
        {
            Debug.Log(NPTrackingTools.Initialize());            
            Debug.Log(NPTrackingTools.LoadProfileW(@"C:\Users\k-miz\Desktop\Motive Profile - 2022-04-08.motive"));
            Debug.Log(NPTrackingTools.LoadCalibration(@"C:\Users\k-miz\Desktop\CalibrationResult 2022-04-08 10.cal"));

            while(NPTrackingTools.CameraCount() == 0)
            {
                NPTrackingTools.Update();
                Task.Delay(100);
            }

            Debug.Log(NPTrackingTools.RigidBodyCount());

            camera1.position = new Vector3(NPTrackingTools.CameraXLocation(0), NPTrackingTools.CameraYLocation(0), NPTrackingTools.CameraZLocation(0));
            camera2.position = new Vector3(NPTrackingTools.CameraXLocation(1), NPTrackingTools.CameraYLocation(1), NPTrackingTools.CameraZLocation(1));
            camera3.position = new Vector3(NPTrackingTools.CameraXLocation(2), NPTrackingTools.CameraYLocation(2), NPTrackingTools.CameraZLocation(2));

            while (NPTrackingTools.Update() == NPRESULT.SUCCESS) { }

        }

        // Update is called once per frame
        void Update()
        {
            if (NPTrackingTools.Update() == NPRESULT.SUCCESS)
            {
                // Debug.Log($"{NPTrackingTools.FrameMarkerX(1)}, {NPTrackingTools.FrameMarkerY(1)}, {NPTrackingTools.FrameMarkerZ(1)}");

                if (NPTrackingTools.FrameMarkerCount() != 0)
                {

                    Debug.Log(NPTrackingTools.CameraFrameBufferSaveAsBMP(0, "image.bmp"));


                    //if (NPTrackingTools.IsRigidBodyTracked(0))
                    //{
                    //    float x, y, z, qx, qy, qz, qw, yaw, pitch, roll;
                    //    NPTrackingTools.RigidBodyLocation(0, out x, out y, out z, out qx, out qy, out qz, out qw, out yaw, out pitch, out roll);
                    //    rigidBody.position = new Vector3(x, y, z);
                    //    rigidBody.rotation = new Quaternion(qx, -qy, -qz, qw);
                    //}
                }
            }
        }

        private void OnApplicationQuit()
        {
            Debug.Log(NPTrackingTools.Shutdown());
        }
    }
}