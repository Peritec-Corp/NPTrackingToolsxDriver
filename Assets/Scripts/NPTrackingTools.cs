namespace NPTrackingToolsx64
{
    using System;
    using System.Runtime.InteropServices;

    public enum NPRESULT : int
    {
        SUCCESS = 0,
        FILENOTFOUND = 1,
        LOADFAILED = 2,
        FAILED = 3,
        INVALIDFILE = 8,
        INVALIDCALFILE = 9,
        UNBLETOINITIALIZE = 10,
        INVALIDLICENSE = 11,
        NOFRAMEAVAILABLE = 14,
        DEVICESINUSE = 15,
    }

    public enum CAMERAVIDEOTYPE : int
    {
        NPVIDEOTYPE_SEGMENT = 0,
        NPVIDEOTYPE_GRAYSCALE = 1,
        NPVIDEOTYPE_OBJECT = 2,
        NPVIDEOTYPE_PRECISION = 4,
        NPVIDEOTYPE_MJPREG = 6,
    }

    public enum RigidBodyRefineStates : int
    {
        TT_RigidBodyRefine_Initialized = 0,
        TT_RigidBodyRefine_Sampling,
        TT_RigidBodyRefine_Solving,
        TT_RigidBodyRefine_Complete,
        TT_RigidBodyRefine_Uninitialized,
    }

    public enum eCameraStates : int
    {
        Camera_Enabled = 0,
        Camera_Disabled_For_Reconstruction = 1,
        Camera_Disabled = 2,
        CameraStatesCount = 3,
    }

    public class NPTrackingTools
    {
        private NPTrackingTools() { }

        #region == Project Management ==
        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_Initialize", CallingConvention = CallingConvention.Cdecl)]
        public static extern NPRESULT Initialize();

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_Shutdown", CallingConvention = CallingConvention.Cdecl)]
        public static extern NPRESULT Shutdown();

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_Update", CallingConvention = CallingConvention.Cdecl)]
        public static extern NPRESULT Update();

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_UpdateSingleFrame", CallingConvention = CallingConvention.Cdecl)]
        public static extern NPRESULT UpdateSingleFrame();

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_LoadCalibrationW", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Unicode)]
        public static extern NPRESULT LoadCalibrationW(string filename);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_LoadCalibration", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern NPRESULT LoadCalibration(string filename);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_LoadRigidBodiesW", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Unicode)]
        public static extern NPRESULT LoadRigidBodiesW(string filename);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_LoadRigidBodies", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern NPRESULT LoadRigidBodies(string filename);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SaveRigidBodiesW", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Unicode)]
        public static extern NPRESULT SaveRigidBodiesW(string filename);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SaveRigidBodies", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern NPRESULT SaveRigidBodies(string filename);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_AddRigidBodiesW", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Unicode)]
        public static extern NPRESULT AddRigidBodiesW(string filename);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_AddRigidBodies", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern NPRESULT AddRigidBodies(string filename);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_LoadProfileW", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Unicode)]
        public static extern NPRESULT LoadProfileW(string filename);
        
        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_LoadProfile", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern NPRESULT LoadProfile(string filename);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SaveProfileW", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Unicode)]
        public static extern NPRESULT SaveProfileW(string filename);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SaveProfile", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern NPRESULT SaveProfile(string filename);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_LoadProjectW", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Unicode)]
        public static extern NPRESULT LoadProjectW(string filename);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_LoadProject", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern NPRESULT LoadProject(string filename);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SaveProjectW", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Unicode)]
        public static extern NPRESULT SaveProjectW(string filename);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SaveProject", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern NPRESULT SaveProject(string filename);

        //[DllImport("NPTrackingToolsx64", EntryPoint = "TT_LoadCalibrationFromMemory", CallingConvention = CallingConvention.Cdecl)]
        //public static extern NPRESULT LoadCalibrationFromMemory(System.Text.StringBuilder buffer, int bufferSize);

        #endregion == Project Management ==

        #region == DATA STREAMING ==
        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_StreamNP", CallingConvention = CallingConvention.Cdecl)]
        public static extern NPRESULT StreamNP(bool enabled);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_StreamTrackd", CallingConvention = CallingConvention.Cdecl)]
        public static extern NPRESULT StreamTrackd(bool enabled);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_StreamVRPN", CallingConvention = CallingConvention.Cdecl)]
        public static extern NPRESULT StreamVRPN(bool enabled, int port);
        #endregion == DATA STREAMING ==
        
        #region == 3D FRAME DATA ==
        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_FrameMarkerCount", CallingConvention = CallingConvention.Cdecl)]
        public static extern int FrameMarkerCount();

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_FrameMarkerX", CallingConvention = CallingConvention.Cdecl)]
        public static extern float FrameMarkerX(int markerIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_FrameMarkerY", CallingConvention = CallingConvention.Cdecl)]
        public static extern float FrameMarkerY(int markerIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_FrameMarkerZ", CallingConvention = CallingConvention.Cdecl)]
        public static extern float FrameMarkerZ(int markerIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_FrameMarkerResidual", CallingConvention = CallingConvention.Cdecl)]
        public static extern float FrameMarkerResidual(int markerIndex);

        //TTAPI   Core::cUID TT_FrameMarkerLabel( int markerIndex );
        
        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_FrameTimeStamp", CallingConvention = CallingConvention.Cdecl)]
        public static extern double FrameTimeStamp();

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_FrameCameraCentroid", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool FrameCameraCentroid(int markerIndex, int cameraIndex, out float x, out float y);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_FlushCameraQueues", CallingConvention = CallingConvention.Cdecl)]
        public static extern void FlushCameraQueues();
        #endregion == 3D FRAME DATA ==

        #region == RIGID BODY==
        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_IsRigidBodyTracked", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool IsRigidBodyTracked(int rbIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RigidBodyLocation", CallingConvention = CallingConvention.Cdecl)]
        public static extern void RigidBodyLocation(int rbIndex, out float x, out float y, out float z, 
            out float qx, out float qy, out float qz, out float qw, out float yaw, out float pitch, out float roll);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_ClearRigidBodyList", CallingConvention = CallingConvention.Cdecl)]
        public static extern void ClearRigidBodyList();

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RemoveRigidBody", CallingConvention = CallingConvention.Cdecl)]
        public static extern NPRESULT RemoveRigidBody(int rbIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RigidBodyCount", CallingConvention = CallingConvention.Cdecl)]
        public static extern int RigidBodyCount();

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RigidBodyUserData", CallingConvention = CallingConvention.Cdecl)]
        public static extern int RigidBodyUserData(int rbIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SetRigidBodyUserData", CallingConvention = CallingConvention.Cdecl)]
        public static extern void SetRigidBodyUserData(int rbIndex, int ID);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RigidBodyMeanError", CallingConvention = CallingConvention.Cdecl)]
        public static extern void RigidBodyMeanError(int rbIndex, int ID);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RigidBodyName", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        private static extern IntPtr _RigidBodyName(int rbIndex);

        public static string RigidBodyName(int rbIndex)
        { 
            return Marshal.PtrToStringAnsi(_RigidBodyName(rbIndex));
        }

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RigidBodyNameW", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Unicode)]
        private static extern IntPtr _RigidBodyNameW(int rbIndex);

        public static string RigidBodyNameW(int rbIndex)
        {
            return Marshal.PtrToStringUni(_RigidBodyNameW(rbIndex));
        }

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SetRigidBodyEnabled", CallingConvention = CallingConvention.Cdecl)]
        public static extern void SetRigidBodyEnabled(int rbIndex, bool enabled);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RigidBodyEnabled", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool RigidBodyEnabled(int rbIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RigidBodyTranslatePivot", CallingConvention = CallingConvention.Cdecl)]
        public static extern NPRESULT RigidBodyTranslatePivot(int index, float x, float y, float z);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RigidBodyResetOrientation", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool RigidBodyResetOrientation(int rbIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RigidBodyMarkerCount", CallingConvention = CallingConvention.Cdecl)]
        public static extern int RigidBodyMarkerCount(int rbIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RigidBodyMarker", CallingConvention = CallingConvention.Cdecl)]
        public static extern void RigidBodyMarker(int rbIndex, int markerIndex, out float x, out float y, out float z);
        
        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RigidBodyUpdateMarker", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool RigidBodyUpdateMarker(int rbIndex, int markerIndex, ref float x, ref float y, ref float z);
        
        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RigidBodyPointCloudMarker", CallingConvention = CallingConvention.Cdecl)]
        public static extern void RigidBodyPointCloudMarker(int rbIndex, int markerIndex, out bool tracked, out float x, out float y, out float z);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RigidBodyPlacedMarker", CallingConvention = CallingConvention.Cdecl)]
        public static extern void RigidBodyEnabled(int rbIndex, int markerIndex, out bool tracked, out float x, out float y, out float z);

        //TTAPI Core::cUID  TT_RigidBodyID(int rbIndex );

        //TTAPI NPRESULT TT_CreateRigidBody( const char* name, int id, int markerCount, float* markerList );

        //TTAPI NPRESULT TT_RigidBodySettings(int rbIndex, RigidBodySolver::cRigidBodySettings &settings );

        //TTAPI NPRESULT TT_SetRigidBodySettings(int rbIndex, RigidBodySolver::cRigidBodySettings &settings );

        //TTAPI bool TT_RigidBodyRefineStart(Core::cUID rigidBodyID, int sampleCount);
        
        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RigidBodyRefineSample", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool RigidBodyRefineSample();

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RigidBodyRefineState", CallingConvention = CallingConvention.Cdecl)]
        public static extern RigidBodyRefineStates RigidBodyRefineState();

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RigidBodyRefineProgress", CallingConvention = CallingConvention.Cdecl)]
        public static extern float RigidBodyRefineProgress();

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RigidBodyRefineInitialError", CallingConvention = CallingConvention.Cdecl)]
        public static extern float RigidBodyRefineInitialError();

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RigidBodyRefineResultError", CallingConvention = CallingConvention.Cdecl)]
        public static extern float RigidBodyRefineResultError();

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RigidBodyRefineApplyResult", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool RigidBodyRefineApplyResult();

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RigidBodyRefineReset", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool RigidBodyRefineReset();

        #endregion == RIGID BODY==

        #region == Camera group
        //TTAPI CameraLibrary::CameraManager* TT_GetCameraManager();

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_BuildNumber", CallingConvention = CallingConvention.Cdecl)]
        public static extern int BuildNumber();

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraGroupCount", CallingConvention = CallingConvention.Cdecl)]
        public static extern int CameraGroupCount();

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CreateCameraGroup", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool CreateCameraGroup();

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_RemoveCameraGroup", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool RemoveCameraGroup(int groupIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CamerasGroups", CallingConvention = CallingConvention.Cdecl)]
        public static extern int CamerasGroup(int cameraIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SetGroupShutterDelay", CallingConvention = CallingConvention.Cdecl)]
        public static extern void SetGroupShutterDelay(int groupIndex, int microseconds);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SetCameraGroups", CallingConvention = CallingConvention.Cdecl)]
        public static extern void SetCameraGroups(int cameraIndex, int groupIndex);

        //TTAPI NPRESULT TT_CameraGroupFilterSettings(int groupIndex, cCameraGroupFilterSettings &settings );

        //TTAPI NPRESULT TT_SetCameraGroupFilterSettings(int groupIndex, cCameraGroupFilterSettings &settings );

        //TTAPI NPRESULT TT_CameraGroupMarkerSize(int groupIndex, cCameraGroupMarkerSizeSettings &settings );

        //TTAPI NPRESULT TT_SetCameraGroupMarkerSize(int groupIndex, cCameraGroupMarkerSizeSettings &settings );

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SetCameraGroupReconstruction", CallingConvention = CallingConvention.Cdecl)]
        public static extern NPRESULT SetCameraGroupReconstruction(int groupIndex, bool enable);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SetEnabledFilterSwitch", CallingConvention = CallingConvention.Cdecl)]
        public static extern NPRESULT SetEnabledFilterSwitch(bool enabled);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_IsFilterSwitchEnabled", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool IsFilterSwitchEnabled();
        #endregion == Camera group ==

        #region == Camera==
        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraCount", CallingConvention = CallingConvention.Cdecl)]
        public static extern int CameraCount();

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraXLocation", CallingConvention = CallingConvention.Cdecl)]
        public static extern float CameraXLocation(int cameraIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraYLocation", CallingConvention = CallingConvention.Cdecl)]
        public static extern float CameraYLocation(int cameraIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraZLocation", CallingConvention = CallingConvention.Cdecl)]
        public static extern float CameraZLocation(int cameraIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraOrientationMatrix", CallingConvention = CallingConvention.Cdecl)]
        public static extern float CameraOrientationMatrix(int cameraIndex, int matrixIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraName", CharSet = CharSet.Unicode, CallingConvention = CallingConvention.StdCall)]
        private static extern IntPtr _CameraName(int cameraIndex);

        public static string CameraName(int cameraInex)
        {
            return Marshal.PtrToStringAnsi(_CameraName(cameraInex));
        }

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraSerial", CallingConvention = CallingConvention.Cdecl)]
        public static extern int CameraSerial(int cameraIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraMarkerCount", CallingConvention = CallingConvention.Cdecl)]
        public static extern int CameraMarkerCount(int cameraIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraMarker", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool CameraMarker(int cameraIndex, int markerIndex, out float x, out float y);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraPixelResolution", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool CameraPixelResolution(int cameraIndex, out int width, out int height);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraMarkerPredistorted", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool CameraMarkerPredistorted(int cameraIndex, int markerIndex, out float x, out float y);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SetCameraSettings", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool SetCameraSettings(int cameraIndex, int videoType, int exposure, int threshold, int intensity);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SetCameraFrameRate", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool SetCameraFrameRate(int cameraIndex, int framerate);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraFrameRate", CallingConvention = CallingConvention.Cdecl)]
        public static extern int CameraFrameRate(int cameraIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraVideoType", CallingConvention = CallingConvention.Cdecl)]
        public static extern int CameraVideoType(int cameraIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraExposure", CallingConvention = CallingConvention.Cdecl)]
        public static extern int CameraExposure(int cameraIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraThreshold", CallingConvention = CallingConvention.Cdecl)]
        public static extern int CameraThreshold(int cameraIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraIntensity", CallingConvention = CallingConvention.Cdecl)]
        public static extern int CameraIntensity(int cameraIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraTemperature", CallingConvention = CallingConvention.Cdecl)]
        public static extern float CameraTemperature(int cameraIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraRinglightTemperature", CallingConvention = CallingConvention.Cdecl)]
        public static extern float CameraRinglightTemperature(int cameraIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraGrayscaleDecimation", CallingConvention = CallingConvention.Cdecl)]
        public static extern int CameraGrayscaleDecimation(int cameraIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SetCameraGrayscaleDecimation", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool SetCameraGrayscaleDecimation(int cameraIndex, int value);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SetCameraFilterSwitch", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool SetCameraFilterSwitch(int cameraIndex, bool enableIRFilter);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SetCameraAGC", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool SetCameraAGC(int cameraIndex, bool enableIRFilter);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SetCameraAEC", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool SetCameraAEC(int cameraIndex, bool ebnabledAutomaticExposureControl);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SetCameraHighPower", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool SetCameraHighPower(int cameraIndex, bool enableHighPowerMode);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SetCameraMJPEGHighQuality", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool SetCameraMJPEGHighQuality(int cameraIndex, int mjpegquality);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraImagerGain", CallingConvention = CallingConvention.Cdecl)]
        public static extern int CameraImagerGain(int cameraIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraImagerGainLevels", CallingConvention = CallingConvention.Cdecl)]
        public static extern int CameraImagerGainLevels(int cameraIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SetCameraImagerGain", CallingConvention = CallingConvention.Cdecl)]
        public static extern void SetCameraImagerGain(int cameraIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_IsContinuousIRAvailable", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool IsContinuousIRAvailable(int cameraIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_ContinuousIR", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool ContinuousIR(int cameraIndex);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SetContinuousIR", CallingConvention = CallingConvention.Cdecl)]
        public static extern void SetContinuousIR(int cameraIndex, bool enable);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_ClearCameraMask", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool ClearCameraMask(int cameraIndex);

        //TTAPI bool TT_SetCameraMask(int cameraIndex, unsigned char* buffer, int bufferSize);

        //TTAPI bool TT_CameraMask(int cameraIndex, unsigned char* buffer, int bufferSize);
        
        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraMaskInfo", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool CameraMaskInfo(int cameraIndex, out int blockingMaskWidth, out int blockingMaskHeight, out int blockingMaskGrid);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_SetCameraState", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool SetCameraState(int cameraIndex, eCameraStates state);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraState", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool CameraState(int cameraIndex, out eCameraStates state);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraID", CallingConvention = CallingConvention.Cdecl)]
        public static extern int CameraID(int cameraIndex);

        //TTAPI bool TT_CameraFrameBuffer(int cameraIndex, int bufferPixelWidth, int bufferPixelHeight, int bufferByteSpan, int bufferPixelBitDepth, unsigned char* buffer);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraFrameBufferSaveAsBMP", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern bool CameraFrameBufferSaveAsBMP(int cameraIndex, string filename);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraBackproject", CallingConvention = CallingConvention.Cdecl)]
        public static extern void CameraBackproject(int cameraIndex, float x, float y, float z, out float cameraX, out float cameraY);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraUndistort2DPoint", CallingConvention = CallingConvention.Cdecl)]
        public static extern void CameraUndistort2DPoint(int cameraIndex, out float x, out float y);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraDistort2DPoint", CallingConvention = CallingConvention.Cdecl)]
        public static extern void CameraDistort2DPoint(int cameraIndex, out float x, out float y);

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_CameraRay", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool CameraRay(int cameraIndex, float x, float y, 
            out float rayStartX, out float rayStartY, out float rayStartZ, out float rayEndX, out float rayEndY, out float rayEndZ);

        //TTAPI bool TT_CameraModel(int cameraIndex, float x, float y, float z, 
        //    float* orientation, float principleX, float principleY, float focalLengthX, float focalLengthY, 
        //    float kc1, float kc2, float kc3, float tangential0, float tangential1);

        //TTAPI CameraLibrary::Camera * TT_GetCamera(int cameraIndex );

        #endregion == Camera group ==

        #region == Additional ==
        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_OrientTrackingBar", CallingConvention = CallingConvention.Cdecl)]
        public static extern NPRESULT OrientTrackingBar(float positionX, float positionY, float positionZ, 
            float orientationX, float orientationY, float orientationZ, float orientationW);

        //TTAPI void TT_AttachCameraModule(int cameraIndex, CameraLibrary::cCameraModule* module);

        //TTAPI void TT_DetachCameraModule(int cameraIndex, CameraLibrary::cCameraModule* module);

        //TTAPI void TT_AttachRigidBodySolutionTest(int rbIndex, cRigidBodySolutionTest* test);

        //TTAPI void TT_DetachRigidBodySolutionTest(int rbIndex, cRigidBodySolutionTest* test);

        //TTAPI void TT_AttachListener(cTTAPIListener* listener);

        //TTAPI void TT_DetachListener(cTTAPIListener* listener);
        
        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_GetResultString", CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr _GetResultString(NPRESULT result);

        public static string GetResultString(NPRESULT result)
        {
            return Marshal.PtrToStringAnsi(_GetResultString(result));
        }

        [DllImport("NPTrackingToolsx64", EntryPoint = "TT_TestSoftwareMutex", CallingConvention = CallingConvention.Cdecl)]
        public static extern NPRESULT TestSoftwareMutex();
        #endregion == Additional ==
    }
}