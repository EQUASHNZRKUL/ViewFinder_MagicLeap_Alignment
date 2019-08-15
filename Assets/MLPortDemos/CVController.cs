// %BANNER_BEGIN%
// ---------------------------------------------------------------------
// %COPYRIGHT_BEGIN%
//
// Copyright (c) 2019 Magic Leap, Inc. All Rights Reserved.
// Use of this file is governed by the Creator Agreement, located
// here: https://id.magicleap.com/creator-terms
//
// %COPYRIGHT_END%
// ---------------------------------------------------------------------
// %BANNER_END%
using System; 
using System.Collections; 
using System.Collections.Generic; 

using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.MagicLeap;

using OpenCVForUnity;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.UnityUtils;
using OpenCVForUnity.ImgprocModule;
using OpenCVForUnity.Calib3dModule;
using OpenCVForUnity.ArucoModule;

namespace MagicLeap
{
    /// <summary>
    /// On Touch: 
    /// Stage I: Detects ArUco markers in Screen space and uses the AR Controller to convert to world space. 
    /// World space coordinates are cached and collected along with the initial captured image and camera world
    /// position. Detected points are marked on the image. 
    ///
    /// Stage II: If ArUco markers form a face, the face surface is extracted from the captured image, rectified 
    /// and cached, and then displayed on the left side of the display. 
    /// 
    /// On Each Frame: 
    /// Stage III: Current camera position and rotation are used to get new screen points of projected world points.
    /// Current camera position is also used to find cached rectified texture that was captured closest to 
    /// current location to obtain most accurate reconstructed view. 
    ///
    /// With new screen coordinates, Homography is created and used towarp cached rectified texture for each 
    /// face. Then the faces are combined and displayed as a single image.
    /// </summary>
    public class CVController : MonoBehaviour
    {
        #region Unity Parameters
        [Tooltip("Reference to the controller object's transform.")]
        public Transform ControllerTransform;

        [SerializeField]
        [Tooltip("Instantiates this prefab on a gameObject at the touch location.")]
        Camera m_deviceCamera;
        public Camera rgb_camera
        {
            get { return m_deviceCamera; }
            set { m_deviceCamera = value; }
        }

        [Header("Constants")]
        [SerializeField, Tooltip("True if display is attached to hand")]
        public bool HAND_DISPLAY = true; 

        [Header("Visuals")]
        [SerializeField, Tooltip("Object that will show up when recording")]
        private GameObject _recordingIndicator = null;

        [SerializeField, Tooltip("Object to set new images on.")]
        private GameObject _hudObject = null;

        [SerializeField, Tooltip("Object to set new images on.")]
        private GameObject _handObject = null;

        [SerializeField, Tooltip("The renderer to show the video capture on")]
        private Renderer _screenRenderer = null;

        [SerializeField, Tooltip("Object to set new images on.")]
        private RawImage m_TopImage1 = null;

        [SerializeField, Tooltip("Object to set new images on.")]
        private RawImage m_TopImage2 = null;

        [SerializeField, Tooltip("Object to set new images on.")]
        private RawImage m_TopImage3 = null;

        [SerializeField, Tooltip("Object to set new images on.")]
        private RawImage m_OutImage = null;
        #endregion

        #region Private Variables
        // Constants
        private static int POINT_COUNT = 7; 
        private static int FACE_COUNT = 3; 
        private static int CACHE_COUNT = 5; 
        private static float X_OFFSET = -1.0f; 
        public static double HOMOGRAPHY_WIDTH = 640.0;
        public static double HOMOGRAPHY_HEIGHT = 360.0;
        public static int RECT_SIZE = 400; 
        public static int SCALE_FACTOR = 3; 

        // Mats & Mat Arrays
        public Mat outMat = new Mat(360, 640, CvType.CV_8UC1);
        private Mat cached_bigMat = new Mat (1080, 1920, CvType.CV_8UC1);
        private Mat cached_initMat = null; 
        private Mat warpedMat = new Mat (360, 640, CvType.CV_8UC1);
        private Mat ids = new Mat(360, 640, CvType.CV_8UC1);
        
        private List<Mat> corners = new List<Mat>();
        private Mat[,] rectMat_array = new Mat[FACE_COUNT, CACHE_COUNT];
        private Mat[] homoMat_array = new Mat[FACE_COUNT];

        // Face corner indices for each face
        private int[,] face_index = { {3, 6, 4, 5}, {0, 1, 3, 6}, {6, 1, 5, 2} };

        // Point Lists
        private Vector3[] src_ray_array = new Vector3[POINT_COUNT];
        private Vector3[] src_world_array = new Vector3[POINT_COUNT];
        private Point[] c1_point_array = new Point[POINT_COUNT];
        private Point[] hand_point_array = new Point[POINT_COUNT];
        private Vector3[] camerapos_array = new Vector3[CACHE_COUNT]; 

        // Indices
        private int world_idx; 
        private int cap_i = 0; 

        // Face Lists
        private bool[] faceX_full = new bool[3];

        // Textures
        private Texture2D rawVideoTexture; 
        private Texture2D out_texture;

        private GameObject _previewObject = null; 

        private Matrix4x4 camera_pose; 
        #endregion

        #region Helper Functions
        int count_src_nulls() {
            int acc = 0;
            for (int i = 0; i < 7; i++)
            {
                if (src_world_array[i] == null) {
                    acc++; 
                }
            }
            return (7 - acc); 
        }

        bool check_faces(int face_i) {
            for (int i = 0; i < 4; i++) {
                int src_i = face_index[face_i, i]; 
                if (src_world_array[src_i] == null) {
                    return false; 
                }
            }
            return true; 
        }
        
        static int arucoTosrc(int a) {
            if (a == 7) { return 4; }
            else if (a == 6) { return 5; }
            else if (a == 10) { return 6; }
            else {return a; }
        }

        static int srcToarcuo(int s) {
            if (s == 4) { return 7; }
            else if (s == 5) {return 6; }
            else if (s == 6) {return 10; }
            else {return s; }
        }

        static void ProcessImage(byte[] data, byte levels)
        {
            byte factor = (byte) (byte.MaxValue / levels);
            for (int i = 0; i < data.Length; i++)
            {
                data[i] = (byte) (data[i] / factor * factor);
            }
        }

        static void MatrixToTransform(Matrix4x4 m, Camera c)
        {
            Transform t = c.transform; 

            // T Section
            Vector3 position; 
            position.x = m.m03;
            position.y = m.m13;
            position.z = m.m23; 

            t.position = position; 

            // R Section
            Vector3 fwd; 
            fwd.x = m.m02; 
            fwd.y = m.m12; 
            fwd.z = m.m22; 

            Vector3 up; 
            up.x = m.m01; 
            up.y = m.m11;
            up.z = m.m21; 

            t.rotation = Quaternion.LookRotation(fwd, up); 

            // S Section
            Vector3 scale; 
            scale.x = new Vector4(m.m00, m.m10, m.m20, m.m30).magnitude; 
            scale.y = new Vector4(m.m01, m.m11, m.m21, m.m31).magnitude; 
            scale.z = new Vector4(m.m02, m.m12, m.m22, m.m32).magnitude; 

            t.localScale = scale; 
        }

        /// Normalizes [I] such that min(I) -> 0 and max(I) -> 255
        static void DesaturateMat(ref Mat I) {
            Core.MinMaxLocResult res = Core.minMaxLoc(I);
            double alpha = 255/(res.maxVal - res.minVal);
            double beta = -res.minVal * alpha; 
            Mat O = (I - new Scalar(res.minVal))*alpha;
            Debug.LogFormat("Mat Max and Min: {0}, {1} \n Alpha & Beta: {2}, {3} \n I(200, 200) -> O(200, 200): {4} vs. {5} vs. {6}", 
                res.maxVal, res.minVal, alpha, beta, I.get(200, 200)[0], (alpha * I.get(200, 200)[0]) + beta, O.get(200,200)[0] );
            I = O; 
        }

        // Caches the camera's world points when textures are captured. 
        public void CacheCamPoints()
        {
            camerapos_array[cap_i] = (Camera.main.transform.position);
            cap_i++;
        }

        public int GetClosestIndex() {
            Vector3 curr_cam; 
            if (HAND_DISPLAY) {
                curr_cam = ControllerTransform.position; 
            }
            else {
                curr_cam = Camera.main.transform.position;
            }

            int min_i = 0; 
            float min_dist = Vector3.Distance(curr_cam, camerapos_array[0]);
            for (int i = 0; i < cap_i; i++) {
                Vector3 camera_pos = camerapos_array[i];
                float dist = Vector3.Distance(curr_cam, camera_pos); 
                if (dist < min_dist) {
                    min_dist = dist; 
                    min_i = i; 
                }
            }
            return min_i; 
        }
        #endregion

        #region Master Functions
        void ArucoDetection() {
            // Detect ArUco markers
            Dictionary dict = Aruco.getPredefinedDictionary(Aruco.DICT_4X4_1000);
            Aruco.detectMarkers(cached_initMat, dict, corners, ids);
            Aruco.drawDetectedMarkers(cached_initMat, corners, ids);
            // Debug.Log("AD - 93: Markers Detected");
            // Debug.LogFormat("Corners: {0}", corners.Count);

            // Get desired corner of marker
            Point[] src_point_array = new Point[POINT_COUNT];
            for (int i = 0; i < corners.Count; i++) {
                int aruco_id = (int) (ids.get(i, 0)[0]);
                int src_i = arucoTosrc(aruco_id);
                int corner_i = aruco_id % 4;

                // Debug.LogFormat("AD - 101: aruco_id: {0}; corner_i: {1}; src_i: {2}", aruco_id, corner_i, src_i);

                // Store corner[i] into spa[src_i]
                src_point_array[src_i] = new Point(corners[i].get(0, corner_i)[0], corners[i].get(0, corner_i)[1]);

                // Display the corner as circle on outMat. 
                Imgproc.circle(cached_initMat, src_point_array[src_i], 10, new Scalar(255, 255, 0));
            }

            // Converting to Ray values for Raycast
            Camera _cam = Camera.main;
            if (_cam != null) {
                for (int i = 0; i < POINT_COUNT; i++) {
                    if (src_point_array[i] != null) {
                        src_ray_array[i] = _cam.ScreenPointToRay(
                            new Vector3((float) src_point_array[i].x,(float) src_point_array[i].y, 0)).direction;
                    }
                }
            }
            // Debug.LogFormat("Detected Direction: {0}", src_ray_array[0]);
            // Debug.LogFormat("Camera Direction: {0}", _cam.transform.forward);

            // Count non-null source points 
            bool spa_full = (count_src_nulls() == 7);

            // Check if have valid faces
            for (int i = 0; i < FACE_COUNT; i++) {
                // faceX_full[i] = check_faces(i); 
                faceX_full[i] = check_faces(i); 
            }

            Core.flip(cached_initMat, outMat, 0);
        }

        /// Sets the projected ScreenPoints of the world coordinate values in src_world_array 
        /// from the PoV of the RGB Camera. 
        void SetC1ScreenPoints() { 
            Camera _camera = Camera.main;

            MatrixToTransform(camera_pose, rgb_camera);

            Debug.LogFormat("Camera Pose: {0} \n old transform: {1}, {2}, {3} \n new transform: {4}, {5}, {6}", camera_pose, _camera.transform.position, _camera.transform.rotation, _camera.transform.localScale, rgb_camera.transform.position, rgb_camera.transform.rotation, rgb_camera.transform.localScale);

            MLCVCameraIntrinsicCalibrationParameters intrinsicParam; 
            MLCamera.GetIntrinsicCalibrationParameters(out intrinsicParam); 

            Debug.LogFormat("Camera Pose: {0} \n Left Eye Pose: {1} \n Intrinsics: FOV -- {5} vs {2} \n Focal Length -- {6} vs. {3} \n Principal Point -- {7} vs. {4} \n Sensor Size {8} vs. {9} x {10} \n Camera Size: {11} x {12} \n Camera Rect: {13}", 
                camera_pose, 
                rgb_camera.GetStereoViewMatrix(Camera.StereoscopicEye.Left), 
                intrinsicParam.FOV, intrinsicParam.FocalLength, intrinsicParam.PrincipalPoint, 
                _camera.fieldOfView, _camera.focalLength, _camera.lensShift, 
                _camera.sensorSize, intrinsicParam.Width, intrinsicParam.Height, 
                rgb_camera.pixelWidth, rgb_camera.pixelHeight, 
                rgb_camera.pixelRect);

            rgb_camera.fieldOfView = intrinsicParam.FOV; 
            rgb_camera.focalLength = intrinsicParam.FocalLength.x; 
            rgb_camera.sensorSize = new Vector2(intrinsicParam.Width, intrinsicParam.Height);
            rgb_camera.usePhysicalProperties = true;

            for (int i = 0; i < POINT_COUNT; i++)
            {
                Vector3 world_pos = src_world_array[i];
                Vector3 c1_vector3 = rgb_camera.WorldToScreenPoint(world_pos); 
                c1_point_array[i] = new Point(((c1_vector3.x * 2) - 128)/SCALE_FACTOR, (c1_vector3.y * 2)/SCALE_FACTOR);
            }
        }

        /// Sets the projected ScreenPoints of the world coordinate values in src_world_array 
        /// from the PoV of the Controller. 
        void SetControllerScreenPoints() {
            // Camera _camera = Camera.main;
            // rgb_camera.CopyFrom(_camera);
            rgb_camera.transform.position = ControllerTransform.position; 
            rgb_camera.transform.rotation = ControllerTransform.rotation;
            // rgb_camera.transform.eulerAngles = ControllerTransform.eulerAngles; 

            for (int i = 0; i < POINT_COUNT; i++)
            {
                Vector3 world_pos = src_world_array[i];
                Vector3 c2_vector3 = 
                    rgb_camera.WorldToScreenPoint(world_pos);

                hand_point_array[i] = new Point(c2_vector3.x/SCALE_FACTOR, c2_vector3.y/SCALE_FACTOR);
            }
        }

        /// Draws circles of radius 24/Scale_factor around the projected c1_point_array in [ref imageMat]
        void DrawC1ScreenPoints(ref Mat imageMat) {
            for (int i = 0; i < POINT_COUNT; i++)
            {
                Imgproc.circle(imageMat, c1_point_array[i], 24/SCALE_FACTOR, new Scalar(255, 255, 0));
            }
        }

        /// Displays [ref outMat] onto _previewObject as a texture. 
        void ShowMat(ref Mat outMat)
        {
            if (out_texture == null) {
                out_texture = new Texture2D(640, 360, TextureFormat.RGBA32, false);
            }
            
            // Debug.LogFormat("ShowMat Debug Info: \n outMat size: {0} \n out_texture size: {1} x {2}", outMat.size(), out_texture.width, out_texture.height);

            Utils.matToTexture2D(outMat, out_texture, false, 0);

            if(_previewObject.activeSelf == false)
            {
                _previewObject.SetActive(true);
                Renderer renderer = _previewObject.GetComponent<Renderer>();
                if(renderer != null)
                {
                    renderer.material.mainTexture = out_texture;
                }
                if (HAND_DISPLAY) {
                    _previewObject.transform.localScale = new Vector3(0.2f, 0.1f, 1);
                    _previewObject.transform.localPosition = new Vector3(0f, 0.13f, 0f);
                }
            }
        }

        /// (Helper to GetFaces()) Rectifies cached_initMat with corner points of [ref face_point_array] as source points. 
        /// reg_point_array is used as output points. Resulting rectified image stored as face [i] 
        /// in rectMat_array. 
        ///
        /// face_point_array should follow standards (like reg_point_array):
        /// [0] ----- [1]
        ///  |         |
        /// [2] ----- [3]
        void Rectify(ref Point[] face_point_array, int i) {
            rectMat_array[i, cap_i] = new Mat (360, 640, CvType.CV_8UC1);
            
            Point[] reg_point_array = new Point[4];
            reg_point_array[0] = new Point(0.0, HOMOGRAPHY_HEIGHT);
            reg_point_array[1] = new Point(HOMOGRAPHY_WIDTH, HOMOGRAPHY_HEIGHT);
            reg_point_array[2] = new Point(0.0, 0.0);
            reg_point_array[3] = new Point(HOMOGRAPHY_WIDTH, 0.0);
            
            MatOfPoint2f srcPoints = new MatOfPoint2f(face_point_array);
            MatOfPoint2f regPoints = new MatOfPoint2f(reg_point_array);

                // Debug.LogFormat("Rectify Face Points; {0} \n {1} \n {2} \n {3}", 
                    // face_point_array[0], face_point_array[1], face_point_array[2], face_point_array[3]);

            // Creating the H Matrix
            Mat Homo_Mat = Calib3d.findHomography(srcPoints, regPoints);

            Debug.LogFormat("Setting rectMat_array[{0}, {1}]", i, cap_i);

            Imgproc.warpPerspective(cached_initMat, rectMat_array[i, cap_i], 
                Homo_Mat, new Size(HOMOGRAPHY_WIDTH, HOMOGRAPHY_HEIGHT));
        }

        /// Rectifies and stores FACE_COUNT faces of object defined by [source_points] and stores into 
        /// rectMat_array. 
        void GetFaces(ref Point[] source_points) {
            for (int i = 0; i < FACE_COUNT; i++) { // i :: face count
                if (faceX_full[i]) { // For each valid face
                    Debug.LogFormat("Getting Face {0}", i);
                    // Build Face Point Array
                    Point[] face_point_array = new Point[4]; 
                    for (int j = 0; j < 4; j++) { // j :: face point count
                        int src_i = face_index[i, j];
                        face_point_array[j] = source_points[src_i];
                    }
                    // Rectify and get the face texture
                    Rectify(ref face_point_array, i);
                }
            }
        }

        /// Displays 3 stored faces of rectMat_array. 
        void ShowFaces() {
            if (faceX_full[0]) {
                Texture2D topTexture1 = new Texture2D(640, 360, TextureFormat.RGBA32, false);
                Utils.matToTexture2D(rectMat_array[0, cap_i], topTexture1, false, 0);
                m_TopImage1.texture = (Texture) topTexture1;
            }
            if (faceX_full[1]) {
                Texture2D topTexture2 = new Texture2D(640, 360, TextureFormat.RGBA32, false);
                Utils.matToTexture2D(rectMat_array[1, cap_i], topTexture2, false, 0);
                m_TopImage2.texture = (Texture) topTexture2;
            }
            if (faceX_full[2]) {
                Texture2D topTexture3 = new Texture2D(640, 360, TextureFormat.RGBA32, false);
                Utils.matToTexture2D(rectMat_array[2, cap_i], topTexture3, false, 0);
                m_TopImage3.texture = (Texture) topTexture3;
            }
        }

        // Combines 3 warped images in homoMat_array into one single image and stores into warped_Mat.
        void CombineWarped() {
            warpedMat = homoMat_array[0] + homoMat_array[1];
            warpedMat = homoMat_array[2] + warpedMat; 
            // Core.flip(warpedMat, warpedMat, 0);
        }

        /// Takes Rectified face [i] and capture index [c] of rectMat_array, and warps them into images specified by 
        /// [proj_point_array[i]]. 
        void HomographyTransform(int i, int c, ref Point[] proj_point_array) {
            // Init homography result Mat
            homoMat_array[i] = new Mat (360, 640, CvType.CV_8UC1);

            // Init regular point array
            Point[] reg_point_array = new Point[4]; 
            reg_point_array[0] = new Point(0.0, HOMOGRAPHY_HEIGHT);
            reg_point_array[1] = new Point(HOMOGRAPHY_WIDTH, HOMOGRAPHY_HEIGHT);
            reg_point_array[2] = new Point(0.0, 0.0);
            reg_point_array[3] = new Point(HOMOGRAPHY_WIDTH, 0.0);

            // Extract face_points corresponding with reg_points
            Point[] out_point_array = new Point[4]; 
                for (int j = 0; j < 4; j++) { // j :: face point count
                    int src_i = face_index[i, j];
                    out_point_array[j] = proj_point_array[src_i];
                }

            MatOfPoint2f regPoints = new MatOfPoint2f(reg_point_array);
            MatOfPoint2f outPoints = new MatOfPoint2f(out_point_array);

            Mat Homo_Mat = Calib3d.findHomography(regPoints, outPoints);

            Debug.LogFormat("Accessing rectMat_array[{0}, {1}]", i, c);

            Imgproc.warpPerspective(rectMat_array[i, c], homoMat_array[i], 
                Homo_Mat, new Size(HOMOGRAPHY_WIDTH, HOMOGRAPHY_HEIGHT));
        }
        #endregion

        #region Unity Methods
        /// Triggered on start by Unity. 
        /// 
        /// Currently just handles setting the main display. 
        void Awake()
        {
            // Main display handling
            if (HAND_DISPLAY) {
                _previewObject = _handObject; 
                _hudObject.SetActive(false);
            }
            else {
                _previewObject = _hudObject; 
                _handObject.SetActive(false);
            }
            _previewObject.SetActive(false);
        }

        /// Triggered on each frame by Unity. 
        ///<summary>
        /// If captured Mat is cached, obtains the controller's projected screenpoints and stores them in [hand_point_array].
        /// Then draws C2 Screen points onto [cached_initMat] to display them. Then each face's homography transform is 
        /// calculated and warped face textures are cropped from [cached_initMat] and stored in [homoMat_array]. These textures
        /// are then combined and displayed as a single image -- the controller's perspective. 
        ///</summary>
        ///
        void Update() 
        {
            // Checks have valid number of points
            if (cached_initMat == null)
                return; 

            // Takes world points and extracts c1 screen points (and displays them)
            if (world_idx >= POINT_COUNT) {
                SetControllerScreenPoints();
            }

            // STAGE III
            for (int i = 0; i < FACE_COUNT; i++) {
                if (faceX_full[i]) {
                    int closest_capture = GetClosestIndex();
                    // int closest_capture = 0; 
                    HomographyTransform(i, closest_capture, ref hand_point_array);
                }
            }
            CombineWarped();

            // Output cached_initMat
            ShowMat(ref warpedMat);
        }
        #endregion

        #region Event Handlers        
        /// <summary>
        /// Takes captured image in the form of [texture] and converts into and downsizes OpenCV Mat [cached_initMat]. 
        /// The Mat is desaturated/normalized, and then the C1 screen are detected and drawn. The rectified faces are 
        /// then extracted from [cached_initMat]. 
        /// </summary>
        /// <param name="texture">The new image that got captured.</param>
        public void OnImageCaptured(Texture2D texture)
        {
            // Convert Texture to Mat, downsize, and store as cached_initMat
            cached_bigMat = new Mat(1080, 2048, CvType.CV_8UC1);
            cached_initMat = new Mat(360, 640, CvType.CV_8UC1); 

            Utils.texture2DToMat(texture, cached_bigMat, true, 0);

            Imgproc.resize(cached_bigMat, cached_initMat, new Size(640, 360), 
                // 1.0/SCALE_FACTOR, 1.0/SCALE_FACTOR, 1);
                640.0/2048.0, 360.0/1080.0, 1); 
            
            // Mat Operations
            DesaturateMat(ref cached_initMat); 

            // Finds existing screen points
            SetC1ScreenPoints();
            DrawC1ScreenPoints(ref cached_initMat);

            // Faces Extracted (and displayed)
            GetFaces(ref c1_point_array);

            CacheCamPoints(); 

            // outMat = cached_initMat;
            // ShowMat(ref outMat);
        }

        /// <summary>
        /// Handles converting [frameData] from byte data into Unity Texture and gets frame pose, and
        /// stores into [camera_pose]. OnImageCaptured triggered by this function. 
        /// </summary>
        public void OnFrameCaptured(MLCameraResultExtras extras, YUVFrameInfo frameData, MLCameraFrameMetadata frameMetadata) {
            ulong vcamtimestamp = extras.VcamTimestampUs;  
            YUVBuffer yData = frameData.Y; 
            byte[] imageData = yData.Data; 

            Texture2D texture = new Texture2D((int) yData.Stride, (int) yData.Height, TextureFormat.R8, false);

            texture.LoadRawTextureData(imageData); 
            texture.Apply(); 

            MLCamera.GetFramePose(vcamtimestamp * 1000, out camera_pose); 

            OnImageCaptured(texture);
        }

        /// <summary>
        /// Handles world_point caching and checking if faces are captured yet. 
        /// </summary>
        public void OnWorldpointFound(Vector3 world_point) 
        {
            src_world_array[world_idx] = world_point;
            world_idx++;

            for (int i = 0; i < FACE_COUNT; i++) {
                faceX_full[i] = check_faces(i); 
            }
        }

        /// <summary>
        /// Handles video capture being started.
        /// </summary>
        public void OnCaptureStarted()
        {
            // Manage canvas visuals
            _recordingIndicator.SetActive(true);
        }

        /// <summary>
        /// Handles video capture ending.
        /// </summary>
        public void OnCaptureEnded()
        {
            // Manage canvas visuals
            _recordingIndicator.SetActive(false);
        }
        #endregion
    }
}
