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
    public class CVController : MonoBehaviour
    {
        #region Public Variables
        [SerializeField, Tooltip("Object to set new images on.")]
        private RawImage m_TopImage1 = null;

        [SerializeField, Tooltip("Object to set new images on.")]
        private RawImage m_TopImage2 = null;

        [SerializeField, Tooltip("Object to set new images on.")]
        private RawImage m_TopImage3 = null;

        [SerializeField, Tooltip("Object to set new images on.")]
        private RawImage m_OutImage = null;

        [Tooltip("Reference to the controller object's transform.")]
        public Transform ControllerTransform;
        #endregion

        #region Private Variables
        [SerializeField, Tooltip("Object to set new images on.")]
        private GameObject _previewObject = null;

        [SerializeField, Tooltip("The renderer to show the video capture on")]
        private Renderer _screenRenderer = null;

        // Constants
        private static int POINT_COUNT = 7; 
        private static int FACE_COUNT = 3; 
        private static float X_OFFSET = -1.0f; 
        // private static float SCALE_FACTOR = 2.0f;
        public static double HOMOGRAPHY_WIDTH = 640.0;
        public static double HOMOGRAPHY_HEIGHT = 360.0;
        public static int RECT_SIZE = 400; 
        public static int SCALE_FACTOR = 3; 

        // Mats
        public Mat outMat = new Mat(360, 640, CvType.CV_8UC1);
        private Mat cached_bigMat = new Mat (1080, 1920, CvType.CV_8UC1);
        // private Mat cached_initMat = new Mat (360, 640, CvType.CV_8UC1);
        private Mat cached_initMat = null; 
        private Mat warpedMat = new Mat (360, 640, CvType.CV_8UC1);
        private Mat ids = new Mat(360, 640, CvType.CV_8UC1);
        private List<Mat> corners = new List<Mat>();

        private Mat[] rectMat_array = new Mat[3];
        private Mat[] homoMat_array = new Mat[3];

        private Matrix4x4 camera_pose; 

        // Face corner indices for each face
        private int[,] face_index = { {3, 6, 4, 5}, {0, 1, 3, 6}, {6, 1, 5, 2} };

        // Point Lists
        private Vector3[] src_ray_array = new Vector3[POINT_COUNT];
        private Vector3[] src_world_array = new Vector3[POINT_COUNT];
        private Point[] c1_point_array = new Point[POINT_COUNT];
        private Point[] hand_point_array = new Point[POINT_COUNT];

        // Index
        private int world_idx; 

        // Face Lists
        private bool[] faceX_full = new bool[3];

        // Textures
        private Texture2D rawVideoTexture; 
        private Texture2D out_texture;

        [SerializeField]
        [Tooltip("Instantiates this prefab on a gameObject at the touch location.")]
        Camera m_deviceCamera;
        public Camera rgb_camera
        {
            get { return m_deviceCamera; }
            set { m_deviceCamera = value; }
        }

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

        // TODO: More sophisticated implementation
        void SetC2ScreenPoints() { 
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

        void DrawC2ScreenPoints(ref Mat imageMat) {
            for (int i = 0; i < POINT_COUNT; i++)
            {
                Imgproc.circle(imageMat, c1_point_array[i], 24/SCALE_FACTOR, new Scalar(255, 255, 0));
            }
        }

        void ShowMat(ref Mat outMat)
        {
            if (out_texture == null) {
                out_texture = new Texture2D(640, 360, TextureFormat.RGBA32, false);
            }
            
            // Debug.LogFormat("ShowMat Debug Info: \n outMat size: {0} \n out_texture size: {1} x {2}", outMat.size(), out_texture.width, out_texture.height);

            Utils.matToTexture2D(outMat, out_texture, false, 0);

            if(_previewObject != null)
            {
                _previewObject.SetActive(true);
                Renderer renderer = _previewObject.GetComponent<Renderer>();
                if(renderer != null)
                {
                    renderer.material.mainTexture = out_texture;
                }
                // _previewObject.transform.localScale = _previewObject.transform.localScale * 4;
                // _previewObject.transform.localScale = new Vector3(1.2f, 0.8f, 1);
            }
        }

        void Rectify(ref Point[] face_point_array, int i) {
            rectMat_array[i] = new Mat (360, 640, CvType.CV_8UC1);
            
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

            Imgproc.warpPerspective(cached_initMat, rectMat_array[i], Homo_Mat, new Size(HOMOGRAPHY_WIDTH, HOMOGRAPHY_HEIGHT));
        }

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

        void ShowFaces() {
            if (faceX_full[0]) {
                Texture2D topTexture1 = new Texture2D(640, 360, TextureFormat.RGBA32, false);
                Utils.matToTexture2D(rectMat_array[0], topTexture1, false, 0);
                m_TopImage1.texture = (Texture) topTexture1;
            }
            if (faceX_full[1]) {
                Texture2D topTexture2 = new Texture2D(640, 360, TextureFormat.RGBA32, false);
                Utils.matToTexture2D(rectMat_array[1], topTexture2, false, 0);
                m_TopImage2.texture = (Texture) topTexture2;
            }
            if (faceX_full[2]) {
                Texture2D topTexture3 = new Texture2D(640, 360, TextureFormat.RGBA32, false);
                Utils.matToTexture2D(rectMat_array[2], topTexture3, false, 0);
                m_TopImage3.texture = (Texture) topTexture3;
            }
        }

        void CombineWarped() {
            warpedMat = homoMat_array[0] + homoMat_array[1];
            warpedMat = homoMat_array[2] + warpedMat; 
            // Core.flip(warpedMat, warpedMat, 0);
        }

        void HomographyTransform(int i, ref Point[] proj_point_array) {
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

            Imgproc.warpPerspective(rectMat_array[i], homoMat_array[i], Homo_Mat, new Size(HOMOGRAPHY_WIDTH, HOMOGRAPHY_HEIGHT));
        }
        #endregion

        #region Unity Methods
        void Awake()
        {
            // _camera = Camera.main; 
            _previewObject.SetActive(false);
            // UnityEngine.Rect new_rect = new UnityEngine.Rect(0, 0, 1920, 1080);
            // rgb_camera.pixelRect = new_rect;
        }

        void Start()
        {
            // UnityEngine.Rect new_rect = new UnityEngine.Rect(0, 0, 1920, 1080);
            // rgb_camera.pixelRect = new_rect;
            // Debug.LogFormat("In Start: Rect -- {0} vs. {1}", rgb_camera.pixelRect, new_rect);
        }

        void Update() 
        {
            // Checks have valid number of points
            if (cached_initMat == null)
                return; 

            // Takes world points and extracts c2 screen points (and displays them)
            if (world_idx >= POINT_COUNT) {
                SetControllerScreenPoints();
                DrawC2ScreenPoints(ref cached_initMat);
            }

            // STAGE III
            for (int i = 0; i < FACE_COUNT; i++) {
                if (faceX_full[i])
                    HomographyTransform(i, ref hand_point_array);
            }
            CombineWarped();

            // Output cached_initMat
            // ShowMat(ref warpedMat);
        }
        #endregion

        #region Event Handlers        
        /// <summary>
        /// Updates preview object with new captured image
        /// </summary>
        /// <param name="texture">The new image that got captured.</param>
        public void OnImageCaptured(Texture2D texture)
        {
            Debug.LogFormat("395 - {0} x {1}", texture.width, texture.height);
            // Convert Texture to Mat and store as cached_initMat
            cached_bigMat = new Mat(1080, 2048, CvType.CV_8UC1);
            cached_initMat = new Mat(360, 640, CvType.CV_8UC1); 

            Debug.Log("400 - Converting to Mat");
            Utils.texture2DToMat(texture, cached_bigMat, true, 0);
            Debug.LogFormat("Texture Dimensions: {0} -- {1} x {2}", 
                cached_bigMat.size(), texture.width, texture.height);

            Imgproc.resize(cached_bigMat, cached_initMat, new Size(640, 360), 
                // 1.0/SCALE_FACTOR, 1.0/SCALE_FACTOR, 1);
                640.0/2048.0, 360.0/1080.0, 1); 
            
            Debug.LogFormat("404 - resized Mats -- Matsize : {0} x {1}", 
                cached_initMat.width(), cached_initMat.height());
            ShowMat(ref cached_initMat); 

            // Finds existing screen points
            SetC2ScreenPoints();
            DrawC2ScreenPoints(ref cached_initMat);

            GetFaces(ref c1_point_array);
            // ShowFaces();

            // Debug.Log("Showing Mat");
            outMat = cached_initMat;
            ShowMat(ref outMat);
        }

        public void OnFrameCaptured(MLCameraResultExtras extras, YUVFrameInfo frameData, MLCameraFrameMetadata frameMetadata) {
            Debug.LogFormat("Entered OFC -- {0}", frameData.Y.Data.Length); 
            ulong vcamtimestamp = extras.VcamTimestampUs;  
            YUVBuffer yData = frameData.Y; 
            byte[] imageData = yData.Data; 

            Texture2D texture = new Texture2D((int) yData.Stride, (int) yData.Height, TextureFormat.R8, false);
            // texture.filterMode = FilterMode.Point; 
            Debug.LogFormat("Created Texture from imageData: {0}x{1}", yData.Stride, yData.Height); 

            // ProcessImage(yData.Data, 4); 
            texture.LoadRawTextureData(imageData); 
            texture.Apply(); 

            MLCamera.GetFramePose(vcamtimestamp * 1000, out camera_pose); 

            OnImageCaptured(texture);
        }

        public void OnWorldpointFound(Vector3 world_point) 
        {
            src_world_array[world_idx] = world_point;
            world_idx++;

            for (int i = 0; i < FACE_COUNT; i++) {
                faceX_full[i] = check_faces(i); 
            }

        }
        #endregion
    }
}
