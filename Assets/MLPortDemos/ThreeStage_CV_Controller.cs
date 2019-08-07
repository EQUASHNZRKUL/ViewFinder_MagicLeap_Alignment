// using System;
// using System.Collections.Generic;
// using Unity.Collections;
// using Unity.Collections.LowLevel.Unsafe;
// using UnityEngine;
// using UnityEngine.UI;
// using UnityEngine.XR.ARFoundation;
// using UnityEngine.XR.ARSubsystems;

// using OpenCVForUnity;
// using OpenCVForUnity.CoreModule;
// using OpenCVForUnity.UnityUtils;
// using OpenCVForUnity.ImgprocModule;
// using OpenCVForUnity.Features2dModule;
// using OpenCVForUnity.Calib3dModule;
// using OpenCVForUnity.Xfeatures2dModule;
// using OpenCVForUnity.ArucoModule;

// /// <summary>
// /// Listens for touch events and performs an AR raycast from the screen touch point.
// /// AR raycasts will only hit detected trackables like feature points and planes.
// ///
// /// If a raycast hits a trackable, the <see cref="placedPrefab"/> is instantiated
// /// and moved to the hit position.
// /// </summary>
// public class ThreeStage_CV_Controller : MonoBehaviour
// {
//     public static double THRESH_VAL = 150.0;
//     public static int K_ITERATIONS = 10;
//     public static double HOMOGRAPHY_WIDTH = 640.0;
//     public static double HOMOGRAPHY_HEIGHT = 480.0;

//     // CV Mats
//     public Mat imageMat = new Mat(480, 640, CvType.CV_8UC1);
//     public Mat outMat = new Mat(480, 640, CvType.CV_8UC1);
//     private Mat cached_initMat = new Mat (480, 640, CvType.CV_8UC1);
//     private List<Mat> corners = new List<Mat>();
//     private Mat ids = new Mat(480, 640, CvType.CV_8UC1);
//     private Mat[] rectMat_array = new Mat[3];
//     private Mat[] homoMat_array = new Mat[3];

//     // Face corner indices for each face
//     private int[,] face_index = { {3, 6, 4, 5}, {0, 1, 3, 6}, {6, 1, 5, 2} };

//     // Populated booleans
//     public bool spa_full = false; 
//     private bool[] faceX_full = new bool[3]; 
//     private bool[] faceX_recent_full = new bool[3]; 

//     // Point Arrays
//     private Point[] src_point_array = new Point[7];
//     private Point[] src_recent_array = new Point[7];
//     private Point[] reg_point_array = new Point[4];
//     private Point[] proj_point_array = new Point[7];

//     private ScreenOrientation? m_CachedOrientation = null;
//     private Texture2D m_Texture;

//     [SerializeField]
//     ARCameraManager m_ARCameraManager;
//     public ARCameraManager cameraManager
//     {
//         get {return m_ARCameraManager; }
//         set {m_ARCameraManager = value; }
//     }

//     [SerializeField]
//     RawImage m_RawImage;
//     public RawImage rawImage 
//     {
//         get { return m_RawImage; }
//         set { m_RawImage = value; }
//     }

//     [SerializeField]
//     RawImage m_TopImage1;
//     public RawImage topImage1 
//     {
//         get { return m_TopImage1; }
//         set { m_TopImage1 = value; }
//     }

//     [SerializeField]
//     RawImage m_TopImage2;
//     public RawImage topImage2 
//     {
//         get { return m_TopImage2; }
//         set { m_TopImage2 = value; }
//     }

//     [SerializeField]
//     RawImage m_TopImage3;
//     public RawImage topImage3 
//     {
//         get { return m_TopImage3; }
//         set { m_TopImage3 = value; }
//     }

//     // private RawImage[] topImage_array = {m_TopImage1, m_TopImage2, m_TopImage3};

//     [SerializeField]
//     Text m_ImageInfo;
//     public Text imageInfo
//     {
//         get { return m_ImageInfo; }
//         set { m_ImageInfo = value; }
//     }

//     [SerializeField]
//     ARSessionOrigin m_ARSessionManager;
//     public ARSessionOrigin sessionManager
//     {
//         get { return m_ARSessionManager; }
//         set { m_ARSessionManager = value; }
//     }

//     void Awake()
//     {
//         Debug.Log("StartTest");
//         Screen.autorotateToLandscapeLeft = true; 
//         m_ARRaycastManager = GetComponent<ARRaycastManager>();
//     }

//     void OnEnable()
//     {
//         if (m_ARCameraManager != null)
//         {
//             m_ARCameraManager.frameReceived += OnCameraFrameReceived;
//         }
//     }

//     void OnDisable()
//     {
//         if (m_ARCameraManager != null)
//             m_ARCameraManager.frameReceived -= OnCameraFrameReceived;
//     }

//     float CameraToPixelX(double x) { return (float) (3.4375 * x); }

//     float CameraToPixelY(double y) { return (float) (1080.0 - (3.375*(y - 80.0))); }

//     int arucoTosrc(int a) {
//         if (a == 7) { return 4; }
//         else if (a == 6) { return 5; }
//         else if (a == 10) { return 6; }
//         else {return a; }
//     }

//     int srcToarcuo(int s) {
//         if (s == 4) { return 7; }
//         else if (s == 5) {return 6; }
//         else if (s == 6) {return 10; }
//         else {return s; }
//     }

//     int count_src_nulls() {
//         int acc = 0;
//         for (int i = 0; i < 7; i++)
//         {
//             if (src_point_array[i] == null) {
//                 acc++; 
//             }
//         }
//         return (7 - acc); 
//     }
    
//     bool check_faces(int face_i) {
//         for (int i = 0; i < 4; i++) {
//             int src_i = face_index[face_i, i]; 
//             if (src_point_array[src_i] == null) {
//                 return false; 
//             }
//         }
//         return true; 
//     }

//     bool check_recent_faces(int face_i) {
//         for (int i = 0; i < 4; i++) {
//             int src_i = face_index[face_i, i]; 
//             if (src_recent_array[src_i] == null) {
//                 return false; 
//             }
//         }
//         return true; 
//     }

//     public Point[] GetC1Points() { return src_point_array; }

//     public Point[] GetRecentC1Points() { return src_recent_array; }

//     void ConfigureRawImageInSpace(Vector2 img_dim)
//     {
//         Vector2 ScreenDimension = new Vector2(Screen.width, Screen.height);
//         int scr_w = Screen.width;
//         int scr_h = Screen.height; 

//         float img_w = img_dim.x;
//         float img_h = img_dim.y;

//         float w_ratio = (float)scr_w/img_w;
//         float h_ratio = (float)scr_h/img_h;
//         float scale = Math.Max(w_ratio, h_ratio);

//         m_RawImage.SetNativeSize();
//         m_RawImage.transform.position = new Vector3(scr_w/4, scr_h/4, 0.0f);
//         m_RawImage.transform.localScale = new Vector3(scale/4, scale/4, 0.0f);
//         // m_RawImage.transform.position = new Vector3(scr_w/2, scr_h/2, 0.0f);
//         // m_RawImage.transform.localScale = new Vector3(scale, scale, 0.0f);

//         m_TopImage1.SetNativeSize();
//         m_TopImage1.transform.position = new Vector3(5*scr_w/6, scr_h/6, 0.0f);
//         m_TopImage1.transform.localScale = new Vector3(scale/6, scale/6, 0.0f);

//         m_TopImage2.SetNativeSize();
//         m_TopImage2.transform.position = new Vector3(5*scr_w/6, scr_h/2, 0.0f);
//         m_TopImage2.transform.localScale = new Vector3(scale/6, scale/6, 0.0f);

//         m_TopImage3.SetNativeSize();
//         m_TopImage3.transform.position = new Vector3(5*scr_w/6, 5*scr_h/6, 0.0f);
//         m_TopImage3.transform.localScale = new Vector3(scale/6, scale/6, 0.0f);
//     }

//     void ArucoDetection() {
//         Dictionary dict = Aruco.getPredefinedDictionary(Aruco.DICT_4X4_1000);
//         Aruco.detectMarkers(cached_initMat, dict, corners, ids);
//         Aruco.drawDetectedMarkers(cached_initMat, corners, ids);
//         src_recent_array = new Point[7];

//         int markerCount = count_src_nulls();

//         for (int i = 0; i < corners.Count; i++) {
//             int aruco_id = (int) (ids.get(i, 0)[0]);
//             int src_i = arucoTosrc(aruco_id);
//             int corner_i = aruco_id % 4;

//             // Store corner[i] into spa[src_i]
//             src_point_array[src_i] = new Point(corners[i].get(0, corner_i)[0], corners[i].get(0, corner_i)[1]);
//             src_recent_array[src_i] = new Point(corners[i].get(0, corner_i)[0], corners[i].get(0, corner_i)[1]);

//             // Display the corner as circle on outMat. 
//             // Imgproc.circle(cached_initMat, src_point_array[src_i], 5, new Scalar(255, 0, 255));
//         }

//         // Count non-null source points 
//         spa_full = (markerCount == 7);

//         // Check if have valid faces
//         for (int i = 0; i < 3; i++) {
//             // faceX_full[i] = check_faces(i); 
//             faceX_full[i] = check_faces(i); 
//         }

//         for (int i = 0; i < 3; i++) {
//             // faceX_full[i] = check_faces(i); 
//             faceX_recent_full[i] = check_recent_faces(i); 
//         }
//     }

//     void Rectify(ref Point[] face_point_array, int i) {
//         rectMat_array[i] = new Mat (480, 640, CvType.CV_8UC1);
        
//         reg_point_array[0] = new Point(0.0, HOMOGRAPHY_HEIGHT);
//         reg_point_array[1] = new Point(HOMOGRAPHY_WIDTH, HOMOGRAPHY_HEIGHT);
//         reg_point_array[2] = new Point(0.0, 0.0);
//         reg_point_array[3] = new Point(HOMOGRAPHY_WIDTH, 0.0);
        
//         MatOfPoint2f srcPoints = new MatOfPoint2f(face_point_array);
//         MatOfPoint2f regPoints = new MatOfPoint2f(reg_point_array);

//             Debug.LogFormat("Rectify Face Points; {0} \n {1} \n {2} \n {3}", 
//                 face_point_array[0], face_point_array[1], face_point_array[2], face_point_array[3]);

//         // Creating the H Matrix
//         Mat Homo_Mat = Calib3d.findHomography(srcPoints, regPoints);

//         Imgproc.warpPerspective(cached_initMat, rectMat_array[i], Homo_Mat, new Size(HOMOGRAPHY_WIDTH, HOMOGRAPHY_HEIGHT));
//     }

//     void GetFaces(ref Point[] source_points) {
//         for (int i = 0; i < 3; i++) { // i :: face count
//             if (faceX_full[i]) { // For each valid face
//                 // Build Face Point Array
//                 Point[] face_point_array = new Point[4]; 
//                 for (int j = 0; j < 4; j++) { // j :: face point count
//                     int src_i = face_index[i, j];
//                     face_point_array[j] = source_points[src_i];
//                 }
//                 // Rectify and get the face texture
//                 Rectify(ref face_point_array, i);
//             }
//         }
//     }

//     void ShowFaces(Vector2 img_dim) {
//         int scr_w = Screen.width;
//         int scr_h = Screen.height; 

//         float img_w = img_dim.x;
//         float img_h = img_dim.y;

//         float w_ratio = (float)scr_w/img_w;
//         float h_ratio = (float)scr_h/img_h;
//         float scale = Math.Max(w_ratio, h_ratio);

//         if (faceX_full[0]) {
//             Texture2D topTexture1 = new Texture2D((int) img_dim.x, (int) img_dim.y, TextureFormat.RGBA32, false);
//             Utils.matToTexture2D(rectMat_array[0], topTexture1, false, 0);
//             m_TopImage1.texture = (Texture) topTexture1;

//             m_TopImage1.SetNativeSize();
//             m_TopImage1.transform.position = new Vector3(5*scr_w/6, scr_h/6, 0.0f);
//             m_TopImage1.transform.localScale = new Vector3(scale/6, scale/6, 0.0f);
//         }
//         if (faceX_full[1]) {
//             Texture2D topTexture2 = new Texture2D((int) img_dim.x, (int) img_dim.y, TextureFormat.RGBA32, false);
//             Utils.matToTexture2D(rectMat_array[1], topTexture2, false, 0);
//             m_TopImage2.texture = (Texture) topTexture2;

//             m_TopImage2.SetNativeSize();
//             m_TopImage2.transform.position = new Vector3(5*scr_w/6, scr_h/2, 0.0f);
//             m_TopImage2.transform.localScale = new Vector3(scale/6, scale/6, 0.0f);
//         }
//         if (faceX_full[2]) {
//             Texture2D topTexture3 = new Texture2D((int) img_dim.x, (int) img_dim.y, TextureFormat.RGBA32, false);
//             Utils.matToTexture2D(rectMat_array[2], topTexture3, false, 0);
//             m_TopImage3.texture = (Texture) topTexture3;

//             m_TopImage3.SetNativeSize();
//             m_TopImage3.transform.position = new Vector3(5*scr_w/6, 5*scr_h/6, 0.0f);
//             m_TopImage3.transform.localScale = new Vector3(scale/6, scale/6, 0.0f);
//         }
//     }

//     void DrawScreenPoints(ThreeStage_AR_Controller ARC) {
//         Point[] c2_scrpoints = ARC.GetScreenpoints();

//         for (int i = 0; i < 7; i++) {
//             Imgproc.circle(cached_initMat, c2_scrpoints[i], 10, new Scalar(255, 255, 0));
//         }
//     }

//     void HomographyTransform(int i) {
//         // Init homography result Mat
//         homoMat_array[i] = new Mat (480, 640, CvType.CV_8UC1);

//         // Init regular point array
//         reg_point_array[0] = new Point(0.0, HOMOGRAPHY_HEIGHT);
//         reg_point_array[1] = new Point(HOMOGRAPHY_WIDTH, HOMOGRAPHY_HEIGHT);
//         reg_point_array[2] = new Point(0.0, 0.0);
//         reg_point_array[3] = new Point(HOMOGRAPHY_WIDTH, 0.0);

//         // Extract face_points corresponding with reg_points
//         Point[] out_point_array = new Point[4]; 
//             for (int j = 0; j < 4; j++) { // j :: face point count
//                 int src_i = face_index[i, j];
//                 out_point_array[j] = proj_point_array[src_i];
//             }

//         MatOfPoint2f regPoints = new MatOfPoint2f(reg_point_array);
//         MatOfPoint2f outPoints = new MatOfPoint2f(out_point_array);

//         Mat Homo_Mat = Calib3d.findHomography(regPoints, outPoints);

//         Imgproc.warpPerspective(rectMat_array[i], homoMat_array[i], Homo_Mat, new Size(HOMOGRAPHY_WIDTH, HOMOGRAPHY_HEIGHT));
//     }

//     void CombineWarped() {
//         outMat = homoMat_array[0] + homoMat_array[1];
//         outMat = homoMat_array[2] + outMat; 
//         Core.flip(outMat, outMat, 0);
//     }

//     void OnCameraFrameReceived(ARCameraFrameEventArgs eventArgs)
//     {
//         // Camera data extraction
//         XRCameraImage image;
//         if (!cameraManager.TryGetLatestImage(out image))
//         {
//             Debug.Log("Uh Oh");
//             return;
//         }

//         Vector2 img_dim = image.dimensions;
//         XRCameraImagePlane greyscale = image.GetPlane(0);

//         // Instantiates new m_Texture if necessary
//         if (m_Texture == null || m_Texture.width != image.width)
//         {
//             var format = TextureFormat.RGBA32;
//             m_Texture = new Texture2D(image.width, image.height, format, false);
//         }

//         image.Dispose();

//         ThreeStage_AR_Controller ARC = m_ARSessionManager.GetComponent<ThreeStage_AR_Controller>();

//         // Process the image here: 
//         unsafe {
//             IntPtr greyPtr = (IntPtr) greyscale.data.GetUnsafePtr();

//             // TOUCH: Detect corners and set as source points
//             if (Input.touchCount > 0)
//             {
//                 Touch touch = Input.GetTouch(0);
//                 if (touch.phase == TouchPhase.Began)
//                 {
//                     // Cache original image
//                     Utils.copyToMat(greyPtr, cached_initMat);

//                     if (!spa_full) { // Stage 1: Finding World Markers
//                         m_ImageInfo.text = string.Format("Number of markers detected: {0} \n world_nulls {1}", 
//                             count_src_nulls(), ARC.count_world_nulls());
//                         ArucoDetection();

//                         ARC.SetWorldPoints();
//                         ARC.SetScreenPoints();
//                         DrawScreenPoints(ARC);
//                     }
//                     else { // Stage 2: Rectification of Captured Image Faces
//                         m_ImageInfo.text = String.Format("world_nulls: {0}", ARC.count_world_nulls());
//                         ARC.SetScreenPoints();
//                         DrawScreenPoints(ARC);

//                         proj_point_array = ARC.GetScreenpoints();

//                         GetFaces(ref proj_point_array);
//                         ShowFaces(img_dim);
//                     }
                    
//                     Core.flip(cached_initMat, outMat, 0);
//                 }
//             }

//             // Displays OpenCV Mat as a Texture
//             Utils.matToTexture2D(outMat, m_Texture, false, 0);
//         }

//         if (spa_full) { // Stage 3: Real-time warping
//             ARC.SetScreenPoints();
//             proj_point_array = ARC.GetScreenpoints();

//             for (int i = 0; i < 3; i++) {
//                 m_ImageInfo.text = String.Format("Stage 3: {0}", i);
//                 HomographyTransform(i);
//             }

//             CombineWarped();

//             Utils.matToTexture2D(outMat, m_Texture, false, 0);
//         }

//         // Sets orientation of screen if necessary
//         if (m_CachedOrientation == null || m_CachedOrientation != Screen.orientation)
//         {
//             // TODO: Debug why doesn't initiate with ConfigRawimage(). The null isn't triggering here. Print cached Orientation
//             m_CachedOrientation = Screen.orientation;
//             ConfigureRawImageInSpace(img_dim);
//         }

//         m_RawImage.texture = (Texture) m_Texture;

//         // m_ImageInfo.text = string.Format("Number of Blobs: {0}", ids.size());
//     }

//     static List<ARRaycastHit> s_Hits = new List<ARRaycastHit>();

//     ARRaycastManager m_ARRaycastManager;

//     ARSessionOrigin m_SessionOrigin;
// }