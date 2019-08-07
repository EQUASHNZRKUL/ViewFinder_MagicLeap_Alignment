// using System;
// using System.Collections.Generic;
// using Unity.Collections;
// using Unity.Collections.LowLevel.Unsafe;
// using UnityEngine;
// using UnityEngine.UI;
// using UnityEngine.XR.ARFoundation;
// using UnityEngine.XR.ARSubsystems;
// using OpenCVForUnity.CoreModule;

// [RequireComponent(typeof(ARRaycastManager))]
// public class ThreeStage_AR_Controller : MonoBehaviour
// {
//     [SerializeField]
//     ARCameraManager m_ARCameraManager;
//     public ARCameraManager cameraManager
//     {
//         get {return m_ARCameraManager; }
//         set {m_ARCameraManager = value; }
//     }

//     [SerializeField]
//     [Tooltip("Instantiates this prefab on a gameObject at the touch location.")]
//     GameObject m_PlacedPrefab;
//     public GameObject placedPrefab
//     {
//         get { return m_PlacedPrefab; }
//         set { m_PlacedPrefab = value; }
//     }

//     public GameObject spawnedObject { get; private set; }
    
//     [SerializeField]
//     GameObject m_CvControllerObject;
//     public GameObject CV_Controller_Object 
//     {
//         get { return m_CvControllerObject; }
//         set { m_CvControllerObject = value; } 
//     } 

//     private ThreeStage_CV_Controller m_cv;
//     public static float DATA_SCALE = 0.05f;
//     private TrackableId cached_trackableid;

//     private Vector3[] world_points = new Vector3[7];

//     private Point[] c1_scr_points = new Point[7];
//     private Point[] c2_scr_points = new Point[7];

//     private GameObject[] spawnedObjects = new GameObject[7];

//     public Point[] GetScreenpoints() { return c2_scr_points; }

//     void Awake()
//     {
//         Debug.Log("StartTest");
//         m_ARRaycastManager = GetComponent<ARRaycastManager>();
//         m_SessionOrigin = GetComponent<ARSessionOrigin>();
//         m_cv = CV_Controller_Object.GetComponent<ThreeStage_CV_Controller>();
//         spawnedObjects[0] = Instantiate(m_PlacedPrefab, new Vector3(0.0f, 0.0f, 0.0f), new Quaternion(0.0f, 0.0f, 0.0f, 0.0f));
//         spawnedObjects[1] = Instantiate(m_PlacedPrefab, new Vector3(0.0f, 0.0f, 0.0f), new Quaternion(0.0f, 0.0f, 0.0f, 0.0f));
//         spawnedObjects[2] = Instantiate(m_PlacedPrefab, new Vector3(0.0f, 0.0f, 0.0f), new Quaternion(0.0f, 0.0f, 0.0f, 0.0f));
//         spawnedObjects[3] = Instantiate(m_PlacedPrefab, new Vector3(0.0f, 0.0f, 0.0f), new Quaternion(0.0f, 0.0f, 0.0f, 0.0f));
//         spawnedObjects[4] = Instantiate(m_PlacedPrefab, new Vector3(0.0f, 0.0f, 0.0f), new Quaternion(0.0f, 0.0f, 0.0f, 0.0f));
//         spawnedObjects[5] = Instantiate(m_PlacedPrefab, new Vector3(0.0f, 0.0f, 0.0f), new Quaternion(0.0f, 0.0f, 0.0f, 0.0f));
//         spawnedObjects[6] = Instantiate(m_PlacedPrefab, new Vector3(0.0f, 0.0f, 0.0f), new Quaternion(0.0f, 0.0f, 0.0f, 0.0f));
//     }

//     int count_c1_nulls() {
//         int acc = 0;
//         for (int i = 0; i < 7; i++)
//         {
//             if (c1_scr_points[i] == null) {
//                 acc++; 
//             }
//         }
//         return (7 - acc); 
//     }

//     public int count_world_nulls() {
//         int acc = 0; 
//         for (int i = 0; i < 7; i++)
//         {
//             if (world_points[i] == null) {
//                 acc++; 
//             }
//         }
//         return acc; 
//     }

//     public bool WorldFull() { return (count_world_nulls() == 0); }

//     float PixelToCameraX(double x) { return (float) ((640.0/2200.0) * x); }

//     float PixelToCameraY(double y) { return (float) ((320.0/1080.0)*(1080.0 - y) + 80.0); }

//     float CameraToPixelX(double x) { return (float) (3.4375 * x); }

//     float CameraToPixelY(double y) { return (float) (1080.0 - (3.375*(y - 80.0))); }

//     public void SetWorldPoints()
//     {
//         Debug.Log("SWP: 119");
//         ThreeStage_CV_Controller CV_Controller = GameObject.Find("CV_Controller").GetComponent<ThreeStage_CV_Controller>();
//         c1_scr_points = CV_Controller.GetRecentC1Points();

//         Debug.Log("SWP: 123");
//         for (int i = 0; i < 7; i++) {
//             if (c1_scr_points[i] != null) {
//                 Debug.LogFormat("SWP: Entering scr_rec[{0}]", i);
//                 Vector2 screen_vec = 
//                     new Vector2(CameraToPixelX(c1_scr_points[i].x), CameraToPixelY(c1_scr_points[i].y));
//                 bool arRayBool = m_ARRaycastManager.Raycast(screen_vec, s_Hits, TrackableType.PlaneWithinPolygon);
//                 if (arRayBool) {
//                     world_points[i] = s_Hits[0].pose.position; 
//                     spawnedObjects[i].transform.position = world_points[i];
//                 }
//             }
//         }
//     }

//     // Sets the C2 screen point values from world points
//     public void SetScreenPoints()
//     {
//         Camera cam = GameObject.Find("AR Camera").GetComponent<Camera>();

//         for (int i = 0; i < 7; i++)
//         {
//             if (world_points[i] != null) {
//                 Vector3 scr_point = cam.WorldToScreenPoint(world_points[i]);
//                 c2_scr_points[i] = new Point(PixelToCameraX(scr_point.x), PixelToCameraY(scr_point.y));
//             }
//         }
//     }

//     void Update()
//     {
//         // TOUCH SECTION
//         // if (Input.touchCount > 0)
//         // {
//         //     Touch touch = Input.GetTouch(0);
//         //     if (touch.phase == TouchPhase.Began)
//         //     {
//         //         // Cache worldpoints 
//         //         SetWorldPoints(); 
//         //         Debug.LogFormat("c1 count: {0} -- world count: {1} ", count_c1_nulls(), count_world_nulls());
//         //         SetScreenPoints(); 
//         //     }
//         // }

//         // FRAME SECTION
//         // SetScreenPoints();
//     }

//     static List<ARRaycastHit> s_Hits = new List<ARRaycastHit>();
//     static List<ARRaycastHit> e_Hits = new List<ARRaycastHit>();

//     ARRaycastManager m_ARRaycastManager;
//     ARSessionOrigin m_SessionOrigin;
//     ARRaycastHit face;
// }