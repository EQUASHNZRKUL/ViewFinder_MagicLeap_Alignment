using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.MagicLeap;
using Unity.Collections;

namespace MagicLeap
{
    public class ControllerBallSpawner : MonoBehaviour
    {
        #region Private Variables
        [SerializeField]
        [Tooltip("Instantiates this prefab on a gameObject at the touch location.")]
        GameObject m_PlacedPrefab;
        public GameObject placedPrefab
        {
            get { return m_PlacedPrefab; }
            set { m_PlacedPrefab = value; }
        }

        [SerializeField]
        [Tooltip("Instantiates this prefab on a gameObject at the touch location.")]
        GameObject m_CursorPrefab;
        public GameObject cursorPrefab
        {
            get { return m_CursorPrefab; }
            set { m_CursorPrefab = value; }
        }

        [SerializeField]
        Text m_InstructionText;
        public Text text
        {
            get { return m_InstructionText; }
            set { m_InstructionText = value; }
        }

        [Space, SerializeField, Tooltip("ControllerConnectionHandler reference.")]
        private ControllerConnectionHandler _controllerConnectionHandler = null;

        public GameObject cursorObject { get; private set; }
        private GameObject[] spawnedObjects = new GameObject[7];

        // current index of spawnedObjects ready to spawn
        private int idx = 0; 

        private Camera _camera; 

        #endregion

        #region Unity Methods
        void Awake()
        {
            MLInput.OnControllerButtonDown += OnButtonDown; 
            MagicLeapDevice.RegisterOnHeadTrackingMapEvent(OnHeadTrackingMapEvent);
            cursorObject = Instantiate(m_PlacedPrefab, new Vector3(0.0f, 0.0f, 0.0f), new Quaternion(0.0f, 0.0f, 0.0f, 0.0f));
            _camera = Camera.main; 
        }

        void Start() {

        }

        void Update()
        {
            UpdateMarker(); 
        }

        void OnDestroy()
        {
            MagicLeapDevice.UnregisterOnHeadTrackingMapEvent(OnHeadTrackingMapEvent);
            MLInput.OnControllerButtonDown -= OnButtonDown;
        }
        #endregion

        #region Private Methods
        // Handles the Frame Event
        private void UpdateMarker()
        {
            MLInputController controller = _controllerConnectionHandler.ConnectedController;
            cursorObject.transform.position = controller.Position;
            cursorObject.transform.rotation = controller.Orientation;
        }

        // Handles the Press Event
        private void BumperPress()
        {
            spawnedObjects[idx] = Instantiate(m_PlacedPrefab, cursorObject.transform.position, cursorObject.transform.rotation);
            idx++; 
            m_InstructionText.text = string.Format("Placing Marker {0}", idx);
        }
        #endregion

        #region Event Handlers
        private void OnButtonDown(byte controllerId, MLInputControllerButton button)
        {
            if (_controllerConnectionHandler.IsControllerValid(controllerId) && button == MLInputControllerButton.HomeTap)
                BumperPress(); 
        }

        private void OnHeadTrackingMapEvent(MLHeadTrackingMapEvent mapEvents)
        {
            if (mapEvents.IsLost())
            {
                // Destroy(cursorObject);
                for (int i = 0; i < 7; i++) {
                    Destroy(spawnedObjects[i]);
                }
                idx = 0; 
            }
        }

        #endregion
    }
}