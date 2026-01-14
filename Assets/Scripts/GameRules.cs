using UnityEngine;

public class GameRules : MonoBehaviour
{
    [SerializeField] private bool debugMode;
    public static bool DebugMode { get; private set; }

    private void Awake()
    {
        DebugMode = debugMode;
       // Application.targetFrameRate = 60;
       // QualitySettings.vSyncCount = 0;
    }
    
}
