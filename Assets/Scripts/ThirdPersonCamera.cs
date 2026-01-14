using UnityEngine;
using UnityEngine.InputSystem;

public class ThirdPersonCamera : MonoBehaviour
{
    [Header("Target")]
    [SerializeField] private Transform target;
    [SerializeField] private float heightOffset = 1.5f;

    [Header("Distance")]
    [SerializeField] private float distance = 6f;

    [Header("Rotation")]
    [SerializeField] private float mouseSensitivity = 120f;
    [SerializeField] private float minPitch = -35f;
    [SerializeField] private float maxPitch = 70f;

    [Header("Smoothing")]
    [SerializeField] private float rotationSmoothTime = 0.05f;

    // Desired angles (input writes here)
    private float yaw;
    private float pitch;

    // Smoothed angles (camera uses these)
    private float yawSmoothed;
    private float pitchSmoothed;

    private float yawVel;
    private float pitchVel;

    private void Awake()
    {
        // Initialize from current rotation to avoid snapping
        Vector3 e = transform.eulerAngles;
        yaw = e.y;
        pitch = e.x;

        yawSmoothed = yaw;
        pitchSmoothed = pitch;

        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;
    }

    /// <summary>
    /// Hook this to PlayerInput -> Look (Performed) when using "Invoke Unity Events".
    /// </summary>
    public void OnLook(InputAction.CallbackContext ctx)
    {
        // Usually you'll hook this on Performed only, but this makes it safe either way:
        if (!ctx.performed)
            return;

        Vector2 delta = ctx.ReadValue<Vector2>();

        yaw += delta.x * mouseSensitivity * Time.deltaTime;
        pitch -= delta.y * mouseSensitivity * Time.deltaTime;
        pitch = Mathf.Clamp(pitch, minPitch, maxPitch);
    }

    private void LateUpdate()
    {
        if (!target)
            return;

        // Smooth toward desired angles
        yawSmoothed = Mathf.SmoothDampAngle(yawSmoothed, yaw, ref yawVel, rotationSmoothTime);
        pitchSmoothed = Mathf.SmoothDampAngle(pitchSmoothed, pitch, ref pitchVel, rotationSmoothTime);

        Quaternion rot = Quaternion.Euler(pitchSmoothed, yawSmoothed, 0f);

        Vector3 targetPos = target.position + Vector3.up * heightOffset;
        Vector3 camOffset = rot * new Vector3(0f, 0f, -distance);

        transform.position = targetPos + camOffset;
        transform.LookAt(targetPos);
    }
}
