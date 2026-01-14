using UnityEngine;

public class SkierVisualAlign : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private Rigidbody rb;
    [SerializeField] private SkierMovement movement;
    [SerializeField] private Transform visual;

    [Header("Align")]
    [SerializeField] private bool lockYawToRigidbody = true;
    [SerializeField] private float alignSharpness = 12f;

    private void Reset()
    {
        rb = GetComponent<Rigidbody>();
        movement = GetComponent<SkierMovement>();
    }

    private void FixedUpdate()
    {
        if (!rb || !movement || !visual)
            return;

        var up = movement.IsOnSurfaceStable ? movement.GroundNormal : Vector3.up;

        var heading = lockYawToRigidbody
            ? (rb.rotation * Vector3.forward)
            : transform.forward;

        var forwardOnPlane = Vector3.ProjectOnPlane(heading, up);

        if (forwardOnPlane.sqrMagnitude < 0.0001f)
        {
            var right = rb.rotation * Vector3.right;
            forwardOnPlane = Vector3.Cross(up, right);
        }

        if (forwardOnPlane.sqrMagnitude < 0.0001f)
        {
            var right = transform.right;
            forwardOnPlane = Vector3.Cross(up, right);
        }

        forwardOnPlane.Normalize();

        var targetRot = Quaternion.LookRotation(forwardOnPlane, up);
        var a = 1f - Mathf.Exp(-alignSharpness * Time.fixedDeltaTime);

        visual.rotation = Quaternion.Slerp(visual.rotation, targetRot, a);
    }
}