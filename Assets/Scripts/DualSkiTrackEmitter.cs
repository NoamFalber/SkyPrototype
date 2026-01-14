using UnityEngine;

public class DualSkiTrackEmitter : MonoBehaviour
{
    [Header("Contact points")]
    public Transform leftContact;
    public Transform rightContact;

    [Header("Track prefab (must have SkiTrackRibbon)")]
    public SkiTrackRibbon trackPrefab;

    [Header("Snow detection")]
    public LayerMask snowLayerMask;
    public float raycastHeight = 0.2f;
    public float raycastDistance = 1.6f;

    [Header("Emission")]
    public float minMoveDistance = 0.05f;
    public float minSpeedToDraw = 0.2f;

    [Header("Segmentation")]
    public int maxPointsPerTrack = 300;

    [Header("Orientation Source")]
    public Transform orientationSource; // if null -> uses this.transform

    SkiTrackRibbon _leftTrack;
    SkiTrackRibbon _rightTrack;

    Vector3 _leftLast, _rightLast;
    bool _leftHasLast, _rightHasLast;

    Vector3 _prevPos;

    void Start()
    {
        _prevPos = transform.position;
        if (!orientationSource) orientationSource = transform;
    }

    void Update()
    {
        float speed = (transform.position - _prevPos).magnitude / Mathf.Max(Time.deltaTime, 0.0001f);
        _prevPos = transform.position;

        if (speed < minSpeedToDraw)
            return;

        HandleContact(leftContact, ref _leftTrack, ref _leftLast, ref _leftHasLast);
        HandleContact(rightContact, ref _rightTrack, ref _rightLast, ref _rightHasLast);
    }

    void HandleContact(
        Transform contact,
        ref SkiTrackRibbon track,
        ref Vector3 lastPoint,
        ref bool hasLastPoint)
    {
        if (!contact) return;

        Vector3 origin = contact.position + Vector3.up * raycastHeight;

        if (!Physics.Raycast(origin, Vector3.down, out RaycastHit hit, raycastDistance, snowLayerMask))
        {
            track = null;
            hasLastPoint = false;
            return;
        }

        Vector3 point = hit.point;
        Vector3 normal = hit.normal;
        if (Vector3.Dot(normal, Vector3.up) < 0f) normal = -normal;

        // Use ski/player forward projected onto the surface (so it lies on the slope)
        Vector3 srcForward = orientationSource ? orientationSource.forward : transform.forward;
        Vector3 tangentForward = Vector3.ProjectOnPlane(srcForward, normal);
        if (tangentForward.sqrMagnitude < 0.000001f)
            tangentForward = Vector3.ProjectOnPlane(transform.forward, normal);

        tangentForward = tangentForward.sqrMagnitude > 0.000001f ? tangentForward.normalized : Vector3.forward;

        if (!track)
        {
            track = Instantiate(trackPrefab);

            // Neutralize transform effects (matches your working version)
            track.transform.SetParent(null);
            track.transform.position = Vector3.zero;
            track.transform.rotation = Quaternion.identity;
            track.transform.localScale = Vector3.one;

            track.Begin();
            hasLastPoint = false;
        }

        if (!hasLastPoint || Vector3.Distance(lastPoint, point) >= minMoveDistance)
        {
            track.AddPoint(point, normal, tangentForward);
            lastPoint = point;
            hasLastPoint = true;

            if (track.PointCount >= maxPointsPerTrack)
            {
                track = null;
                hasLastPoint = false;
            }
        }
    }
}
