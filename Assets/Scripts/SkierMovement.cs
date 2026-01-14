using UnityEngine;
using UnityEngine.InputSystem;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody), typeof(Collider))]
public class SkierMovement : MonoBehaviour
{
    // =========================================================
    // Inspector tuning
    // =========================================================

    [Header("Forward Speed")]
    [SerializeField] private float targetSpeed = 12f;
    [SerializeField] private float acceleration = 25f;
    [SerializeField] private float maxSpeed = 25f;

    [Header("Braking / Back")]
    [SerializeField] private float brakeStrength = 25f;
    [SerializeField] private float reverseSlowStrength = 10f;

    [Header("Turning (speed affects yaw)")]
    [SerializeField] private float turnRateMinSpeed = 200f;
    [SerializeField] private float turnRateMaxSpeed = 80f;
    [SerializeField] private float turnRateHighSpeedAt = 18f;
    [SerializeField] private float turnRateResponse = 12f;

    [Header("Align to Ground")]
    [SerializeField] private float alignResponse = 16f;

    [Header("Ground Normal Smoothing")]
    [SerializeField] private float groundNormalResponse = 25f;

    [Tooltip("Lets jump still work if grounded flickers for a split second.")]
    [SerializeField] private float coyoteTime = 0.08f;

    [Header("Coasting Steering")]
    [SerializeField] private float coastSteerRate = 650f;
    [SerializeField] private float steerBoostWhenTurning = 1.25f;

    [Header("Side Slip")]
    [SerializeField] private float lateralFriction = 10f;
    [SerializeField] private float turnFrictionBoost = 1.5f;

    [Header("Ground")]
    [SerializeField] private LayerMask groundMask;

    [Tooltip("Max surface angle (degrees) that still counts as ground. For pipes: 85–89.")]
    [Range(0f, 89.9f)]
    [SerializeField] private float maxGroundAngle = 88f;

    [Header("Stick To Ground")]
    [SerializeField] private float groundStickForce = 45f;
    [SerializeField] private float stickForceBoost = 120f;
    [SerializeField] private float stickBoostSpeedAt = 18f;
    [SerializeField] private float stickBoostSteepness = 1.0f;

    [Header("Drag")]
    [SerializeField] private float groundDrag = 0.4f;
    [SerializeField] private float coastDrag = 0.9f;
    [SerializeField] private float airDrag = 0.02f;

    [Header("Jump (charge then release)")]
    [SerializeField] private float jumpImpulseMin = 4.0f;
    [SerializeField] private float jumpImpulseMax = 9.0f;
    [SerializeField] private float maxJumpChargeTime = 2.0f;
    [SerializeField] private float jumpUngroundLockTime = 0.10f;

    [Header("Physics Material (Unity 6)")]
    [Tooltip("Optional. If set, a runtime clone is used so friction edits don't touch the asset.")]
    [SerializeField] private PhysicsMaterial physicsMaterial;

    [SerializeField] private float physicsStaticFriction = 0.001f;
    [SerializeField] private float physicsDynamicFriction = 0.001f;

    [Header("Tuning")]
    [SerializeField] private float inputDeadzone = 0.01f;
    [SerializeField] private float minSpeedToSteer = 0.15f;

    // =========================================================
    // Runtime state
    // =========================================================

    private const float EPS = 1e-4f;

    private Rigidbody _rb;
    private Collider _col;

    private Vector2 _moveInput;
    private bool _brakeHeld;

    private bool _grounded;
    private Vector3 _groundNormal = Vector3.up;

    private float _currentTurnRate;
    private Quaternion _desiredRbRotation;

    // Jump state: hold -> charge -> release
    private bool _jumpHeld;
    private float _jumpHoldTime;
    private bool _jumpQueued;
    private float _queuedJumpCharge01;
    private float _ungroundLockTimer;

    // Contacts gathered in collision callbacks, consumed in FixedUpdate.
    private bool _contactGrounded;
    private Vector3 _contactNormalSum;
    private int _contactCount;

    private float _lastGroundedFixedTime;

    // Cached "walkable angle" threshold: n·up >= cos(maxGroundAngle)
    private float _minGroundDot;

    // Runtime-cloned material so project assets never get mutated.
    private PhysicsMaterial _runtimeMaterial;

    // =========================================================
    // Public (mostly for visuals)
    // =========================================================

    public bool IsGrounded => _grounded;
    public Vector3 GroundNormal => _groundNormal;

    // Small buffer after losing contact, so jump doesn't feel ultra picky on bumps.
    public bool IsOnSurfaceStable => _grounded || (Time.fixedTime - _lastGroundedFixedTime) <= coyoteTime;

    // =========================================================
    // Unity lifecycle
    // =========================================================

    private void Awake()
    {
        _rb = GetComponent<Rigidbody>();
        _col = GetComponent<Collider>();

        // Rotation is driven manually, so don't let physics torque spin the body.
        _rb.freezeRotation = true;

        // Keeps motion smooth and reduces tunneling on fast ramps.
        _rb.interpolation = RigidbodyInterpolation.Interpolate;
        _rb.collisionDetectionMode = CollisionDetectionMode.Continuous;

        // Stabilizes sharp transitions (pipe lips, seams).
        _rb.solverIterations = Mathf.Max(_rb.solverIterations, 8);
        _rb.solverVelocityIterations = Mathf.Max(_rb.solverVelocityIterations, 8);

        _currentTurnRate = turnRateMinSpeed;
        _desiredRbRotation = _rb.rotation;

        RecomputeGroundDotThreshold();
        SetupPhysicsMaterialInstance();
    }

    private void OnValidate()
    {
        if (_col == null)
            _col = GetComponent<Collider>();

        RecomputeGroundDotThreshold();

        // If a runtime clone exists (play mode), keep it synced with inspector values.
        ApplyPhysicsMaterialSettingsTo(_runtimeMaterial);
    }

    private void OnDestroy()
    {
        // Cleanup runtime material clone (play mode).
        if (_runtimeMaterial != null)
            Destroy(_runtimeMaterial);
    }

    private void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        if (_ungroundLockTimer > 0f)
            _ungroundLockTimer -= dt;

        // Convert last physics step's contacts into grounded + a smoothed normal.
        UpdateGroundFromContacts(dt);

        // Clear contact buffer for the next physics step.
        _contactGrounded = false;
        _contactNormalSum = Vector3.zero;
        _contactCount = 0;

        // Cached per-step values (cuts repeated projections).
        Vector3 up = _grounded ? _groundNormal : Vector3.up;
        Vector3 vel = _rb.linearVelocity;

        // Charge jump while held, as long as contact is "basically still there".
        if (_jumpHeld && IsOnSurfaceStable && _ungroundLockTimer <= 0f)
        {
            _jumpHoldTime += dt;
            if (_jumpHoldTime > maxJumpChargeTime)
                _jumpHoldTime = maxJumpChargeTime;
        }

        // Apply queued jump impulse on the physics step.
        if (_jumpQueued)
        {
            _jumpQueued = false;
            DoJump(_queuedJumpCharge01);

            _jumpHoldTime = 0f;
            _queuedJumpCharge01 = 0f;

            // Jump can change grounded/velocity, so refresh cached values.
            up = _grounded ? _groundNormal : Vector3.up;
            vel = _rb.linearVelocity;
        }

        bool hasThrottleGround = _grounded && !_brakeHeld && Mathf.Abs(_moveInput.y) > inputDeadzone;

        // Damping is the "feel" knob: lower when pushing, higher when coasting, tiny in air.
        _rb.linearDamping = _grounded
            ? (hasThrottleGround ? groundDrag : coastDrag)
            : airDrag;

        // Planar speed (relative to current up) drives turn-rate scaling.
        Vector3 planarVel = Vector3.ProjectOnPlane(vel, up);
        float planarSpeed = planarVel.magnitude;

        UpdateTurnRate(planarSpeed, dt);

        // Rotation happens every step for alignment + visuals.
        // In air this only affects orientation; velocity stays untouched.
        ApplyRotation(_moveInput.x, up, dt);

        // Ground-only gameplay forces: no air throttle, no air brake, no air steering.
        if (_grounded)
        {
            up = _groundNormal;
            vel = _rb.linearVelocity;
            planarVel = Vector3.ProjectOnPlane(vel, up);
            planarSpeed = planarVel.magnitude;

            Vector3 fwd = ForwardOnSlope(up);
            Vector3 right = Vector3.Cross(up, fwd);
            right = (right.sqrMagnitude > EPS) ? right.normalized : Vector3.right;

            ApplyStickToGround(hasThrottleGround, up, planarSpeed);
            ApplyThrottle(_moveInput.y, fwd, vel);
            ApplyBrakeIfNeeded(planarVel);
            ApplyCoastSteeringAndLateralFriction(up, fwd, right, planarVel, planarSpeed, dt);
        }

        ClampSpeed();
    }

    // =========================================================
    // Input callbacks (PlayerInput / Input System)
    // =========================================================

    public void SetMove(InputAction.CallbackContext ctx) => _moveInput = ctx.ReadValue<Vector2>();
    public void SetBrake(InputAction.CallbackContext ctx) => _brakeHeld = ctx.ReadValueAsButton();

    // "Hold to charge, release to jump".
    // The callback just stores intent; the actual impulse is applied in FixedUpdate.
    public void SetJump(InputAction.CallbackContext ctx)
    {
        if (ctx.started)
        {
            if (IsOnSurfaceStable && _ungroundLockTimer <= 0f)
            {
                _jumpHeld = true;
                _jumpHoldTime = 0f;
            }
            else
            {
                _jumpHeld = false;
                _jumpHoldTime = 0f;
            }

            return;
        }

        if (ctx.canceled)
        {
            if (_jumpHeld && IsOnSurfaceStable && _ungroundLockTimer <= 0f)
            {
                float denom = Mathf.Max(0.0001f, maxJumpChargeTime);
                _queuedJumpCharge01 = Mathf.Clamp01(_jumpHoldTime / denom);
                _jumpQueued = true;
            }

            _jumpHeld = false;
        }
    }

    // =========================================================
    // Collision contacts -> grounded + normal
    // =========================================================

    private void OnCollisionEnter(Collision collision) => AccumulateGroundContacts(collision);
    private void OnCollisionStay(Collision collision) => AccumulateGroundContacts(collision);

    // Collect "walkable" contact normals into a buffer.
    // FixedUpdate consumes this buffer and smooths the normal to prevent jitter.
    private void AccumulateGroundContacts(Collision collision)
    {
        int otherLayerBit = 1 << collision.gameObject.layer;
        if ((groundMask.value & otherLayerBit) == 0)
            return;

        int count = collision.contactCount;
        for (int i = 0; i < count; i++)
        {
            ContactPoint c = collision.GetContact(i);
            Vector3 n = c.normal;

            // Fast angle test vs world-up (avoids Vector3.Angle / acos).
            if (Vector3.Dot(n, Vector3.up) < _minGroundDot)
                continue;

            _contactGrounded = true;
            _contactNormalSum += n;
            _contactCount++;
        }
    }

    // Turns the contact buffer into:
    // - grounded bool
    // - smoothed ground normal (less snapping on seams/edges)
    private void UpdateGroundFromContacts(float dt)
    {
        if (_ungroundLockTimer > 0f)
        {
            _grounded = false;

            // While locked, relax the normal back toward up so takeoff feels clean.
            float relaxA = 1f - Mathf.Exp(-groundNormalResponse * dt);
            _groundNormal = Vector3.Slerp(_groundNormal, Vector3.up, relaxA);
            return;
        }

        bool shouldBeGrounded = _contactGrounded && _contactCount > 0;

        _grounded = shouldBeGrounded;
        if (_grounded)
            _lastGroundedFixedTime = Time.fixedTime;

        Vector3 targetNormal = _grounded
            ? (_contactNormalSum / _contactCount).normalized
            : Vector3.up;

        float a = 1f - Mathf.Exp(-groundNormalResponse * dt);
        _groundNormal = Vector3.Slerp(_groundNormal, targetNormal, a);
    }

    // =========================================================
    // Setup helpers
    // =========================================================

    // Precompute cos(maxGroundAngle) so contact checks stay cheap.
    private void RecomputeGroundDotThreshold()
    {
        float clamped = Mathf.Clamp(maxGroundAngle, 0f, 89.9f);
        _minGroundDot = Mathf.Cos(clamped * Mathf.Deg2Rad);
    }

    // Clones the material at runtime so friction tweaks are per-object.
    private void SetupPhysicsMaterialInstance()
    {
        if (physicsMaterial == null || _col == null)
            return;

        _runtimeMaterial = Instantiate(physicsMaterial);
        _runtimeMaterial.name = physicsMaterial.name + " (Runtime Instance)";

        ApplyPhysicsMaterialSettingsTo(_runtimeMaterial);
        _col.sharedMaterial = _runtimeMaterial;
    }

    private void ApplyPhysicsMaterialSettingsTo(PhysicsMaterial mat)
    {
        if (mat == null)
            return;

        mat.staticFriction = physicsStaticFriction;
        mat.dynamicFriction = physicsDynamicFriction;
        mat.frictionCombine = PhysicsMaterialCombine.Minimum;
    }

    // =========================================================
    // Movement + rotation
    // =========================================================

    private void DoJump(float charge01)
    {
        if (!IsOnSurfaceStable)
            return;

        float impulse = Mathf.Lerp(jumpImpulseMin, jumpImpulseMax, Mathf.Clamp01(charge01));
        Vector3 jumpDir = _groundNormal.normalized;

        // Remove downward component along jumpDir so the jump feels consistent.
        float vDown = Vector3.Dot(_rb.linearVelocity, -jumpDir);
        if (vDown > 0f)
            _rb.linearVelocity += jumpDir * vDown;

        _rb.AddForce(jumpDir * impulse, ForceMode.Impulse);

        _grounded = false;
        _ungroundLockTimer = jumpUngroundLockTime;
    }

    // Turn rate lerps down as speed goes up (more speed = less yaw authority).
    private void UpdateTurnRate(float planarSpeed, float dt)
    {
        float t = turnRateHighSpeedAt <= 0.001f ? 1f : Mathf.Clamp01(planarSpeed / turnRateHighSpeedAt);
        float desired = Mathf.Lerp(turnRateMinSpeed, turnRateMaxSpeed, t);

        float a = 1f - Mathf.Exp(-turnRateResponse * dt);
        _currentTurnRate = Mathf.Lerp(_currentTurnRate, desired, a);
    }

    // Builds a target rotation from slope-aligned forward + up, then smooths toward it.
    private void ApplyRotation(float turnInput, Vector3 up, float dt)
    {
        Quaternion baseRot = _desiredRbRotation;

        // Project current heading onto the slope plane. Handles steep surfaces safely.
        Vector3 heading = baseRot * Vector3.forward;
        Vector3 forwardOnPlane = Vector3.ProjectOnPlane(heading, up);

        if (forwardOnPlane.sqrMagnitude < EPS)
        {
            Vector3 right = baseRot * Vector3.right;
            forwardOnPlane = Vector3.Cross(up, right);
        }

        if (forwardOnPlane.sqrMagnitude < EPS)
        {
            Vector3 right = transform.right;
            forwardOnPlane = Vector3.Cross(up, right);
        }

        if (forwardOnPlane.sqrMagnitude < EPS)
            forwardOnPlane = Vector3.ProjectOnPlane(transform.forward, up);

        if (forwardOnPlane.sqrMagnitude > EPS)
            forwardOnPlane.Normalize();
        else
            forwardOnPlane = Vector3.forward;

        float yawDeg = turnInput * _currentTurnRate * dt;
        if (Mathf.Abs(yawDeg) > 0.000001f)
            forwardOnPlane = Quaternion.AngleAxis(yawDeg, up) * forwardOnPlane;

        Quaternion target = Quaternion.LookRotation(forwardOnPlane, up);

        float a = 1f - Mathf.Exp(-alignResponse * dt);
        _desiredRbRotation = Quaternion.Slerp(baseRot, target, a);

        _rb.MoveRotation(_desiredRbRotation);
    }

    // Slope-forward based on current desired rotation, projected onto the slope plane.
    private Vector3 ForwardOnSlope(Vector3 up)
    {
        Vector3 fwd = Vector3.ProjectOnPlane(_desiredRbRotation * Vector3.forward, up);
        if (fwd.sqrMagnitude < EPS)
        {
            Vector3 right = _desiredRbRotation * Vector3.right;
            fwd = Vector3.Cross(up, right);
        }

        if (fwd.sqrMagnitude > EPS)
            return fwd.normalized;

        fwd = Vector3.ProjectOnPlane(transform.forward, up);
        return (fwd.sqrMagnitude > EPS) ? fwd.normalized : Vector3.forward;
    }

    // Extra downward force to keep contact on steep/fast surfaces.
    private void ApplyStickToGround(bool hasThrottle, Vector3 up, float planarSpeed)
    {
        if (!_grounded)
            return;

        float speedT = stickBoostSpeedAt <= 0.001f ? 1f : Mathf.Clamp01(planarSpeed / stickBoostSpeedAt);

        // 0 on flat, closer to 1 on near-vertical surfaces.
        float steepT = 1f - Mathf.Clamp01(Vector3.Dot(up, Vector3.up));
        float throttleT = hasThrottle ? 1f : 0f;

        float boost =
            stickForceBoost *
            Mathf.Max(throttleT, speedT) *
            Mathf.Lerp(1f, steepT, stickBoostSteepness);

        _rb.AddForce(-up * (groundStickForce + boost), ForceMode.Acceleration);
    }

    // Pushes toward targetSpeed along slope-forward when holding W.
    private void ApplyThrottle(float forwardInput, Vector3 fwd, Vector3 vel)
    {
        if (_brakeHeld)
            return;

        float forwardSpeed = Vector3.Dot(vel, fwd);

        if (forwardInput > inputDeadzone)
        {
            float speedError = targetSpeed - forwardSpeed;
            if (speedError <= 0f)
                return;

            float accelAmt = Mathf.Clamp(speedError, 0f, acceleration);
            _rb.AddForce(fwd * accelAmt, ForceMode.Acceleration);
        }
        else if (forwardInput < -inputDeadzone)
        {
            _rb.AddForce(-fwd * reverseSlowStrength, ForceMode.Acceleration);
        }
    }

    // Braking is a planar velocity damp (so it doesn't fight gravity on steep slopes too hard).
    private void ApplyBrakeIfNeeded(Vector3 planarVel)
    {
        if (!_brakeHeld || !_grounded)
            return;

        _rb.AddForce(-planarVel * brakeStrength, ForceMode.Acceleration);
    }

    // Two things on ground:
    // - gently rotate planar velocity toward the skis direction (helps follow curves)
    // - damp sideways slip so it feels like snow, not ice
    private void ApplyCoastSteeringAndLateralFriction(
        Vector3 up,
        Vector3 fwd,
        Vector3 right,
        Vector3 planarVel,
        float planarSpeed,
        float dt)
    {
        if (planarSpeed < minSpeedToSteer)
            return;

        bool turning = Mathf.Abs(_moveInput.x) > inputDeadzone;

        if (planarSpeed > 0.1f)
        {
            Vector3 desiredPlanarVel = fwd * planarSpeed;

            float rate = coastSteerRate * (turning ? steerBoostWhenTurning : 1f);
            float maxRad = rate * Mathf.Deg2Rad * dt;

            Vector3 newPlanarVel = Vector3.RotateTowards(planarVel, desiredPlanarVel, maxRad, 0f);

            // Only change the planar component; keep the "up" component untouched.
            Vector3 vel = _rb.linearVelocity;
            Vector3 normalVel = Vector3.Project(vel, up);
            _rb.linearVelocity = newPlanarVel + normalVel;

            // Refresh planarVel after the write.
            vel = _rb.linearVelocity;
            planarVel = Vector3.ProjectOnPlane(vel, up);
        }

        float lateralSpeed = Vector3.Dot(planarVel, right);
        float friction = lateralFriction * (turning ? turnFrictionBoost : 1f);
        _rb.AddForce(-right * lateralSpeed * friction, ForceMode.Acceleration);
    }

    // Hard speed cap to avoid silly values if something launches the skier.
    private void ClampSpeed()
    {
        float speed = _rb.linearVelocity.magnitude;
        if (speed <= maxSpeed)
            return;

        _rb.linearVelocity *= (maxSpeed / speed);
    }
}
