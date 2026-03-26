using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class tennisBall : MonoBehaviour
{

    public Vector3 impactPos;
    public Vector3 leavePos;

    // Ball
    public float m = 0.058f;
    public float r = 0.033f;

    // Initial Velocity
    public float initialVelocity = 35f;
    public float launchAngle = 7f;
    public Vector3 initialDirection;

    public Vector3 acceleration;
    public Vector3 velocity;
    public Vector3 pos = new Vector3(11.885f, 1f, 0f);
    public float maxY = 0;

    public float dt; // Time-step

    public Vector3 angularVelocity;

    public Vector3 v_rel;

    public float g = 9.81f;
    public float groundY = 0f;
    public float netY = 0.914f;

    // Boolean to keep track of bounces
    public int bounceCounter = 0;
    public Vector3 firstImpactPos;
    public bool wasInAir = true;

    // Constants for drag and lift (Magnus effect)
    public float rho = 1.22f; // Air density at 20C
    public float A; // Cross sectional area

    // Constants for impact
    public float COR = 0.75f; // Coefficient of restitution
    public float COF = 0.7f; // Coefficient of friction (kan behöva ändras)
    public float k = 20000f; // Spring constant for ball (kan behöva ändras)
    public float c; // Spring dampening for ball (kan behöva ändras)
    public bool isGripping = false;
    public Vector3 gripDisplacement = Vector3.zero;

    public enum SpinType 
    {
        None,
        topSpin,
        underSpin,
        sideSpinRight,
        sideSpinLeft
    }

    public SpinType spinType = SpinType.None;

    // Angular velocities for different spins
    public float spinRPM = 3000; //rpm


    // Start is called before the first frame update
    void Start()
    {
        launchAngle *= Mathf.Deg2Rad;
        initialDirection = new Vector3(-Mathf.Cos(launchAngle), Mathf.Sin(launchAngle), 0f);

        // Calculation of c (dampening) based on COR and k
        float lnCOR = Mathf.Log(COR);
        float zeta = -lnCOR / Mathf.Sqrt(Mathf.PI * Mathf.PI + lnCOR * lnCOR);
        c = 2f * zeta * Mathf.Sqrt(k * m);

        // Cross sectional area of ball
        A = Mathf.PI * r * r;

        velocity = initialVelocity * initialDirection.normalized;

        float spin = (2f * Mathf.PI * spinRPM) / 60f; // rad/s

    // Räkna ut bollens horisontella riktning
    Vector3 forwardH = new Vector3(initialDirection.x, 0f, initialDirection.z).normalized;
    
    // Räkna ut axeln som pekar "åt sidan" relativt färdriktningen
    // Om vi kryssar Upp-vektorn med Framåt-vektorn får vi en perfekt höger-axel
    Vector3 sideAxis = Vector3.Cross(Vector3.up, forwardH);

    switch (spinType)
    {
        case SpinType.None:
            angularVelocity = Vector3.zero;
            break;
        case SpinType.topSpin:
            // Rotera runt sido-axeln för topspin
            angularVelocity = sideAxis * spin;
            break;
        case SpinType.underSpin:
            // Rotera åt andra hållet för underspin
            angularVelocity = -sideAxis * spin;
            break;
        case SpinType.sideSpinRight:
            // Sidoskruv roterar runt den vertikala axeln (Y)
            angularVelocity = Vector3.down * spin;
            break;
        case SpinType.sideSpinLeft:
            angularVelocity = Vector3.up * spin;
            break;
    }

        Time.fixedDeltaTime = 0.0005f;
    }

    Vector3 applyAeroForces()
    {
        // Aerodynamic Forces

        // Air Resistance
        float C_D = 0.6204f - 9.76e-4f * (velocity.magnitude - 50) + (1.027e-4f - 2.24e-6f * (velocity.magnitude - 50)) * angularVelocity.magnitude; // Equation 3
        Vector3 drag = -0.5f * rho * A * C_D * velocity.magnitude * velocity;

        // Lift Force (Magnus Effect)
        float C_L = (4.68e-4f - 2.0984e-5f * (velocity.magnitude - 50)) * angularVelocity.magnitude; // Equation 4

        // Clamp values to physically reasonable ranges
        C_D = Mathf.Max(0.2f, C_D);
        C_L = Mathf.Max(0f, C_L);

        Vector3 lift = Vector3.zero;

        if (angularVelocity.magnitude > 0.001)
        {
            lift = 0.5f * rho * A * C_L * velocity.magnitude * (Vector3.Cross(angularVelocity, velocity) / angularVelocity.magnitude);
        }
        
        return drag + lift;
    }

    Vector3 applyImpactForces() //TTEEESSTT
    {
        // Compression of the ball
        float compression = r - pos.y;

        // Spring force and dampening force
        float springForce = k * compression;
        float dampeningForce = -c * velocity.y;

        // Normal force magnitude
        float magnitudeN = Mathf.Max(0, springForce + dampeningForce);

        // Normal force
        Vector3 vertical = new Vector3(0f, 1f, 0f);
        Vector3 F_normal = magnitudeN * vertical;

        // Relative velocity on impact
        Vector3 v_horizontal = new Vector3(velocity.x, 0f, velocity.z);
        Vector3 radiusVector = new Vector3(0f, -r, 0f); 
        Vector3 v_rel = v_horizontal + Vector3.Cross(angularVelocity, radiusVector);

        Vector3 F_friction = Vector3.zero;

        if (!isGripping) // Sliding
        {

            F_friction = - v_rel.normalized * COF * magnitudeN;

            if (v_rel.magnitude < 0.01f && magnitudeN > 0)
            {
                isGripping = true;
                gripDisplacement = Vector3.zero;
            }

        }
        else // Gripping
        {
            // Elastic force on ball
            gripDisplacement += v_rel * dt;
            F_friction = -k * gripDisplacement;

            if (F_friction.magnitude > COF * magnitudeN)
            {
                isGripping = false;
                // Return to sliding
                F_friction = - v_rel.normalized * COF * magnitudeN;
            }

        }

        // I = 2/3 * m * r^2 (hollow shell of a sphere)
        float I = (2f / 3f) * m * r * r;
        Vector3 torque = Vector3.Cross(radiusVector, F_friction);
        Vector3 angularAcceleration = torque / I;
        angularVelocity += angularAcceleration * dt;

        return F_normal + F_friction;        

    }

    void FixedUpdate()
    {
        
        // Ground Level
        float contactY = groundY + r;
        float contactNetY = netY + r;

        // Time-step
        dt = Time.fixedDeltaTime;

        // Gravitational Force
        Vector3 gravitation = new Vector3(0f, -m*g, 0f);
        Vector3 totalForce = gravitation;

        if (pos.y <= contactY)
        {

            if (wasInAir)
            {
                impactPos = pos;

                bounceCounter++;

                if (bounceCounter == 1)
                {
                    //Debug.Log("Första Studs");
                    Debug.Log("Första studs x-position " + spinType + ": " + pos);
                }

                // Cancels simulation on second bounce
                if (bounceCounter == 2)
                {
                    Debug.Log("Peak height between bounces: " + maxY);
                    //Debug.Log("Simulering avbruten vid andra studs");
                    Debug.Log("Slutposition " + spinType + ":" + pos);
                    Debug.Log("Sluthastighet " + spinType + ": " + velocity.magnitude);
                    
                    enabled = false;
                    return;
                }

                firstImpactPos = new Vector3(pos.x, 0, pos.z);
                wasInAir = false;
            }

            // If the ball is in contact with the court
            totalForce += applyImpactForces();

        }
        else
        {
            if (!wasInAir)
            {
                leavePos = pos;
                Vector3 distance = leavePos - impactPos;
                float d = distance.magnitude;
                Debug.Log("Distans under träff " + spinType + ": " + d);
            }
            wasInAir = true;

            // Saves highest position between first and second bounce
            if (bounceCounter == 1)
            {
                if (pos.y > maxY)
                {
                    maxY = pos.y;
                }
            }

            // If the ball is in the air
            totalForce += applyAeroForces();
        }

        // Checks if ball hits the net
        if (Mathf.Abs(pos.x) < 0.01f && pos.y <= contactNetY)
        {
            Debug.Log(spinType + ": Bollen gick i nät");
            enabled = false;
            return;
        }
        /*if (pos.x == 0 && pos.y <= contactNetY)
        {
            Debug.Log("Bollen gick i nät");
            enabled = false;
            return;
        }*/

        // Eulers method
        acceleration = totalForce / m;
        velocity += acceleration * dt;
        pos += velocity * dt;

        // Updates the position of the sphere in Unity
        transform.position = pos;

        // Rotates the sphere in Unity according to angular velocity
        transform.Rotate(angularVelocity * (180 / Mathf.PI) * dt); // .Rotate wants degrees and not rad



        // TA BORT??
        
        // Fixing simulation so that the ball never goes beneath the ground visually
        Vector3 visualPos = pos;
        if (visualPos.y < contactY) 
        {
            visualPos.y = contactY; // Bollen ser ut att ligga på marken
        }
        transform.position = visualPos;

    }
}
