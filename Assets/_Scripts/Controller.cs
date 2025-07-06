using System;
using Unity.Mathematics;
using UnityEditor;
using UnityEditor.Search;
using UnityEngine;
using UnityEngine.Rendering.VirtualTexturing;
using UnityEngine.Serialization;

[RequireComponent(typeof(Rigidbody))]
public class Controller : MonoBehaviour
{
    [Header("Common References")]
    public Renderer renderer;

    [Header("Rotation")]
    public float uprightSpringStiffness;
    public float upRightDampingForce;
    private Quaternion idleRot;

    [Header("External Forces")]
    public float gravity = -10f;
    
    [Header("Spring Settings")]
    public float springStiffness = 100f;            //Constant
    public float springDampingForce = 100f;
    public float restLength = 0.2f;                 //length of the spring in rest excluding bodyheigth
    public float maxTravelLength = 0.1f;            //Max compression of Extension length
    public float extraBufferLength = 0.2f;          //extra length for raycast
    private float rideLength;                       
    private float rayLength;
    
    [Header("Float")]
    public Vector3 rayDir = Vector3.down;
    
    [Header("Debug")]
    public bool drawDebug = true;
    private Vector3 rotationAxis;
    private float rotationAngle;
    
    Rigidbody rb;

    float bodyHeight;
    
    Vector3 Center => transform.position + Vector3.up * rideLength;
    
    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        
        if(renderer != null) bodyHeight = renderer.bounds.size.y;
        rideLength = restLength + bodyHeight * 0.5f;
        rayLength = rideLength + maxTravelLength + extraBufferLength;
        
        idleRot = transform.rotation;
    }

    private void Update()
    {
        DoDebug();
    }

    private void FixedUpdate()
    {
        Float();
        UpRightForce();
    }
    
    private void UpRightForce()
    {
        Quaternion currentRot = transform.rotation;
        Quaternion targetRot = UtilsMath.ShortestRotation(idleRot, currentRot);

        Vector3 rotAxis;
        float rotDegrees;
        targetRot.ToAngleAxis(out rotDegrees, out rotAxis);
        rotAxis.Normalize();
        
        float rotRadians = rotDegrees * Mathf.Deg2Rad;
        
        //torque = force * r
        rb.AddTorque(rotAxis * (rotRadians * uprightSpringStiffness) - (rb.angularVelocity * upRightDampingForce));
        
        // for debug
        rotationAngle = rotDegrees;
        rotationAxis = rotAxis;
    }
    
    private void Float()
    {
        bool rayDidHit = Physics.Raycast(Center, rayDir, out RaycastHit hit, rayLength);

        //debug start
        if(rayDidHit){
            Debug.DrawLine(Center, hit.point, Color.green);
            float remainingLength = rayLength - Vector3.Distance(Center, hit.point);
            Debug.DrawLine(hit.point, hit.point + rayDir * remainingLength, Color.red);
        }
        else
            Debug.DrawLine(Center, hit.point, Color.red);
        //debug end
        
        if (!rayDidHit) return;
        
        Rigidbody otherRB = hit.collider.GetComponent<Rigidbody>();
        Vector3 otherVel = Vector3.zero;

        if (otherRB != null)
            otherVel = otherRB.linearVelocity;
        
        Vector3 dir = transform.TransformDirection(rayDir);

        var rayDirVel = Vector3.Dot(dir, rb.linearVelocity);
        var otherDirVel = Vector3.Dot(dir, otherVel);
        
        var relVel = rayDirVel - otherDirVel;
        
        float x = hit.distance - rideLength;
        float springForce = (x * springStiffness) - (relVel * springDampingForce);
        
        rb.AddForce(dir * springForce);
        
        if(otherRB != null)
            otherRB.AddForceAtPosition(-rayDir * springForce * 0.1f, hit.point);
        
        //debug
        Debug.Log($"Spring Force: {springForce}");
        Debug.DrawLine(transform.TransformPoint(rb.centerOfMass + Vector3.right * 0.5f),
            transform.TransformPoint(transform.position + dir * springForce + Vector3.right * 0.5f),
            Color.blue);
    }

    private void DoDebug()
    {
        if(!drawDebug) return;
            
#if UNITY_EDITOR
        
#endif
    }

    private void OnDrawGizmos()
    {
        if(!drawDebug) return;
        
        /*Gizmos.color = Color.magenta;
        
        if(!Application.isPlaying){
            //Scene View debug
            
            return;
        }

        Gizmos.color = Color.blue;
        Gizmos.DrawSphere(Center, 0.1f);*/
        
        //rotation
        Vector3 center = transform.TransformDirection(rb.centerOfMass);
        Gizmos.color = Color.blue;
        Gizmos.DrawRay(center,rotationAxis);

        Gizmos.color = Color.red;
        Gizmos.DrawLine(center, center + transform.forward * 5f);
        Gizmos.DrawLine(center, center + (Quaternion.AngleAxis(rotationAngle, rotationAxis) * transform.forward) * 5f);
    }
}