using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class particle {
    public float m;
    public Vector3 position;
    public Vector3 velocity;
    public Vector3 force;

    public float distance2Point(Vector3 point)
    {
        return Vector3.Magnitude(point - position);
    }

    public Vector3 vector2Point(Vector3 point)
    {
        return point - position;
    }

}

public class spring
{

    public particle parA, parB;
    public float springConstant, damperConstant, restLength, labs;
    public Vector3 l, ldot, addForce;
    public spring(particle inParA, particle inParB, float inSpringConstant, float inDamperConstant)
    {
        springConstant = inSpringConstant;
        damperConstant = inDamperConstant;
        parA = inParA;
        parB = inParB;
        l = parA.position - parB.position;
        labs = Vector3.Magnitude(parA.position - parB.position);
        ldot = parA.velocity - parB.velocity;
        restLength = Vector3.Magnitude(parA.position - parB.position);
    }

    public void calcSpringForces(float inSpringConstant, float inDampenerConstant) 
    {
        springConstant = inSpringConstant;
        damperConstant = inDampenerConstant;
        l = parA.position - parB.position;
        ldot = parA.velocity - parB.velocity;
        labs = Vector3.Magnitude(l);
        addForce = -springConstant * (labs - restLength) * Vector3.Normalize(l) - damperConstant * ldot;
        parA.force += addForce;
        parB.force -= addForce;
    }

}

public class face
{
    public particle p1, p2, p3;
    public float area;
    public Vector3 normal;
    public Vector3 pressureForce;

    public face(particle inp1, particle inp2, particle inp3)
    {
        p1 = inp1;
        p2 = inp2;
        p3 = inp3;
    }

    public float calculateArea()
    {
        float v321 = p3.position.x * p2.position.y * p1.position.z;
        float v231 = p2.position.x * p3.position.y * p1.position.z;
        float v312 = p3.position.x * p1.position.y * p2.position.z;
        float v132 = p1.position.x * p3.position.y * p2.position.z;
        float v213 = p2.position.x * p1.position.y * p3.position.z;
        float v123 = p1.position.x * p2.position.y * p3.position.z;
        area = Mathf.Abs((1.0f / 6.0f) * (-v321 + v231 + v312 - v132 - v213 + v123));
        return area;
    }

    public void calculateNormal()
    {
        Vector3 p1p2 = p2.position - p1.position;
        Vector3 p1p3 = p3.position - p1.position;
        normal = Vector3.Normalize(Vector3.Cross(p1p2, p1p3));
    }

    public Vector3 calculateMidPoint()
    {
        return ((p1.position + p2.position + p3.position) / 3f);
    }

}

public class particleSystem
{
    public particle[] particleArray;
    public int particleAmount;
    public Mesh objectMesh;
    public Transform objectTransform;
    public List<spring> springList;
    public List<face> faceList;
    public float volume;
    public float pressure;
    public float springConstant = 5f;
    public float damperConstant = 0.1f;
    public float frictionConstant = 0.8f;

    public void setPressure(float inPressure)
    {
        pressure = inPressure;
    }

    public void setFrictionConstant(float inFrictionConstant)
    {
        frictionConstant = inFrictionConstant;
    }

    public void setSpringConstant(float inSpringConstant)
    {
        springConstant = inSpringConstant;
    }

    public void setDampenerConstant(float inDampenerConstant)
    {
        damperConstant = inDampenerConstant;
    }

    public void calculateVolume()
    {
        for(int i = 0; i < faceList.Count; i++)
        {
            volume += faceList[i].calculateArea();
        }
    }

    public particleSystem(int inAmount)
    {
        particleAmount = inAmount;
        particleArray = new particle[particleAmount];
        objectMesh = null;
        objectTransform = null;

        for(int i = 0; i < particleAmount; i++)
        {
            particleArray[i] = new particle();
            particleArray[i].position = new Vector3(0f, 0f, 0f);
            particleArray[i].velocity = new Vector3(0f, 0f, 0f);
        }
    }

    public particleSystem(Transform inTransform)
    {

        objectMesh = inTransform.GetComponent<MeshFilter>().mesh;
        objectMesh.MarkDynamic();
        particleAmount = objectMesh.vertexCount;
        particleArray = new particle[particleAmount];
        springList = new List<spring>();
        faceList = new List<face>();
        objectTransform = inTransform;


        for (int i = 0; i < particleAmount; i++)
        {
            particleArray[i] = new particle();
            particleArray[i].m = 1f;
            particleArray[i].position = objectTransform.TransformPoint(objectMesh.vertices[i]);
            particleArray[i].velocity = new Vector3(0f, 0f, 0f);
            particleArray[i].force = new Vector3(0f, 0f, 0f);

        }

       for(int i = 0; i < particleAmount; i++)
        {
            for(int j = 0; j < particleAmount; j++)
            {
                if(i != j) {
                    springList.Add(new spring(particleArray[i], particleArray[j], springConstant, damperConstant));
                }
            }
        }

       for(int i = 0; i < objectMesh.triangles.Length; i = i + 3)
        {
            faceList.Add(new face(particleArray[objectMesh.triangles[i]], particleArray[objectMesh.triangles[i + 1]], particleArray[objectMesh.triangles[i + 2]]));
        }

        calculateVolume();

    }
}

abstract public class force {

    public abstract void applyForce(particleSystem pSys);

}

public class dragForce : force
{
    public override void applyForce(particleSystem pSys)
    {
        for(int i = 0; i < pSys.particleAmount; i++)
        {
            pSys.particleArray[i].force += pSys.particleArray[i].velocity * -0.1f;
        }
    }

}

public class gravityForce : force
{
    public override void applyForce(particleSystem pSys)
    {
        for(int i = 0; i < pSys.particleAmount; i++)
        {
            pSys.particleArray[i].force += Physics.gravity;
        }
    }
}

public class springForce : force
{
    public override void applyForce(particleSystem pSys)
    {
        for(int i = 0; i < pSys.springList.Count; i++)
        {
            pSys.springList[i].calcSpringForces(pSys.springConstant, pSys.damperConstant);
        }
    }
}

public class collisionForce : force
{
    public List<Transform> planeList;

    public collisionForce(List<Transform> inPlaneList)
    {
        planeList = new List<Transform>();
        planeList = inPlaneList;
    }

    public override void applyForce(particleSystem pSys)
    {
        float epsilon = 1e-6f;
        for(int i = 0; i < pSys.particleAmount; i++)
        {
            for (int j = 0; j < planeList.Count; j++)
            {
                Mesh planeMesh = planeList[j].GetComponent<MeshFilter>().mesh;

                Vector3 pointOnPlane = planeList[j].TransformPoint(planeMesh.vertices[30]);
                Vector3 pointOnMesh = pSys.particleArray[i].position;
                Vector3 point2Plane = pointOnMesh - pointOnPlane;
                Vector3 planeNormal = planeList[j].TransformDirection(planeMesh.normals[30]);
                if (Vector3.Dot(point2Plane, planeNormal) < -epsilon) {

                    pSys.particleArray[i].position += -Vector3.Dot(point2Plane, planeNormal) * planeNormal * 1.05f;

                    Vector3 nVel = Vector3.Dot(pSys.particleArray[i].velocity, planeNormal) * planeNormal;
                    Vector3 pVel = pSys.particleArray[i].velocity - nVel;
                    pSys.particleArray[i].velocity = pVel;

                    pSys.particleArray[i].force += pSys.frictionConstant * (Vector3.Dot(pSys.particleArray[i].force, planeNormal)) * pVel;

                    Vector3 nForce = Vector3.Dot(pSys.particleArray[i].force, planeNormal) * planeNormal;
                    pSys.particleArray[i].force += nForce;

                }
            }
        }
    }
}

public class pressureForce : force
{
    public float pressureValue;
    public float pressureStrength;
    public override void applyForce(particleSystem pSys)
    {
        pressureStrength = pSys.pressure;
        pSys.calculateVolume();
        pressureValue = pressureStrength / pSys.volume;
        Vector3 pressureForce;
        for(int i = 0; i < pSys.faceList.Count; i++)
        {
            pSys.faceList[i].calculateNormal();
            pSys.faceList[i].calculateArea();
            pressureForce = pSys.faceList[i].normal * pSys.faceList[i].area * pressureValue;
            pSys.faceList[i].p1.force += pressureForce;
            pSys.faceList[i].p2.force += pressureForce;
            pSys.faceList[i].p3.force += pressureForce;
            pSys.faceList[i].pressureForce = pressureForce;
        }
    }
}

public class mouseForce : force
{
    public Camera camera;

    public mouseForce(Camera inCamera)
    {
        camera = inCamera;
    }

    public override void applyForce(particleSystem pSys)
    {
        Vector3 averagePos = Vector3.zero;
        for(int i = 0; i < pSys.particleAmount; i++)
        {
            averagePos += pSys.particleArray[i].position;
        }
        averagePos /= pSys.particleAmount;

        float distance2Mesh = Vector3.Magnitude(averagePos - camera.transform.position);
        Vector3 pullPoint = camera.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y, distance2Mesh));
        
        for(int i = 0; i < pSys.particleAmount; i++)
        {
            float distance = pSys.particleArray[i].distance2Point(pullPoint);

            if (distance > 1f)
            {
                float pullForce = 200f / (distance * distance);
                pSys.particleArray[i].force += Vector3.Normalize(pSys.particleArray[i].vector2Point(pullPoint)) * pullForce;
            }
        }
        Debug.DrawLine(averagePos, pullPoint, Color.green);
    }
}

public class SimulationScript : MonoBehaviour
{
    public Camera camera;
    public List<Transform> planeList;
    public Transform planeParentObject;

    public int particleDims(particleSystem pSys)
    {
        return (6 * pSys.particleAmount);
    }

    public void clearForces(particleSystem pSys)
    {
        for(int i = 0; i < pSys.particleAmount; i++) 
        {
            pSys.particleArray[i].force = new Vector3(0f, 0f, 0f);
        }
    }

    public particleSystem particleDerivative(particleSystem pSys)
    {
        clearForces(pSys);
        particleSystem tempSys = new particleSystem(pSys.particleAmount);
        calculateForces(pSys);
       // float[] particleState;
        for(int i = 0; i < pSys.particleAmount; i++)
        {
            tempSys.particleArray[i].position = pSys.particleArray[i].velocity;
            tempSys.particleArray[i].velocity.x += pSys.particleArray[i].force.x / pSys.particleArray[i].m;
            tempSys.particleArray[i].velocity.y += pSys.particleArray[i].force.y / pSys.particleArray[i].m;
            tempSys.particleArray[i].velocity.z += pSys.particleArray[i].force.z / pSys.particleArray[i].m;
        }
        return tempSys;
    }

    public particleSystem addSystems(particleSystem sys1, particleSystem sys2)
    {
        for(int i = 0; i < sys1.particleAmount; i++)
        {
            sys1.particleArray[i].position += sys2.particleArray[i].position * Time.deltaTime;
            sys1.particleArray[i].velocity += sys2.particleArray[i].velocity * Time.deltaTime;
        }
        return sys1;
    }

    public void EulerStep(particleSystem pSys)
    {
        pSys = addSystems(pSys,particleDerivative(pSys));
        updateMesh(pSys);
    }

    public void calculateForces(particleSystem pSys)
    {
        gravityForce gravity = new gravityForce();
        collisionForce collision = new collisionForce(planeList);
        springForce spring = new springForce();
        pressureForce pressure = new pressureForce();
        mouseForce mouse = new mouseForce(camera);
        gravity.applyForce(pSys);
        collision.applyForce(pSys);
        spring.applyForce(pSys);
        pressure.applyForce(pSys);
        if(Input.GetMouseButton(0))
        {
            mouse.applyForce(pSys);
        }


    }

    public void updateMesh(particleSystem pSys)
    {
        Mesh updateMesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertexArr = new Vector3[pSys.particleAmount];
        for (int i = 0; i < pSys.particleAmount; i++)
        {
            vertexArr[i].x =  pSys.particleArray[i].position.x;
            vertexArr[i].y =  pSys.particleArray[i].position.y;
            vertexArr[i].z =  pSys.particleArray[i].position.z;
            vertexArr[i] = pSys.objectTransform.InverseTransformPoint(vertexArr[i]);

        }
        updateMesh.vertices = vertexArr;
    }

    particleSystem pSys;
    public float pressure = 50000f;
    public Vector3 gravity = Vector3.up * -9.82f;
    public float springConstant = 5f;
    public float dampenerConstant = 0.1f;
    public float frictionConstant = 0.8f;
    public bool showNormals = false;
    public bool showPressureForce = false;
    public bool showParticeVelocity = false;
    public bool showParticeForce = false;


    void Start()
    {
        foreach(Transform child in planeParentObject) {
            planeList.Add(child);
        }
        pSys = new particleSystem(gameObject.transform);
    }

    // Update is called once per frame
    void Update()
    {
        EulerStep(pSys);
        pSys.setPressure(pressure);
        pSys.setSpringConstant(springConstant);
        pSys.setDampenerConstant(dampenerConstant);
        pSys.setFrictionConstant(frictionConstant);
        Physics.gravity = gravity;

        if (showNormals) { 
            for (int i = 0; i < pSys.faceList.Count; i++)
            {
                Debug.DrawRay(pSys.faceList[i].calculateMidPoint(), pSys.faceList[i].normal, Color.green);
            }
        }
        if (showPressureForce)
        {
            for (int i = 0; i < pSys.faceList.Count; i++)
            {
                Debug.DrawRay(pSys.faceList[i].calculateMidPoint(), pSys.faceList[i].pressureForce, Color.cyan);
            }
        }
        if (showParticeVelocity)
        {
            for(int i = 0; i < pSys.particleAmount; i++)
            {
                Debug.DrawRay(pSys.particleArray[i].position, pSys.particleArray[i].velocity, Color.yellow);
            }
        }
        if (showParticeForce)
        {
            for (int i = 0; i < pSys.particleAmount; i++)
            {
                Debug.DrawRay(pSys.particleArray[i].position, pSys.particleArray[i].force, Color.red);
            }
        }

    }


}


/*  for(int i = 0; i < particleAmount; i++)
        {
            int counter = 0;
            for (int j = 0; j < objectMesh.triangles.Length; j++)
            {
                if (objectMesh.triangles[j].Equals(i))
                {
                    if (counter == 0)
                    {
                        if (!particleArray[i].neighbors.Contains(particleArray[objectMesh.triangles[j + 1]])) {
                            particleArray[i].neighbors.Add(particleArray[objectMesh.triangles[j + 1]]);
                            particleArray[i].neighborDistances.Add(Vector3.Magnitude(calculateDistance(particleArray[i], particleArray[objectMesh.triangles[j + 1]])));
                        }
                        if (!particleArray[i].neighbors.Contains(particleArray[objectMesh.triangles[j + 2]]))
                        {
                            particleArray[i].neighbors.Add(particleArray[objectMesh.triangles[j + 2]]);
                            particleArray[i].neighborDistances.Add(Vector3.Magnitude(calculateDistance(particleArray[i], particleArray[objectMesh.triangles[j + 2]])));
                        }
                    }
                    else if (counter == 1)
                    {
                        if (!particleArray[i].neighbors.Contains(particleArray[objectMesh.triangles[j - 1]]))
                        {
                            particleArray[i].neighbors.Add(particleArray[objectMesh.triangles[j - 1]]);
                            particleArray[i].neighborDistances.Add(Vector3.Magnitude(calculateDistance(particleArray[i], particleArray[objectMesh.triangles[j - 1]])));
                        }
                        if (!particleArray[i].neighbors.Contains(particleArray[objectMesh.triangles[j + 1]]))
                        {
                            particleArray[i].neighbors.Add(particleArray[objectMesh.triangles[j + 1]]);
                            particleArray[i].neighborDistances.Add(Vector3.Magnitude(calculateDistance(particleArray[i], particleArray[objectMesh.triangles[j + 1]])));
                        }
                    }
                    else
                    {
                        if (!particleArray[i].neighbors.Contains(particleArray[objectMesh.triangles[j - 1]]))
                        {
                            particleArray[i].neighbors.Add(particleArray[objectMesh.triangles[j - 1]]);
                            particleArray[i].neighborDistances.Add(Vector3.Magnitude(calculateDistance(particleArray[i], particleArray[objectMesh.triangles[j - 1]])));
                        }
                        if (!particleArray[i].neighbors.Contains(particleArray[objectMesh.triangles[j - 2]]))
                        {
                            particleArray[i].neighbors.Add(particleArray[objectMesh.triangles[j - 2]]);
                            particleArray[i].neighborDistances.Add(Vector3.Magnitude(calculateDistance(particleArray[i], particleArray[objectMesh.triangles[j - 2]])));
                        }
                    }
                }
                counter++;
                if (counter == 3)
                {
                    counter = 0;
                }
            }
        }*/