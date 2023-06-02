using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OrbitHandler : MonoBehaviour
{
    public enum OrbitTypes
    {
        CIRCULAR,
        ELLIPTICAL,
        PARABLOIC,
        HYPERBOLIC,
    }

    // vectors
    private Vector3 mPosition;
    private Vector3 mVelocity;
    private Vector3 mAngularMomentum;
    private Vector3 mNodeLine;
    private Vector3 mEccentricity;

    // scalars
    private float mH;
    private float mN;
    private float mE;
    private float mR;
    private float mV;

    // angles
    private float mInclination;
    private float mArgumentOfPerigee;
    private float mRightAscention;
    private float mTrueAnomaly;
    private float mTrueAnomolyInfinity;

    // Other
    private float mMu;
    private float[,] mRotationMatrix = new float[3, 3];
    private Vector3[] mOrbitPath;
    private float mScale;
    public OrbitTypes mOrbitType;
    [SerializeField] public LineRenderer mOrbitPathLineRenderer;

    void Start()
    {
        // Initial Conditions
        // mu
        // position
        // velocity
        // number of points in path
        float mu = 398600f;

        // Hyperbolic
        //Vector3 position = new Vector3(-6000.0f, -3605.6f, 0.0f);
        //Vector3 velocity = new Vector3(22.0f, 7.5460f, 0.0f);

        // Circle around equator
        //Vector3 position = new Vector3(7000.0f, 0.0f, 0.0f);
        //Vector3 velocity = new Vector3(0.0f, 7.5460f, 0.0f);

        // Ellipse around equator
        //Vector3 position = new Vector3(7000.0f, 0.0f, 0.0f);
        //Vector3 velocity = new Vector3(7.0f, 7.5460f, 0.0f);

        // Ellipse with some inclination
        Vector3 position = new Vector3(-6045.0f, -3490.0f, 2500.0f);
        Vector3 velocity = new Vector3(-3.457f, 6.618f, 2.533f);

        // set methods
        mScale = 1.0f / 1000.0f;                                        // I am going to scale everything down to 1/1000th, this is arbitrary and can be anything. I just didn't want to move the camera much to fit everything in screen
        setMu(mu);
        setPosition(position);
        setVelocity(velocity);
        calculateOrbitalElements();
        generatePath(200);
        setGameObjectLineRenderer();
        setGameObjectPosition();
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    // Generate the path/orbit of object
    void generatePath(int aNumberOfPoints)
    {
        int segments = aNumberOfPoints;
        Vector3[] points = new Vector3[segments + 1];
        float deltaAngle = 2.0f * Mathf.PI / segments;
        float angle = 0.0f;

        if (mOrbitType == OrbitTypes.HYPERBOLIC)
        {
            deltaAngle = mTrueAnomolyInfinity * 0.97f * 2.0f / segments;
            angle = -1.0f * mTrueAnomolyInfinity * 0.97f;
        }
        else if (mOrbitType == OrbitTypes.PARABLOIC)
        {
            deltaAngle = Mathf.PI * 0.90f * 2.0f / segments;
            angle = -1.0f * Mathf.PI * 0.90f;
        }

        for (int i = 0; i < segments; i++)
        {
            float localRange = (mH * mH / mMu) / (1.0f + mE * Mathf.Cos(angle));
            float perifocalX = localRange * Mathf.Cos(angle);
            float perifocalY = localRange * Mathf.Sin(angle);
            angle = angle + deltaAngle;
            //Debug.LogFormat("localRange = {0}", localRange);
            points[i] = rotatePerifocalVectorToStateVector(new Vector3(perifocalX, perifocalY, 0.0f));
        }

        if (mOrbitType == OrbitTypes.CIRCULAR || mOrbitType == OrbitTypes.ELLIPTICAL)
        {
            points[segments] = points[0];
        }
        else
        {
            points[segments] = points[segments - 1];
        }

        mOrbitPath = points;
    }

    // calculate methods
    void calculateOrbitalElements()
    {
        calculateAngularMomentum();
        calculateInclination();
        calculateEccentricity();
        calculateTrueAnomaly();
        calculateNodeLine();
        calculateRightAscention();
        calculateArgumentOfPerigee();
        calculatePerifocal2GeocentricEquatorialTransformation();
        calculateTrueAnomalyInfinity();
    }

    void calculateAngularMomentum()
    {
        Vector3 angMom = Vector3.Cross(mPosition, mVelocity);
        setAngularMomentum(angMom);
    }

    void calculateNodeLine()
    {
        if (mInclination % Mathf.PI == 0.0f)
        {
            setNodeLine(new Vector3(1.0f, 0.0f, 0.0f));
            return;
        }

        Vector3 zDir = new Vector3(0.0f, 0.0f, 1.0f);
        Vector3 nodeLine = Vector3.Cross(zDir, mAngularMomentum);
        setNodeLine(nodeLine);
    }

    void calculateEccentricity()
    {
        float vro = Vector3.Dot(mPosition, mVelocity) / mR;
        Vector3 ecc = 1.0f / mMu * ((Mathf.Pow(mV, 2) - mMu / mR) * mPosition - mR * vro * mVelocity);
        setEccentricity(ecc);
    }

    void calculateInclination()
    {
        float inc = Mathf.Acos(mAngularMomentum.z / mH);

        // Not sure what's wrong, the path doesn't generate correctly with 180 degrees
        if (inc == Mathf.PI)
        {
            inc = 0.0f;
        }
        setInclination(inc);
    }

    void calculateRightAscention()
    {
        if (mInclination % Mathf.PI == 0.0f)
        {
            setRightAscention(0.0f);
            return;
        }

        float ra = Mathf.Acos(mNodeLine.x / mN);
        setRightAscention(ra);
    }

    void calculateArgumentOfPerigee()
    {
        if (mInclination % Mathf.PI == 0.0f)
        {
            float ap1 = calculateAngleThroughDot(mPosition, new Vector3(1.0f, 0.0f, 0.0f));
            if (mPosition.y < 0.0f)
            {
                ap1 = 2.0f * Mathf.PI - ap1;
            }

            float omega = ap1 - mTrueAnomaly;
            if (omega < 0)
            {
                omega = 2.0f * Mathf.PI + omega;
            }

            setArgumentOfPerigee(omega);
            return;
        }

        float ap = calculateAngleThroughDot(mNodeLine, mEccentricity);
        setArgumentOfPerigee(ap);
    }

    void calculateTrueAnomaly()
    {
        float ta = calculateAngleThroughDot(mEccentricity, mPosition);
        setTrueAnomaly(ta);
    }

    void calculateTrueAnomalyInfinity()
    {
        float taInf = 0.0f;
        if (mOrbitType == OrbitTypes.HYPERBOLIC)
        {
            taInf = Mathf.Acos(-1.0f / mE);
        }
        setTrueAnomalyInfinity(taInf);
    }

    // set methods
    void setGameObjectPosition()
    {
        transform.position = mPosition * mScale;
    }

    void setGameObjectLineRenderer()
    {
        mOrbitPathLineRenderer.positionCount = mOrbitPath.Length;
        mOrbitPathLineRenderer.SetPositions(mOrbitPath);
        mOrbitPathLineRenderer.startWidth = 0.2f;
        mOrbitPathLineRenderer.endWidth = 0.2f;
        mOrbitPathLineRenderer.startColor = Color.green;
        mOrbitPathLineRenderer.endColor = Color.green;
    }

    void setMu(float aMu)
    {
        mMu = aMu;
    }

    void setPosition(Vector3 aPosition)
    {
        mPosition = aPosition;
        mR = aPosition.magnitude;
    }

    void setVelocity(Vector3 aVelocity)
    {
        mVelocity = aVelocity;
        mV = aVelocity.magnitude;
    }

    void setAngularMomentum(Vector3 aAngularMomentum)
    {
        mAngularMomentum = aAngularMomentum;
        mH = aAngularMomentum.magnitude;
    }

    void setNodeLine(Vector3 aNodeLine)
    {
        mNodeLine = aNodeLine;
        mN = aNodeLine.magnitude;
    }

    void setEccentricity(Vector3 aEccentricity)
    {
        float mag = aEccentricity.magnitude;
        if (mag <= 0.0001)
        {
            aEccentricity = new Vector3(0.0f, 0.0f, 0.0f);
        }

        mEccentricity = aEccentricity;
        mE = mag;
        setOrbitType();
    }

    void setOrbitType()
    {
        float error = 0.0001f;
        if (mE <= error)
        {
            mOrbitType = OrbitTypes.CIRCULAR;
        }
        else if (mE > error && mE <= 1.0f-error)
        {
            mOrbitType = OrbitTypes.ELLIPTICAL;
        }
        else if (mE > 1.0f-error && mE <= 1.0+error)
        {
            mOrbitType = OrbitTypes.PARABLOIC;
        }
        else
        {
            mOrbitType = OrbitTypes.HYPERBOLIC;
        }
    }

    void setInclination(float aInclination)
    {
        // inclination is between 0 and 180 degrees
        mInclination = Mathf.Clamp(aInclination, 0.0f, Mathf.PI);
    }

    void setRightAscention(float aRightAscention)
    {
        // quadrant check
        mRightAscention = aRightAscention;
        if (mNodeLine.y < 0.0f)
        {
            mRightAscention = 2.0f * Mathf.PI - aRightAscention;
        }
    }

    void setArgumentOfPerigee(float aArgumentOfPerigee)
    {
        // quadrant check
        mArgumentOfPerigee = aArgumentOfPerigee;
        if (mEccentricity.z < 0)
        {
            mArgumentOfPerigee = 2.0f * Mathf.PI - aArgumentOfPerigee;
        }
    }

    void setTrueAnomaly(float aTrueAnomaly)
    {
        // quadrant check
        mTrueAnomaly = aTrueAnomaly;
        float vro = Vector3.Dot(mPosition, mVelocity) / mR;
        if (vro < 0.0f)
        {
            mTrueAnomaly = 2.0f * Mathf.PI - aTrueAnomaly;
        }
    }

    void setTrueAnomalyInfinity(float aTrueAnomalyInfinity)
    {
        mTrueAnomolyInfinity = aTrueAnomalyInfinity;
    }

    // other functions
    float calculateAngleThroughDot(Vector3 aVec1, Vector3 aVec2)
    {
        float error = 0.0001f;
        float mag1 = aVec1.magnitude;
        float mag2 = aVec2.magnitude;
        float numerator = Vector3.Dot(aVec1, aVec2);
        float denominator = mag1 * mag2;

        if (denominator <= error)
        {
            return 0.0f;
        }

        float argument = numerator / denominator;
        float angle = 0.0f;
        if (Mathf.Abs(argument) >= 1.0f)
        {
            if (argument < 0.0f)
            {
                angle = Mathf.PI;
            }
        }
        else
        {
            angle = Mathf.Acos(argument);
        }

        return angle;
    }

    void calculatePerifocal2GeocentricEquatorialTransformation()
    {
        // Rotation from perifocal frame to the geocentric equitorial frame
        float raS = Mathf.Sin(mRightAscention);
        float raC = Mathf.Cos(mRightAscention);
        float iS = Mathf.Sin(mInclination);
        float iC = Mathf.Cos(mInclination);
        float apS = Mathf.Sin(mArgumentOfPerigee);
        float apC = Mathf.Cos(mArgumentOfPerigee);

        mRotationMatrix[0, 0] = -raS * iC * apS + raC * apC;
        mRotationMatrix[0, 1] = -raS * iC * apC - raC * apS;
        mRotationMatrix[0, 2] = raS * iS;

        mRotationMatrix[1, 0] = raC * iC * apS + raS * apC;
        mRotationMatrix[1, 1] = raC * iC * apC - raS * apS;
        mRotationMatrix[1, 2] = -raC * iS;

        mRotationMatrix[2, 0] = iS * apS;
        mRotationMatrix[2, 1] = iS * apC;
        mRotationMatrix[2, 2] = iC;
    }

    Vector3 rotatePerifocalVectorToStateVector(Vector3 aPerifocalFrame)
    {
        Vector3 stateVectorFull = new Vector3(mRotationMatrix[0, 0] * aPerifocalFrame.x + mRotationMatrix[0, 1] * aPerifocalFrame.y + mRotationMatrix[0, 2] * aPerifocalFrame.z,
                                              mRotationMatrix[1, 0] * aPerifocalFrame.x + mRotationMatrix[1, 1] * aPerifocalFrame.y + mRotationMatrix[1, 2] * aPerifocalFrame.z,
                                              mRotationMatrix[2, 0] * aPerifocalFrame.x + mRotationMatrix[2, 1] * aPerifocalFrame.y + mRotationMatrix[2, 2] * aPerifocalFrame.z);

        Vector3 stateVectorScalled = new Vector3(stateVectorFull.x * mScale,
                                                 stateVectorFull.y * mScale,
                                                 stateVectorFull.z * mScale);

        return stateVectorScalled;
    }
}
