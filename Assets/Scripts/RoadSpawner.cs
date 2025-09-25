using System.Runtime.CompilerServices;
using UnityEngine;

public class RoadSpawner
{
    private Transform parentRoads;
    private Vector3[][] possibleLocations;

    public RoadSpawner(Transform parentRoads)
    {
        this.parentRoads = parentRoads;
        BuildPossibleLocations();
    }

    public void RandomLocation(Transform obj)
    {
        if (parentRoads == null || possibleLocations == null) return;

        // collect valid (non-null) segments
        int n = possibleLocations.Length;
        int tries = 0;
        Vector3[] linePicked = null;

        while (tries < n)
        {
            int index = Random.Range(0, n);
            linePicked = possibleLocations[index];
            if (linePicked != null) break;
            tries++;
        }
        if (linePicked == null) return;

        float t = Random.Range(0.1f, 0.9f);
        Vector3 randomPointLocalToParent = Vector3.Lerp(linePicked[0], linePicked[1], t);

        // place in correct space
        if (obj.parent == parentRoads)
            obj.localPosition = randomPointLocalToParent;
        else
            obj.position = parentRoads.TransformPoint(randomPointLocalToParent);

        float fixedX = obj.eulerAngles.x;
        float fixedZ = obj.eulerAngles.z;
        float randomY = Random.Range(0f, 360f);
        obj.rotation = Quaternion.Euler(fixedX, randomY, fixedZ);
    }
    private void BuildPossibleLocations()
    {
        if (parentRoads == null) return;

        int n = parentRoads.childCount;
        possibleLocations = new Vector3[n][];

        Transform common = parentRoads;
        //Sets possible location for each road and adds them to possibleLocations
        for (int i = 0; i < n; i++)
        {
            Transform child = parentRoads.GetChild(i);
            BoxCollider box = child.GetComponent<BoxCollider>();
            if (box == null)
            {
                possibleLocations[i] = null;
                continue;
            }
            //normalized positions of the four top face corners of a unit cube collider
            Vector3[] localTopCorners = new Vector3[4]
            {
                    new Vector3(-0.5f, 0.5f, -0.5f),
                    new Vector3(-0.5f, 0.5f, 0.5f),
                    new Vector3( 0.5f, 0.5f, -0.5f),
                    new Vector3( 0.5f, 0.5f,  0.5f),
            };

            /*
                Scale the unit corners by the collider’s actual size
                Shift by the collider’s center
                Transform from local space into world coordinates
                Raise them up by 1 unit (+1f on y), so the spawn point is slightly above the road
            */
            Vector3[] worldTop = new Vector3[4];
            for (int j = 0; j < 4; j++)
            {
                Vector3 scaled = Vector3.Scale(localTopCorners[j], box.size);
                worldTop[j] = box.transform.TransformPoint(scaled + box.center);
                worldTop[j].y += 1f;
            }

            //express coordinates relative to parentRoads
            Vector3[] commonLocalTop = new Vector3[4];
            for (int j = 0; j < 4; j++)
            {
                commonLocalTop[j] = common.InverseTransformPoint(worldTop[j]);
            }

            /*
	            Finds the smallest and largest x, z values among the 4 corners
	            Finds the highest y (so you know the flat top level)
            */
            float minX = Mathf.Min(commonLocalTop[0].x, Mathf.Min(commonLocalTop[1].x, Mathf.Min(commonLocalTop[2].x, commonLocalTop[3].x)));
            float maxX = Mathf.Max(commonLocalTop[0].x, Mathf.Max(commonLocalTop[1].x, Mathf.Max(commonLocalTop[2].x, commonLocalTop[3].x)));
            float minZ = Mathf.Min(commonLocalTop[0].z, Mathf.Min(commonLocalTop[1].z, Mathf.Min(commonLocalTop[2].z, commonLocalTop[3].z)));
            float maxZ = Mathf.Max(commonLocalTop[0].z, Mathf.Max(commonLocalTop[1].z, Mathf.Max(commonLocalTop[2].z, commonLocalTop[3].z)));
            float topY = Mathf.Max(commonLocalTop[0].y, Mathf.Max(commonLocalTop[1].y, Mathf.Max(commonLocalTop[2].y, commonLocalTop[3].y)));

            //Determine road orientation
            float spanX = maxX - minX;
            float spanZ = maxZ - minZ;

            //Pick line segment across the top
            Vector3 pA, pB;
            if (spanX >= spanZ)
            {
                float midZ = 0.5f * (minZ + maxZ);
                pA = new Vector3(minX, topY, midZ);
                pB = new Vector3(maxX, topY, midZ);
            }
            else
            {
                float midX = 0.5f * (minX + maxX);
                pA = new Vector3(midX, topY, minZ);
                pB = new Vector3(midX, topY, maxZ);
            }

            possibleLocations[i] = new Vector3[2] { pA, pB };
        }
    }       
}


