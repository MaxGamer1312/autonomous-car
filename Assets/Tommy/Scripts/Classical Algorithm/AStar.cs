using System;
using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;

namespace Tommy.Scripts.Classical_Algorithm
{
    [System.Serializable]
    public class AStar
    {
        // all numbers must be 0 to 100
        public int[,] grid;
        private bool[,] obstacles;
        
        
        // for debugging purposes
        private Texture2D texture;
        private const int textureSize = 128;
        private byte[] pixelData; // normally would use color array, but byte operation is faster.

        public Vector2 carPosition; // represented as a percent
        public float carAngle = 0;

        public AStar(int width, int height)
        {
            grid = new int[width, height];
            obstacles = new bool[width, height];
            
            pixelData = new byte[3 * textureSize * textureSize];
        }

        public void InitDebugGrid(RawImage image)
        {
            if (image.texture == null)
            {
                texture = new Texture2D(textureSize, textureSize, TextureFormat.RGB24, false);
                image.texture = texture;
            }
        }

        public void DebugGrid(RawImage image)
        {
            Vector2 carHead = new Vector2(Mathf.Cos(carAngle), Mathf.Sin(carAngle)) * .04f + carPosition;
            for (int i = 0; i < pixelData.Length; i+=3)
            {
                int r = i / 3 / textureSize;
                int c = i / 3 % textureSize;
                int gr = (int) Mathf.Lerp(0, grid.GetLength(0), (float)r / textureSize);
                int gc = (int) Mathf.Lerp(0, grid.GetLength(1), (float)c / textureSize);

                gr = obstacles[gr, gc] ? 100 : gr;
                gc = obstacles[gr, gc] ? 100 : gc;
                
                byte color = (byte)Mathf.Lerp(0, 255, grid[gr, gc] / 100f);
                pixelData[i + 0] = color; // R
                pixelData[i + 1] = color; // G
                pixelData[i + 2] = color; // B

                Vector2 pixelXY = new Vector2((float)r / textureSize, (float)c / textureSize);
                if (Vector2.Distance(pixelXY, carPosition) < .03f || Vector2.Distance(pixelXY, carHead) < .01f)
                {
                    pixelData[i + 0] = 204; // R
                    pixelData[i + 1] = 85; // G
                    pixelData[i + 2] = 0; // B
                }
                else if (Vector2.Distance(pixelXY, carPosition) < .045f || Vector2.Distance(pixelXY, carHead) < .025f)
                {
                    pixelData[i + 0] = 255; // R
                    pixelData[i + 1] = 255; // G
                    pixelData[i + 2] = 255; // B
                }
            }
            texture.SetPixelData(pixelData, 0);
            texture.Apply();
        }

        public List<AStarState> ComputeShortestPath(Vector3 startPose, Vector3 endPose)
        {
            // params:
            // (x,y,heading)
            
            //(inputPower, turnAngle, cost)
            Vector3[] actions =
            {
                new Vector3(-1, -14.3f, .2f),
                new Vector3(-1, 0, .1f),
                new Vector3(-1, 14.3f, .2f),
                new Vector3(1, -14.3f, .05f),
                new Vector3(1, 0, 0f),
                new Vector3(1, 14.3f, .05f),
            };
            Heap<AStarState> openList = new Heap<AStarState>(3);
            HashSet<AStarState> closeList = new HashSet<AStarState>();
            AStarState initialPose = new AStarState();
            initialPose.pose = startPose;
            initialPose.gCost = 0;
            initialPose.hCost = Vector3.Distance(startPose, endPose);
            
            openList.Add(initialPose);
            int iterations = 0;
            while (openList.Count >= 1)
            {
                if (iterations >= 4000)
                {
                    Debug.Log("Ended early");
                    break;
                }
                AStarState pop = openList.RemoveFirst();

                if (Mathf.Sqrt(pop.pose.x * pop.pose.x + pop.pose.y * pop.pose.y) < 3 &&
                    Math.Abs(Mathf.DeltaAngle(pop.pose.z, endPose.z)) <= 30)
                {
                    // goal found
                    return reconstructList(pop);
                }
                if (closeList.Contains(pop))
                {
                    
                }

                iterations++;
            }
            
            Debug.Log(closeList.Count);
            return null;
        }

        private List<AStarState> reconstructList(AStarState endPose)
        {
            Stack<AStarState> stack = new Stack<AStarState>();
            List<AStarState> result = new List<AStarState>();
            AStarState current = endPose;
            while (current != null)
            {
                stack.Push(current);
                current = current.parent;
            }
            
            while (stack.Count >= 1)
            {
                result.Add(stack.Pop());
            }

            return result;
        }
    }
}