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
        
        
        // // for debugging purposes
        // private Texture2D texture;
        // private const int textureSize = 128;
        // private byte[] pixelData; // normally would use color array, but byte operation is faster.
        //
        // public Vector2 carPosition; // represented as a percent
        // public float carAngle = 0;
        
        public AStar(int width, int height)
        {
            grid = new int[width, height];
            obstacles = new bool[width, height];
            
            //pixelData = new byte[3 * textureSize * textureSize];
        }
        
        // public void InitDebugGrid(RawImage image)
        // {
        //     if (image.texture == null)
        //     {
        //         texture = new Texture2D(textureSize, textureSize, TextureFormat.RGB24, false);
        //         image.texture = texture;
        //     }
        // }
        //
        // public void DebugGrid(RawImage image)
        // {
        //     Vector2 carHead = new Vector2(Mathf.Cos(carAngle), Mathf.Sin(carAngle)) * .04f + carPosition;
        //     for (int i = 0; i < pixelData.Length; i+=3)
        //     {
        //         int r = i / 3 / textureSize;
        //         int c = i / 3 % textureSize;
        //         int gr = (int) Mathf.Lerp(0, grid.GetLength(0), (float)r / textureSize);
        //         int gc = (int) Mathf.Lerp(0, grid.GetLength(1), (float)c / textureSize);
        //
        //         gr = obstacles[gr, gc] ? 100 : gr;
        //         gc = obstacles[gr, gc] ? 100 : gc;
        //         
        //         byte color = (byte)Mathf.Lerp(0, 255, grid[gr, gc] / 100f);
        //         pixelData[i + 0] = color; // R
        //         pixelData[i + 1] = color; // G
        //         pixelData[i + 2] = color; // B
        //
        //         Vector2 pixelXY = new Vector2((float)r / textureSize, (float)c / textureSize);
        //         if (Vector2.Distance(pixelXY, carPosition) < .03f || Vector2.Distance(pixelXY, carHead) < .01f)
        //         {
        //             pixelData[i + 0] = 204; // R
        //             pixelData[i + 1] = 85; // G
        //             pixelData[i + 2] = 0; // B
        //         }
        //         else if (Vector2.Distance(pixelXY, carPosition) < .045f || Vector2.Distance(pixelXY, carHead) < .025f)
        //         {
        //             pixelData[i + 0] = 255; // R
        //             pixelData[i + 1] = 255; // G
        //             pixelData[i + 2] = 255; // B
        //         }
        //     }
        //     texture.SetPixelData(pixelData, 0);
        //     texture.Apply();
        // }
    }
}