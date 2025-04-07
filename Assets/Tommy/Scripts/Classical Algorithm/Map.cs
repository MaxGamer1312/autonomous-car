using System;
using UnityEditor.Experimental.GraphView;
using UnityEngine;

namespace Tommy.Scripts.Classical_Algorithm
{
    public class Map : MonoBehaviour
    {
        public BoxCollider bounds;
        [Range(1, 200)]
        public int boardWidth;  // the number of squares that make up the length of the board
        public float tileWidth;
        public bool debugMode;

        public bool[,] grid;
        private int prevWidth = 0;
        private LayerMask layer;
        private void OnValidate()
        {
            if (prevWidth != boardWidth)
            {
                grid = new bool[boardWidth, boardWidth];
                prevWidth = boardWidth;
                
                
                layer = LayerMask.GetMask("Road");
                Bounds boundingBox = bounds.bounds;
                tileWidth = boundingBox.extents.z * 2 / boardWidth;
                
                Vector3 min = boundingBox.min + new Vector3(tileWidth, 0, tileWidth) / 2;
                for (int row = 0; row < grid.GetLength(0); row++)
                {
                    for (int col = 0; col < grid.GetLength(1); col++)
                    {
                        Vector3 position = min;
                        position.y = 0.6f;
                        position.x = position.x + tileWidth * row;
                        position.z = position.z + tileWidth * col;
                        grid[row, col] = Physics.Raycast(position, Vector3.down, 1, layer);
                    }
                }
            }
            
        }
        
        void OnDrawGizmos()
        {
            if (!debugMode || grid == null) return;
            Bounds boundingBox = bounds.bounds;
            tileWidth = boundingBox.extents.z * 2 / boardWidth;

            Vector3 debugTileSize = new Vector3(tileWidth * .9f, 0f, tileWidth * .9f);
            Vector3 min = boundingBox.min + new Vector3(tileWidth, 0, tileWidth) / 2;
            for (int row = 0; row < grid.GetLength(0); row++)
            {
                for (int col = 0; col < grid.GetLength(1); col++)
                {
                    Vector3 position = min;
                    position.y = 0.6f;
                    position.x = position.x + tileWidth * row;
                    position.z = position.z + tileWidth * col;
                    Gizmos.color = grid[row, col] ? new Color(1, 0, 0, .5f) : new Color(0, 0, 0, .5f);
                    Gizmos.DrawCube(position, debugTileSize);
                }
            }
        }

        public ref bool WorldToCell(float x, float z)
        {
            x = Mathf.InverseLerp( -bounds.bounds.extents.x, bounds.bounds.extents.x, x - .5f);
            z = Mathf.InverseLerp( -bounds.bounds.extents.z, bounds.bounds.extents.z, z - .5f);

            int row = Mathf.RoundToInt(Mathf.Lerp(0, boardWidth, x));
            int col = Mathf.RoundToInt(Mathf.Lerp(0, boardWidth, z));

            row = Mathf.Clamp(row, 0, grid.GetLength(0) - 1);
            col = Mathf.Clamp(col, 0, grid.GetLength(1) - 1);
            
            return ref grid[row, col];
        }

        public bool OutOfBounds(float x, float z)
        {
            return x < bounds.bounds.min.x || x > bounds.bounds.max.x ||
                   z < bounds.bounds.min.z || z > bounds.bounds.max.z;
        }
    }
}