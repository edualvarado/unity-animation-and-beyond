using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Wave : MonoBehaviour
{
    private MeshRenderer meshRenderer;
    private MeshFilter meshFilter;
    
    // Waves strenght
    [SerializeField]
    private float wavesStrenght = 1;
    
    // Waves speed
    [SerializeField]
    private float wavesSpeed = 1f;
    
    // Waves direction in degrees
    [SerializeField]
    private float wavesDirection = 0f;
    
    // Texture scroll speed
    [SerializeField]
    private float textureScrollSpeed = 0.5f;
    
    /// <summary>
    /// Unity method called on first frame.
    /// </summary>
    private void Start()
    {
        // Getting references to components.
        meshRenderer = GetComponent<MeshRenderer>();
        meshFilter = GetComponent<MeshFilter>();
    }
    
    /// <summary>
    /// Unity method called every frame.
    /// </summary>
    private void Update()
    {
        // Scrolling water texture
        float xSpeed = Mathf.Sin(wavesDirection * Mathf.Deg2Rad);
        float zSpeed = Mathf.Cos(wavesDirection * Mathf.Deg2Rad);
        
        meshRenderer.material.mainTextureOffset += new Vector2(xSpeed, zSpeed) * textureScrollSpeed * Time.deltaTime;
        // Getting references
        var mesh = meshFilter.mesh;
        var verts = mesh.vertices;
        
        // Changing vertice elevation.
        for (int i = 0; i < verts.Length; i++)
        {
            float xOffset = verts[i].x * xSpeed;
            float zOffset = verts[i].z * zSpeed;
            float elevation = Mathf.Sin(xOffset + zOffset + Time.time * wavesSpeed) * wavesStrenght;
            verts[i].y = elevation;
        }
        
        // Applying changes to the mesh
        mesh.vertices = verts;
    }
}