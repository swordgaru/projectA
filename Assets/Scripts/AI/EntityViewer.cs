using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Ting.AI
{
    public class EntityViewer : MonoBehaviour
    {
        GameObject triangleObject;
        Triange triange;

        public struct Triange
        {
            public Vector3[] vertices;
            public int[] triangles;
        }
        
        // https://www.youtube.com/watch?list=PLp2zpxnnEFue9M04Q6Dkqxkp2vPrt7EO8&time_continue=18&v=ThWyvoqWjhE
        public void CreateTriange()
        {
            triangleObject = new GameObject("triangleObject");
            triangleObject.transform.parent = this.transform;
            triangleObject.transform.localPosition = Vector3.zero;
            triangleObject.transform.localRotation = Quaternion.AngleAxis(90, Vector3.right);
            triangleObject.transform.localScale = new Vector3(1, 1, 1);

            MeshFilter mf = triangleObject.AddComponent<MeshFilter>();
            MeshRenderer mr = triangleObject.AddComponent<MeshRenderer>();

            mr.material = Resources.Load<Material>("Material/MatTriange");
            Mesh m = new Mesh();
            mf.mesh = m;

            triange.vertices = new[]
            {
                new Vector3(0, 0, 0),
                //new Vector3(0, 1, 0),
                new Vector3(0.5f, 0.866025404f, 0),
                new Vector3(1, 0, 0)
            };

            m.vertices = triange.vertices;

            triange.triangles = new[] { 0, 1, 2 };
            m.triangles = triange.triangles;

            GameObject pfArrow = Resources.Load("pfArrow") as GameObject;
            GameObject arrow = Instantiate(pfArrow, Vector3.zero, Quaternion.AngleAxis(90, Vector3.right));
            if (arrow)
            {
                arrow.transform.parent = this.transform;
                arrow.transform.localScale = new Vector3(1, 1, 1);
                arrow.transform.localPosition = new Vector3(0.5f, 0, 1f);
            }
        }
    }
}