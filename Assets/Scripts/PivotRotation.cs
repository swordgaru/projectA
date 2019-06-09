using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PivotRotation : MonoBehaviour
{
    public const int SQUAD_MEMBER_COUNT = 12;

    GameObject[] squadMembers;

	// Use this for initialization
	void Start () {        
        squadMembers = new GameObject[SQUAD_MEMBER_COUNT];

        int lineMax = 6;

        int colIndex = 0;
        int rowIndex = 0;
        
        for (int i = 0; i < SQUAD_MEMBER_COUNT; i++)
        {
            squadMembers[i] = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            squadMembers[i].name = $"Squad {i}";

            rowIndex = i % lineMax;
            colIndex = i / lineMax;

            squadMembers[i].transform.position = new Vector3(rowIndex, 0, colIndex);
        }

        
    }
	
	// Update is called once per frame
	void Update () {
		
	}
}
