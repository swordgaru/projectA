using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Ting.PerformanceTest
{
    // C# 메모리 관리 – 구조체 vs 클래스
    // 유니티는 3개에서 4개 정도의 변수를 가지는 경우 구조체를 사용합니다. position, color, quaternion, rotation, scale 등이 모두 구조체 입니다
    // https://ronniej.sfuh.tk/c-%EB%A9%94%EB%AA%A8%EB%A6%AC-%EA%B4%80%EB%A6%AC-%EA%B5%AC%EC%A1%B0%EC%B2%B4-vs-%ED%81%B4%EB%9E%98%EC%8A%A4/

    // 유니티 Vector3 new 는 스택에 생성된다
    // https://3dmpengines.tistory.com/1566

    // 닷넷 가비지 컬렉션
    // http://www.simpleisbest.net/post/2011/04/01/Review-NET-Garbage-Collection.aspx

    public struct DataStruct
    {
        public string name { get; set; }
        public int id { get; set;  }        
    }

    public class DataClass
    {
        public string name { get; set; }
        public int id { get; set; }
    }

    public class MemoryTest : MonoBehaviour
    {
        [SerializeField]
        bool structTest = false;
        [SerializeField]
        bool classTest = false;

        private void Update()
        {
            if(Input.GetKeyDown(KeyCode.S))
                structTest = !structTest;

            if (Input.GetKeyDown(KeyCode.C))
                classTest = !classTest;

            GcTest();
        }

        public void GcTest()
        {            
            for(int i  = 0; i < 100000; i++)
            {
                if (structTest)
                {
                    DataStruct dataStruct = new DataStruct();
                    dataStruct.name = "DataStruct";
                    dataStruct.id = i;
                }

                if (classTest)
                {
                    DataClass dataClass = new DataClass();
                    dataClass.name = "DataClass";
                    dataClass.id = i;
                }
            }
        }
    }
}