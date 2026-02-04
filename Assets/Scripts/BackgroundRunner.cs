using UnityEngine;

public class BackgroundRunner : MonoBehaviour
{
    void Start() => Application.runInBackground = true;
}
