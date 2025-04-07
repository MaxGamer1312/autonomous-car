using System;
using UnityEngine;
using UnityEngine.UI;

public class EnviromentManager : MonoBehaviour
{
    [Range(-24, 19)]
    public static float distance = 0;
    [Range(0, 25)]
    public static float sideVariance = 0;
    [Range(0, 180)]
    public static float rotationVariance = 0;

    public static bool alternateRoads;

    public GameObject settings;

    public Slider distSlider;
    public Slider sideSlider;
    public Slider rotSlider;
    public Slider altSlider;

    public void Start()
    {
        distSlider.onValueChanged.AddListener((val) =>
        {
            distance = val;
        });
        sideSlider.onValueChanged.AddListener((val) =>
        {
            sideVariance = val;
        });
        rotSlider.onValueChanged.AddListener((val) =>
        {
            rotationVariance = val;
        });
        altSlider.onValueChanged.AddListener((val) =>
        {
            alternateRoads = Mathf.Approximately(val, 1);
        });
    }

    public void Hide()
    {
        settings.SetActive(!settings.activeInHierarchy);
    }
    
}
