using System.Collections;
using Tommy.Scripts.Training;
using UnityEngine;
using System.Collections.Generic;

public class MultipleCarSpawner : MonoBehaviour
{
    // Main
    [SerializeField] private int maxCars;
    [SerializeField] private GameObject car;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        for (int i = 0; i < maxCars-1; i++){
            SpawnCar();
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void SpawnCar(){
        Instantiate(car.transform, new Vector3(0,0,0), new Quaternion(0,0,0,0));
        return;
    }


}
