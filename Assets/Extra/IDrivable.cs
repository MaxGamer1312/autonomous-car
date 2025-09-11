using System;
using UnityEngine;

public abstract class Drivable : MonoBehaviour
{
    public abstract void Drive(float inputPower, float turnPower);

    public abstract void Restart();
}