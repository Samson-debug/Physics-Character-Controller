using Unity.Mathematics;
using UnityEngine;

public static class UtilsMath
{
    /// <returns>returns shortest rotation to get to rotation A from rotation B</returns>
    public static Quaternion ShortestRotation(Quaternion _a, Quaternion _b)
    {
        return _b * Quaternion.Inverse(_a);
    }
}