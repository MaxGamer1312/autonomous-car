using UnityEditor;
using UnityEngine;

public class LinuxBuilder
{
    public static void BuildLinux64()
    {
        #if UNITY_EDITOR
        string[] scenes = { "Assets/Tommy/Scenes/CrunchDay.unity" };
        string buildPath = "Builds/Linux/MyEnv.x86_64";

        BuildPlayerOptions buildPlayerOptions = new BuildPlayerOptions();
        buildPlayerOptions.scenes = scenes;
        buildPlayerOptions.locationPathName = buildPath;
        buildPlayerOptions.target = BuildTarget.StandaloneLinux64;
        buildPlayerOptions.options = BuildOptions.EnableHeadlessMode;

        BuildPipeline.BuildPlayer(buildPlayerOptions);
        #endif
    }
}
