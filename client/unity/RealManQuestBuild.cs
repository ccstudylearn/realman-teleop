using System;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

public static class RealManQuestBuild
{
    public static void Build()
    {
        var output = Environment.GetEnvironmentVariable("REALMAN_QUEST_APK");
        if (string.IsNullOrEmpty(output))
            throw new Exception("REALMAN_QUEST_APK output path is required");
        Directory.CreateDirectory(Path.GetDirectoryName(output));
        PlayerSettings.SetApplicationIdentifier(BuildTargetGroup.Android, "com.xrobotoolkit.client.quest.realman");
        PlayerSettings.productName = "XRoboToolkit RealMan";
        PlayerSettings.bundleVersion = "1.0.1-realman.2";
        PlayerSettings.Android.bundleVersionCode = 10102;
        PlayerSettings.Android.targetArchitectures = AndroidArchitecture.ARM64;
        PlayerSettings.Android.minSdkVersion = (AndroidSdkVersions)32;
        PlayerSettings.Android.targetSdkVersion = (AndroidSdkVersions)35;
        PlayerSettings.SetScriptingBackend(BuildTargetGroup.Android, ScriptingImplementation.IL2CPP);
        PlayerSettings.Android.useCustomKeystore = false;
        EditorUserBuildSettings.buildAppBundle = false;
        var scenes = EditorBuildSettings.scenes.Where(s => s.enabled).Select(s => s.path).ToArray();
        if (scenes.Length == 0) throw new Exception("No enabled build scenes");
        var report = BuildPipeline.BuildPlayer(new BuildPlayerOptions {
            scenes = scenes, locationPathName = output, target = BuildTarget.Android,
            options = BuildOptions.None
        });
        if (report.summary.result != BuildResult.Succeeded)
            throw new Exception("Quest build failed: " + report.summary.result);
        Debug.Log("REALMAN_QUEST_BUILD_SUCCEEDED " + output);
    }
}
