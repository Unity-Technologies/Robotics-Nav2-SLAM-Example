using UnityEditor;

namespace Unity.Multiplayer.Center.Recommendations
{
    /// <summary>
    /// Current way to fetch recommendation data from disk. Will probably change to fetching something from a server.
    /// </summary>
    [FilePath(PathConstants.RecommendationDataPath, FilePathAttribute.Location.ProjectFolder)]
    internal class RecommenderSystemDataObject : ScriptableSingleton<RecommenderSystemDataObject>
    {
        public RecommenderSystemData RecommenderSystemData;
        
#if MULTIPLAYER_CENTER_DEV_MODE  
        [MenuItem("Multiplayer/Recommendations/Populate Default Recommendation Data")]
        public static void CreateDefaultInstance()
        {
            instance.RecommenderSystemData = RecommendationAssetUtils.PopulateDefaultRecommendationData();
            instance.ForceSave();
        }

        void ForceSave()
        {
            base.Save(saveAsText:true);
            AssetDatabase.Refresh();
            DestroyImmediate(this);
        }
#endif
    }

    static class PathConstants
    {
        const string k_RootPath = "Packages/com.unity.multiplayer.center/Editor/Recommendations/";
        public const string RecommendationDataPath = k_RootPath + "RecommendationData_6000.0.recommendations";
    }
}