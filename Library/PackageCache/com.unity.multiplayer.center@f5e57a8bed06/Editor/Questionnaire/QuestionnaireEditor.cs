using UnityEngine;
using UnityEditor;
using UnityEditor.Callbacks;
using UnityEditor.UIElements;
using UnityEngine.UIElements;

namespace Unity.Multiplayer.Center.Questionnaire
{
    /// <summary>
    /// Because of the "questionnaire" extension, the default inspector is not shown.
    /// Double clicking on the asset will open this custom inspector (in debug mode), which has a way to force saving
    /// the asset.
    /// </summary>
    [CustomEditor(typeof(QuestionnaireObject))]
    internal class QuestionnaireEditor : Editor
    {
        public override VisualElement CreateInspectorGUI()
        {
            var so = new SerializedObject(target);
            var root = new VisualElement();
            var questionnaire = (QuestionnaireObject) target;
            root.Add(new Button(() => questionnaire.ForceReload() ){text = "Load Changes From Disk", tooltip = "Use when editing with external editor."});
            root.Add(new Button(() => questionnaire.ForceSave() ){text = "Save local changes", tooltip = "Use when editing in inspector."});
            var inspector = new PropertyField(so.FindProperty("Questionnaire"));
            root.Add(inspector);
            return root;
        }
        
        [OnOpenAsset(1)]
        public static bool OpenMyCustomAsset(int instanceID, int line)
        {
            if (!EditorPrefs.GetBool("DeveloperMode")) return false;
            var asset = EditorUtility.InstanceIDToObject(instanceID);
            var path = AssetDatabase.GetAssetPath(asset);
            if(string.IsNullOrEmpty(path) || !path.EndsWith("questionnaire"))
                return false;
        
            Selection.activeObject = QuestionnaireObject.instance;
            return true;
        }
    }
}