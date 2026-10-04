using System;
using UnityEditor;
using UnityEngine;

namespace Unity.Multiplayer.Center.Questionnaire
{
    /// <summary>
    /// The questionnaire scriptable object, used to store and edit the data
    /// </summary>
    [FilePath("Packages/com.unity.multiplayer.center/Editor/Questionnaire/Questionnaire.questionnaire", FilePathAttribute.Location.ProjectFolder)]
    internal class QuestionnaireObject : ScriptableSingleton<QuestionnaireObject>
    {
        public QuestionnaireData Questionnaire;
        
        
        public void ForceReload()
        {
            DestroyImmediate(QuestionnaireObject.instance);
            var questions = QuestionnaireObject.instance.Questionnaire;
        }
        
        public void ForceSave()
        {
            base.Save(saveAsText:true);
            AssetDatabase.Refresh();
        }
    }
}
