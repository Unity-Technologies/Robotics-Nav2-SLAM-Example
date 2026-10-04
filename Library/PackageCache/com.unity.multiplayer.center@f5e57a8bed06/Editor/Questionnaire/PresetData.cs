using System;
using Unity.Multiplayer.Center.Common;
using UnityEngine;

namespace Unity.Multiplayer.Center.Questionnaire
{
    /// <summary>
    /// The predefined answers for presets. This is meant to be read only and saved with the questionnaire
    /// </summary>
    [Serializable]
    internal class PresetData
    {
        /// <summary>
        /// The list of presets for which we have predefined answers (hopefully all possible values except None)
        /// This should contain as many values as the <see cref="Answers"/> array.
        /// </summary>
        public Preset[] Presets;
        
        /// <summary>
        /// The predefined answers for each preset, in the same order as <see cref="Presets"/>.
        /// This should contain as many values as the <see cref="Presets"/> array.
        /// </summary>
        public AnswerData[] Answers;
    }
}
