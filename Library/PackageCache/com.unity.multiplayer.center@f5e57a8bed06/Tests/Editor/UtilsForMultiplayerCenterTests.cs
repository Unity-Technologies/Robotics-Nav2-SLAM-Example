using System;
using System.IO;
using NUnit.Framework;
using Unity.Multiplayer.Center.Common;
using Unity.Multiplayer.Center.Questionnaire;
using Unity.Multiplayer.Center.Window;
using UnityEditor;
using UnityEngine;
using Object = UnityEngine.Object;

namespace Unity.MultiplayerCenterTests
{
    /// <summary>
    /// This is meant to be a utility class for MultiplayerCenter tests (including the project tests).
    /// </summary>
    internal static class UtilsForMultiplayerCenterTests
    {
        public static void CloseMultiplayerCenterWindow()
        {
            var allWindows = Resources.FindObjectsOfTypeAll<MultiplayerCenterWindow>();
            if (allWindows.Length <= 0) return;
            foreach (var window in allWindows)
            {
                window.Close();
                UnityEngine.Object.DestroyImmediate(window);
            }
        }
        
        public static void ResetUserChoices()
        {
            UserChoicesObject.instance.UserAnswers = new AnswerData();
            UserChoicesObject.instance.Preset = new Preset();
            UserChoicesObject.instance.SelectedSolutions = new SelectedSolutionsData();
            UserChoicesObject.instance.Save();
        }

        public static void PopulateUserAnswersForPresetAndPlayerCount(Preset preset, int playerCount)
        {
            //TODO: also assert player count has proper value
            Assert.AreNotEqual(Preset.None, preset);
            UserChoicesObject.instance.Preset = preset;
            Logic.Update(UserChoicesObject.instance.UserAnswers, CreatePlayerCountAnswer(playerCount));
            UserChoicesObject.instance.Save();
            
            var (resultAnswerData, recommendation) = Logic.ApplyPresetToAnswerData(
                UserChoicesObject.instance.UserAnswers, preset, QuestionnaireObject.instance.Questionnaire);

            Assert.NotNull(recommendation);
            
            UserChoicesObject.instance.UserAnswers = resultAnswerData;
            UserChoicesObject.instance.Save();
        }
        
        public static AnsweredQuestion CreatePlayerCountAnswer(int playerCount)
        {
            // simplified validation for the sake of the test
            Assert.True(playerCount is 2 or 4 or 8, "Please use a player count of 2, 4 or 8.");
            return new AnsweredQuestion() {QuestionId = "PlayerCount", Answers = new() {playerCount.ToString()}};
        }

        public static AnsweredQuestion GetAnsweredQuestionThatIsNotInAdventurePreset()
        {
            // Corresponds to answer "I do not know" for the question "Netcode Architecture"
            return new AnsweredQuestion() { QuestionId = "NetcodeArchitecture", Answers = new() {"NoNetcode"}};
        }
        
        // Copy the UserChoices file to a temp file to be able to restore it after the tests.
        public static void CopyUserChoicesToTempFile()
        {
            var sourceFilePath = GetUserChoicesFullFilePath();
            var tempFilePath = GetUserChoicesTempFilePath();

            // Can happen if the there are no UserChoices saved to disk yet.
            if (!File.Exists(sourceFilePath))
            {
                // Save to have something to copy.
                UserChoicesObject.instance.Save();
            }

            try
            {
                File.Copy(sourceFilePath, tempFilePath, true);
            }
            catch (Exception e)
            {
                Assert.Fail($"Could not create Temp File from {sourceFilePath} {e.Message}");
            }
        }
        
        // Restore the UserChoices file from the temp file and delete the temp file.
        public static void RestoreUserChoicesFromTempFile()
        {
            var sourceFilePath = GetUserChoicesFullFilePath();
            var tempFilePath = GetUserChoicesTempFilePath();
            
            try
            {
                File.Copy(tempFilePath,sourceFilePath, true);
                File.Delete(tempFilePath);
            }
            catch (Exception e)
            {
                Assert.Fail($"Could not restore UserChoices from temp file {tempFilePath}, or could not delete temp file {tempFilePath} {e.Message}");
            }
            Object.DestroyImmediate(UserChoicesObject.instance);
        }
        
        public static void OpenTabByIndex(int tabIndex)
        {
            var multiplayerWindow = EditorWindow.GetWindow<MultiplayerCenterWindow>();
            multiplayerWindow.CurrentTabTest = tabIndex;
        }

        public static void SetNetcodeSolutionToCustomNetcode(bool flag)
        {
            UserChoicesObject.instance.SelectedSolutions.SelectedNetcodeSolution = flag ? SelectedSolutionsData.NetcodeSolution.CustomNetcode : SelectedSolutionsData.NetcodeSolution.NGO;
        }
        
        public static void SetSelectedSolutionToDistributedAuthority(bool flag)
        {
            if (!flag)
            {
                UserChoicesObject.instance.SelectedSolutions.SelectedHostingModel = SelectedSolutionsData.HostingModel.ClientHosted;
                return;
            }

            UserChoicesObject.instance.SelectedSolutions.SelectedNetcodeSolution = SelectedSolutionsData.NetcodeSolution.NGO;
            UserChoicesObject.instance.SelectedSolutions.SelectedHostingModel = SelectedSolutionsData.HostingModel.DistributedAuthority;
        }
        
        static string GetUserChoicesFullFilePath()
        {
            var choicesObject = UserChoicesObject.instance; 
            var dirName = Path.GetDirectoryName(Application.dataPath);
            return Path.Combine(dirName, choicesObject.FilePath);
        }

        static string GetUserChoicesTempFilePath()
        {
            return  Path.ChangeExtension(GetUserChoicesFullFilePath(), ".tmp");
        }
    }
}
