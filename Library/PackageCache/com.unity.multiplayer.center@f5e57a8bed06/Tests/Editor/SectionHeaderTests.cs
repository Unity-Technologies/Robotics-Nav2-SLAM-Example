using System.Collections;
using System.Linq;
using NUnit.Framework;
using Unity.Multiplayer.Center.Recommendations;
using Unity.Multiplayer.Center.Window.UI.RecommendationView;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;
using UnityEngine.UIElements;

namespace Unity.MultiplayerCenterTests
{
    internal class SectionHeaderTests
    {
        [SetUp]
        public void Setup()
        {
            var sectionHeader = new SectionHeader("Test");
            var dummyWindow = EditorWindow.GetWindow<DummyWindow>();
            dummyWindow.rootVisualElement.Clear();
            dummyWindow.rootVisualElement.Add(sectionHeader);
        }

        SectionHeader GetSectionHeader()
        {
            return EditorWindow.GetWindow<DummyWindow>().rootVisualElement.Q<SectionHeader>();
        }

        IEnumerator WaitUntilDropdownIsShown()
        {
            DropdownField dropDown = null;
            var maxWait = 100;
            
            // wait for UI to be ready
            while (dropDown == null || maxWait-- > 0)
            {
                dropDown = GetSectionHeader().Q<DropdownField>();
                yield return null;
            }
            
            Assert.NotNull(dropDown, "Dropdown could not be found");
        }

        [UnityTest]
        public IEnumerator SectionHeader_AppendsRecommendationText()
        {
            yield return WaitUntilDropdownIsShown();
           
            var dropDown = GetSectionHeader().Q<DropdownField>();
            var recommendation = UtilsForRecommendationTests.GetSomeRecommendation();
            var serverChoices = recommendation.ServerArchitectureOptions;
            var serverChoice = serverChoices.First(sol => sol.RecommendationType == RecommendationType.MainArchitectureChoice);
            GetSectionHeader().UpdateData(recommendation.ServerArchitectureOptions);
            Assert.AreEqual(serverChoice.Title +" - Recommended", dropDown.value);
        }
        
        [UnityTest]
        public IEnumerator SectionHeader_IgnoresIncompatibleSolutions()
        {
            yield return WaitUntilDropdownIsShown();
            
            var dropDown = GetSectionHeader().Q<DropdownField>();
            var recommendation = UtilsForRecommendationTests.GetSomeRecommendation();
            var serverChoices = recommendation.ServerArchitectureOptions;
            serverChoices.Last().RecommendationType = RecommendationType.Incompatible;
            GetSectionHeader().UpdateData(recommendation.ServerArchitectureOptions);
            Assert.AreEqual(serverChoices.Count(sol => sol.RecommendationType !=RecommendationType.Incompatible), dropDown.choices.Count);
        }
        
        [TearDown]
        public void TearDown()
        {
            var dummyWindow = EditorWindow.GetWindow<DummyWindow>();
            dummyWindow.rootVisualElement.Clear();
            dummyWindow.Close();
        }

        class DummyWindow : EditorWindow { }
    }
}
