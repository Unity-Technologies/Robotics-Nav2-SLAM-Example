using System;
using NUnit.Framework;
using Unity.Multiplayer.Center.Window;
using UnityEditor;
using UnityEngine.UIElements;

namespace Unity.MultiplayerCenterTests
{
    class QuickstartTabTests
    {
        bool m_IsQuickstartPackageInstalled = false;
        
        [OneTimeSetUp]
        public void OneTimeSetup()
        {
             var packInfo = UnityEditor.PackageManager.PackageInfo.FindForPackageName("com.unity.multiplayer.center.quickstart");
             m_IsQuickstartPackageInstalled = packInfo != null;
        }
        
        [SetUp]
        public void Setup()
        {
            EditorWindow.GetWindow<MultiplayerCenterWindow>();
        }

        [Test]
        public void QuickstartTab_QuickstartPackageMissingHelpboxExists()
        {
            if (m_IsQuickstartPackageInstalled)
            {
                Assert.Ignore("Skip test because it needs the quickstart package to be missing");
            }
            
            UtilsForGettingStartedTabTests.OpenGettingStartedTab();
            var helpBox = EditorWindow.GetWindow<MultiplayerCenterWindow>().rootVisualElement.Q<HelpBox>();
            Assert.NotNull(helpBox, "Helpbox for missing quickstart package not found");
        }
    }
}
