using UnityEngine.UIElements;
namespace Unity.Multiplayer.Center.Onboarding
{
    /// <summary>
    /// A lot of the getting started content is in the package com.unity.multiplayer.center.quickstart
    /// This class is responsible for handling the package and its installation
    /// </summary>
    internal class QuickstartIsMissingView
    {
        public const string PackageId = "com.unity.multiplayer.center.quickstart";

        Button m_Button;
        public VisualElement RootVisualElement { get; private set; }

        public static bool ShouldShow => !PackageManagement.IsInstalled(PackageId);

        public QuickstartIsMissingView()
        {
            RootVisualElement = new HelpBox("The Quickstart package is not installed, so not all the content will be available in this view.", HelpBoxMessageType.Warning);
            m_Button = new Button(InstallQuickstart) {text = "Install"};
            RootVisualElement.style.marginLeft = RootVisualElement.style.marginRight = RootVisualElement.style.marginTop = RootVisualElement.style.marginBottom = 8;
            RootVisualElement.Add(m_Button);
        }

        void InstallQuickstart()
        {
            PackageManagement.InstallPackage(PackageId, OnInstallFinished);
        }

        void OnInstallFinished(bool success)
        {
            if (!success)
                return;

            if(m_Button != null)
                m_Button.clicked -= InstallQuickstart;
            RootVisualElement?.RemoveFromHierarchy();
        }
    }
}
