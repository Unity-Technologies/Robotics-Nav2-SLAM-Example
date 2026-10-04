using System;
using System.Collections.Generic;
using Unity.Multiplayer.Center.Recommendations;
using UnityEngine;
using UnityEngine.UIElements;

namespace Unity.Multiplayer.Center.Window.UI.RecommendationView
{
    /// <summary>
    /// View to show one option to the user
    /// Option can be shown as a radio button or as a checkbox.
    /// </summary>
    internal class RecommendationItemView : VisualElement
    {
        string m_DocsUrl;

        BaseBoolField m_RadioButton;
        Label m_PackageNameLabel = new();
        Label m_Catchphrase = new();
        RecommendationBadge m_RecommendedBadge;
        PreReleaseBadge m_PreReleaseBadge;
        Image m_PackageManagerIcon;
        Image m_HelpIcon;
        Image m_InstalledIcon;

        /// <summary>
        /// Feature Id stores a unique identifier that identifies the feature.
        /// Usually this is the title of the solution or the package id.
        /// </summary>
        public string FeatureId { get; set; }

        /// <summary>
        /// Triggered when the user changes the selection.
        /// </summary>
        public Action<RecommendationItemView, bool> OnUserChangedSelection;

        public RecommendationItemView(bool isRadio = true)
        {
            AddToClassList("recommendation-item");

            var topContainer = new VisualElement();
            topContainer.name = "header";

            m_RadioButton = isRadio ? new RadioButton() : new Toggle();
            m_RadioButton.RegisterValueChangedCallback(evt =>
            {
                if (OnUserChangedSelection != null)
                    OnUserChangedSelection(this, evt.newValue);
            });

            var topContainerLeft = new VisualElement();
            m_RecommendedBadge = new RecommendationBadge();
            m_PreReleaseBadge = new PreReleaseBadge();
            
            m_InstalledIcon = new Image() { name = "icon-package-installed" };
            m_InstalledIcon.AddToClassList("icon");
            m_InstalledIcon.AddToClassList("icon-package-installed");
            m_InstalledIcon.tooltip = "Package is installed";
            
            topContainerLeft.Add(m_RadioButton);
            topContainerLeft.Add(m_PackageNameLabel);
            topContainerLeft.Add(m_RecommendedBadge);
            topContainerLeft.Add(m_PreReleaseBadge);

            topContainerLeft.Add(m_InstalledIcon);
            topContainerLeft.AddToClassList("recommendation-item-top-left-container");
            topContainer.Add(topContainerLeft);

            var topContainerRight = new VisualElement();
            topContainerRight.AddToClassList("recommendation-item-top-right-container");

            m_HelpIcon = new Image() { name = "info-icon"};
            m_HelpIcon.AddToClassList("icon");
            m_HelpIcon.AddToClassList("icon-questionmark");
            m_HelpIcon.tooltip = "Open documentation";
            m_HelpIcon.RegisterCallback<ClickEvent>(OpenInBrowser);
            topContainerRight.Add(m_HelpIcon);

            m_PackageManagerIcon = new Image() { name = "package-manager-icon" };
            m_PackageManagerIcon.AddToClassList("icon");
            m_PackageManagerIcon.AddToClassList("icon-package-manager");
            m_PackageManagerIcon.tooltip = "Open Package Manager";
            m_PackageManagerIcon.RegisterCallback<ClickEvent>(_ => PackageManagement.OpenPackageManager(FeatureId));
            topContainerRight.Add(m_PackageManagerIcon);
            
            topContainer.Add(topContainerRight);
            Add(topContainer);

            var bottomContainer = new VisualElement();
            bottomContainer.Add(m_Catchphrase);
            bottomContainer.name = "sub-info-text";
            Add(bottomContainer);
        }

        public void UpdateData(RecommendedPackageViewData package)
        {
            var featureName = package.Name;
            
            m_PreReleaseBadge.style.display = string.IsNullOrEmpty(package.PreReleaseVersion)? DisplayStyle.None : DisplayStyle.Flex;
            
            var featureId = package.PackageId;
            FeatureId = featureId;
            
            SetFeatureName(featureName);
            SetIsSelected(package.Selected);
            SetRecommendationType(package.RecommendationType);
            SetReasonText(package.Reason);
            SetDocUrl(package.DocsUrl);
            SetCatchPhrase(package.ShortDescription);
            SetupInstalledIcon(package, featureId);
            SetupPackageManagerIcon(featureId);
        }

        void SetupPackageManagerIcon(string featureId)
        {
            m_PackageManagerIcon.SetEnabled(!string.IsNullOrEmpty(featureId));
        }

        internal void SetIsSelected(bool value)
        {
            m_RadioButton.SetValueWithoutNotify(value);
        }
        
        internal void SetCheckboxEnabled(bool value)
        {
            m_RadioButton.SetEnabled(value);
        }

        void SetupInstalledIcon(RecommendedItemViewData item, string featureId)
        {
            if (!item.IsInstalledAsProjectDependency)
            {
                m_InstalledIcon.style.display = DisplayStyle.None;
                return;
            }

            m_InstalledIcon.style.display = DisplayStyle.Flex;
            m_InstalledIcon.tooltip = $"Installed version: {item.InstalledVersion}\nClick to open Package Manager";
        }

        void SetFeatureName(string value)
        {
            m_PackageNameLabel.text = value;
        }

        void SetRecommendationType(RecommendationType value)
        {
            m_RecommendedBadge.SetRecommendationType(value);
            m_RadioButton.SetEnabled(true);
            style.opacity = 1f;
            if (!value.IsInstallableAsDirectDependency())
            {
                style.opacity = 0.8f;
                m_RadioButton.SetEnabled(false);
                m_RadioButton.SetValueWithoutNotify(false);
            }
        }

        public void SetRecommendedBadgeVisible(bool value)
        {
            m_RecommendedBadge.style.display = value ? DisplayStyle.Flex : DisplayStyle.None;
        }
        
        void SetReasonText(string value)
        {
            m_PackageNameLabel.tooltip = value;
            m_RecommendedBadge.tooltip = value;
        }

        void SetCatchPhrase(string value)
        {
            m_Catchphrase.text = value;

            // Deleted the reason text for now
            m_Catchphrase.style.display = DisplayStyle.Flex;
            if (string.IsNullOrEmpty(value))
                m_Catchphrase.style.display = DisplayStyle.None;
        }

        void SetDocUrl(string url)
        {
            m_DocsUrl = url;
            m_HelpIcon.SetEnabled(!string.IsNullOrEmpty(url));
        }

        void OpenInBrowser(ClickEvent evt)
        {
            // For a better solution look at PackageLinkButton.cs in PackageManagerUI, there seems to be a version with analytics etc.
            Application.OpenURL(m_DocsUrl);
        }
    }
    
    internal class RecommendationBadge : Label
    {
        List<string> m_PossibleLabelStyles = new ()
        {
            "color-grey",
            "color-recommendation-badge",
        };

        public void SetRecommendationType(RecommendationType value)
        {
            style.display = DisplayStyle.Flex;
            m_PossibleLabelStyles.ForEach(RemoveFromClassList);
            switch (value)
            {
                case RecommendationType.NetcodeFeatured or
                    RecommendationType.HostingFeatured or
                    RecommendationType.OptionalStandard:
                    AddToClassList("color-recommendation-badge");
                    text = "Recommended";
                    break;
                case RecommendationType.NotRecommended:
                    AddToClassList("color-grey");
                    text = "Not Recommended";
                    break;
                case RecommendationType.Incompatible:
                    AddToClassList("color-grey");
                    text = "Incompatible";
                    break;
                default:
                    style.display = DisplayStyle.None;
                    break;
            }
        }

        public RecommendationBadge()
        {
            AddToClassList("badge");
        }
    }

    internal class PreReleaseBadge : Label
    {
        public PreReleaseBadge() : base ("Pre") 
        {
            AddToClassList("badge");
            AddToClassList("pre-release-badge");
        }
    }
}
