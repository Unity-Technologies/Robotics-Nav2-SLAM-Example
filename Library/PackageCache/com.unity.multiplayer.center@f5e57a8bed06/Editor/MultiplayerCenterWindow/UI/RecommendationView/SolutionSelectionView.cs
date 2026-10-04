using System;
using System.Collections.Generic;
using Unity.Multiplayer.Center.Recommendations;
using UnityEngine;
using UnityEngine.UIElements;

namespace Unity.Multiplayer.Center.Window.UI.RecommendationView
{
    internal class NetcodeSelectionView : SolutionSelectionView
    {
        public NetcodeSelectionView() : base("Netcode Solution") {}
    }

    internal class HostingModelSelectionView : SolutionSelectionView
    {
        public HostingModelSelectionView() : base("Hosting Model") {}
    }

    internal class SolutionSelectionView : PackageSelectionView
    {
        readonly SectionHeader m_Header;
        readonly Label m_AssociatedFeaturesHeader;
        readonly RecommendationItemView m_MainHostingModelItem;
        readonly VisualElement m_ContainerRoot;
        readonly HelpBox m_Warning;
        
        protected override VisualElement ContainerRoot => m_ContainerRoot;
        
        /// <summary>
        /// Gets fired when the user changes the solution.
        /// </summary>
        public event Action OnUserChangedSolution;

        protected SolutionSelectionView(string title)
        {
            AddToClassList("main-section");
            m_Header = new SectionHeader(title);
            Add(m_Header);
            m_AssociatedFeaturesHeader = new Label() { text = "Associated Packages", name = "associated-features-headline" };
            Add(m_AssociatedFeaturesHeader);
            m_MainHostingModelItem = new RecommendationItemView(isRadio: false);
            Add(m_MainHostingModelItem);
            m_Header.OnSolutionSelected += () => OnUserChangedSolution?.Invoke();
            m_ContainerRoot = new VisualElement();
            Add(m_ContainerRoot);
            m_Warning = new HelpBox(null, HelpBoxMessageType.Info);
            Add(m_Warning);
            SetVisible(m_Warning, false);
        }
        
        public void UpdateData(RecommendedSolutionViewData[] availableSolutions, List<RecommendedPackageViewData> allPackages)
        {
            m_Header.UpdateData(availableSolutions);
            var selectedSolution = RecommendationUtils.GetSelectedSolution(availableSolutions);
            UpdateMainPackageView(selectedSolution);
            UpdatePackageData(allPackages);
            SetVisible(m_AssociatedFeaturesHeader, allPackages.Count > 0 || selectedSolution.MainPackage != null);
        }

        void UpdateMainPackageView(RecommendedSolutionViewData selectedSolution)
        {
            var hasMainPackage = selectedSolution.MainPackage != null;
            m_Warning.text = selectedSolution.WarningString;
            SetVisible(m_Warning, !string.IsNullOrEmpty(selectedSolution.WarningString));
            SetVisible(m_MainHostingModelItem, hasMainPackage);
            if (!hasMainPackage) return;
            
            m_MainHostingModelItem.UpdateData(selectedSolution.MainPackage);
            m_MainHostingModelItem.SetIsSelected(true);
            m_MainHostingModelItem.SetCheckboxEnabled(false);
        }
    }
}
