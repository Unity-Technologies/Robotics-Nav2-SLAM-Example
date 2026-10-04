using System;
using System.Collections.Generic;
using Unity.Multiplayer.Center.Recommendations;
using UnityEngine;
using UnityEngine.UIElements;

namespace Unity.Multiplayer.Center.Window.UI.RecommendationView
{
    internal class RecommendedPackagesSection : PackageSelectionView
    {
        VisualElement m_Container;
        protected override VisualElement ContainerRoot => m_Container;
        public RecommendedPackagesSection()
        {
            var headline = new Label("Recommended Multiplayer Packages");
            headline.AddToClassList(k_HeadlineUssClass);
            Add(headline);
            Add(m_Container = new VisualElement());
            m_Container.AddToClassList(k_SubSectionClass);
        }
    }
    
    internal class OtherSection : PackageSelectionView
    {
        readonly Foldout m_Foldout;
        
        protected override VisualElement ContainerRoot => m_Foldout;

        public OtherSection()
        {
            m_Foldout = new Foldout(){text = "Other Packages"};
            m_Foldout.AddToClassList(k_HeadlineUssClass);
            Add(m_Foldout);
            m_Foldout.contentContainer.AddToClassList(k_SubSectionClass);
        }
    }
    
    internal class PackageSelectionView : VisualElement
    {
        List<RecommendedPackageViewData> m_Packages;
        List<RecommendationItemView> PackageViews => ContainerRoot.Query<RecommendationItemView>().ToList();

        protected virtual VisualElement ContainerRoot => this;

        protected const string k_HeadlineUssClass = "subsection-headline";
        
        protected const string k_SubSectionClass = "sub-section";
        
        public event Action OnPackageSelectionChanged;

        protected PackageSelectionView()
        {
        }
        
        public void UpdatePackageData(List<RecommendedPackageViewData> packages)
        {
            m_Packages = packages;
            foreach (var view in PackageViews)
            {
                view.RemoveFromHierarchy();
            }
            
            SetVisible(ContainerRoot, packages.Count > 0);
            
            for (var index = 0; index < packages.Count; index++)
            {
                var feature = packages[index];
                if (PackageViews.Count <= index)
                    ContainerRoot.Add(new RecommendationItemView(isRadio: false));

                var view = PackageViews[index];
                view.UpdateData(feature);
                view.OnUserChangedSelection -= OnSelectionChanged;
                view.OnUserChangedSelection += OnSelectionChanged;
                
                // Todo: remove dirty hack to select multiplayer sdk for distributed authority.
                // i tried to put it as mainPackage but it currently is not really supported, to have a main package that is an optional package
                // in another hostingmodel choice. It caused issues with analytics.
                if (this is HostingModelSelectionView)
                {
                    if (feature.PackageId == "com.unity.services.multiplayer")
                    {
                        view.SetCheckboxEnabled(false);
                        view.SetIsSelected(true);
                        view.SetRecommendedBadgeVisible(false);
                    }
                }
            }
        }
        
        void OnSelectionChanged(RecommendationItemView view, bool isSelected)
        {
            var featureToSet = RecommendationUtils.FindRecommendedPackageViewById(m_Packages, view.FeatureId);

            if (featureToSet == null)
            {
                Debug.LogError($"Feature {view.FeatureId} not found");
                return;
            }
            featureToSet.Selected = isSelected;
            OnPackageSelectionChanged?.Invoke();
        }

        protected static void SetVisible(VisualElement element, bool visible)
        {
            element.style.display = visible ? DisplayStyle.Flex : DisplayStyle.None;
        }
    }
}
