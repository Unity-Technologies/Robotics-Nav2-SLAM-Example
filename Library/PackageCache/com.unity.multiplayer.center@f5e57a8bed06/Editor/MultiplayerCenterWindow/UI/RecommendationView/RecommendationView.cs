using System;
using System.Collections.Generic;
using Unity.Multiplayer.Center.Questionnaire;
using Unity.Multiplayer.Center.Recommendations;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;

namespace Unity.Multiplayer.Center.Window.UI.RecommendationView
{
    internal class RecommendationView
    {
        RecommendationViewData m_Recommendation;
        
        SolutionsToRecommendedPackageViewData m_AllPackages;
        public SolutionsToRecommendedPackageViewData AllPackages 
            => m_AllPackages ??= RecommenderSystem.GetSolutionsToRecommendedPackageViewData();

        public RecommendationViewData CurrentRecommendation => m_Recommendation;

        
        public ScrollView Root { get; } = new();

        NetcodeSelectionView m_NetcodeSelectionView = new();
        HostingModelSelectionView m_HostingModelSelectionView = new();
        RecommendedPackagesSection m_RecommendedPackagesSection = new();
        OtherSection m_OtherSection = new();
        VisualElement m_NoRecommendationsView;

        VisualElement m_Content;

        PreReleaseHandling m_PreReleaseHandling;

        //Todo: for now check on the view but actually should be on the model
        public Action OnPackageSelectionChanged;

        public RecommendationView()
        {
            Root.AddToClassList("recommendation-view");
            Root.horizontalScrollerVisibility = ScrollerVisibility.Hidden;
            var title = new VisualElement();
            title.Add(new Label {text = "Multiplayer Solutions and Recommendations"});
            title.AddToClassList("recommendation-view-headline");
            Root.Add(title);
            
            Root.Add(m_Content = new VisualElement() {name = "recommendation-view-section-container"});
            var topContainer = new VisualElement();
            topContainer.name = "main-sections-container";
            topContainer.Add(m_NetcodeSelectionView);
            topContainer.Add(m_HostingModelSelectionView);
            m_Content.Add(topContainer);
            
            m_NetcodeSelectionView.OnUserChangedSolution += OnUserChangedSolution;
            m_HostingModelSelectionView.OnUserChangedSolution += OnUserChangedSolution;
            
            m_RecommendedPackagesSection.OnPackageSelectionChanged += RaisePackageSelectionChanged;
            m_HostingModelSelectionView.OnPackageSelectionChanged += RaisePackageSelectionChanged;
            m_NetcodeSelectionView.OnPackageSelectionChanged += RaisePackageSelectionChanged;
            
            m_Content.Add(m_RecommendedPackagesSection);
            m_Content.Add(m_OtherSection);
            m_NoRecommendationsView = EmptyView();

            Root.Add(m_NoRecommendationsView);

            UpdateView(false);
        }

        public void UpdateRecommendation(RecommendationViewData recommendation, PreReleaseHandling preReleaseHandling)
        {
            m_Recommendation = recommendation;
            m_PreReleaseHandling = preReleaseHandling;
            UpdateView(false);
        }

        public void Clear()
        {
            m_NetcodeSelectionView.OnUserChangedSolution -= OnUserChangedSolution;
            m_HostingModelSelectionView.OnUserChangedSolution -= OnUserChangedSolution;
            m_RecommendedPackagesSection.OnPackageSelectionChanged -= RaisePackageSelectionChanged;
            m_HostingModelSelectionView.OnPackageSelectionChanged -= RaisePackageSelectionChanged;
            m_NetcodeSelectionView.OnPackageSelectionChanged -= RaisePackageSelectionChanged;
            
            Root.Clear();
        }

        void RaisePackageSelectionChanged()
        {
            OnPackageSelectionChanged?.Invoke();
        }

        void OnUserChangedSolution()
        {
            UpdateView(true);
        }
        
        void UpdateView(bool recordUndo)
        {
            var hideRecommendation = m_Recommendation == null;
            SetRecommendationHidden(hideRecommendation);

            if (hideRecommendation)
            {
                OnPackageSelectionChanged?.Invoke();
                return;
            }
            
            RecommenderSystem.AdaptRecommendationToNetcodeSelection(m_Recommendation);
            m_PreReleaseHandling.PatchPackages(m_Recommendation);
            
            var selectedNetcode = RecommendationUtils.GetSelectedNetcode(m_Recommendation);
            var selectedHostingModel = RecommendationUtils.GetSelectedHostingModel(m_Recommendation);
            var selection = new SolutionSelection(selectedNetcode.Solution, selectedHostingModel.Solution);
            var allPackages = AllPackages.GetPackagesForSelection(selection); 

            // Debug(selection, allPackages);
            m_NetcodeSelectionView.UpdateData(m_Recommendation.NetcodeOptions,
                RecommendationUtils.FilterByType(allPackages, RecommendationType.NetcodeFeatured));
            m_HostingModelSelectionView.UpdateData(m_Recommendation.ServerArchitectureOptions,
                RecommendationUtils.FilterByType(allPackages, RecommendationType.HostingFeatured));
            m_RecommendedPackagesSection.UpdatePackageData(
                RecommendationUtils.FilterByType(allPackages, RecommendationType.OptionalStandard));
            
            var otherPackages = RecommendationUtils.FilterByType(allPackages, RecommendationType.NotRecommended);
            otherPackages.AddRange(RecommendationUtils.FilterByType(allPackages, RecommendationType.Incompatible));
            m_OtherSection.UpdatePackageData(otherPackages);
            OnPackageSelectionChanged?.Invoke();
            
            UpdateUserInputObject(m_Recommendation, recordUndo);
        }

        static void Debug(SolutionSelection selection, List<RecommendedPackageViewData> allPackages)
        {
            UnityEngine.Debug.Log("/////////////////////////////////////// " + selection);
            foreach (var pack in allPackages)
            {
                UnityEngine.Debug.Log(pack.Name + " " + pack.RecommendationType);
            }
        }

        void SetRecommendationHidden(bool hideRecommendation)
        {
            var flex = new StyleEnum<DisplayStyle>(DisplayStyle.Flex);
            var none = new StyleEnum<DisplayStyle>(DisplayStyle.None);
            m_Content.style.display = hideRecommendation ? none : flex;
            m_NoRecommendationsView.style.display = hideRecommendation ? flex : none;

            // To show the EmptyView in the center, we have to change the behavior of the scroll view container.
            Root.Q<VisualElement>("unity-content-container").style.flexGrow = hideRecommendation ? 1 : 0;
        }

        static void UpdateUserInputObject(RecommendationViewData recommendation, bool recordUndo)
        {
            if(recordUndo)
                Undo.RecordObject(UserChoicesObject.instance,"Selection Change");
            
            var currentSelectedNetcode = UserChoicesObject.instance.SelectedSolutions.SelectedNetcodeSolution;
            foreach (var netcodeOption in recommendation.NetcodeOptions)
            {
                if (netcodeOption.Selected)
                    currentSelectedNetcode = Logic.ConvertNetcodeSolution(netcodeOption);
            }
            
            var selectedHostingModel = UserChoicesObject.instance.SelectedSolutions.SelectedHostingModel;
            foreach (var serverArchitectureOption in recommendation.ServerArchitectureOptions)
            {
                if (serverArchitectureOption.Selected)
                    selectedHostingModel = Logic.ConvertInfrastructure(serverArchitectureOption);
            }

            UserChoicesObject.instance.SetUserSelection(selectedHostingModel, currentSelectedNetcode);
            UserChoicesObject.instance.Save();
        }

        static VisualElement EmptyView()
        {
            var emptyView = new VisualElement();
            emptyView.name = "empty-view";

            var emptyViewContentContainer = new VisualElement();
            emptyViewContentContainer.name = "empty-view-content";
            var emptyViewMessage = new Label
            {
                text = "You will see your recommendations and be able to explore Unity’s multiplayer offerings here once you specify the genre of your game, and the number of players per session.",
                name = "empty-view-message"
            };
            var emptyViewIcon = new VisualElement();
            emptyViewIcon.name = "empty-view-icon";
            emptyViewContentContainer.Add(emptyViewIcon);
            emptyViewContentContainer.Add(emptyViewMessage);
            emptyView.Add(emptyViewContentContainer);
            return emptyView;
        }
    }
}
