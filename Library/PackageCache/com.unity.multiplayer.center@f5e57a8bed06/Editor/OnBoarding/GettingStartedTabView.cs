using System;
using System.Collections.Generic;
using System.Reflection;
using Unity.Multiplayer.Center.Analytics;
using Unity.Multiplayer.Center.Common;
using Unity.Multiplayer.Center.Onboarding;
using Unity.Multiplayer.Center.Questionnaire;
using Unity.Multiplayer.Center.Window.UI;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;

namespace Unity.Multiplayer.Center.Window
{
    [Serializable]
    internal class QuickstartCategory
    {
        [SerializeField]
        public OnboardingSectionCategory Category;

        [SerializeReference]
        public IOnboardingSection[] Sections;
    }
    
    /// <summary>
    /// This is the main view for the Quickstart tab.
    /// Note that in the code, the Quickstart tab is referred to as the Getting Started tab.
    /// </summary>
    [Serializable]
    internal class GettingStartedTabView : ITabView
    {
        const string k_SectionUssClass = "onboarding-section-category-container";
        const string k_CategoryButtonUssClass = "onboarding-category-button";
        const string k_OnboardingCategoriesUssClass = "onboarding-categories";
        const string k_OnboardingContentUssClass = "onboarding-content";

        [field: SerializeField]
        public string Name { get; private set; }
        
        public bool IsEnabled => PackageManagement.IsAnyMultiplayerPackageInstalled();
      
        public string ToolTip => IsEnabled ? "" : "Please install some multiplayer packages to access quickstart content.";
        
        public VisualElement RootVisualElement { get; set; }

        [SerializeField]
        int m_SelectedCategory;
        
        Dictionary<OnboardingSectionCategory, int> m_CategoryIndices;
        
        VisualElement[] m_CategoryContainers;
        
        [SerializeField]
        QuickstartCategory[] m_SectionCategories;
        /// <summary>
        /// To find out if new section appeared, we need to keep track of the last section types we found.
        /// </summary>
        [SerializeField]
        AvailableSectionTypes m_LastFoundSectionTypes;
        
        public IMultiplayerCenterAnalytics MultiplayerCenterAnalytics { get; set; }

        public GettingStartedTabView(string name = "Quickstart")
        {
            Name = name;
        }

        public void Refresh()
        {
            Debug.Assert(MultiplayerCenterAnalytics != null, "MultiplayerCenterAnalytics != null");
            UserChoicesObject.instance.OnSolutionSelectionChanged -= NotifyChoicesChanged;
            UserChoicesObject.instance.OnSolutionSelectionChanged += NotifyChoicesChanged;
            
            var currentSectionTypes = SectionsFinder.FindSectionTypes();
            
            if (m_SectionCategories == null || m_SectionCategories.Length == 0 || m_LastFoundSectionTypes.HaveTypesChanged(currentSectionTypes))
            {
                m_LastFoundSectionTypes = currentSectionTypes;
                ConstructSectionInstances();
                CreateViews();
            }
            else if(RootVisualElement == null || RootVisualElement.childCount == 0)
            {
                CreateViews(); 
            }
        }

        public void Clear()
        {
            RootVisualElement?.Clear();
            if (m_SectionCategories == null)
                return;
            foreach (var category in m_SectionCategories)
            {
                if(category == null) continue;
                foreach (var section in category.Sections)
                {
                    section?.Unload();    
                }
            }

            Array.Clear(m_SectionCategories, 0, m_SectionCategories.Length);
        }
        
        public void SetVisible(bool visible)
        {
            RootVisualElement.style.display = visible ? DisplayStyle.Flex : DisplayStyle.None;
        }

        void ConstructSectionInstances()
        {
            var enumValues = Enum.GetValues(typeof(OnboardingSectionCategory));
            var allCategories = new QuickstartCategory[enumValues.Length];
            foreach (var categoryObject in enumValues)
            {
                var category = (OnboardingSectionCategory) categoryObject;
                var categoryData = new QuickstartCategory {Category = category, Sections = Array.Empty<IOnboardingSection>()};
                allCategories[(int) category] = categoryData;
                if (!m_LastFoundSectionTypes.TryGetValue(category, out var sectionTypes))
                {
                    continue; // no section for that category
                }

                categoryData.Sections = new IOnboardingSection[sectionTypes.Length];
                for (var index = 0; index < sectionTypes.Length; index++)
                {
                    var sectionType = sectionTypes[index];
                    var newSection = SectionFromType(sectionType);
                    
                    // TODO: check what to do with null sections
                    if (newSection == null) continue;

                    categoryData.Sections[index] = newSection;
                }
            }
 
            m_SectionCategories = allCategories;
        }

        void SetSelectedCategory(int categoryIndex)
        {
            m_SelectedCategory = categoryIndex;
            for (var index = 0; index < m_CategoryContainers.Length; index++)
            {
                var categoryContainer = m_CategoryContainers[index];
                if(categoryContainer != null)
                    categoryContainer.style.display = index == categoryIndex ? DisplayStyle.Flex : DisplayStyle.None;
            }
        }
        
        void CreateViews()
        {
            RootVisualElement ??= new VisualElement();
            RootVisualElement.Clear();

            if (QuickstartIsMissingView.ShouldShow)
            {
                RootVisualElement.Add(new QuickstartIsMissingView().RootVisualElement);
            }
            
            m_CategoryIndices = new Dictionary<OnboardingSectionCategory, int>();
            m_CategoryContainers = new VisualElement[m_SectionCategories.Length];
            
            var horizontalContainer = new TwoPaneSplitView(0, 250, TwoPaneSplitViewOrientation.Horizontal);
            RootVisualElement.Add(horizontalContainer);
            horizontalContainer.AddToClassList(StyleClasses.MainSplitView);
            var buttonGroup = new ToggleButtonGroup() { allowEmptySelection = false, isMultipleSelection = false};
            buttonGroup.AddToClassList(k_OnboardingCategoriesUssClass);
            buttonGroup.AddToClassList(StyleClasses.MainSplitViewLeft);
            horizontalContainer.Add(buttonGroup);
            
            var scrollView = new ScrollView(ScrollViewMode.Vertical) {horizontalScrollerVisibility = ScrollerVisibility.Hidden};
            scrollView.AddToClassList(StyleClasses.MainSplitViewRight);
            scrollView.AddToClassList(k_OnboardingContentUssClass);
            
            horizontalContainer.Add(scrollView);

            var index = -1;
            foreach (var categoryData in m_SectionCategories)
            {
                if (categoryData == null || categoryData.Sections.Length == 0) continue;

                ++index;
                var category = categoryData.Category;
                var currentContainer = StartNewSection(scrollView, category);
                scrollView.Add(currentContainer);
                
                m_CategoryIndices[category] = index;
                m_CategoryContainers[index] = currentContainer;
                
                var button = new Button { text = SectionCategoryToString(category)};
                button.AddToClassList(k_CategoryButtonUssClass);
                buttonGroup.Add(button);
                
                CreateSectionViewsIn(currentContainer, categoryData);
            }
            
            // Hide the SplitView if we have nothing to show
            var noContentToShow = index == -1;
            horizontalContainer.style.display = noContentToShow ? DisplayStyle.None : DisplayStyle.Flex;

            if (noContentToShow && !QuickstartIsMissingView.ShouldShow)
            {
                var noContentLabel = new Label("No content is available for the current selection in Netcode Solution and Hosting Model.");
                noContentLabel.style.marginLeft = noContentLabel.style.marginRight = noContentLabel.style.marginTop = noContentLabel.style.marginBottom = 8;
                RootVisualElement.Add(noContentLabel);
            }
            
            SetSelectedCategory(m_SelectedCategory);
            ulong mask = (ulong) 1 << m_SelectedCategory; 
            buttonGroup.SetValueWithoutNotify(new ToggleButtonGroupState(mask, m_CategoryIndices.Count));

            // MTT-8918 Block the callback on register as it will always return index 0,
            // which can result in a mismatch between toggle group and selected category.
            var onCreateFrame = EditorApplication.timeSinceStartup;
            buttonGroup.RegisterValueChangedCallback(evt =>
            {
                if (Math.Abs(onCreateFrame - EditorApplication.timeSinceStartup) < 0.05f)
                    return;
                
                var selectedIndex = evt.newValue.GetActiveOptions(stackalloc int[evt.newValue.length])[0];
                SetSelectedCategory(selectedIndex);
            });
            NotifyChoicesChanged();
        }

        void CreateSectionViewsIn(VisualElement currentContainer, QuickstartCategory categoryData)
        {
            foreach (var section in categoryData.Sections)
            {
                try
                {
                    if (section is ISectionWithAnalytics sectionWithAnalytics)
                    {
                        var attribute = section.GetType().GetCustomAttribute<OnboardingSectionAttribute>();
                        sectionWithAnalytics.AnalyticsProvider = new OnboardingSectionAnalyticsProvider(MultiplayerCenterAnalytics,
                            targetPackageId: attribute.TargetPackageId, sectionId: attribute.Id);
                    }

                    section.Load();
                    section.Root.name = section.GetType().Name;
                    currentContainer.Add(section.Root);
                }
                catch (Exception e)
                {
                    Debug.LogWarning($"Could not load onboarding section {section?.GetType()}: {e}");
                }
            }
        }

        void NotifyChoicesChanged()
        {
            if (m_SectionCategories == null)
                return;
            
            foreach (var category in m_SectionCategories)
            {
                if (category == null) continue;
                foreach (var section in category.Sections)
                {
                    if (section is not ISectionDependingOnUserChoices dependentSection) continue;

                    try
                    {
                        dependentSection.HandleAnswerData(UserChoicesObject.instance.UserAnswers);
                        dependentSection.HandlePreset(UserChoicesObject.instance.Preset);
                        dependentSection.HandleUserSelectionData(UserChoicesObject.instance.SelectedSolutions);
                    }
                    catch (Exception e)
                    {
                        Debug.LogWarning($"Could not set data for onboarding section {section.GetType()}: {e}");
                    }   
                }
            }
        }
        
        static VisualElement StartNewSection(VisualElement parent, OnboardingSectionCategory category)
        {
            var container = new VisualElement();
            if (category != OnboardingSectionCategory.Intro)
            {
                var titleContainer = new VisualElement();
                titleContainer.AddToClassList(StyleClasses.ViewHeadline);
                
                var title = new Label(SectionCategoryToString(category));
                titleContainer.Add(title);
                container.Add(titleContainer);
            }

            container.AddToClassList(k_SectionUssClass);
            parent.Add(container);
            return container;
        }

        static IOnboardingSection SectionFromType(Type type)
        {
            var constructed = type.GetConstructor(Type.EmptyTypes)?.Invoke(null);
            if (constructed is IOnboardingSection section) return section;

            Debug.LogWarning($"Could not create onboarding section {type}");
            return null;
        }

        static string SectionCategoryToString(OnboardingSectionCategory category)
        {
            return category switch
            {
                OnboardingSectionCategory.Intro => "Intro",
                OnboardingSectionCategory.Netcode => "Netcode and Tools",
                OnboardingSectionCategory.ConnectingPlayers => "Connecting Players",
                OnboardingSectionCategory.ServerInfrastructure => "Hosting",
                OnboardingSectionCategory.Other => "Other",
                _ => category.ToString()
            };
        }
    }
}
