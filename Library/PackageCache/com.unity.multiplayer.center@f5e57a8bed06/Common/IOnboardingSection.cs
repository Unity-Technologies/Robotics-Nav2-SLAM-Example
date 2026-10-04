using System;
using System.Reflection;
using Unity.Multiplayer.Center.Common.Analytics;
using UnityEngine.UIElements;

namespace Unity.Multiplayer.Center.Common
{
    /// <summary>
    /// Onboarding section metadata to be picked up by the multiplayer center.
    /// This can only be used once per type. If you wish to make the same section appear in multiple categories/conditions,
    /// please create two types inheriting from the same base class.
    /// </summary>
    [AttributeUsage(AttributeTargets.All, Inherited = false, AllowMultiple = false)]
    public sealed class OnboardingSectionAttribute : Attribute
    {
        /// <summary>
        /// The UI category the section will fall into.
        /// </summary>
        public OnboardingSectionCategory Category { get; }

        /// <summary>
        /// The id of that section (defines uniqueness and whether priority should be used)
        /// </summary>
        public readonly string Id;

        /// <summary>
        /// Optional: condition to display the section. By default, if the type exists in the project, the section is shown.
        /// </summary>
        public DisplayCondition DisplayCondition { get; set; }
        
        /// <summary>
        /// Optional: dependency on a certain hosting model choice.
        /// </summary>
        public SelectedSolutionsData.HostingModel HostingModelDependency { get; set; }
        
        /// <summary>
        /// Optional: dependency on a certain netcode choice.
        /// </summary>
        public SelectedSolutionsData.NetcodeSolution NetcodeDependency { get; set; }

        /// <summary>
        /// Optional: priority in case several onboarding sections are defined for the same package/id.
        /// Use-case: new version of a package needs a different onboarding and overrides what we ship with the Multiplayer Center.
        /// </summary>
        public int Priority { get; set; }

        /// <summary>
        /// Optional: this is the order in which the sections will be displayed in the UI within the section.
        /// (the higher the Order value, the further down)
        /// </summary>
        public int Order { get; set; }

        /// <summary>
        /// Optional: the package identifier that this section is related to, e.g. "com.unity.transport".
        /// </summary>
        public string TargetPackageId { get; set; }

        /// <summary>
        /// Creates a new instance of the attribute.
        /// </summary>
        /// <param name="category">The section category.</param>
        /// <param name="id">The identifier.</param>
        public OnboardingSectionAttribute(OnboardingSectionCategory category, string id)
        {
            Category = category;
            Id = id;
        }
    }

    /// <summary>
    /// The UI category the section will fall into.
    /// </summary>
    public enum OnboardingSectionCategory
    {
        /// <summary>
        /// Comes at the top and should cover overarching topics for beginners
        /// </summary>
        Intro,
        
        /// <summary>
        /// Section about the fundamentals of gameplay synchronization implementation and debugging.
        /// This includes netcode and tools related to netcode, as well as alternative solutions.
        /// </summary>
        Netcode,
        
        /// <summary>
        /// Section gathering information about connecting players together, such as lobbies, voice chat, matchmaking
        /// and widgets.
        /// </summary>
        ConnectingPlayers,
        
        /// <summary>
        /// Section gathering information about deploying, running and optimizing a game server.
        /// </summary>
        ServerInfrastructure,
        
        /// <summary>
        /// Something else.
        /// </summary>
        Other,
        
        /// <summary>
        /// LiveOps sections which are meant to be used after some development happened on the game.
        /// </summary>
        LiveOps
    }

    /// <summary>
    /// A condition for a section to be displayed.
    /// </summary>
    public enum DisplayCondition
    {
        /// <summary>
        /// As long as the type exists in the project, the section is shown.
        /// Exception: a section with a higher priority is defined for the same id.
        /// </summary>
        None,
        
        /// <summary>
        /// A target package is defined in TargetPackageId and the package is installed
        /// If multiple types share the same id, the one with the highest priority is shown.
        /// </summary>
        PackageInstalled,
        
        /// <summary>
        /// Shown if no multiplayer package is installed (e.g. for the first time user that has not installed anything)
        /// Check SectionsFinder.k_TargetPackages to see which packages are checked
        /// </summary>
        NoPackageInstalled
    }

    /// <summary>
    /// Defines if a section depends on a certain hosting model.
    /// </summary>
    public enum InfrastructureDependency
    {
        /// <summary>
        /// No dependency, the section is shown if all other conditions are also met.
        /// </summary>
        None,
        
        /// <summary>
        /// Only available when the user has selected a client hosted infrastructure.
        /// </summary>
        ClientHosted,
        
        /// <summary>
        /// Only available when the user has selected a dedicated server infrastructure.
        /// </summary>
        DedicatedServer,
        
        /// <summary>
        /// Only available when the user has selected Cloud Code as their hosting model.
        /// </summary>
        CloudCode
    }

    /// <summary>
    /// A view for a single onboarding section. Classes implementing this interface should be marked with the
    /// <see cref="OnboardingSectionAttribute"/>.
    /// </summary>
    public interface IOnboardingSection
    {
        /// <summary>
        /// The visual element that will be added to the onboarding window.
        /// After Load is called, it should never be null.
        /// </summary>
        public VisualElement Root { get; }

        /// <summary>
        /// Makes the section ready to be displayed.
        /// May be called several times in a row.
        /// </summary>
        public void Load();

        /// <summary>
        /// Frees anything that needs to be.
        /// </summary>
        public void Unload();
    }
    
    /// <summary>
    /// For sections that depend on what the user selected in either the game specs or the solution selection.
    /// </summary>
    public interface ISectionDependingOnUserChoices : IOnboardingSection
    {
        /// <summary>
        /// Receives the answer data and handles it. This is called after Load.
        /// </summary>
        /// <param name="answerData">The latest value of answerData</param>
        public void HandleAnswerData(AnswerData answerData) { }
        
        /// <summary>
        /// Receives the user selection data and handles it. This is called after Load.
        /// </summary>
        /// <param name="selectedSolutionsData">The latest value of the selection</param>
        public void HandleUserSelectionData(SelectedSolutionsData selectedSolutionsData) { }

        /// <summary>
        /// Receives the preset data and handles it. This is called after Load.
        /// </summary>
        /// <param name="preset">The latest preset value</param>
        public void HandlePreset(Preset preset) { }
    }

    /// <summary>
    /// Implement this interface to have access to the Multiplayer Center analytics provider.
    /// Use the analytics provider to log events when the user interacts with the section.
    /// </summary>
    public interface ISectionWithAnalytics
    {
        /// <summary>
        /// This will be set before the load function is called.
        /// The implementor can then use the Analytics provider to send events to the analytics backend.
        /// </summary>
        public IOnboardingSectionAnalyticsProvider AnalyticsProvider { get; set; }
    }
}
