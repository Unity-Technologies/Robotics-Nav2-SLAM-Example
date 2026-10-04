using System;

namespace Unity.Multiplayer.Center.Recommendations
{
    /// <summary>
    /// Types of recommendation that are both used in ViewData and in the ground truth data for the recommendation.
    /// Architecture choices must be made. Packages are typically optional.
    /// </summary>
    [Serializable]
    internal enum RecommendationType
    {
        /// <summary>
        /// Invalid value, indicates error.
        /// </summary>
        None,

        /// <summary>
        /// Featured option (e.g. NGO if NGO is the recommended architecture)
        /// Note that in the case of architecture choice, the user should select something.
        /// </summary>
        MainArchitectureChoice,

        /// <summary>
        /// Non-featured option (e.g. N4E if NGO is the recommended architecture)
        /// </summary>
        SecondArchitectureChoice,

        /// <summary>
        /// Associated feature in the Netcode section
        /// </summary>
        NetcodeFeatured,
        
        /// <summary>
        /// Associated feature in the Hosting Model section
        /// </summary>
        HostingFeatured,

        /// <summary>
        /// Optional but not highlighted
        /// </summary>
        OptionalStandard,

        /// <summary>
        /// Not recommended, but not incompatible with the user intent.
        /// </summary>
        NotRecommended,

        /// <summary>
        /// Incompatible with the user intent. Might even break something, we need to warn the user
        /// </summary>
        Incompatible,
        
        /// <summary>
        /// Packages that are not visible for the User but useful for the analytics
        /// </summary>
        Hidden
    }
    
    internal static class RecommendationTypeExtensions
    {
        public static bool IsRecommendedPackage(this RecommendationType type)
            => type is RecommendationType.OptionalStandard or RecommendationType.NetcodeFeatured or RecommendationType.HostingFeatured;
        
        public static bool IsRecommendedSolution(this RecommendationType type)
            => type is RecommendationType.MainArchitectureChoice;
        
        public static bool IsInstallableAsDirectDependency(this RecommendationType type)
            => type is not RecommendationType.Incompatible and not RecommendationType.Hidden;
    }
}
