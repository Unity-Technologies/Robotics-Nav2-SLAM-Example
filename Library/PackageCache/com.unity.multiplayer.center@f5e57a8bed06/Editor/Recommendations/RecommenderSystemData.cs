using System;
using System.Collections.Generic;
using Unity.Multiplayer.Center.Questionnaire;

namespace Unity.Multiplayer.Center.Recommendations
{
    /// <summary>
    /// The source of data for the recommender system. Based on scoring and this data, the recommender system will
    /// populate the recommendation view data. 
    /// </summary>
    [Serializable]
    internal class RecommenderSystemData
    {
        /// <summary>
        /// The Unity version for which this recommendation data is valid.
        /// </summary>
        public string TargetUnityVersion;

        /// <summary>
        /// Stores all the recommended solutions. This is serialized.
        /// </summary>
        public RecommendedSolution[] RecommendedSolutions;

        /// <summary>
        /// Stores all the package details.
        /// This is serialized.
        /// </summary>
        public PackageDetails[] Packages;

        /// <summary>
        /// Provides convenient access to the package details by package id.
        /// </summary>
        public Dictionary<string, PackageDetails> PackageDetailsById
        {
            get
            {
                if (m_PackageDetailsById != null) return m_PackageDetailsById;
                m_PackageDetailsById = Utils.ToDictionary(Packages, p => p.Id);
                return m_PackageDetailsById;
            }
        }

        public Dictionary<PossibleSolution, RecommendedSolution> SolutionsByType
        {
            get
            {
                if (m_SolutionsByType != null) return m_SolutionsByType;
                m_SolutionsByType = Utils.ToDictionary(RecommendedSolutions, s => s.Type);
                return m_SolutionsByType;
            }
        }

        /// <summary>
        /// Checks for incompatibility between the netcode and hosting model.
        /// </summary>
        /// <param name="netcode">The netcode type</param>
        /// <param name="hostingModel">The hosting model</param>
        /// <param name="reason">The reason for the incompatibility, filled when this function returns false.</param>
        /// <returns>True if compatible (default), False otherwise</returns>
        public bool IsHostingModelCompatibleWithNetcode(PossibleSolution netcode, PossibleSolution hostingModel, out string reason)
        {
            m_IncompatibleHostingModels ??= Utils.ToDictionary(RecommendedSolutions);
            return !m_IncompatibleHostingModels.TryGetValue(new SolutionSelection(netcode, hostingModel), out reason);
        }

        /// <summary>
        /// Patch incompatibility values.
        /// </summary>
        /// <param name="netcode">Netcode solution</param>
        /// <param name="hostingModel">Hosting model solution</param>
        /// <param name="newIsCompatible">Whether we should now consider the two solutions compatible</param>
        /// <param name="reason">If incompatible, why it is incompatible.</param>
        internal void UpdateIncompatibility(PossibleSolution netcode, PossibleSolution hostingModel, bool newIsCompatible, string reason=null)
        {
            Utils.UpdateIncompatibilityInSolutions(RecommendedSolutions, netcode, hostingModel, newIsCompatible, reason);
            m_IncompatibleHostingModels = Utils.ToDictionary(RecommendedSolutions);
            m_SolutionsByType = Utils.ToDictionary(RecommendedSolutions, s => s.Type);
        }
        
        Dictionary<string, PackageDetails> m_PackageDetailsById;
        Dictionary<PossibleSolution, RecommendedSolution> m_SolutionsByType;
        Dictionary<SolutionSelection, string> m_IncompatibleHostingModels;
    }

    [Serializable]
    internal struct SolutionSelection
    {
        public PossibleSolution Netcode;
        public PossibleSolution HostingModel;
        public SolutionSelection(PossibleSolution netcode, PossibleSolution hostingModel)
        {
            Netcode = netcode;
            HostingModel = hostingModel;
        }
    }
    
    /// <summary>
    /// A possible solution and whether packages are recommended or not
    /// </summary>
    [Serializable]
    internal class RecommendedSolution
    {
        /// <summary>
        /// The type of solution
        /// </summary>
        public PossibleSolution Type;
        
        /// <summary>
        ///  The name of the solution as shown in the UI.
        /// </summary>
        public string Title;
        
        /// <summary>
        /// Optional package ID associated with that solution (e.g. a netcode package or the cloud code package).
        /// Use this field if the package has to mandatorily be installed when the solution is selected. 
        /// </summary>
        public string MainPackageId;// only id because scoring will impact the rest
        
        /// <summary>
        /// Url to documentation describing the solution.
        /// </summary>
        public string DocUrl;
        
        /// <summary>
        /// Short description of the solution.
        /// </summary>
        public string ShortDescription;
        
        /// <summary>
        /// The packages and the associated recommendation type.
        /// If the Type is a netcode Type, all the possible packages should be in this array.
        /// If the Type is a hosting model, this will contain only overrides in case a package is incompatible or
        /// featured for the hosting model.
        /// </summary>
        public RecommendedPackage[] RecommendedPackages;
        
        /// <summary>
        /// Solutions that are incompatible with this solution.
        /// Typically used for netcode solutions.
        /// </summary>
        public IncompatibleSolution[] IncompatibleSolutions = Array.Empty<IncompatibleSolution>();
    }

    /// <summary>
    /// Stores why a solution is incompatible with something and why.
    /// </summary>
    [Serializable]
    internal class IncompatibleSolution
    {
        /// <summary>
        /// What is incompatible.
        /// </summary>
        public PossibleSolution Solution;
        
        /// <summary>
        /// Why it is incompatible.
        /// </summary>
        public string Reason;
        
        public IncompatibleSolution(PossibleSolution solution, string reason)
        {
            Solution = solution;
            Reason = reason;
        }
    }

    /// <summary>
    /// A package, whether it is recommended or not (context dependent), and why.
    /// </summary>
    [Serializable]
    internal class RecommendedPackage
    {
        /// <summary>
        /// The package id (e.g. com.unity.netcode)
        /// </summary>
        public string PackageId;
        
        /// <summary>
        /// Whether it is recommended or not.
        /// </summary>
        public RecommendationType Type;
        
        /// <summary>
        /// Why it is recommended or not.
        /// </summary>
        public string Reason;

        public RecommendedPackage(string packageId, RecommendationType type, string reason)
        {
            PackageId = packageId;
            Type = type;
            Reason = reason;
        }
    }

    [Serializable]
    internal class PackageDetails
    {
        public string Id;
        public string Name;
        public string ShortDescription;
        public string DocsUrl;
        public string[] AdditionalPackages;

        /// <summary>
        /// In case we want to promote a specific pre-release version, this is set (by default, this is null
        /// and the default package manager version is used).
        /// </summary>
        public string PreReleaseVersion;

        /// <summary>
        /// Details about the package.
        /// </summary>
        /// <param name="id">Package ID</param>
        /// <param name="name">Package Name (for display)</param>
        /// <param name="shortDescription">Short description.</param>
        /// <param name="docsUrl">Link to Docs</param>
        /// <param name="additionalPackages">Ids of packages that should be installed along this one, but are not formal dependencies.</param>
        public PackageDetails(string id, string name, string shortDescription, string docsUrl, string[] additionalPackages = null)
        {
            Id = id;
            Name = name;
            ShortDescription = shortDescription;
            DocsUrl = docsUrl;
            AdditionalPackages = additionalPackages;
        }
    }

    static class Utils
    {
        public static Dictionary<TKey, T> ToDictionary<T, TKey>(T[] array, Func<T, TKey> keySelector)
        {
            if (array == null) return null;
            var result = new Dictionary<TKey, T>();
            foreach (var item in array)
            {
                result[keySelector(item)] = item;
            }

            return result;
        }
        
        public static Dictionary<SolutionSelection, string> ToDictionary(RecommendedSolution[] solutions)
        {
            var result = new Dictionary<SolutionSelection, string>();
            foreach (var recommendedSolution in solutions)
            {
                foreach (var incompatibleHostingModel in recommendedSolution.IncompatibleSolutions)
                {
                    var key = new SolutionSelection(recommendedSolution.Type, incompatibleHostingModel.Solution);
                    result.Add(key, incompatibleHostingModel.Reason);
                }
            }

            return result;
        }
        
        public static void UpdateIncompatibilityInSolutions(RecommendedSolution[] solutions, PossibleSolution netcode, 
            PossibleSolution hostingModel, bool newIsCompatible, string reason)
        {
            foreach (var recommendedSolution in solutions)
            {
                if (recommendedSolution.Type != netcode) continue;
                
                var incompatibleSolution = Array.Find(recommendedSolution.IncompatibleSolutions, s => s.Solution == hostingModel);
                if (newIsCompatible && incompatibleSolution != null)
                {
                    recommendedSolution.IncompatibleSolutions = Array.FindAll(recommendedSolution.IncompatibleSolutions, s => s.Solution != hostingModel);
                }
                else if (!newIsCompatible && incompatibleSolution == null)
                {
                    Array.Resize(ref recommendedSolution.IncompatibleSolutions, recommendedSolution.IncompatibleSolutions.Length + 1);
                    recommendedSolution.IncompatibleSolutions[^1] = new IncompatibleSolution(hostingModel, reason);
                }
            }
        }
    }
}