using System;
using Unity.Multiplayer.Center.Questionnaire;
using UnityEditor.PackageManager;
using UnityEngine;

namespace Unity.Multiplayer.Center.Recommendations
{
    [Serializable]
    internal class PreReleaseHandling
    {
        [SerializeReference]
        PreReleaseHandlingBase[] m_PreReleaseHandlings =
        {
        };

        public event Action OnAllChecksFinished;

        public bool IsReady => m_PreReleaseHandlings != null && 
            Array.TrueForAll(m_PreReleaseHandlings, p => p is {IsReady: true});

        public void CheckForUpdates()
        {
            foreach (var package in m_PreReleaseHandlings)
            {
                package.OnCheckFinished += OnOnePackageVersionCheckFinished;
                package.CheckForUpdates();
            }
        }

        public void PatchPackages(RecommendationViewData toPatch)
        {
            foreach (var package in m_PreReleaseHandlings)
            {
                package.PatchPackages(toPatch);
            }
        }

        public void PatchRecommenderSystemData()
        {
            foreach (var package in m_PreReleaseHandlings)
            {
                package.PatchRecommenderSystemData();
            }
        }

        void OnOnePackageVersionCheckFinished()
        {
            var allVersionChecksDone = true;
            foreach (var package in m_PreReleaseHandlings)
            {
                allVersionChecksDone &= package.IsReady;
                if (package.IsReady)
                {
                    package.OnCheckFinished -= OnOnePackageVersionCheckFinished;
                }
            }

            if (allVersionChecksDone)
            {
                OnAllChecksFinished?.Invoke();
            }
        }
    }

    [Serializable]
    internal abstract class PreReleaseHandlingBase
    {
        /// <summary>
        /// Whether the versions data is ready to be used.
        /// </summary>
        public bool IsReady => m_VersionsInfo != null && !string.IsNullOrEmpty(m_VersionsInfo.latestCompatible);

        /// <summary>
        /// Triggered when this instance is ready
        /// </summary>
        public event Action OnCheckFinished;

        /// <summary>
        /// The package id e.g. com.unity.netcode
        /// </summary>
        public abstract string PackageId { get; }

        /// <summary>
        /// The minimum version that we target.
        /// </summary>
        public abstract string MinVersion { get; }

        /// <summary>
        /// The cached versions of the package.
        /// </summary>
        [SerializeField]
        protected VersionsInfo m_VersionsInfo;

        /// <summary>
        /// version which would be installed by package manager 
        /// </summary>
        [SerializeField]
        protected string m_DefaultVersion;

        PackageManagement.VersionChecker m_VersionChecker;

        /// <summary>
        /// Start (online) request to check for available versions.
        /// </summary>
        public void CheckForUpdates()
        {
            m_VersionChecker = new PackageManagement.VersionChecker(PackageId);
            m_VersionChecker.OnVersionFound += OnVersionFound;
        }

        internal string GetPreReleaseVersion(string version, VersionsInfo versionsInfo)
        {
            if (version != null && version.StartsWith(MinVersion))
                return null; // no need for a pre-release version
            return versionsInfo.latestCompatible.StartsWith(MinVersion) ? versionsInfo.latestCompatible : null;
        }

        /// <summary>
        /// Patch the recommendation view data directly.
        /// </summary>
        /// <param name="toPatch">The view data to patch</param>
        public abstract void PatchPackages(RecommendationViewData toPatch);

        /// <summary>
        /// Patch the recommender system data, which will be used for every use of the recommendation.
        /// </summary>
        public abstract void PatchRecommenderSystemData();

        protected virtual void BeforeRaisingCheckFinished() { }

        void OnVersionFound(PackageInfo packageInfo)
        {
            m_DefaultVersion = packageInfo.version;
            m_VersionsInfo = packageInfo.versions;
            if (m_VersionChecker != null) // null observed in tests 
                m_VersionChecker.OnVersionFound -= OnVersionFound;
            m_VersionChecker = null;
            BeforeRaisingCheckFinished();
            OnCheckFinished?.Invoke();
        }
    }

    /// <summary>
    /// Implementation that fetches unconditionally the version starting with a prefix for a given package, even if it
    /// is an experimental package
    /// </summary>
    [Serializable]
    internal class SimplePreReleaseHandling : PreReleaseHandlingBase
    {
        [SerializeField] string m_MinVersion;
        [SerializeField] string m_PackageId;

        public override string MinVersion => m_MinVersion;
        public override string PackageId => m_PackageId;

        public SimplePreReleaseHandling(string packageId, string minVersion)
        {
            m_PackageId = packageId;
            m_MinVersion = minVersion;
        }

        private SimplePreReleaseHandling() { }

        public override void PatchPackages(RecommendationViewData toPatch)
        {
            // Nothing to do, we only patch the package details in the recommender system data
        }

        public override void PatchRecommenderSystemData()
        {
            if (!IsReady) return;

            var allPackages = RecommenderSystemDataObject.instance.RecommenderSystemData.Packages;
            foreach (var package in allPackages)
            {
                if (package.Id == PackageId)
                {
                    package.PreReleaseVersion = GetPreReleaseVersion(m_DefaultVersion, m_VersionsInfo);
                }
            }
        }
    }
}
