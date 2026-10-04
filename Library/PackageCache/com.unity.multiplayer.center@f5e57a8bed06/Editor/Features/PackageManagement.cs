using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEditor.PackageManager;
using UnityEditor.PackageManager.Requests;
using UnityEngine;

namespace Unity.Multiplayer.Center
{
    internal static class PackageManagement
    {
        static PackageInstaller s_Installer;
        
        /// <summary>
        /// Opens the package manager window with selected package name and hides error
        /// </summary>
        public static void OpenPackageManager(string packageName)
        {
            try
            {
                UnityEditor.PackageManager.UI.Window.Open(packageName);
            }
            catch (Exception)
            {
                // Hide the error in the PackageManager API until the team fixes it
                // Debug.Log("Error opening Package Manager: " + e.Message);
            }
        }

        /// <summary>
        /// Checks if the package is a direct dependency of the project
        /// </summary>
        /// <param name="packageId">The package name/id e.g. com.unity.netcode</param>
        /// <returns>True if the package is a direct dependency</returns>
        public static bool IsDirectDependency(string packageId)
        {
            var package = GetInstalledPackage(packageId);
            return package != null && package.isDirectDependency;
        }

        /// <summary>
        /// Checks if a package is installed.
        /// </summary>
        /// <param name="packageId">The package name, e.g. com.unity.netcode</param>
        /// <returns>True if the package is installed, false otherwise</returns>
        public static bool IsInstalled(string packageId) => GetInstalledPackage(packageId) != null;

        /// <summary>
        /// Checks if a package is embedded, linked locally, installed via Git or local Tarball.
        /// </summary>
        /// <param name="packageId">The package name, e.g. com.unity.netcode</param>
        /// <returns>True if the package is linked locally, false otherwise</returns>
        public static bool IsLinkedLocallyOrEmbeddedOrViaGit(string packageId) => 
            GetInstalledPackage(packageId) is { source: PackageSource.Embedded or PackageSource.Local or PackageSource.Git or PackageSource.LocalTarball };

        /// <summary>
        /// Finds the installed package with the given packageId or returns null.
        /// </summary>
        /// <param name="packageId">The package name/id e.g. com.unity.netcode</param>
        /// <returns>The package info</returns>
        public static UnityEditor.PackageManager.PackageInfo GetInstalledPackage(string packageId)
        {
            return UnityEditor.PackageManager.PackageInfo.FindForPackageName(packageId);
        }

        /// <summary>
        /// Filters out the packages that are already embedded, linked locally, installed via Git or local Tarball and returns this new list.
        /// </summary>
        /// <param name="installCandidates">A list of package IDs that are candidates for installation.</param>
        /// <returns>A new filtered list of packages.</returns>
        public static IEnumerable<string> RemoveLocallyLinkedOrEmbeddedOrViaGitPackagesFromList(IEnumerable<string> installCandidates)
        {
            var filteredList = new List<string>();

            foreach (var packageId in installCandidates)
            {
                if (!IsLinkedLocallyOrEmbeddedOrViaGit(packageId))
                {
                    filteredList.Add(packageId);
                }
                else
                {
                    Debug.Log($"Removing {packageId} from install candidates.\n" +
                        "This package is already embedded, linked locally, installed via Git, or from a local tarball. " +
                        "Please check the Package Manager for more information or to upgrade manually.");
                }
            }

            return filteredList;
        }

        /// <summary>
        /// Returns true if any of the given packageIds is installed.
        /// </summary>
        /// <param name="packageIds">List of package is e.g com.unity.netcode</param>
        /// <returns>True if any package is installed, false otherwise</returns>
        public static bool IsAnyPackageInstalled(params string[] packageIds)
        {
            var installedPackages = UnityEditor.PackageManager.PackageInfo.GetAllRegisteredPackages();
            var hashset = new HashSet<string>();

            foreach (var package in installedPackages)
            {
                hashset.Add(package.name);
            }

            foreach (var packageId in packageIds)
            {
                if (hashset.Contains(packageId))
                {
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// Installs a single package and invokes the callback when the package is installed/when the install failed.
        /// </summary>
        /// <param name="packageId">The package name/id e.g. com.unity.netcode</param>
        /// <param name="onInstalled">The callback</param>
        public static void InstallPackage(string packageId, Action<bool> onInstalled = null)
        {
            s_Installer = new PackageInstaller(packageId);
            s_Installer.OnInstalled += onInstalled;
            s_Installer.OnInstalled += _ => s_Installer = null;
        }

        /// <summary>
        /// Register to an existing installation callback. This has no effect if no installation is ongoing (check
        /// <see cref="IsInstallationFinished"/> to see if that is the case).
        /// </summary>
        /// <param name="onInstalled">The callback</param>
        public static void RegisterToExistingInstaller(Action<bool> onInstalled)
        {
            if (s_Installer != null)
            {
                s_Installer.OnInstalled += onInstalled;
            }
        }
        
        /// <summary>
        /// Installs several packages and invokes the callback when all packages are installed/when the installation failed.
        /// </summary>
        /// <param name="packageIds">The package names/ids e.g. com.unity.netcode</param>
        /// <param name="onAllInstalled">The callback</param>
        /// <param name="packageIdsToRemove">Optional package name/ids to remove</param>
        public static void InstallPackages(IEnumerable<string> packageIds, Action<bool> onAllInstalled = null, IEnumerable<string> packageIdsToRemove = null)
        {
            s_Installer = new PackageInstaller(RemoveLocallyLinkedOrEmbeddedOrViaGitPackagesFromList(packageIds), packageIdsToRemove);
            s_Installer.OnInstalled += onAllInstalled;
            s_Installer.OnInstalled += _ => s_Installer = null;
        }

        /// <summary>
        /// Create a dictionary with package names as keys and versions as values    
        /// </summary>
        /// <returns>The mapping (package id, installed version) </returns>
        internal static Dictionary<string, string> InstalledPackageDictionary()
        {
            var installedPackages = UnityEditor.PackageManager.PackageInfo.GetAllRegisteredPackages();
            var installedPackageDictionary = new Dictionary<string, string>();

            foreach (var package in installedPackages)
            {
                var splitPackageId = package.packageId.Split('@');
                if (splitPackageId.Length == 2)
                {
                    installedPackageDictionary[splitPackageId[0]] = splitPackageId[1];
                }
            }

            return installedPackageDictionary;
        }
        
        internal class VersionChecker
        {
            SearchRequest m_Request;
            public VersionChecker(string packageID)
            {
                m_Request = Client.Search(packageID, false);
                EditorApplication.update += Progress;
            }

            public event Action<UnityEditor.PackageManager.PackageInfo> OnVersionFound;

            void Progress()
            {
                if (!m_Request.IsCompleted) return;
            
                EditorApplication.update -= Progress;
                var foundPackage = m_Request.Result;
                foreach (var packageInfo in foundPackage)
                {
                    OnVersionFound?.Invoke(packageInfo);
                }
            }
        }

        class PackageInstaller
        {
            Request m_Request;
            string[] m_PackagesToAddIds;
            public event Action<bool> OnInstalled;

            public PackageInstaller(string packageId)
            {
                // Add a package to the project
                m_Request = Client.Add(packageId);
                m_PackagesToAddIds = new[] {packageId};
                EditorApplication.update += Progress;
            }

            public PackageInstaller(IEnumerable<string> packageIds, IEnumerable<string> packageIdsToRemove = null)
            {
                var packageIdsList = new List<string>();
                foreach (var id in packageIds)
                {
                    packageIdsList.Add(id);
                }
                
                var packageIdsArray = packageIdsList.ToArray();
                
                string[] packageIdsToRemoveArray = null;
                if (packageIdsToRemove != null)
                {
                    var packageIdsToRemoveList = new List<string>();
                    foreach (var id in packageIdsToRemove)
                    {
                        packageIdsToRemoveList.Add(id);
                    }
                    packageIdsToRemoveArray = packageIdsToRemoveList.ToArray();
                }

                // Add a package to the project
                m_Request = Client.AddAndRemove(packageIdsArray, packageIdsToRemoveArray);
                m_PackagesToAddIds = packageIdsArray;
                EditorApplication.update += Progress;
            }

            public bool IsCompleted()
            {
                return m_Request == null || m_Request.IsCompleted;
            }
            
            void Progress()
            {
                if (!m_Request.IsCompleted) return;

                EditorApplication.update -= Progress;
                if (m_Request.Status == StatusCode.Success)
                {
                    Debug.Log("Installed: " + GetInstalledPackageId());
                }
                else if (m_Request.Status >= StatusCode.Failure)
                {
                    // if the request has more than one package, it will only prompt error message for one  
                    // We should prompt all the failed packages
                    Debug.Log("Package installation request with selected packages: " + String.Join(", ", m_PackagesToAddIds) +
                        " failed. \n Reason: "+ m_Request.Error.message);
                }

                OnInstalled?.Invoke(m_Request.Status == StatusCode.Success);
            }

            string GetInstalledPackageId()
            {
                switch (m_Request)
                {
                    case AddRequest addRequest:
                        return addRequest.Result.packageId;
                    case AddAndRemoveRequest addAndRemoveRequest:
                        var packageIds = new List<string>();
                        foreach (var packageInfo in addAndRemoveRequest.Result)
                        {
                            packageIds.Add(packageInfo.packageId);
                        }
                        return string.Join(", ", packageIds);
                    default:
                        throw new InvalidOperationException("Unknown request type");
                }
            }
        }

        /// <summary>
        /// Detects if any multiplayer package is installed by checking for services and Netcode installed packages.
        /// </summary>
        /// <returns>True if any package was detected, False otherwise</returns>
        public static bool IsAnyMultiplayerPackageInstalled()
        {
            var packagesToCheck = new []
            {
                "com.unity.netcode",
                "com.unity.netcode.gameobjects",
                "com.unity.services.multiplayer",
                "com.unity.transport",
                "com.unity.dedicated-server",
                "com.unity.services.cloudcode",
                "com.unity.multiplayer.playmode",
                "com.unity.services.vivox"
                // Note about "com.unity.services.core": it used to be installed only with multiplayer packages, but it is also a dependency of the analytics, which is now always installed.
            };

            foreach (var package in packagesToCheck)
            {
                if (IsInstalled(package))
                {
                    return true;
                }
            }

            return false;
        }
        
        /// <summary>
        /// Checks if the installation process has finished.
        /// </summary>
        /// <returns>True if there is no current installer instance or installation is finished on the installer</returns>
        public static bool IsInstallationFinished()
        {
            return s_Installer == null || s_Installer.IsCompleted();
        }
    }
}
