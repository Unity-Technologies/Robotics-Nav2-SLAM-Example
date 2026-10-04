using System;
using System.Collections.Generic;
using System.Text;
using Unity.Multiplayer.Center.Common;
using Unity.Multiplayer.Center.Onboarding;
using Unity.Multiplayer.Center.Questionnaire;
using UnityEngine;

namespace Unity.Multiplayer.Center.Recommendations
{
    internal static class RecommendationUtils
    {
        public static List<RecommendedPackageViewData> PackagesToInstall(RecommendationViewData recommendation,
            SolutionsToRecommendedPackageViewData solutionToPackageData)
        {
            var packagesToInstall = new List<RecommendedPackageViewData>();

            // Can happen on first load
            if (recommendation?.NetcodeOptions == null)
                return packagesToInstall;

            var selectedNetcode = GetSelectedNetcode(recommendation);
            if (selectedNetcode == null)
                return packagesToInstall;

            // add features based on netcode
            if (selectedNetcode.MainPackage != null)
                packagesToInstall.Add(selectedNetcode.MainPackage);

            var selectedServerArchitecture = GetSelectedHostingModel(recommendation);
            if (selectedServerArchitecture.MainPackage != null)
                packagesToInstall.Add(selectedServerArchitecture.MainPackage);

            foreach (var package in solutionToPackageData.GetPackagesForSelection(selectedNetcode.Solution, selectedServerArchitecture.Solution))
            {
                if (package.Selected)
                {
                    packagesToInstall.Add(package);
                }
            }

            return packagesToInstall;
        }

        public static RecommendedSolutionViewData GetSelectedHostingModel(RecommendationViewData recommendation)
        {
            return GetSelectedSolution(recommendation.ServerArchitectureOptions);
        }

        public static RecommendedSolutionViewData GetSelectedNetcode(RecommendationViewData recommendation)
        {
            return GetSelectedSolution(recommendation.NetcodeOptions);
        }

        /// <summary>
        /// Finds the first selected solution in the input array.
        /// </summary>
        /// <param name="availableSolutions">The available solutions.</param>
        /// <returns>Returns the first selected solution. If no solution is selected, it returns null.</returns>
        public static RecommendedSolutionViewData GetSelectedSolution(RecommendedSolutionViewData[] availableSolutions)
        {
            foreach (var solution in availableSolutions)
            {
                if (solution.Selected)
                {
                    return solution;
                }
            }

            return default;
        }

        public static PackageDetails GetPackageDetailForPackageId(string packageId)
        {
            var idToPackageDetailDict = RecommenderSystemDataObject.instance.RecommenderSystemData.PackageDetailsById;

            if (idToPackageDetailDict.TryGetValue(packageId, out var packageDetail))
                return packageDetail;

            Debug.LogError("Trying to get package detail for package id that does not exist: " + packageId);
            return null;
        }

        /// <summary>
        /// Returns all the packages passed via packageIds and their informal dependencies (stored in AdditionalPackages)
        /// </summary>
        /// <returns>List of PackageDetails</returns>
        /// <param name="packages">List of package id</param>
        /// <param name="toolTip">tooltip text</param>
        public static void GetPackagesWithAdditionalPackages(List<RecommendedPackageViewData> packages,
            out List<string> ids, out List<string> names, out string toolTip)
        {
            ids = new List<string>();
            names = new List<string>();
            var toolTipBuilder = new StringBuilder();
            foreach (var package in packages)
            {
                var packageDetail = GetPackageDetailForPackageId(package.PackageId);

                var id = string.IsNullOrEmpty(package.PreReleaseVersion)? package.PackageId : $"{package.PackageId}@{package.PreReleaseVersion}";
                var name = string.IsNullOrEmpty(package.PreReleaseVersion)? packageDetail.Name : $"{packageDetail.Name} {package.PreReleaseVersion}";
                ids.Add(id);
                names.Add(name);
                toolTipBuilder.Append(name);

                if (packageDetail.AdditionalPackages is {Length: > 0})
                {
                    toolTipBuilder.Append(" + ");
                    foreach (var additionalPackageId in packageDetail.AdditionalPackages)
                    {
                        var additionalPackage = GetPackageDetailForPackageId(additionalPackageId);
                        ids.Add(additionalPackage.Id);
                        names.Add(additionalPackage.Name);
                        toolTipBuilder.Append(additionalPackage.Name);
                        toolTipBuilder.Append(",");
                    }
                }

                toolTipBuilder.Append("\n");
            }

            // remove last newline
            toolTip = toolTipBuilder.ToString().TrimEnd('\n');
            ids.Add(QuickstartIsMissingView.PackageId);
        }

        /// <summary>
        /// Reapplies the previous selection on the view data
        /// </summary>
        /// <param name="recommendation">The recommendation view data to update</param>
        /// <param name="data">The previous selection data</param>
        public static void ApplyPreviousSelection(RecommendationViewData recommendation, SelectedSolutionsData data)
        {
            if (data == null || recommendation == null)
                return;

            if (data.SelectedNetcodeSolution != SelectedSolutionsData.NetcodeSolution.None)
            {
                foreach (var d in recommendation.NetcodeOptions)
                {
                    d.Selected = Logic.ConvertNetcodeSolution(d) == data.SelectedNetcodeSolution;
                }
            }

            if (data.SelectedHostingModel != SelectedSolutionsData.HostingModel.None)
            {
                foreach (var view in recommendation.ServerArchitectureOptions)
                {
                    view.Selected = Logic.ConvertInfrastructure(view) == data.SelectedHostingModel;
                }
            }
        }

        /// <summary>
        /// Returns the packages that are of the given recommendation type
        /// </summary>
        /// <param name="packages">All the package view data as returned by the recommender system</param>
        /// <param name="type">The target recommendation type</param>
        /// <returns>The filtered list</returns>
        public static List<RecommendedPackageViewData> FilterByType(IEnumerable<RecommendedPackageViewData> packages, RecommendationType type)
        {
            var filteredPackages = new List<RecommendedPackageViewData>();

            foreach (var package in packages)
            {
                if (package.RecommendationType == type)
                {
                    filteredPackages.Add(package);
                }
            }

            return filteredPackages;
        }

        public static int IndexOfMaximumScore(RecommendedSolutionViewData[] array)
        {
            var maxIndex = -1;
            var maxScore = float.MinValue;
            for (var index = 0; index < array.Length; index++)
            {
                var solution = array[index];
                if (solution.RecommendationType == RecommendationType.Incompatible)
                    continue;

                if (solution.Score > maxScore)
                {
                    maxScore = solution.Score;
                    maxIndex = index;
                }
            }

            return maxIndex;
        }

        /// <summary>
        /// Finds the recommended solution among the scored solutions.
        /// </summary>
        /// <param name="scoredSolutions">An array of tuples, where each tuple contains a PossibleSolution and its corresponding Scoring.</param>
        /// <returns>Returns the recommended solution with the maximum total score.</returns>
        public static PossibleSolution FindRecommendedSolution((PossibleSolution, Scoring)[] scoredSolutions)
        {
            var maxScore = float.MinValue;
            PossibleSolution recommendedSolution = default;
            foreach (var (possibleSolution, scoring) in scoredSolutions)
            {
                if (scoring.TotalScore > maxScore)
                {
                    maxScore = scoring.TotalScore;
                    recommendedSolution = possibleSolution;
                }
            }

            return recommendedSolution;
        }

        /// <summary>
        /// Looks through the hosting models and marks them incompatible if necessary (deselected and recommendation
        /// type incompatible).
        /// Note that this might leave the recommendation in an invalid state (no hosting model selected)
        /// </summary>
        /// <param name="recommendation">The recommendation data to modify.</param>
        public static void MarkIncompatibleHostingModels(RecommendationViewData recommendation)
        {
            var netcode = GetSelectedNetcode(recommendation);

            for (var index = 0; index < recommendation.ServerArchitectureOptions.Length; index++)
            {
                var hosting = recommendation.ServerArchitectureOptions[index];
                var isCompatible = RecommenderSystemDataObject.instance.RecommenderSystemData.IsHostingModelCompatibleWithNetcode(netcode.Solution, hosting.Solution, out _);
                if (!isCompatible)
                {
                    hosting.RecommendationType = RecommendationType.Incompatible;
                    hosting.Selected = false;
                }
                else
                {
                    hosting.RecommendationType = RecommendationType.SecondArchitectureChoice;
                }
            }
        }

        /// <summary>
        /// Finds a recommended package view by its ID from a list.
        /// </summary>
        /// <param name="packages">A list of Recommended Package ViewData representing the packages.</param>
        /// <param name="id">The ID of the package to find.</param>
        /// <returns>Returns the RecommendedPackageViewData object with the matching ID. If none is found, it returns null.</returns>
        public static RecommendedPackageViewData FindRecommendedPackageViewById( List<RecommendedPackageViewData> packages, string id)
        {
            RecommendedPackageViewData featureToSet = default;
            foreach (var package in packages)
            {
                if (package.PackageId == id)
                {
                    featureToSet = package;
                    break;
                }
            }

            return featureToSet;
        }

        /// <summary>
        /// Checks if the specific question has been answered.
        /// </summary>
        /// <param name="question">The question to check.</param>
        /// <returns>Returns true if the question has been answered by the user, otherwise returns false.</returns>
        public static bool IsQuestionAnswered(Question question)
        {
            return Logic.TryGetAnswerByQuestionId(UserChoicesObject.instance.UserAnswers, question.Id, out _);
        }

        /// <summary>
        /// Compares two arrays of type <typeparamref name="T"/> for equality.
        /// </summary>
        /// <typeparam name="T">The type of the elements in the arrays.</typeparam>
        /// <param name="a">The first array to compare.</param>
        /// <param name="b">The second array to compare.</param>
        /// <returns>
        /// <c>true</c> if all elements are equal and they are in the same order,
        /// <c>false</c> otherwise.
        /// </returns>
        public static bool AreArraysEqual<T>(T[] a, T[] b)
        {
            if (a.Length != b.Length)
            {
                return false;
            }

            for (var i = 0; i < a.Length; i++)
            {
                if (!EqualityComparer<T>.Default.Equals(a[i], b[i]))
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Gets the section type names in order from the provided section mapping.
        /// </summary>
        /// <param name="sectionMapping">A dictionary mapping OnboardingSectionCategory to SectionList types.</param>
        /// <returns>A list of section type names sorted in ascending order.</returns>
        public static List<string> GetSectionTypeNamesInOrder(Dictionary<OnboardingSectionCategory, Type[]> sectionMapping)
        {
            var sectionTypeNamesList = new List<string>();

            foreach (var sectionTypeArray in sectionMapping.Values)
            {
                foreach (var sectionType in sectionTypeArray)
                {
                    sectionTypeNamesList.Add(sectionType.AssemblyQualifiedName);
                }
            }

            sectionTypeNamesList.Sort(StringComparer.InvariantCulture);
            return sectionTypeNamesList;
        }
    }
}
