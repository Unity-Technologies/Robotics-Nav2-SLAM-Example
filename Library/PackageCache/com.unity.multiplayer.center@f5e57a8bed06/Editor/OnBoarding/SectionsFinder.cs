using System;
using System.Collections.Generic;
using System.Reflection;
using Unity.Multiplayer.Center.Common;
using Unity.Multiplayer.Center.Questionnaire;
using Unity.Multiplayer.Center.Recommendations;
using UnityEditor;
using UnityEngine;
using DisplayCondition = Unity.Multiplayer.Center.Common.DisplayCondition;

namespace Unity.Multiplayer.Center.Onboarding
{
    using SectionCategoryToSectionIdToSectionType = Dictionary<OnboardingSectionCategory, Dictionary<string, Type>>;
    
    using SectionCategoryToSectionList = Dictionary<OnboardingSectionCategory, Type[]>;
    
    /// <summary>
    /// Stores the available section types in a serializable way, but for comparison purposes only.
    /// Only the assembly qualified names are serialized in a sorted array.
    /// </summary>
    [Serializable]
    internal class AvailableSectionTypes
    {
        readonly SectionCategoryToSectionList m_SectionMapping;
        
        [SerializeField]
        string[] m_SectionTypeNames;

        public IEnumerable<string> SectionTypeNames => m_SectionTypeNames;

        public AvailableSectionTypes(SectionCategoryToSectionList sectionTypes)
        {
            m_SectionMapping = sectionTypes;
            m_SectionTypeNames = RecommendationUtils.GetSectionTypeNamesInOrder(m_SectionMapping).ToArray();
        }

        public bool TryGetValue(OnboardingSectionCategory category, out Type[] sectionTypes)
        {
            return m_SectionMapping.TryGetValue(category, out sectionTypes);
        }

        public bool HaveTypesChanged(AvailableSectionTypes other)
        {
            return m_SectionTypeNames == null || !RecommendationUtils.AreArraysEqual(m_SectionTypeNames, other.m_SectionTypeNames);
        }
    }
    
    internal static class SectionsFinder
    {
        /// <summary>
        /// The target packages that are used to determine if "nothing is installed", which results in sections with
        /// the display condition "NoPackageInstalled" to be displayed.
        /// </summary>
        static readonly string[] k_TargetPackages = {
            "com.unity.services.core", // This is a dependency of all the services, so if any service is installed, this is installed
            "com.unity.netcode.gameobjects", "com.unity.netcode", // Netcodes
        };

        public static AvailableSectionTypes FindSectionTypes()
        {
            var dico = GetSectionCategoryToSectionIdToSectionType();
            return SortSectionsByOrder(dico);
        }

        static AvailableSectionTypes SortSectionsByOrder(SectionCategoryToSectionIdToSectionType dico)
        {
            var result = new SectionCategoryToSectionList();
            foreach (var (category, idToTypeDictionary) in dico)
            {
                var typeAttributeList = GetOnboardingTypeAttributeList(idToTypeDictionary);
                typeAttributeList.Sort((typeA, typeB) => typeA.Attribute.Order.CompareTo(typeB.Attribute.Order));
                result[category] = GetOnboardingTypeArray(typeAttributeList);
            }

            return new AvailableSectionTypes(result);
        }

        static Type[] GetOnboardingTypeArray(List<(Type Type, OnboardingSectionAttribute Attribute)> typeAttributeList)
        {
            var typeArray = new Type[typeAttributeList.Count];
            for (var i = 0; i < typeAttributeList.Count; i++)
            {
                typeArray[i] = typeAttributeList[i].Type;
            }

            return typeArray;
        }

        static List<(Type Type, OnboardingSectionAttribute Attribute)> GetOnboardingTypeAttributeList(Dictionary<string, Type> idToTypeDictionary)
        {
            var typeAttributeList = new List<(Type Type, OnboardingSectionAttribute Attribute)>(idToTypeDictionary.Count);

            foreach (var (_, type) in idToTypeDictionary)
            {
                var attribute = type.GetCustomAttribute<OnboardingSectionAttribute>();
                if (attribute != null)
                {
                    typeAttributeList.Add((type, attribute));
                }
            }

            return typeAttributeList;
        }

        static SectionCategoryToSectionIdToSectionType GetSectionCategoryToSectionIdToSectionType()
        {
            var dico = new Dictionary<OnboardingSectionCategory, Dictionary<string, System.Type>>();
            foreach (var sectionType in TypeCache.GetTypesDerivedFrom<IOnboardingSection>())
            {
                if (sectionType.IsAbstract) continue;

                // check if type has default constructor
                if (sectionType.GetConstructor(System.Type.EmptyTypes) == null)
                {
                    Debug.LogWarning($"Onboarding section type {sectionType} does not have a default constructor and will be ignored.");
                    continue;
                }

                var sectionAttribute = sectionType.GetCustomAttribute<OnboardingSectionAttribute>();
                if(sectionAttribute == null)
                    continue;
                
                if (!IsDisplayConditionFulfilledForSection(sectionAttribute))
                        continue;
                
                if (!IsSectionSupportedBySelectedInfrastructure(sectionAttribute))
                    continue;
                
                if (!IsSectionSupportedBySelectedNetcodeChoice(sectionAttribute))
                    continue;

                if (!dico.ContainsKey(sectionAttribute.Category))
                    dico[sectionAttribute.Category] = new Dictionary<string, System.Type>();

                if (dico[sectionAttribute.Category].TryGetValue(sectionAttribute.Id, out var existing))
                {
                    var existingAttr = existing.GetCustomAttribute<OnboardingSectionAttribute>();

                    if (existingAttr!= null && existingAttr.Id == sectionAttribute.Id 
                        && sectionAttribute.Priority > existingAttr.Priority)
                    {
                        dico[sectionAttribute.Category][sectionAttribute.Id] = sectionType;
                    }
                }
                else
                {
                    dico[sectionAttribute.Category][sectionAttribute.Id] = sectionType;
                }
            }

            return dico;
        }

        static bool IsDisplayConditionFulfilledForSection(OnboardingSectionAttribute attribute)
        {
            return attribute.DisplayCondition switch
            {
                DisplayCondition.None => true,
                DisplayCondition.PackageInstalled when string.IsNullOrEmpty(attribute.TargetPackageId)
                    => throw new ArgumentException("PackageInstalled condition requires a target package id"),
                DisplayCondition.PackageInstalled => PackageManagement.IsInstalled(attribute.TargetPackageId),
                DisplayCondition.NoPackageInstalled => NothingInstalled(),
                _ => throw new NotImplementedException($"Unknown display condition: {attribute.DisplayCondition}")
            };
        }
        
      static bool IsSectionSupportedBySelectedInfrastructure(OnboardingSectionAttribute attribute)
        {
            if (UserChoicesObject.instance.SelectedSolutions == null) return true;
            
            var selectedInfrastructure = UserChoicesObject.instance.SelectedSolutions.SelectedHostingModel;
            return attribute.HostingModelDependency == SelectedSolutionsData.HostingModel.None || attribute.HostingModelDependency == selectedInfrastructure;
        }
        
        static bool IsSectionSupportedBySelectedNetcodeChoice(OnboardingSectionAttribute attribute)
        {
            if (UserChoicesObject.instance.SelectedSolutions == null) return true;
            
            var selectedNetcode = UserChoicesObject.instance.SelectedSolutions.SelectedNetcodeSolution;
            return attribute.NetcodeDependency == SelectedSolutionsData.NetcodeSolution.None || attribute.NetcodeDependency == selectedNetcode;
        }

        static bool NothingInstalled()
        {
            return !PackageManagement.IsAnyPackageInstalled(k_TargetPackages);
        }
    }
}
