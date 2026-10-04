using System;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Multiplayer.Center.Common;
using Unity.Multiplayer.Center.Questionnaire;

namespace Unity.MultiplayerCenterTests
{
    partial class RecommendationTests
    {
        public static IEnumerable<TestCaseData> AdventurePresetCases
            => new TestCaseData[]
            {
                new("2", PossibleSolution.NGO, PossibleSolution.DS, Preset.Adventure),
                new("4", PossibleSolution.NGO, PossibleSolution.DS, Preset.Adventure),
                new("8", PossibleSolution.NGO, PossibleSolution.DS, Preset.Adventure),
                new("16", PossibleSolution.NGO, PossibleSolution.DS, Preset.Adventure),
                new("64+", PossibleSolution.NGO, PossibleSolution.DS, Preset.Adventure),
                new("128", PossibleSolution.N4E, PossibleSolution.DS, Preset.Adventure)
            };

        public static IEnumerable<TestCaseData> SandboxPresetCases
            => new TestCaseData[]
            {
                new("2", PossibleSolution.NGO, PossibleSolution.DA, Preset.Sandbox),
                new("4", PossibleSolution.NGO, PossibleSolution.DA, Preset.Sandbox),
                new("8", PossibleSolution.NGO, PossibleSolution.DA, Preset.Sandbox),
                new("16", PossibleSolution.NGO, PossibleSolution.DA, Preset.Sandbox),
                new("64+", PossibleSolution.NGO, PossibleSolution.DA, Preset.Sandbox),
                new("128", PossibleSolution.N4E, PossibleSolution.DS, Preset.Sandbox)
            };

        public static IEnumerable<TestCaseData> AsyncPresetCases
            => new TestCaseData[]
            {
                new("2", PossibleSolution.NoNetcode, PossibleSolution.CloudCode, Preset.Async),
                new("4", PossibleSolution.NoNetcode, PossibleSolution.CloudCode, Preset.Async),
                new("8", PossibleSolution.NoNetcode, PossibleSolution.CloudCode, Preset.Async),
                new("16", PossibleSolution.NoNetcode, PossibleSolution.CloudCode, Preset.Async),
                new("64+", PossibleSolution.NoNetcode, PossibleSolution.CloudCode, Preset.Async),
                new("128", PossibleSolution.NoNetcode, PossibleSolution.CloudCode, Preset.Async)
            };

        public static IEnumerable<TestCaseData> TurnBasedPresetCases
            => new TestCaseData[]
            {
                new("2", PossibleSolution.NoNetcode, PossibleSolution.CloudCode, Preset.TurnBased),
                new("4", PossibleSolution.NoNetcode, PossibleSolution.CloudCode, Preset.TurnBased),
                new("8", PossibleSolution.NoNetcode, PossibleSolution.CloudCode, Preset.TurnBased),
                new("16", PossibleSolution.NoNetcode, PossibleSolution.CloudCode, Preset.TurnBased),
                new("64+", PossibleSolution.NoNetcode, PossibleSolution.CloudCode, Preset.TurnBased),
                new("128", PossibleSolution.NoNetcode, PossibleSolution.CloudCode, Preset.TurnBased)
            };

        public static IEnumerable<TestCaseData> FightingPresetCases
            => new TestCaseData[]
            {
                new("2", PossibleSolution.CustomNetcode, PossibleSolution.LS, Preset.Fighting),
                new("4", PossibleSolution.CustomNetcode, PossibleSolution.LS, Preset.Fighting),
                new("8", PossibleSolution.CustomNetcode, PossibleSolution.LS, Preset.Fighting),
                new("16", PossibleSolution.CustomNetcode, PossibleSolution.LS, Preset.Fighting),
                new("64+", PossibleSolution.CustomNetcode, PossibleSolution.LS, Preset.Fighting),
                new("128", PossibleSolution.CustomNetcode, PossibleSolution.LS, Preset.Fighting)
            };

        public static IEnumerable<TestCaseData> RacingPresetCases
            => new TestCaseData[]
            {
                new("2", PossibleSolution.N4E, PossibleSolution.DS, Preset.Racing),
                new("4", PossibleSolution.N4E, PossibleSolution.DS, Preset.Racing),
                new("8", PossibleSolution.N4E, PossibleSolution.DS, Preset.Racing),
                new("16", PossibleSolution.N4E, PossibleSolution.DS, Preset.Racing),
                new("64+", PossibleSolution.N4E, PossibleSolution.DS, Preset.Racing),
                new("128", PossibleSolution.N4E, PossibleSolution.DS, Preset.Racing)
            };

        public static IEnumerable<TestCaseData> RolePlayingPresetCases
            => new TestCaseData[]
            {
                new("2", PossibleSolution.CustomNetcode, PossibleSolution.DS, Preset.RolePlaying),
                new("4", PossibleSolution.CustomNetcode, PossibleSolution.DS, Preset.RolePlaying),
                new("8", PossibleSolution.CustomNetcode, PossibleSolution.DS, Preset.RolePlaying),
                new("16", PossibleSolution.CustomNetcode, PossibleSolution.DS, Preset.RolePlaying),
                new("64+", PossibleSolution.CustomNetcode, PossibleSolution.DS, Preset.RolePlaying),
                new("128", PossibleSolution.CustomNetcode, PossibleSolution.DS, Preset.RolePlaying)
            };

        public static IEnumerable<TestCaseData> ShooterPresetCases
            => new TestCaseData[]
            {
                new("2", PossibleSolution.N4E, PossibleSolution.DS, Preset.Shooter),
                new("4", PossibleSolution.N4E, PossibleSolution.DS, Preset.Shooter),
                new("8", PossibleSolution.N4E, PossibleSolution.DS, Preset.Shooter),
                new("16", PossibleSolution.N4E, PossibleSolution.DS, Preset.Shooter),
                new("64+", PossibleSolution.N4E, PossibleSolution.DS, Preset.Shooter),
                new("128", PossibleSolution.N4E, PossibleSolution.DS, Preset.Shooter)
            };

        public static IEnumerable<TestCaseData> SimulationPresetCases
            => new TestCaseData[]
            {
                new("2", PossibleSolution.NGO, PossibleSolution.DA, Preset.Simulation),
                new("4", PossibleSolution.NGO, PossibleSolution.DA, Preset.Simulation),
                new("8", PossibleSolution.NGO, PossibleSolution.DA, Preset.Simulation),
                new("16", PossibleSolution.NGO, PossibleSolution.DA, Preset.Simulation),
                new("64+", PossibleSolution.NGO, PossibleSolution.DA, Preset.Simulation),
                new("128", PossibleSolution.N4E, PossibleSolution.DS, Preset.Simulation)
            };

        public static IEnumerable<TestCaseData> StrategyPresetCases
            => new TestCaseData[]
            {
                new("2", PossibleSolution.CustomNetcode, PossibleSolution.LS, Preset.Strategy),
                new("4", PossibleSolution.CustomNetcode, PossibleSolution.LS, Preset.Strategy),
                new("8", PossibleSolution.CustomNetcode, PossibleSolution.LS, Preset.Strategy),
                new("16", PossibleSolution.CustomNetcode, PossibleSolution.LS, Preset.Strategy),
                new("64+", PossibleSolution.CustomNetcode, PossibleSolution.LS, Preset.Strategy),
                new("128", PossibleSolution.CustomNetcode, PossibleSolution.LS, Preset.Strategy)
            };

        public static IEnumerable<TestCaseData> SportPresetCases
            => new TestCaseData[]
            {
                new("2", PossibleSolution.N4E, PossibleSolution.DS, Preset.Sports),
                new("4", PossibleSolution.N4E, PossibleSolution.DS, Preset.Sports),
                new("8", PossibleSolution.N4E, PossibleSolution.DS, Preset.Sports),
                new("16", PossibleSolution.N4E, PossibleSolution.DS, Preset.Sports),
                new("64+", PossibleSolution.N4E, PossibleSolution.DS, Preset.Sports),
                new("128", PossibleSolution.N4E, PossibleSolution.DS, Preset.Sports)
            };
    }
}
