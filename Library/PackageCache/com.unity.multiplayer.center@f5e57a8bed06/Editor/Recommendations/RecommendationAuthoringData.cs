#if MULTIPLAYER_CENTER_DEV_MODE
using System;
using Unity.Multiplayer.Center.Questionnaire;
namespace Unity.Multiplayer.Center.Recommendations
{
    /// <summary>
    /// This file contains all data relevant for authoring recommendations.
    /// </summary>
    
    internal static class Packages
    {
        public const string VivoxId = "com.unity.services.vivox";
        public const string MultiplayerSdkId = "com.unity.services.multiplayer";
        public const string MultiplayerWidgetsId = "com.unity.multiplayer.widgets";
        public const string NetcodeForEntitiesId = "com.unity.netcode";
        public const string NetcodeGameObjectsId = "com.unity.netcode.gameobjects";
        public const string MultiplayerToolsId = "com.unity.multiplayer.tools";
        public const string MultiplayerPlayModeId = "com.unity.multiplayer.playmode";
        public const string CloudCodeId = "com.unity.services.cloudcode";
        public const string TransportId = "com.unity.transport";
        public const string DeploymentPackageId = "com.unity.services.deployment";
        public const string DedicatedServerPackageId = "com.unity.dedicated-server";
        public const string EntitiesGraphics = "com.unity.entities.graphics";
    }
    internal static class Reasons
    {
        public const string MultiplayerPlayModeRecommended = "Multiplayer Play Mode enables you to iterate faster by testing locally without the need to create builds.";
        public const string VivoxRecommended = "Vivox adds real-time communication and is always recommended to boost players engagement.";
        public const string MultiplayerToolsRecommended = "Tools speed up your development workflow for games using Netcode for GameObjects.";
        public const string MultiplayerToolsIncompatible = "Multiplayer Tools is only compatible with Netcode for GameObjects.";
        public const string MultiplayerSdkRecommended = "The Multiplayer Services package makes it easy to connect players together using Unity Gaming Services.";
        public const string MultiplayerSdkRecommendedNoNetcode = "The Multiplayer Services package makes it easy to connect players together using Unity Gaming Services. This can also be useful in a no-netcode context.";
        public const string MultiplayerWidgetsRecommended = "Widgets provide pre-implemented building blocks to help you connect players together using services as fast as possible.";
        public const string MultiplayerWidgetsNotRecommendedBecauseWrongNetcode = "The Multiplayer Widgets package is currently only recommended with Netcode for GameObjects and Netcode for Entities.";
        public const string DedicatedServerPackageRecommended = "The Dedicated Server package, which includes Content Selection, helps with the development for the dedicated server platforms.";
        public const string DedicatedServerPackageNotRecommended = "The Dedicated Server package is only recommended when you have a dedicated server architecture and use Netcode for Gameobjects.";
        public const string DedicatedServerPackageNotRecommendedN4E = "The Dedicated Server package is currently not recommended when using Netcode for Entities.";
        public const string DistributedAuthorityIncompatible = "Distributed Authority is only available when using Netcode for GameObjects.";
        public const string DistributedAuthorityNotAvailable = "Distributed Authority is not available yet.";
        public const string EntitiesGraphicsRecommended = "While you do not need Entities Graphics to use Netcode for Entities, it will make it easier for you to get started";
        public const string EntitiesGraphicsNotRecommended = "You would typically only use Entities Graphics with Netcode for Entities";
        public const string TransportRecommended = "Unity Transport is the low-level interface for connecting and sending data through a network, and can be the basis of your custom netcode solution.";
        public const string DeploymentRecommended = "The deployment package is necessary to upload your C# Cloud Code scripts.";
    }
    
    internal static class DocLinks
    {
        public const string DedicatedServer = "https://docs-multiplayer.unity3d.com/netcode/current/terms-concepts/network-topologies/#dedicated-game-server";
        public const string ListenServer = "https://docs-multiplayer.unity3d.com/netcode/current/terms-concepts/network-topologies/#client-hosted-listen-server";
        public const string NetcodeForGameObjects = "https://docs-multiplayer.unity3d.com/netcode/current/about/";
        public const string NetcodeForEntities = "https://docs.unity3d.com/Packages/com.unity.netcode@latest";
        public const string Vivox = "https://docs.unity.com/ugs/en-us/manual/vivox-unity/manual/Unity/Unity";
        public const string MultiplayerTools = "https://docs-multiplayer.unity3d.com/tools/current/about/";
        public const string MultiplayerPlayMode = "https://docs-multiplayer.unity3d.com/mppm/current/about/";
        public const string MultiplayerSdk = "https://docs.unity3d.com/Packages/com.unity.services.multiplayer@latest";
        public const string MultiplayerWidgets = "https://docs.unity3d.com/Packages/com.unity.multiplayer.widgets@latest";
        public const string CloudCode = "https://docs.unity.com/ugs/manual/cloud-code/manual";
        public const string Transport = "https://docs.unity3d.com/Packages/com.unity.transport@latest";
        public const string DedicatedServerPackage = "https://docs.unity3d.com/Packages/com.unity.dedicated-server@latest";
        public const string DistributedAuthority = "https://docs-multiplayer.unity3d.com/netcode/current/terms-concepts/distributed-authority/";
        public const string EntitiesGraphics = "https://docs.unity3d.com/Packages/com.unity.entities.graphics@latest";
        public const string DeploymentPackage = "https://docs.unity3d.com/Packages/com.unity.services.deployment@latest";
    }

    internal static class SolutionDescriptionAndReason
    {
        public const string DistributedAuthority = "The authority over the game will be distributed across different players. This hosting model [dynamic].";
        public const string DedicatedServer = "A dedicated server has authority over the game logic. This hosting model [dynamic].";
        public const string ListenServer = "A player will be the host of your game. This hosting model [dynamic].";
        public const string NetcodeForGameObjects = "Netcode for GameObject is a high-level networking library built for GameObjects to abstract networking logic. It is made for simplicity. It [dynamic].";
        public const string NetcodeForEntities = "Netcode for Entities is a multiplayer solution with server authority and client prediction. It is made for performance. It [dynamic].";
        public const string CloudCode = "When gameplay does not require a synchronous multiplayer experience, Cloud Code allows to run player interaction logic directly on the backend side. It [dynamic].";
        public const string CustomNetcode = "Custom or third-party netcode, not provided by Unity. It [dynamic].";
        public const string NoNetcode = "Your game doesn't require realtime synchronization. No Netcode [dynamic].";
    }
    
    internal static class CatchPhrases
    {
        public const string NetcodeForGameObjects = "Multiplayer synchronization for gameplay based on GameObjects.";
        public const string NetcodeForEntities = "Multiplayer synchronization for gameplay based on Entity Component System.";
        public const string Vivox = "Connect players through voice and text chat.";
        public const string MultiplayerTools = "Debug and optimize your multiplayer gameplay.";
        public const string MultiplayerPlayMode = "Test multiplayer gameplay in separated processes from the same project.";
        public const string MultiplayerSdk = "Connect players together in sessions for lobby, matchmaker, etc.";
        public const string MultiplayerWidgets = "Experiment rapidly with multiplayer services.";
        public const string DedicatedServerPackage = "Streamline dedicated server builds.";
        public const string CloudCode = "Run game logic as serverless functions.";
        public const string EntitiesGraphics = "Optimized rendering for Entity Component System.";
        public const string DeploymentPackage = "Deploy assets to Unity Gaming Services from the Editor.";
        public const string Transport = "Low-level networking communication layer.";
    }

    internal static class Titles
    {
        public const string DedicatedServer = "Dedicated Server";
        public const string ListenServer = "Client Hosted";
        public const string NetcodeForGameObjects = "Netcode for GameObjects";
        public const string NetcodeForEntities = "Netcode for Entities";
        public const string Vivox = "Voice/Text chat (Vivox)";
        public const string MultiplayerTools = "Multiplayer Tools";
        public const string MultiplayerPlayMode = "Multiplayer Play Mode";
        public const string NoUnityNetcode = "No Netcode";
        public const string MultiplayerSdk = "Multiplayer Services";
        public const string MultiplayerWidgets = "Multiplayer Widgets";
        public const string CloudCode = "Cloud Code";
        public const string CustomNetcode = "Custom or Third-party Netcode";
        public const string Transport = "Transport";
        public const string DedicatedServerPackage = "Dedicated Server Package";
        public const string DeploymentPackage = "Deployment Package";
        public const string EntitiesGraphics = "Entities Graphics";
        public const string DistributedAuthority = "Distributed Authority";
    }

    static class RecommendationAssetUtils
    {
        public static RecommenderSystemData PopulateDefaultRecommendationData()
        {
            var data = new RecommenderSystemData();
            data.TargetUnityVersion = UnityEngine.Application.unityVersion;
            data.RecommendedSolutions = new RecommendedSolution[]
            {
                new()
                {
                    Type = PossibleSolution.LS,
                    Title = Titles.ListenServer,
                    DocUrl = DocLinks.ListenServer,
                    ShortDescription = SolutionDescriptionAndReason.ListenServer,
                    RecommendedPackages = new RecommendedPackage[]
                    {
                        new(Packages.DedicatedServerPackageId, RecommendationType.Incompatible, Reasons.DedicatedServerPackageNotRecommended),
                    }
                },
                new()
                {
                    Type = PossibleSolution.DS,
                    Title = Titles.DedicatedServer,
                    DocUrl = DocLinks.DedicatedServer,
                    ShortDescription = SolutionDescriptionAndReason.DedicatedServer,
                    RecommendedPackages = new RecommendedPackage[]
                    {
                    }
                },
                
                new()
                {
                    Type = PossibleSolution.DA,
                    Title = Titles.DistributedAuthority,
                    DocUrl = DocLinks.DistributedAuthority,
                    ShortDescription = SolutionDescriptionAndReason.DistributedAuthority,
                    RecommendedPackages = new RecommendedPackage[]
                    { 
                        new(Packages.MultiplayerSdkId, RecommendationType.HostingFeatured, "Distributed Authority needs the Multiplayer Services package to work."),
                        new(Packages.DedicatedServerPackageId, RecommendationType.Incompatible, Reasons.DedicatedServerPackageNotRecommended),
                    }
                },
                
                new()
                {
                    Type = PossibleSolution.CloudCode,
                    Title = Titles.CloudCode,
                    MainPackageId = Packages.CloudCodeId,
                    ShortDescription = SolutionDescriptionAndReason.CloudCode,
                    DocUrl = DocLinks.CloudCode,
                    
                    RecommendedPackages = new RecommendedPackage[]
                    {
                        new(Packages.DedicatedServerPackageId, RecommendationType.Incompatible, Reasons.DedicatedServerPackageNotRecommended),
                        new(Packages.DeploymentPackageId, RecommendationType.HostingFeatured, Reasons.DeploymentRecommended),
                    }
                },
                new()
                {
                    Type = PossibleSolution.NGO,
                    Title = Titles.NetcodeForGameObjects,
                    MainPackageId = Packages.NetcodeGameObjectsId,
                    ShortDescription = SolutionDescriptionAndReason.NetcodeForGameObjects,
                    RecommendedPackages = new RecommendedPackage[]
                    {
                        new(Packages.TransportId, RecommendationType.Hidden, null),
                        new(Packages.DeploymentPackageId, RecommendationType.Hidden, null),
                        new(Packages.MultiplayerToolsId, RecommendationType.NetcodeFeatured, Reasons.MultiplayerToolsRecommended),
                        new(Packages.DedicatedServerPackageId, RecommendationType.HostingFeatured, Reasons.DedicatedServerPackageRecommended  ),
                        new(Packages.EntitiesGraphics, RecommendationType.Hidden, Reasons.EntitiesGraphicsNotRecommended),
                        new(Packages.VivoxId, RecommendationType.OptionalStandard, Reasons.VivoxRecommended),
                        new(Packages.MultiplayerSdkId, RecommendationType.OptionalStandard, Reasons.MultiplayerSdkRecommended),
                        new(Packages.MultiplayerWidgetsId, RecommendationType.OptionalStandard, Reasons.MultiplayerWidgetsRecommended),
                        new(Packages.MultiplayerPlayModeId, RecommendationType.OptionalStandard, Reasons.MultiplayerPlayModeRecommended),
                    }
                },
                new()
                {
                    Type = PossibleSolution.N4E,
                    Title = Titles.NetcodeForEntities,
                    MainPackageId = Packages.NetcodeForEntitiesId,
                    ShortDescription = SolutionDescriptionAndReason.NetcodeForEntities,
                    RecommendedPackages = new RecommendedPackage[]
                    {
                        new(Packages.TransportId, RecommendationType.Hidden, null),
                        new(Packages.DeploymentPackageId, RecommendationType.Hidden, null),
                        new(Packages.EntitiesGraphics, RecommendationType.NetcodeFeatured, Reasons.EntitiesGraphicsRecommended),
                        new(Packages.VivoxId, RecommendationType.OptionalStandard, Reasons.VivoxRecommended),
                        new(Packages.MultiplayerSdkId, RecommendationType.OptionalStandard, Reasons.MultiplayerSdkRecommended),
                        new(Packages.MultiplayerWidgetsId, RecommendationType.OptionalStandard, Reasons.MultiplayerWidgetsRecommended),
                        new(Packages.MultiplayerToolsId, RecommendationType.Incompatible, Reasons.MultiplayerToolsIncompatible),
                        new(Packages.MultiplayerPlayModeId, RecommendationType.OptionalStandard, Reasons.MultiplayerPlayModeRecommended),
                        new(Packages.DedicatedServerPackageId, RecommendationType.NotRecommended, Reasons.DedicatedServerPackageNotRecommended)
                    },
                    IncompatibleSolutions = new IncompatibleSolution[]{new(PossibleSolution.DA, Reasons.DistributedAuthorityIncompatible)}
                },
                new()
                {
                    Type = PossibleSolution.NoNetcode,
                    Title = Titles.NoUnityNetcode,
                    DocUrl = null,
                    MainPackageId = null,
                    ShortDescription = SolutionDescriptionAndReason.NoNetcode,
                    RecommendedPackages = new RecommendedPackage[]
                    {
                        new(Packages.TransportId, RecommendationType.Hidden, null),
                        new(Packages.DeploymentPackageId, RecommendationType.Hidden, null),
                        new(Packages.MultiplayerToolsId, RecommendationType.Incompatible, Reasons.MultiplayerToolsIncompatible),
                        new(Packages.EntitiesGraphics, RecommendationType.Hidden, Reasons.EntitiesGraphicsNotRecommended),
                        new(Packages.VivoxId, RecommendationType.OptionalStandard, Reasons.VivoxRecommended),
                        new(Packages.MultiplayerSdkId, RecommendationType.OptionalStandard, Reasons.MultiplayerSdkRecommendedNoNetcode),
                        new(Packages.MultiplayerWidgetsId, RecommendationType.NotRecommended, Reasons.MultiplayerWidgetsNotRecommendedBecauseWrongNetcode),
                        new(Packages.MultiplayerPlayModeId, RecommendationType.OptionalStandard, Reasons.MultiplayerPlayModeRecommended),
                        new(Packages.DedicatedServerPackageId, RecommendationType.NotRecommended, Reasons.DedicatedServerPackageNotRecommended),
                    },
                    IncompatibleSolutions = new IncompatibleSolution[]{new(PossibleSolution.DA, Reasons.DistributedAuthorityIncompatible)}
                },
                new()
                {
                    Type = PossibleSolution.CustomNetcode,
                    Title = Titles.CustomNetcode,
                    MainPackageId = null,
                    ShortDescription = SolutionDescriptionAndReason.CustomNetcode,
                    RecommendedPackages = new RecommendedPackage[]
                    {
                        new(Packages.TransportId, RecommendationType.NetcodeFeatured, Reasons.TransportRecommended),
                        new(Packages.EntitiesGraphics, RecommendationType.Hidden, Reasons.EntitiesGraphicsNotRecommended),
                        new(Packages.DeploymentPackageId, RecommendationType.Hidden, null),
                        new(Packages.VivoxId, RecommendationType.OptionalStandard, Reasons.VivoxRecommended),
                        new(Packages.MultiplayerSdkId, RecommendationType.OptionalStandard, Reasons.MultiplayerSdkRecommended),
                        new(Packages.MultiplayerWidgetsId, RecommendationType.NotRecommended, Reasons.MultiplayerWidgetsNotRecommendedBecauseWrongNetcode),
                        new(Packages.MultiplayerToolsId, RecommendationType.Incompatible, Reasons.MultiplayerToolsIncompatible),
                        new(Packages.MultiplayerPlayModeId, RecommendationType.OptionalStandard, Reasons.MultiplayerPlayModeRecommended),
                        new(Packages.DedicatedServerPackageId, RecommendationType.NotRecommended, Reasons.DedicatedServerPackageNotRecommendedN4E),
                    },
                    IncompatibleSolutions = new IncompatibleSolution[]{new(PossibleSolution.DA, Reasons.DistributedAuthorityIncompatible)}
                }
            };
            data.Packages = new PackageDetails[]
            {
                new(Packages.VivoxId, Titles.Vivox, CatchPhrases.Vivox, DocLinks.Vivox),
                new(Packages.MultiplayerSdkId, Titles.MultiplayerSdk, CatchPhrases.MultiplayerSdk, DocLinks.MultiplayerSdk),
                new(Packages.MultiplayerWidgetsId, Titles.MultiplayerWidgets, CatchPhrases.MultiplayerWidgets, DocLinks.MultiplayerWidgets),
                new(Packages.NetcodeForEntitiesId, Titles.NetcodeForEntities, CatchPhrases.NetcodeForEntities, DocLinks.NetcodeForEntities),
                new(Packages.NetcodeGameObjectsId, Titles.NetcodeForGameObjects, CatchPhrases.NetcodeForGameObjects, DocLinks.NetcodeForGameObjects),
                new(Packages.MultiplayerToolsId, Titles.MultiplayerTools, CatchPhrases.MultiplayerTools, DocLinks.MultiplayerTools),
                new(Packages.MultiplayerPlayModeId, Titles.MultiplayerPlayMode, CatchPhrases.MultiplayerPlayMode, DocLinks.MultiplayerPlayMode),
                new(Packages.CloudCodeId, Titles.CloudCode, CatchPhrases.CloudCode, DocLinks.CloudCode),
                new(Packages.TransportId, Titles.Transport, CatchPhrases.Transport, DocLinks.Transport),
                new(Packages.DedicatedServerPackageId, Titles.DedicatedServerPackage, CatchPhrases.DedicatedServerPackage, DocLinks.DedicatedServerPackage),
                new(Packages.EntitiesGraphics, Titles.EntitiesGraphics, CatchPhrases.EntitiesGraphics, DocLinks.EntitiesGraphics),
                new(Packages.DeploymentPackageId, Titles.DeploymentPackage, CatchPhrases.DeploymentPackage, DocLinks.DeploymentPackage)
            };

            return data;
        }    
    }
}
#endif