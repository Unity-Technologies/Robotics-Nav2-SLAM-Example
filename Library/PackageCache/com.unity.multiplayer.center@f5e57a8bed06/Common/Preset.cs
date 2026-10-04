using System;
using UnityEngine;

namespace Unity.Multiplayer.Center.Common
{
    /// <summary>
    /// Game genres that can be selected. Each one is associated with pre-selected answers for the questionnaire,
    /// except for `None`.
    /// </summary>
    [Serializable]
    [InspectorOrder()] // will sort the values alphabetically in the inspector
    public enum Preset
    {
        /// <summary>
        /// Start from scratch, no preset.
        /// </summary>
        [InspectorName("-")]
        None,
        
        /// <summary>
        /// Adventure genre.
        /// </summary>
        [InspectorName("Adventure")]
        Adventure,
        
        /// <summary>
        /// Shooter, Battle Royale, Battle Arena genre.
        /// </summary>
        [InspectorName("Shooter, Battle Royale, Battle Arena")]
        Shooter,
        
        /// <summary>
        /// Racing genre.
        /// </summary>
        [InspectorName("Racing")]
        Racing,
        
        /// <summary>
        /// Card Battle, Turn-based, Tabletop genre.
        /// </summary>
        [InspectorName("Card Battle, Turn-based, Tabletop")]
        TurnBased,
        
        /// <summary>
        /// Simulation genre.
        /// </summary>
        [InspectorName("Simulation")]
        Simulation,
        
        /// <summary>
        /// Strategy genre.
        /// </summary>
        [InspectorName("Strategy")]
        Strategy,
        
        /// <summary>
        /// Sports genre.
        /// </summary>
        [InspectorName("Sports")]
        Sports,

        /// <summary>
        /// Role-Playing, MMO genre.
        /// </summary>
        [InspectorName("Role-Playing, MMO")]
        RolePlaying,
        
        /// <summary>
        /// Async, Idle, Hyper Casual, Puzzle genre.
        /// </summary>
        [InspectorName("Async, Idle, Hyper Casual, Puzzle")]
        Async,
        
        /// <summary>
        /// Fighting genre.
        /// </summary>
        [InspectorName("Fighting")]
        Fighting,
        
        /// <summary>
        /// Arcade, Platformer, Sandbox genre.
        /// </summary>
        [InspectorName("Arcade, Platformer, Sandbox")]
        Sandbox
    }
}
