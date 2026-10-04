using System;

namespace Unity.Multiplayer.Center.Common.Analytics
{
    /// <summary>
    /// The type of interaction that the user has with a button in the getting started tab.
    /// </summary>
    public enum InteractionDataType
    {
        /// <summary>
        /// For a button that does something in the editor, e.g. a button that opens a window or imports a sample. 
        /// </summary>
        CallToAction = 0,
        
        /// <summary>
        /// For a button that opens a URL in the browser (e.g. a documentation link).
        /// </summary>
        Link = 1,
    }
    
    /// <summary>
    /// For the object that provides the analytics functionality to send interaction events on some Onboarding section
    /// in the getting started tab.
    /// </summary>
    public interface IOnboardingSectionAnalyticsProvider
    {
        /// <summary>
        /// Send event for a button interaction in the getting started tab.
        /// </summary>
        /// <param name="type"> Whether it is a call to action or a link</param>
        /// <param name="displayName"> The name of the button in the UI</param>
        void SendInteractionEvent(InteractionDataType type, string displayName);
    }
}
