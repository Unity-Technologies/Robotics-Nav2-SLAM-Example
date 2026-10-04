namespace Unity.Multiplayer.Center.Window.UI
{
    /// <summary>
    /// Style common to the multiplayer center, excepted for the getting started tab (which needs shared style)
    /// </summary>
    internal static class StyleClasses
    {
        /// <summary> Game spec standard question </summary>
        public const string QuestionView = "question-view";
        
        /// <summary> Game spec mandatory question, highlighted </summary>
        public const string MandatoryQuestion = "mandatory-question";
        
        /// <summary> Description of game spec section </summary>
        public const string QuestionText = "question-text";
        
        /// <summary> Part of game spec questionnaire that contains several questions </summary>
        public const string QuestionSection = "question-section";
        
        /// <summary> Part of game spec questionnaire that contains several questions </summary>
        public const string QuestionSectionNoScrollbar = "question-section__no-scrollbar";
        
        /// <summary> Thicker button to go to a next step; e.g. Install packages </summary>
        public const string NextStepButton = "next-step-button";

        /// <summary> The splitview that is used in Recommendation and Getting Started Tab </summary>
        public const string MainSplitView = "main-split-view";

        /// <summary> The right container of the main split view </summary>
        public const string MainSplitViewRight = "main-split-view-right";
        
        /// <summary> The left container of the main split view </summary>
        public const string MainSplitViewLeft = "main-split-view-left";
        
        /// <summary> The style for the Headline that is used in the main views </summary>
        public const string ViewHeadline = "view-headline";
       
    }
}
