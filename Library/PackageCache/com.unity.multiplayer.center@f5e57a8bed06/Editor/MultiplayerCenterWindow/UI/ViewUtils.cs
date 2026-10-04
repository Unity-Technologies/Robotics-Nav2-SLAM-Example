using UnityEngine.UIElements;

namespace Unity.Multiplayer.Center.Window.UI
{
    internal static class ViewUtils
    {
        public static void MoveToggleLeft(BaseBoolField button)
        {
            var label = button.Q<Label>();
            button.Insert(button.childCount - 1, label);
            // equivalent to button.Children.First()
            using var iterator = button.Children().GetEnumerator();
            iterator.MoveNext();
            var first = iterator.Current;
            first.style.flexGrow = 0;
        }
    }
}
