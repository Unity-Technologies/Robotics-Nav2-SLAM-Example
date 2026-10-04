using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

namespace Unity.Multiplayer.Center.Recommendations
{
    struct ScoreWithReason
    {
        public float Score;
        public string Reason;

        public ScoreWithReason(float score, string reason)
        {
            Score = score;
            Reason = reason;
        }
    }

    /// <summary>
    /// Aggregates scores for a given solution and fetches the reasons for the highest score
    /// </summary>
    internal class Scoring
    {
        public const string DynamicKeyword = "[dynamic]";
        
        string m_PrimaryDescription = null;
        
        List<ScoreWithReason> m_AllScores = new();
        public float TotalScore { get; private set; } = 0f;
        
        public Scoring(string primaryDescription)
        {
            m_PrimaryDescription = primaryDescription;
        }

        public void AddScore(float score, string reason)
        {
            TotalScore += score;
            if(m_AllScores.Count == 0)
            {
                m_AllScores.Add(new ScoreWithReason(score, reason));
                return;
            }

            int insertIndex = 0;
            for (int i = 0; i < m_AllScores.Count; i++)
            {
                if (score > m_AllScores[i].Score)
                {
                    insertIndex = i;
                    break;
                }
            }
            
            m_AllScores.Insert(insertIndex, new ScoreWithReason(score, reason));
        }

        /// <summary>
        /// Gets the reason for increased scores
        /// </summary>
        /// <returns>The explanatory string</returns>
        public string GetReasonString() => GetAllContributionsReasons();
        
        string GetAllContributionsReasons()
        {
            return string.IsNullOrEmpty(m_PrimaryDescription)? OneReasonPerLine() : CombineReasonsInOneSentence();
        }
        
        string OneReasonPerLine()
        {
            var stringBuilder = new System.Text.StringBuilder();
            foreach (var score in m_AllScores)
            {
                stringBuilder.AppendLine(score.Reason);
            }
            return stringBuilder.ToString();
        }

        string CombineReasonsInOneSentence()
        {
            var length = m_AllScores.Count;
            if(length == 0)
                return RemoveSentenceWithDynamicKeyword(m_PrimaryDescription);

            var explanation = length switch
            {
                1 => m_AllScores[0].Reason,
                2 => $"{m_AllScores[0].Reason} and {m_AllScores[1].Reason}",
                _ => Combine(m_AllScores)
            };

            return m_PrimaryDescription.Replace(DynamicKeyword, explanation);
        }
        
        static string RemoveSentenceWithDynamicKeyword(string primaryDescription)
        {
            var sentences = primaryDescription.Split('.');
            var sentencesWithoutKeyword = new StringBuilder(capacity:sentences.Length);
            var index = 0;
            foreach (var sentence in sentences)
            {
                if (!string.IsNullOrEmpty(sentence) && !sentence.Contains(DynamicKeyword))
                {
                    sentencesWithoutKeyword.Append(sentence);
                    sentencesWithoutKeyword.Append(".");
                    index++;
                }
            }

            return sentencesWithoutKeyword.ToString();
        }

        static string Combine(List<ScoreWithReason> scores)
        {
            var stringBuilder = new System.Text.StringBuilder();
            for(var i = 0; i < scores.Count -2; i++)
            {
                stringBuilder.Append(scores[i].Reason);
                stringBuilder.Append(", ");
            }

            stringBuilder.Append(scores[^2].Reason);
            stringBuilder.Append(" and ");
            stringBuilder.Append(scores[^1].Reason);
            
            return stringBuilder.ToString();
        }
    }
}
