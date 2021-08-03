using System;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

public class RaycastPublisher : MonoBehaviour
{
    [SerializeField]
    protected string m_Topic;
    protected Button m_Button;

    protected ROSConnection m_Ros;
    protected ClickState m_State = ClickState.NONE;

    // Start is called before the first frame update
    public virtual void Start()
    {
        m_Ros = ROSConnection.GetOrCreateInstance();
    }

    protected bool ValidClick()
    {
        return m_State != ClickState.NONE && !EventSystem.current.IsPointerOverGameObject();
    }

    protected (bool, RaycastHit) RaycastCheck(ClickState state)
    {
        var isHit = false;
        if (Physics.Raycast(Camera.main.ScreenPointToRay(Input.mousePosition), out var hit) && m_State == state)
        {
            if (hit.collider.gameObject.name.Contains("Floor"))
            {
                isHit = true;
            }
        }

        return (isHit, hit);
    }
    
    public void BeginInteractionButton()
    {
        m_Button.interactable = false;
        m_State = ClickState.BEGIN;
    }

    protected enum ClickState
    {
        NONE,
        BEGIN,
        UPDATE
    }
}
