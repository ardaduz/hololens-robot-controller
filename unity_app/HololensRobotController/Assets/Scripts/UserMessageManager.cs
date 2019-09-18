using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class UserMessageManager : MonoBehaviour
{
    public Canvas messageCanvas;
    public Text messageText;
    public Button MessageCloseButton;

    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
    }

    public void ShowUserMessage(string message, float delay)
    {
        StartCoroutine(MessageCoroutine(message, delay));
    }

    private IEnumerator MessageCoroutine(string message, float delay)
    {
        messageText.text = message;
        messageCanvas.enabled = true;
        yield return new WaitForSeconds(delay);
        messageCanvas.enabled = false;
    }

    public void OnCloseMessageButtonClick()
    {
        messageCanvas.enabled = false;
    }
}
