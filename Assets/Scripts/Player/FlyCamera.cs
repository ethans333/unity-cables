using UnityEngine;
using UnityEngine.InputSystem;

public class FlyCamera : MonoBehaviour
{
    [SerializeField] float moveSpeed = 10f;
    [SerializeField] float lookSpeed = 0.15f;
    [SerializeField] float fastMultiplier = 4f;

    void Update()
    {
        if (Mouse.current.rightButton.isPressed)
        {
            Cursor.lockState = CursorLockMode.Locked;
            Cursor.visible = false;

            Vector2 mouseDelta = Mouse.current.delta.ReadValue();

            transform.Rotate(
                Vector3.up,
                mouseDelta.x * lookSpeed,
                Space.World);

            transform.Rotate(
                Vector3.left,
                mouseDelta.y * lookSpeed,
                Space.Self);

            Vector3 movement = Vector3.zero;

            if (Keyboard.current.wKey.isPressed) movement += transform.forward;
            if (Keyboard.current.sKey.isPressed) movement -= transform.forward;
            if (Keyboard.current.dKey.isPressed) movement += transform.right;
            if (Keyboard.current.aKey.isPressed) movement -= transform.right;
            if (Keyboard.current.eKey.isPressed) movement += Vector3.up;
            if (Keyboard.current.qKey.isPressed) movement -= Vector3.up;

            float speed = moveSpeed;

            if (Keyboard.current.leftShiftKey.isPressed)
                speed *= fastMultiplier;

            if (movement != Vector3.zero)
                transform.position += movement.normalized * speed * Time.deltaTime;
        }
        else
        {
            Cursor.lockState = CursorLockMode.None;
            Cursor.visible = true;
        }
    }
}