using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
namespace RSplus
{
    [RequireComponent(typeof(PlayerInput))]
    public class Drone_Inputs : MonoBehaviour
    {
        #region variable
        private Vector2 cyclic;
        private float padels;
        private float throttle;
        public Vector2 Cyclic { get => cyclic; }
        public float Padels { get => padels; }
        public float Throttle { get => throttle;}
        #endregion
        #region main method
        private void OnCyclic(InputValue value)
        {
            cyclic = value.Get<Vector2>();
        }
        private void OnPedals(InputValue value)
        {
            padels = value.Get<float>();
        }
        private void OnThrottle(InputValue value)
        {
            throttle = value.Get<float>();
        }
        #endregion
    }
}