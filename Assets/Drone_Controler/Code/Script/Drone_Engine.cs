using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RSplus
{
    [RequireComponent(typeof(BoxCollider))]
    public class Drone_Engine : MonoBehaviour,IEngine
    {
        #region Variables
        [Header("Engine Properties")]
        [SerializeField] private float maxPower = 4f;
        // PID ���Ʋ���0
        [SerializeField] public float Kp = 0.5f;  // ��������
        [SerializeField] public float Ki = 0.1f;  // ��������
        [SerializeField] public float Kd = 0.2f;  // ΢������
        // ���Ʋ���0
        public float previousError = 0;
        public float integral = 0;
        #endregion

        #region Interface Methods
        public void InitEngine()
        {
            throw new System.NotImplementedException();
        }

        public void UpdateEngine(Rigidbody rb,Drone_Inputs input)
        {
            //Debug.Log("running engine: " + gameObject.name);
            
            // Ŀ��x���ٶ�
            float targetRollVelocity = input.Throttle * maxPower;

            // ��ǰ�ٶ�
            float currentVelocity = rb.velocity.y;//�������Ҫ�ĳ���������ϵ�µ��ٶ�

            // �������
            float error = targetRollVelocity - currentVelocity;

            // ���������
            integral += error * Time.fixedDeltaTime;

            // ����΢����
            float derivative = (error - previousError) / Time.fixedDeltaTime;

            // ������������ٶȣ�
            float output = Kp * error + Ki * integral + Kd * derivative;

            // ���������Χ���ɸ�����Ҫ���е�����
            output = Mathf.Clamp(output, -3f, 3f);
            
            Vector3 engineForce;
            engineForce = transform.up * (((rb.mass * Physics.gravity.magnitude) + output) / 4f);
            rb.AddForce(engineForce, ForceMode.Force);
        }
        #endregion
    }
}
