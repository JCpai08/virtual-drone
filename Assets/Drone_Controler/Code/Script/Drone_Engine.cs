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
        // PID 控制参数0
        [SerializeField] public float Kp = 0.5f;  // 比例增益
        [SerializeField] public float Ki = 0.1f;  // 积分增益
        [SerializeField] public float Kd = 0.2f;  // 微分增益
        // 控制参数0
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
            
            // 目标x轴速度
            float targetRollVelocity = input.Throttle * maxPower;

            // 当前速度
            float currentVelocity = rb.velocity.y;//这里可能要改成世界坐标系下的速度

            // 计算误差
            float error = targetRollVelocity - currentVelocity;

            // 计算积分项
            integral += error * Time.fixedDeltaTime;

            // 计算微分项
            float derivative = (error - previousError) / Time.fixedDeltaTime;

            // 计算输出（加速度）
            float output = Kp * error + Ki * integral + Kd * derivative;

            // 限制输出范围（可根据需要进行调整）
            output = Mathf.Clamp(output, -3f, 3f);
            
            Vector3 engineForce;
            engineForce = transform.up * (((rb.mass * Physics.gravity.magnitude) + output) / 4f);
            rb.AddForce(engineForce, ForceMode.Force);
        }
        #endregion
    }
}
