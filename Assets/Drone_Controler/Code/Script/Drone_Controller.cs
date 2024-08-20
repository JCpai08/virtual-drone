using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Unity.VisualScripting;

namespace RSplus
{
    [RequireComponent(typeof(Drone_Inputs))]
    public class Drone_Controller : Drone_Base_Rigidbody
    {
        #region Variables
        [SerializeField] private float minMaxPitch = 10f;
        [SerializeField] private float minMaxroll = -10f;
        [SerializeField] private float yawPower = 4f;
        [SerializeField] private float lerpSpeed = 2f;
        [SerializeField] private float maxSpeed = 2f;
        [SerializeField] private float maxTorquePower = 4f;

        private Drone_Inputs input;

        private List<IEngine> engines = new List<IEngine>();

        private float finalPitch;
        private float finalRoll;
        private float finalYaw;

        // PID 控制参数0
        [SerializeField] public float Kp = 0.5f;  // 比例增益
        [SerializeField] public float Ki = 0.1f;  // 积分增益
        [SerializeField] public float Kd = 0.2f;  // 微分增益
        // 控制参数X0
        public float previousError = 0;
        public float integral = 0;
        // PID 控制参数1
        [SerializeField] public float Kp1 = 1f;  // 比例增益
        [SerializeField] public float Ki1 = 0.1f;  // 积分增益
        [SerializeField] public float Kd1 = 0.2f;  // 微分增益
        // 控制参数X1
        public float previousError1 = 0;
        public float integral1 = 0;
        // 控制参数Z0
        public float previousErrorZ = 0;
        public float integralZ = 0;
        // 控制参数Z1
        public float previousErrorZ1 = 0;
        public float integralZ1 = 0;

        #endregion

        #region Main Mathods
        // Start is called before the first frame update
        void Start()
        {
            input = GetComponent<Drone_Inputs>();
            engines = GetComponentsInChildren<IEngine>().ToList<IEngine>();
        }
        #endregion

        #region Custom Methods
        protected override void HandlePhysics()
        {
            HandleEngines();
            HandleControls();
        }
        protected virtual void HandleEngines()
        {
            foreach (IEngine engine in engines)
            {
                engine.UpdateEngine(rb, input);
            }
        }
        protected virtual void HandleControls()
        {
            
            // 目标x轴速度
            float targetRollVelocity = input.Cyclic.x * maxSpeed*(-1);

            // 当前速度
            float currentVelocity = transform.InverseTransformDirection(rb.velocity).x * (-1);

            // 计算误差
            float error = targetRollVelocity - currentVelocity;
            //Debug.Log("Verror:" + error);

            // 计算积分项
            integral += error * Time.fixedDeltaTime;

            // 计算微分项
            float derivative = (error - previousError) / Time.fixedDeltaTime;

            // 计算输出（加速度）
            float output = Kp * error + Ki * integral + Kd * derivative;

            // 限制输出范围（可根据需要进行调整）
            output = Mathf.Clamp(output, -1f, 1f);

            //-------------------------------------------------------------------

            // 目标滚转角度
            float targetRollAngle = output / 10 * 100;

            // 当前滚转角度
            float currentRollAngle = transform.eulerAngles.z;
            if (currentRollAngle > 180) { currentRollAngle = currentRollAngle - 360; }
            //Debug.Log("currentRollAngle:" + currentRollAngle);
            // 计算误差
            float error1 = targetRollAngle - currentRollAngle;

            //Debug.Log(error1);
            // 计算积分项
            integral1 += error1 * Time.fixedDeltaTime;

            // 计算微分项
            float derivative1 = (error1 - previousError1) / Time.fixedDeltaTime;

            // 计算输出
            float output1 = Kp1 * error1 + Ki1 * integral1 + Kd1 * derivative1;

            // 限制输出范围（可根据需要进行调整）
            output1 = Mathf.Clamp(output1, maxTorquePower*-1, maxTorquePower);

            // 施加滚转力
            rb.AddTorque(transform.forward * output1, ForceMode.Acceleration);

            // 更新上一次的误差
            previousError1 = error1;
            
            //-----------------------------------------------------------------------------------------------++++++
            
            // 目标z轴速度
            float targetRollVelocityZ = input.Cyclic.y * maxSpeed ;

            // 当前速度
            float currentVelocityZ = transform.InverseTransformDirection(rb.velocity).z;

            // 计算误差
            float errorZ = targetRollVelocityZ - currentVelocityZ;
            //Debug.Log("Verror:" + Zerror);

            // 计算积分项
            integralZ += errorZ * Time.fixedDeltaTime;

            // 计算微分项
            float derivativeZ = (errorZ - previousErrorZ) / Time.fixedDeltaTime;

            // 计算输出（加速度）
            float outputZ = Kp * errorZ + Ki * integralZ + Kd * derivativeZ;

            // 限制输出范围（可根据需要进行调整）
            outputZ = Mathf.Clamp(outputZ, -1f, 1f);

            //-------------------------------------------------------------------

            // 目标滚转角度
            float targetRollAngleZ = outputZ / 10 * 100;

            // 当前滚转角度
            float currentRollAngleZ = transform.eulerAngles.x;
            if (currentRollAngleZ > 180) { currentRollAngleZ = currentRollAngleZ - 360; }
            //Debug.Log("currentRollAngleZ:" + currentRollAngleZ);
            // 计算误差
            float errorZ1 = targetRollAngleZ - currentRollAngleZ;

            //Debug.Log(error1);
            // 计算积分项
            integralZ1 += errorZ1 * Time.fixedDeltaTime;

            // 计算微分项
            float derivativeZ1 = (errorZ1 - previousErrorZ1) / Time.fixedDeltaTime;

            // 计算输出
            float outputZ1 = Kp1 * errorZ1 + Ki1 * integralZ1 + Kd1 * derivativeZ1;

            // 限制输出范围（可根据需要进行调整）
            outputZ1 = Mathf.Clamp(outputZ1, maxTorquePower * -1, maxTorquePower);

            // 施加滚转力
            rb.AddTorque(transform. right * outputZ1, ForceMode.Acceleration);

            // 更新上一次的误差
            previousErrorZ1 = errorZ1;
            
            //---------------------------------------------------------------------------------+++++++++++++++++++++++++++++++++++++
            float yawInput = input.Padels; // 获取水平轴向输入，用于控制偏航
            float targetAngularVelocity;
            //Debug.Log("yawinput:" + yawInput);
            if (Mathf.Approximately(yawInput, 0f))
            {
                // 停止旋转
                //rb.angularVelocity = Vector3.zero;
                targetAngularVelocity = 0f;
            }

            else
            {
                // 计算目标角速度的旋转轴
                targetAngularVelocity = yawInput * yawPower;
            }
            // 设置刚体的角速度
            rb.angularVelocity = transform.up * Mathf.Deg2Rad * targetAngularVelocity;
            /*
            float pitch = input.Cyclic.y * minMaxPitch;
            float roll = input.Cyclic.x * minMaxroll;
            float yaw = input.Padels * yawPower;

            finalPitch = Mathf.Lerp(pitch, finalPitch, Time.deltaTime * lerpSpeed);
            finalRoll = Mathf.Lerp(roll, finalRoll, Time.deltaTime * lerpSpeed);
            finalYaw = Mathf.Lerp(yaw, finalYaw, Time.deltaTime * lerpSpeed);
            Quaternion rot = Quaternion.Euler(finalPitch, finalYaw, finalRoll);
            rb.MoveRotation(rot);
            */
        }
        #endregion
    }
}
