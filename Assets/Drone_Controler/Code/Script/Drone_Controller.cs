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

        // PID ���Ʋ���0
        [SerializeField] public float Kp = 0.5f;  // ��������
        [SerializeField] public float Ki = 0.1f;  // ��������
        [SerializeField] public float Kd = 0.2f;  // ΢������
        // ���Ʋ���X0
        public float previousError = 0;
        public float integral = 0;
        // PID ���Ʋ���1
        [SerializeField] public float Kp1 = 1f;  // ��������
        [SerializeField] public float Ki1 = 0.1f;  // ��������
        [SerializeField] public float Kd1 = 0.2f;  // ΢������
        // ���Ʋ���X1
        public float previousError1 = 0;
        public float integral1 = 0;
        // ���Ʋ���Z0
        public float previousErrorZ = 0;
        public float integralZ = 0;
        // ���Ʋ���Z1
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
            
            // Ŀ��x���ٶ�
            float targetRollVelocity = input.Cyclic.x * maxSpeed*(-1);

            // ��ǰ�ٶ�
            float currentVelocity = transform.InverseTransformDirection(rb.velocity).x * (-1);

            // �������
            float error = targetRollVelocity - currentVelocity;
            //Debug.Log("Verror:" + error);

            // ���������
            integral += error * Time.fixedDeltaTime;

            // ����΢����
            float derivative = (error - previousError) / Time.fixedDeltaTime;

            // ������������ٶȣ�
            float output = Kp * error + Ki * integral + Kd * derivative;

            // ���������Χ���ɸ�����Ҫ���е�����
            output = Mathf.Clamp(output, -1f, 1f);

            //-------------------------------------------------------------------

            // Ŀ���ת�Ƕ�
            float targetRollAngle = output / 10 * 100;

            // ��ǰ��ת�Ƕ�
            float currentRollAngle = transform.eulerAngles.z;
            if (currentRollAngle > 180) { currentRollAngle = currentRollAngle - 360; }
            //Debug.Log("currentRollAngle:" + currentRollAngle);
            // �������
            float error1 = targetRollAngle - currentRollAngle;

            //Debug.Log(error1);
            // ���������
            integral1 += error1 * Time.fixedDeltaTime;

            // ����΢����
            float derivative1 = (error1 - previousError1) / Time.fixedDeltaTime;

            // �������
            float output1 = Kp1 * error1 + Ki1 * integral1 + Kd1 * derivative1;

            // ���������Χ���ɸ�����Ҫ���е�����
            output1 = Mathf.Clamp(output1, maxTorquePower*-1, maxTorquePower);

            // ʩ�ӹ�ת��
            rb.AddTorque(transform.forward * output1, ForceMode.Acceleration);

            // ������һ�ε����
            previousError1 = error1;
            
            //-----------------------------------------------------------------------------------------------++++++
            
            // Ŀ��z���ٶ�
            float targetRollVelocityZ = input.Cyclic.y * maxSpeed ;

            // ��ǰ�ٶ�
            float currentVelocityZ = transform.InverseTransformDirection(rb.velocity).z;

            // �������
            float errorZ = targetRollVelocityZ - currentVelocityZ;
            //Debug.Log("Verror:" + Zerror);

            // ���������
            integralZ += errorZ * Time.fixedDeltaTime;

            // ����΢����
            float derivativeZ = (errorZ - previousErrorZ) / Time.fixedDeltaTime;

            // ������������ٶȣ�
            float outputZ = Kp * errorZ + Ki * integralZ + Kd * derivativeZ;

            // ���������Χ���ɸ�����Ҫ���е�����
            outputZ = Mathf.Clamp(outputZ, -1f, 1f);

            //-------------------------------------------------------------------

            // Ŀ���ת�Ƕ�
            float targetRollAngleZ = outputZ / 10 * 100;

            // ��ǰ��ת�Ƕ�
            float currentRollAngleZ = transform.eulerAngles.x;
            if (currentRollAngleZ > 180) { currentRollAngleZ = currentRollAngleZ - 360; }
            //Debug.Log("currentRollAngleZ:" + currentRollAngleZ);
            // �������
            float errorZ1 = targetRollAngleZ - currentRollAngleZ;

            //Debug.Log(error1);
            // ���������
            integralZ1 += errorZ1 * Time.fixedDeltaTime;

            // ����΢����
            float derivativeZ1 = (errorZ1 - previousErrorZ1) / Time.fixedDeltaTime;

            // �������
            float outputZ1 = Kp1 * errorZ1 + Ki1 * integralZ1 + Kd1 * derivativeZ1;

            // ���������Χ���ɸ�����Ҫ���е�����
            outputZ1 = Mathf.Clamp(outputZ1, maxTorquePower * -1, maxTorquePower);

            // ʩ�ӹ�ת��
            rb.AddTorque(transform. right * outputZ1, ForceMode.Acceleration);

            // ������һ�ε����
            previousErrorZ1 = errorZ1;
            
            //---------------------------------------------------------------------------------+++++++++++++++++++++++++++++++++++++
            float yawInput = input.Padels; // ��ȡˮƽ�������룬���ڿ���ƫ��
            float targetAngularVelocity;
            //Debug.Log("yawinput:" + yawInput);
            if (Mathf.Approximately(yawInput, 0f))
            {
                // ֹͣ��ת
                //rb.angularVelocity = Vector3.zero;
                targetAngularVelocity = 0f;
            }

            else
            {
                // ����Ŀ����ٶȵ���ת��
                targetAngularVelocity = yawInput * yawPower;
            }
            // ���ø���Ľ��ٶ�
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
