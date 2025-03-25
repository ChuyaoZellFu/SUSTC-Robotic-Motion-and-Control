;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::BaseCyclic_Command)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'BaseCyclic_Command (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BASECYCLIC_COMMAND")
  (make-package "KORTEX_DRIVER::BASECYCLIC_COMMAND"))

(in-package "ROS")
;;//! \htmlinclude BaseCyclic_Command.msg.html


(defclass kortex_driver::BaseCyclic_Command
  :super ros::object
  :slots (_frame_id _actuators _interconnect ))

(defmethod kortex_driver::BaseCyclic_Command
  (:init
   (&key
    ((:frame_id __frame_id) 0)
    ((:actuators __actuators) ())
    ((:interconnect __interconnect) (instance kortex_driver::InterconnectCyclic_Command :init))
    )
   (send-super :init)
   (setq _frame_id (round __frame_id))
   (setq _actuators __actuators)
   (setq _interconnect __interconnect)
   self)
  (:frame_id
   (&optional __frame_id)
   (if __frame_id (setq _frame_id __frame_id)) _frame_id)
  (:actuators
   (&rest __actuators)
   (if (keywordp (car __actuators))
       (send* _actuators __actuators)
     (progn
       (if __actuators (setq _actuators (car __actuators)))
       _actuators)))
  (:interconnect
   (&rest __interconnect)
   (if (keywordp (car __interconnect))
       (send* _interconnect __interconnect)
     (progn
       (if __interconnect (setq _interconnect (car __interconnect)))
       _interconnect)))
  (:serialization-length
   ()
   (+
    ;; uint32 _frame_id
    4
    ;; kortex_driver/ActuatorCommand[] _actuators
    (apply #'+ (send-all _actuators :serialization-length)) 4
    ;; kortex_driver/InterconnectCyclic_Command _interconnect
    (send _interconnect :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _frame_id
       (write-long _frame_id s)
     ;; kortex_driver/ActuatorCommand[] _actuators
     (write-long (length _actuators) s)
     (dolist (elem _actuators)
       (send elem :serialize s)
       )
     ;; kortex_driver/InterconnectCyclic_Command _interconnect
       (send _interconnect :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _frame_id
     (setq _frame_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/ActuatorCommand[] _actuators
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _actuators (let (r) (dotimes (i n) (push (instance kortex_driver::ActuatorCommand :init) r)) r))
     (dolist (elem- _actuators)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; kortex_driver/InterconnectCyclic_Command _interconnect
     (send _interconnect :deserialize buf ptr-) (incf ptr- (send _interconnect :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::BaseCyclic_Command :md5sum-) "79e9d628025fc54837a4192c98fc5b9f")
(setf (get kortex_driver::BaseCyclic_Command :datatype-) "kortex_driver/BaseCyclic_Command")
(setf (get kortex_driver::BaseCyclic_Command :definition-)
      "
uint32 frame_id
ActuatorCommand[] actuators
InterconnectCyclic_Command interconnect
================================================================================
MSG: kortex_driver/ActuatorCommand

uint32 command_id
uint32 flags
float32 position
float32 velocity
float32 torque_joint
float32 current_motor
================================================================================
MSG: kortex_driver/InterconnectCyclic_Command

InterconnectCyclic_MessageId command_id
uint32 flags
InterconnectCyclic_Command_tool_command oneof_tool_command
================================================================================
MSG: kortex_driver/InterconnectCyclic_MessageId

uint32 identifier
================================================================================
MSG: kortex_driver/InterconnectCyclic_Command_tool_command

GripperCyclic_Command[] gripper_command
================================================================================
MSG: kortex_driver/GripperCyclic_Command

GripperCyclic_MessageId command_id
uint32 flags
MotorCommand[] motor_cmd
================================================================================
MSG: kortex_driver/GripperCyclic_MessageId

uint32 identifier
================================================================================
MSG: kortex_driver/MotorCommand

uint32 motor_id
float32 position
float32 velocity
float32 force
")



(provide :kortex_driver/BaseCyclic_Command "79e9d628025fc54837a4192c98fc5b9f")


