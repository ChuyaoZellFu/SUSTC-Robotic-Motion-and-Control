;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::InterconnectCyclic_Command_tool_command)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'InterconnectCyclic_Command_tool_command (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::INTERCONNECTCYCLIC_COMMAND_TOOL_COMMAND")
  (make-package "KORTEX_DRIVER::INTERCONNECTCYCLIC_COMMAND_TOOL_COMMAND"))

(in-package "ROS")
;;//! \htmlinclude InterconnectCyclic_Command_tool_command.msg.html


(defclass kortex_driver::InterconnectCyclic_Command_tool_command
  :super ros::object
  :slots (_gripper_command ))

(defmethod kortex_driver::InterconnectCyclic_Command_tool_command
  (:init
   (&key
    ((:gripper_command __gripper_command) ())
    )
   (send-super :init)
   (setq _gripper_command __gripper_command)
   self)
  (:gripper_command
   (&rest __gripper_command)
   (if (keywordp (car __gripper_command))
       (send* _gripper_command __gripper_command)
     (progn
       (if __gripper_command (setq _gripper_command (car __gripper_command)))
       _gripper_command)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/GripperCyclic_Command[] _gripper_command
    (apply #'+ (send-all _gripper_command :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/GripperCyclic_Command[] _gripper_command
     (write-long (length _gripper_command) s)
     (dolist (elem _gripper_command)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/GripperCyclic_Command[] _gripper_command
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _gripper_command (let (r) (dotimes (i n) (push (instance kortex_driver::GripperCyclic_Command :init) r)) r))
     (dolist (elem- _gripper_command)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::InterconnectCyclic_Command_tool_command :md5sum-) "cb4b9ede5a008a07d439d113030c34d0")
(setf (get kortex_driver::InterconnectCyclic_Command_tool_command :datatype-) "kortex_driver/InterconnectCyclic_Command_tool_command")
(setf (get kortex_driver::InterconnectCyclic_Command_tool_command :definition-)
      "
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



(provide :kortex_driver/InterconnectCyclic_Command_tool_command "cb4b9ede5a008a07d439d113030c34d0")


