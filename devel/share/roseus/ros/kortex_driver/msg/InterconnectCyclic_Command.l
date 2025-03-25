;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::InterconnectCyclic_Command)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'InterconnectCyclic_Command (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::INTERCONNECTCYCLIC_COMMAND")
  (make-package "KORTEX_DRIVER::INTERCONNECTCYCLIC_COMMAND"))

(in-package "ROS")
;;//! \htmlinclude InterconnectCyclic_Command.msg.html


(defclass kortex_driver::InterconnectCyclic_Command
  :super ros::object
  :slots (_command_id _flags _oneof_tool_command ))

(defmethod kortex_driver::InterconnectCyclic_Command
  (:init
   (&key
    ((:command_id __command_id) (instance kortex_driver::InterconnectCyclic_MessageId :init))
    ((:flags __flags) 0)
    ((:oneof_tool_command __oneof_tool_command) (instance kortex_driver::InterconnectCyclic_Command_tool_command :init))
    )
   (send-super :init)
   (setq _command_id __command_id)
   (setq _flags (round __flags))
   (setq _oneof_tool_command __oneof_tool_command)
   self)
  (:command_id
   (&rest __command_id)
   (if (keywordp (car __command_id))
       (send* _command_id __command_id)
     (progn
       (if __command_id (setq _command_id (car __command_id)))
       _command_id)))
  (:flags
   (&optional __flags)
   (if __flags (setq _flags __flags)) _flags)
  (:oneof_tool_command
   (&rest __oneof_tool_command)
   (if (keywordp (car __oneof_tool_command))
       (send* _oneof_tool_command __oneof_tool_command)
     (progn
       (if __oneof_tool_command (setq _oneof_tool_command (car __oneof_tool_command)))
       _oneof_tool_command)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/InterconnectCyclic_MessageId _command_id
    (send _command_id :serialization-length)
    ;; uint32 _flags
    4
    ;; kortex_driver/InterconnectCyclic_Command_tool_command _oneof_tool_command
    (send _oneof_tool_command :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/InterconnectCyclic_MessageId _command_id
       (send _command_id :serialize s)
     ;; uint32 _flags
       (write-long _flags s)
     ;; kortex_driver/InterconnectCyclic_Command_tool_command _oneof_tool_command
       (send _oneof_tool_command :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/InterconnectCyclic_MessageId _command_id
     (send _command_id :deserialize buf ptr-) (incf ptr- (send _command_id :serialization-length))
   ;; uint32 _flags
     (setq _flags (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/InterconnectCyclic_Command_tool_command _oneof_tool_command
     (send _oneof_tool_command :deserialize buf ptr-) (incf ptr- (send _oneof_tool_command :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::InterconnectCyclic_Command :md5sum-) "4def369ad2d800b446113efeb710b701")
(setf (get kortex_driver::InterconnectCyclic_Command :datatype-) "kortex_driver/InterconnectCyclic_Command")
(setf (get kortex_driver::InterconnectCyclic_Command :definition-)
      "
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



(provide :kortex_driver/InterconnectCyclic_Command "4def369ad2d800b446113efeb710b701")


