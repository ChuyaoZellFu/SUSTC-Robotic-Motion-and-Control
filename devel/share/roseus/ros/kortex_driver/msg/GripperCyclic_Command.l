;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GripperCyclic_Command)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GripperCyclic_Command (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GRIPPERCYCLIC_COMMAND")
  (make-package "KORTEX_DRIVER::GRIPPERCYCLIC_COMMAND"))

(in-package "ROS")
;;//! \htmlinclude GripperCyclic_Command.msg.html


(defclass kortex_driver::GripperCyclic_Command
  :super ros::object
  :slots (_command_id _flags _motor_cmd ))

(defmethod kortex_driver::GripperCyclic_Command
  (:init
   (&key
    ((:command_id __command_id) (instance kortex_driver::GripperCyclic_MessageId :init))
    ((:flags __flags) 0)
    ((:motor_cmd __motor_cmd) ())
    )
   (send-super :init)
   (setq _command_id __command_id)
   (setq _flags (round __flags))
   (setq _motor_cmd __motor_cmd)
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
  (:motor_cmd
   (&rest __motor_cmd)
   (if (keywordp (car __motor_cmd))
       (send* _motor_cmd __motor_cmd)
     (progn
       (if __motor_cmd (setq _motor_cmd (car __motor_cmd)))
       _motor_cmd)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/GripperCyclic_MessageId _command_id
    (send _command_id :serialization-length)
    ;; uint32 _flags
    4
    ;; kortex_driver/MotorCommand[] _motor_cmd
    (apply #'+ (send-all _motor_cmd :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/GripperCyclic_MessageId _command_id
       (send _command_id :serialize s)
     ;; uint32 _flags
       (write-long _flags s)
     ;; kortex_driver/MotorCommand[] _motor_cmd
     (write-long (length _motor_cmd) s)
     (dolist (elem _motor_cmd)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/GripperCyclic_MessageId _command_id
     (send _command_id :deserialize buf ptr-) (incf ptr- (send _command_id :serialization-length))
   ;; uint32 _flags
     (setq _flags (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/MotorCommand[] _motor_cmd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _motor_cmd (let (r) (dotimes (i n) (push (instance kortex_driver::MotorCommand :init) r)) r))
     (dolist (elem- _motor_cmd)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::GripperCyclic_Command :md5sum-) "aa88200f31c06368a2a9758c0985b419")
(setf (get kortex_driver::GripperCyclic_Command :datatype-) "kortex_driver/GripperCyclic_Command")
(setf (get kortex_driver::GripperCyclic_Command :definition-)
      "
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



(provide :kortex_driver/GripperCyclic_Command "aa88200f31c06368a2a9758c0985b419")


