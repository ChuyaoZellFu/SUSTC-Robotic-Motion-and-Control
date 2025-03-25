;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::InterconnectCyclic_Feedback_tool_feedback)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'InterconnectCyclic_Feedback_tool_feedback (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::INTERCONNECTCYCLIC_FEEDBACK_TOOL_FEEDBACK")
  (make-package "KORTEX_DRIVER::INTERCONNECTCYCLIC_FEEDBACK_TOOL_FEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude InterconnectCyclic_Feedback_tool_feedback.msg.html


(defclass kortex_driver::InterconnectCyclic_Feedback_tool_feedback
  :super ros::object
  :slots (_gripper_feedback ))

(defmethod kortex_driver::InterconnectCyclic_Feedback_tool_feedback
  (:init
   (&key
    ((:gripper_feedback __gripper_feedback) ())
    )
   (send-super :init)
   (setq _gripper_feedback __gripper_feedback)
   self)
  (:gripper_feedback
   (&rest __gripper_feedback)
   (if (keywordp (car __gripper_feedback))
       (send* _gripper_feedback __gripper_feedback)
     (progn
       (if __gripper_feedback (setq _gripper_feedback (car __gripper_feedback)))
       _gripper_feedback)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/GripperCyclic_Feedback[] _gripper_feedback
    (apply #'+ (send-all _gripper_feedback :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/GripperCyclic_Feedback[] _gripper_feedback
     (write-long (length _gripper_feedback) s)
     (dolist (elem _gripper_feedback)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/GripperCyclic_Feedback[] _gripper_feedback
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _gripper_feedback (let (r) (dotimes (i n) (push (instance kortex_driver::GripperCyclic_Feedback :init) r)) r))
     (dolist (elem- _gripper_feedback)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::InterconnectCyclic_Feedback_tool_feedback :md5sum-) "39e2425dcc897a387ad5c442c6119c99")
(setf (get kortex_driver::InterconnectCyclic_Feedback_tool_feedback :datatype-) "kortex_driver/InterconnectCyclic_Feedback_tool_feedback")
(setf (get kortex_driver::InterconnectCyclic_Feedback_tool_feedback :definition-)
      "
GripperCyclic_Feedback[] gripper_feedback
================================================================================
MSG: kortex_driver/GripperCyclic_Feedback

GripperCyclic_MessageId feedback_id
uint32 status_flags
uint32 fault_bank_a
uint32 fault_bank_b
uint32 warning_bank_a
uint32 warning_bank_b
MotorFeedback[] motor
================================================================================
MSG: kortex_driver/GripperCyclic_MessageId

uint32 identifier
================================================================================
MSG: kortex_driver/MotorFeedback

uint32 motor_id
float32 position
float32 velocity
float32 current_motor
float32 voltage
float32 temperature_motor
")



(provide :kortex_driver/InterconnectCyclic_Feedback_tool_feedback "39e2425dcc897a387ad5c442c6119c99")


