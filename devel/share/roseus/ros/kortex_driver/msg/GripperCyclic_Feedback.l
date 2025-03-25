;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GripperCyclic_Feedback)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GripperCyclic_Feedback (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GRIPPERCYCLIC_FEEDBACK")
  (make-package "KORTEX_DRIVER::GRIPPERCYCLIC_FEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude GripperCyclic_Feedback.msg.html


(defclass kortex_driver::GripperCyclic_Feedback
  :super ros::object
  :slots (_feedback_id _status_flags _fault_bank_a _fault_bank_b _warning_bank_a _warning_bank_b _motor ))

(defmethod kortex_driver::GripperCyclic_Feedback
  (:init
   (&key
    ((:feedback_id __feedback_id) (instance kortex_driver::GripperCyclic_MessageId :init))
    ((:status_flags __status_flags) 0)
    ((:fault_bank_a __fault_bank_a) 0)
    ((:fault_bank_b __fault_bank_b) 0)
    ((:warning_bank_a __warning_bank_a) 0)
    ((:warning_bank_b __warning_bank_b) 0)
    ((:motor __motor) ())
    )
   (send-super :init)
   (setq _feedback_id __feedback_id)
   (setq _status_flags (round __status_flags))
   (setq _fault_bank_a (round __fault_bank_a))
   (setq _fault_bank_b (round __fault_bank_b))
   (setq _warning_bank_a (round __warning_bank_a))
   (setq _warning_bank_b (round __warning_bank_b))
   (setq _motor __motor)
   self)
  (:feedback_id
   (&rest __feedback_id)
   (if (keywordp (car __feedback_id))
       (send* _feedback_id __feedback_id)
     (progn
       (if __feedback_id (setq _feedback_id (car __feedback_id)))
       _feedback_id)))
  (:status_flags
   (&optional __status_flags)
   (if __status_flags (setq _status_flags __status_flags)) _status_flags)
  (:fault_bank_a
   (&optional __fault_bank_a)
   (if __fault_bank_a (setq _fault_bank_a __fault_bank_a)) _fault_bank_a)
  (:fault_bank_b
   (&optional __fault_bank_b)
   (if __fault_bank_b (setq _fault_bank_b __fault_bank_b)) _fault_bank_b)
  (:warning_bank_a
   (&optional __warning_bank_a)
   (if __warning_bank_a (setq _warning_bank_a __warning_bank_a)) _warning_bank_a)
  (:warning_bank_b
   (&optional __warning_bank_b)
   (if __warning_bank_b (setq _warning_bank_b __warning_bank_b)) _warning_bank_b)
  (:motor
   (&rest __motor)
   (if (keywordp (car __motor))
       (send* _motor __motor)
     (progn
       (if __motor (setq _motor (car __motor)))
       _motor)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/GripperCyclic_MessageId _feedback_id
    (send _feedback_id :serialization-length)
    ;; uint32 _status_flags
    4
    ;; uint32 _fault_bank_a
    4
    ;; uint32 _fault_bank_b
    4
    ;; uint32 _warning_bank_a
    4
    ;; uint32 _warning_bank_b
    4
    ;; kortex_driver/MotorFeedback[] _motor
    (apply #'+ (send-all _motor :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/GripperCyclic_MessageId _feedback_id
       (send _feedback_id :serialize s)
     ;; uint32 _status_flags
       (write-long _status_flags s)
     ;; uint32 _fault_bank_a
       (write-long _fault_bank_a s)
     ;; uint32 _fault_bank_b
       (write-long _fault_bank_b s)
     ;; uint32 _warning_bank_a
       (write-long _warning_bank_a s)
     ;; uint32 _warning_bank_b
       (write-long _warning_bank_b s)
     ;; kortex_driver/MotorFeedback[] _motor
     (write-long (length _motor) s)
     (dolist (elem _motor)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/GripperCyclic_MessageId _feedback_id
     (send _feedback_id :deserialize buf ptr-) (incf ptr- (send _feedback_id :serialization-length))
   ;; uint32 _status_flags
     (setq _status_flags (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _fault_bank_a
     (setq _fault_bank_a (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _fault_bank_b
     (setq _fault_bank_b (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _warning_bank_a
     (setq _warning_bank_a (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _warning_bank_b
     (setq _warning_bank_b (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/MotorFeedback[] _motor
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _motor (let (r) (dotimes (i n) (push (instance kortex_driver::MotorFeedback :init) r)) r))
     (dolist (elem- _motor)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::GripperCyclic_Feedback :md5sum-) "ec3639b6684e8f718fd3128a8215e686")
(setf (get kortex_driver::GripperCyclic_Feedback :datatype-) "kortex_driver/GripperCyclic_Feedback")
(setf (get kortex_driver::GripperCyclic_Feedback :definition-)
      "
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



(provide :kortex_driver/GripperCyclic_Feedback "ec3639b6684e8f718fd3128a8215e686")


