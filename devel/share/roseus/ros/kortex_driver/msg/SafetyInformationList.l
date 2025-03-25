;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SafetyInformationList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SafetyInformationList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SAFETYINFORMATIONLIST")
  (make-package "KORTEX_DRIVER::SAFETYINFORMATIONLIST"))

(in-package "ROS")
;;//! \htmlinclude SafetyInformationList.msg.html


(defclass kortex_driver::SafetyInformationList
  :super ros::object
  :slots (_information ))

(defmethod kortex_driver::SafetyInformationList
  (:init
   (&key
    ((:information __information) ())
    )
   (send-super :init)
   (setq _information __information)
   self)
  (:information
   (&rest __information)
   (if (keywordp (car __information))
       (send* _information __information)
     (progn
       (if __information (setq _information (car __information)))
       _information)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/SafetyInformation[] _information
    (apply #'+ (send-all _information :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SafetyInformation[] _information
     (write-long (length _information) s)
     (dolist (elem _information)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SafetyInformation[] _information
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _information (let (r) (dotimes (i n) (push (instance kortex_driver::SafetyInformation :init) r)) r))
     (dolist (elem- _information)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::SafetyInformationList :md5sum-) "c4049b5c5ed2e1815e6545bc46f6cf34")
(setf (get kortex_driver::SafetyInformationList :datatype-) "kortex_driver/SafetyInformationList")
(setf (get kortex_driver::SafetyInformationList :definition-)
      "
SafetyInformation[] information
================================================================================
MSG: kortex_driver/SafetyInformation

SafetyHandle handle
bool can_change_safety_state
bool has_warning_threshold
bool has_error_threshold
uint32 limit_type
float32 default_warning_threshold
float32 default_error_threshold
float32 upper_hard_limit
float32 lower_hard_limit
uint32 status
uint32 unit
================================================================================
MSG: kortex_driver/SafetyHandle

uint32 identifier
")



(provide :kortex_driver/SafetyInformationList "c4049b5c5ed2e1815e6545bc46f6cf34")


