;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SafetyConfigurationList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SafetyConfigurationList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SAFETYCONFIGURATIONLIST")
  (make-package "KORTEX_DRIVER::SAFETYCONFIGURATIONLIST"))

(in-package "ROS")
;;//! \htmlinclude SafetyConfigurationList.msg.html


(defclass kortex_driver::SafetyConfigurationList
  :super ros::object
  :slots (_configuration ))

(defmethod kortex_driver::SafetyConfigurationList
  (:init
   (&key
    ((:configuration __configuration) ())
    )
   (send-super :init)
   (setq _configuration __configuration)
   self)
  (:configuration
   (&rest __configuration)
   (if (keywordp (car __configuration))
       (send* _configuration __configuration)
     (progn
       (if __configuration (setq _configuration (car __configuration)))
       _configuration)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/SafetyConfiguration[] _configuration
    (apply #'+ (send-all _configuration :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SafetyConfiguration[] _configuration
     (write-long (length _configuration) s)
     (dolist (elem _configuration)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SafetyConfiguration[] _configuration
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _configuration (let (r) (dotimes (i n) (push (instance kortex_driver::SafetyConfiguration :init) r)) r))
     (dolist (elem- _configuration)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::SafetyConfigurationList :md5sum-) "8178a2a82961f46646c77423c6510e4d")
(setf (get kortex_driver::SafetyConfigurationList :datatype-) "kortex_driver/SafetyConfigurationList")
(setf (get kortex_driver::SafetyConfigurationList :definition-)
      "
SafetyConfiguration[] configuration
================================================================================
MSG: kortex_driver/SafetyConfiguration

SafetyHandle handle
float32 error_threshold
float32 warning_threshold
SafetyEnable enable
================================================================================
MSG: kortex_driver/SafetyHandle

uint32 identifier
================================================================================
MSG: kortex_driver/SafetyEnable

SafetyHandle handle
bool enable
")



(provide :kortex_driver/SafetyConfigurationList "8178a2a82961f46646c77423c6510e4d")


