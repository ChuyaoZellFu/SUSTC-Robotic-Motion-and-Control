;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControllerConfigurationList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControllerConfigurationList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLLERCONFIGURATIONLIST")
  (make-package "KORTEX_DRIVER::CONTROLLERCONFIGURATIONLIST"))

(in-package "ROS")
;;//! \htmlinclude ControllerConfigurationList.msg.html


(defclass kortex_driver::ControllerConfigurationList
  :super ros::object
  :slots (_controller_configurations ))

(defmethod kortex_driver::ControllerConfigurationList
  (:init
   (&key
    ((:controller_configurations __controller_configurations) ())
    )
   (send-super :init)
   (setq _controller_configurations __controller_configurations)
   self)
  (:controller_configurations
   (&rest __controller_configurations)
   (if (keywordp (car __controller_configurations))
       (send* _controller_configurations __controller_configurations)
     (progn
       (if __controller_configurations (setq _controller_configurations (car __controller_configurations)))
       _controller_configurations)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/ControllerConfiguration[] _controller_configurations
    (apply #'+ (send-all _controller_configurations :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControllerConfiguration[] _controller_configurations
     (write-long (length _controller_configurations) s)
     (dolist (elem _controller_configurations)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControllerConfiguration[] _controller_configurations
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _controller_configurations (let (r) (dotimes (i n) (push (instance kortex_driver::ControllerConfiguration :init) r)) r))
     (dolist (elem- _controller_configurations)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::ControllerConfigurationList :md5sum-) "4e505f81204befaff98df48e637201d5")
(setf (get kortex_driver::ControllerConfigurationList :datatype-) "kortex_driver/ControllerConfigurationList")
(setf (get kortex_driver::ControllerConfigurationList :definition-)
      "
ControllerConfiguration[] controller_configurations
================================================================================
MSG: kortex_driver/ControllerConfiguration

ControllerHandle handle
string name
MappingHandle active_mapping_handle
string analog_input_identifier_enum
string digital_input_identifier_enum
================================================================================
MSG: kortex_driver/ControllerHandle

uint32 type
uint32 controller_identifier
================================================================================
MSG: kortex_driver/MappingHandle

uint32 identifier
uint32 permission
")



(provide :kortex_driver/ControllerConfigurationList "4e505f81204befaff98df48e637201d5")


