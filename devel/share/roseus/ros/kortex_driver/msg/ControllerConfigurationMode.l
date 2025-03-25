;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControllerConfigurationMode)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControllerConfigurationMode (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLLERCONFIGURATIONMODE")
  (make-package "KORTEX_DRIVER::CONTROLLERCONFIGURATIONMODE"))

(in-package "ROS")
;;//! \htmlinclude ControllerConfigurationMode.msg.html


(defclass kortex_driver::ControllerConfigurationMode
  :super ros::object
  :slots (_enable ))

(defmethod kortex_driver::ControllerConfigurationMode
  (:init
   (&key
    ((:enable __enable) nil)
    )
   (send-super :init)
   (setq _enable __enable)
   self)
  (:enable
   (&optional (__enable :null))
   (if (not (eq __enable :null)) (setq _enable __enable)) _enable)
  (:serialization-length
   ()
   (+
    ;; bool _enable
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _enable
       (if _enable (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _enable
     (setq _enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get kortex_driver::ControllerConfigurationMode :md5sum-) "8c1211af706069c994c06e00eb59ac9e")
(setf (get kortex_driver::ControllerConfigurationMode :datatype-) "kortex_driver/ControllerConfigurationMode")
(setf (get kortex_driver::ControllerConfigurationMode :definition-)
      "
bool enable
")



(provide :kortex_driver/ControllerConfigurationMode "8c1211af706069c994c06e00eb59ac9e")


