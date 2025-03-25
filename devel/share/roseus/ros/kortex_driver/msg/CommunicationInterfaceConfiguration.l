;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CommunicationInterfaceConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CommunicationInterfaceConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::COMMUNICATIONINTERFACECONFIGURATION")
  (make-package "KORTEX_DRIVER::COMMUNICATIONINTERFACECONFIGURATION"))

(in-package "ROS")
;;//! \htmlinclude CommunicationInterfaceConfiguration.msg.html


(defclass kortex_driver::CommunicationInterfaceConfiguration
  :super ros::object
  :slots (_type _enable ))

(defmethod kortex_driver::CommunicationInterfaceConfiguration
  (:init
   (&key
    ((:type __type) 0)
    ((:enable __enable) nil)
    )
   (send-super :init)
   (setq _type (round __type))
   (setq _enable __enable)
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:enable
   (&optional (__enable :null))
   (if (not (eq __enable :null)) (setq _enable __enable)) _enable)
  (:serialization-length
   ()
   (+
    ;; uint32 _type
    4
    ;; bool _enable
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _type
       (write-long _type s)
     ;; bool _enable
       (if _enable (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _type
     (setq _type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _enable
     (setq _enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get kortex_driver::CommunicationInterfaceConfiguration :md5sum-) "5a1523f2cb0646f5be12fd2c14b0a2a7")
(setf (get kortex_driver::CommunicationInterfaceConfiguration :datatype-) "kortex_driver/CommunicationInterfaceConfiguration")
(setf (get kortex_driver::CommunicationInterfaceConfiguration :definition-)
      "
uint32 type
bool enable
")



(provide :kortex_driver/CommunicationInterfaceConfiguration "5a1523f2cb0646f5be12fd2c14b0a2a7")


