;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::OperatingModeInformation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'OperatingModeInformation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::OPERATINGMODEINFORMATION")
  (make-package "KORTEX_DRIVER::OPERATINGMODEINFORMATION"))

(in-package "ROS")
;;//! \htmlinclude OperatingModeInformation.msg.html


(defclass kortex_driver::OperatingModeInformation
  :super ros::object
  :slots (_operating_mode _device_handle ))

(defmethod kortex_driver::OperatingModeInformation
  (:init
   (&key
    ((:operating_mode __operating_mode) 0)
    ((:device_handle __device_handle) (instance kortex_driver::DeviceHandle :init))
    )
   (send-super :init)
   (setq _operating_mode (round __operating_mode))
   (setq _device_handle __device_handle)
   self)
  (:operating_mode
   (&optional __operating_mode)
   (if __operating_mode (setq _operating_mode __operating_mode)) _operating_mode)
  (:device_handle
   (&rest __device_handle)
   (if (keywordp (car __device_handle))
       (send* _device_handle __device_handle)
     (progn
       (if __device_handle (setq _device_handle (car __device_handle)))
       _device_handle)))
  (:serialization-length
   ()
   (+
    ;; uint32 _operating_mode
    4
    ;; kortex_driver/DeviceHandle _device_handle
    (send _device_handle :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _operating_mode
       (write-long _operating_mode s)
     ;; kortex_driver/DeviceHandle _device_handle
       (send _device_handle :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _operating_mode
     (setq _operating_mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; kortex_driver/DeviceHandle _device_handle
     (send _device_handle :deserialize buf ptr-) (incf ptr- (send _device_handle :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::OperatingModeInformation :md5sum-) "c45e380eac0023b4784c990d000302a3")
(setf (get kortex_driver::OperatingModeInformation :datatype-) "kortex_driver/OperatingModeInformation")
(setf (get kortex_driver::OperatingModeInformation :definition-)
      "
uint32 operating_mode
DeviceHandle device_handle
================================================================================
MSG: kortex_driver/DeviceHandle

uint32 device_type
uint32 device_identifier
uint32 order
")



(provide :kortex_driver/OperatingModeInformation "c45e380eac0023b4784c990d000302a3")


