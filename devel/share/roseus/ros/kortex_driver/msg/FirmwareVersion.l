;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::FirmwareVersion)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'FirmwareVersion (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::FIRMWAREVERSION")
  (make-package "KORTEX_DRIVER::FIRMWAREVERSION"))

(in-package "ROS")
;;//! \htmlinclude FirmwareVersion.msg.html


(defclass kortex_driver::FirmwareVersion
  :super ros::object
  :slots (_firmware_version ))

(defmethod kortex_driver::FirmwareVersion
  (:init
   (&key
    ((:firmware_version __firmware_version) 0)
    )
   (send-super :init)
   (setq _firmware_version (round __firmware_version))
   self)
  (:firmware_version
   (&optional __firmware_version)
   (if __firmware_version (setq _firmware_version __firmware_version)) _firmware_version)
  (:serialization-length
   ()
   (+
    ;; uint32 _firmware_version
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _firmware_version
       (write-long _firmware_version s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _firmware_version
     (setq _firmware_version (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::FirmwareVersion :md5sum-) "f0cdf3021edc70c033ace598cc19b3d0")
(setf (get kortex_driver::FirmwareVersion :datatype-) "kortex_driver/FirmwareVersion")
(setf (get kortex_driver::FirmwareVersion :definition-)
      "
uint32 firmware_version
")



(provide :kortex_driver/FirmwareVersion "f0cdf3021edc70c033ace598cc19b3d0")


